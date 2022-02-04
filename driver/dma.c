// SPDX-License-Identifier: BSD-2-Clause

/*
 * LitePCIe driver
 *
 * This file is part of LitePCIe.
 *
 * Copyright (C) 2018-2020 / EnjoyDigital  / florent@enjoy-digital.fr
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/mmtimer.h>
#include <linux/miscdevice.h>
#include <linux/posix-timers.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/math64.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/log2.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/of_net.h>
#include <linux/etherdevice.h>

#include "litepcie.h"
#include "csr.h"
#include "config.h"
#include "flags.h"
#include "mem.h"

struct liteeth {
        void __iomem *base;
        struct net_device *netdev;
//        struct device *dev;
        u32 slot_size;

        /* Polling support */
        int use_polling;
        struct timer_list poll_timer;

        /* Tx */
        u32 tx_slot;
        u32 num_tx_slots;
        void __iomem *tx_base;
	dma_addr_t tx_base_dma;

        /* Rx */
        u32 num_rx_slots;
        void __iomem *rx_base;
	dma_addr_t rx_base_dma;

	struct litepcie_device *lpdev;
	atomic_t tx_pending;
};

#define LITEPCIE_NAME "litepcie"
#define LITEPCIE_MINOR_COUNT 32
#define LITEPCIE_DMA_BUF_SIZE ((ETHMAC_RX_SLOTS + ETHMAC_TX_SLOTS) * ETHMAC_SLOT_SIZE)
#define ALIGN_SIZE (512)

static u8 mac_addr[] = {0x12, 0x2e, 0x60, 0xbe, 0xef, 0xbb};

struct litepcie_device {
	struct pci_dev *dev;
	struct platform_device *uart;
	resource_size_t bar0_size;
	phys_addr_t bar0_phys_addr;
	uint8_t *bar0_addr; /* virtual address of BAR0 */
	spinlock_t lock;
	int irqs;
	dma_addr_t dma_addr;
	void* host_dma_addr;
	struct work_struct irq0_work;
	struct work_struct irq1_work;
	struct liteeth *ethdev;
};


static inline uint32_t litepcie_readl(struct litepcie_device *s, uint32_t addr)
{
	uint32_t val;

	val = readl(s->bar0_addr + addr - CSR_BASE);
	return val;
}

static inline void litepcie_writel(struct litepcie_device *s, uint32_t addr, uint32_t val)
{
	return writel(val, s->bar0_addr + addr - CSR_BASE);
}

static void litepcie_enable_interrupt(struct litepcie_device *s, int irq_num)
{
	uint32_t v;

	v = litepcie_readl(s, CSR_PCIE_MSI_ENABLE_ADDR);
	v |= (1 << irq_num);
	litepcie_writel(s, CSR_PCIE_MSI_ENABLE_ADDR, v);
}

static void litepcie_disable_interrupt(struct litepcie_device *s, int irq_num)
{
	uint32_t v;

	v = litepcie_readl(s, CSR_PCIE_MSI_ENABLE_ADDR);
	v &= ~(1 << irq_num);
	litepcie_writel(s, CSR_PCIE_MSI_ENABLE_ADDR, v);
}

static int liteeth_open(struct net_device *netdev)
{
        struct liteeth *priv = netdev_priv(netdev);
	int err;
	netdev_info(netdev,"liteeth_open\n");
	netif_carrier_on(netdev);
	netif_start_queue(netdev);
	litepcie_writel(priv->lpdev,CSR_ETHMAC_SRAM_WRITER_ENABLE_ADDR,1);
	litepcie_writel(priv->lpdev, CSR_PCIE_HOST_WB2PCIE_DMA_HOST_BASE_ADDR_ADDR, priv->rx_base_dma);
	litepcie_writel(priv->lpdev, CSR_PCIE_HOST_PCIE2WB_DMA_HOST_BASE_ADDR_ADDR, priv->tx_base_dma);

	return 0;
}

static int liteeth_stop(struct net_device *netdev)
{
	struct liteeth *priv = netdev_priv(netdev);

	netdev_info(netdev,"liteeth_stop\n");
	litepcie_writel(priv->lpdev,CSR_ETHMAC_SRAM_WRITER_ENABLE_ADDR,0);
	netif_stop_queue(netdev);
	netif_carrier_off(netdev);

	if (priv->use_polling) {
		del_timer_sync(&priv->poll_timer);
	}

	return 0;
}

static int liteeth_start_xmit(struct sk_buff *skb, struct net_device *netdev)
{
	struct liteeth *priv = netdev_priv(netdev);
	struct litepcie_device *lpdev = priv->lpdev;
	void __iomem *txbuffer;

	if (!litepcie_readl(lpdev,CSR_ETHMAC_SRAM_READER_READY_ADDR)) {
		if (net_ratelimit())
			netdev_err(netdev, "LITEETH_READER_READY not ready\n");
		goto busy;
	}
	
	/* Reject oversize packets */
	if (unlikely(skb->len > priv->slot_size)) {
		if (net_ratelimit())
			netdev_err(netdev, "tx packet too big\n");

		dev_kfree_skb_any(skb);
		netdev->stats.tx_dropped++;
		netdev->stats.tx_errors++;

		return NETDEV_TX_OK;
	}
	while (atomic_cmpxchg(&priv->tx_pending,0,1));

	txbuffer = priv->tx_base + priv->tx_slot * priv->slot_size;
	memcpy(txbuffer, skb->data, skb->len);
	litepcie_writel(lpdev, CSR_ETHMAC_SRAM_READER_SLOT_ADDR, priv->tx_slot);
	litepcie_writel(lpdev, CSR_ETHMAC_SRAM_READER_LENGTH_ADDR, skb->len);
	litepcie_writel(lpdev, CSR_ETHMAC_SRAM_READER_START_ADDR, 1);

	priv->tx_slot = (priv->tx_slot + 1) % priv->num_tx_slots;

	netdev->stats.tx_bytes += skb->len;
	netdev->stats.tx_packets++;

	dev_kfree_skb_any(skb);
	
	return NETDEV_TX_OK;

busy:                
	netif_stop_queue(netdev);
	return NETDEV_TX_BUSY;
}
static const struct net_device_ops liteeth_netdev_ops = {
	.ndo_open		= liteeth_open,
	.ndo_stop		= liteeth_stop,
	.ndo_start_xmit         = liteeth_start_xmit,
};

static void handle_ethtx_interrupt(struct net_device *netdev)
{
	struct liteeth *priv = netdev_priv(netdev);
	struct litepcie_device *lpdev = priv->lpdev;
	u32 reg;

	reg = litepcie_readl(lpdev,CSR_ETHMAC_SRAM_READER_READY_ADDR);
	if (reg) {
		if (netif_queue_stopped(netdev))
			netif_wake_queue(netdev);
	}
}
static int handle_ethrx_interrupt(struct net_device *netdev)
{
 	struct liteeth *priv = netdev_priv(netdev);
 	struct litepcie_device *lpdev = priv->lpdev;
        struct sk_buff *skb;
        unsigned char *data;
        u32 rx_slot;
        int len;

        rx_slot = litepcie_readl(lpdev,CSR_ETHMAC_SRAM_WRITER_SLOT_ADDR);
        len = litepcie_readl(lpdev,CSR_ETHMAC_SRAM_WRITER_LENGTH_ADDR);

        skb = netdev_alloc_skb_ip_align(netdev, len);
        if (!skb) {
		litepcie_writel(lpdev,CSR_ETHMAC_SRAM_WRITER_ACK_ADDR, 1);
                netdev_err(netdev, "couldn't get memory\n");
                goto rx_drop;
        }

        data = skb_put(skb, len);
        memcpy(data, priv->rx_base + rx_slot * priv->slot_size, len);
	
	litepcie_writel(lpdev,CSR_ETHMAC_SRAM_WRITER_ACK_ADDR, 1);
        skb->protocol = eth_type_trans(skb, netdev);

        netdev->stats.rx_packets++;
        netdev->stats.rx_bytes += len;

        return netif_rx(skb);
rx_drop:
        netdev->stats.rx_dropped++;
        netdev->stats.rx_errors++;

        return NET_RX_DROP;
}
static void handle_txdata_interrupt(struct net_device *netdev)
{
	struct liteeth *priv = netdev_priv(netdev);
	struct litepcie_device *lpdev = priv->lpdev;
	WARN_ON(atomic_read(&priv->tx_pending) == 0);
	atomic_set(&priv->tx_pending, 0);
}

static irqreturn_t litepcie_interrupt(int irq, void *data)
{
	struct litepcie_device *s = (struct litepcie_device *) data;
	struct net_device *netdev = s->ethdev->netdev;
	uint32_t irq_vector, irq_enable;
	int i;

	irq_vector = 0;
	for (i = 0; i < s->irqs; i++) {
		if (irq == pci_irq_vector(s->dev, i)) {
			irq_vector = (1 << i);
			break;
		}
	}

	if (i == ETHRX_INTERRUPT)
		handle_ethrx_interrupt(netdev);
	if (i == ETHTX_INTERRUPT)
		handle_ethtx_interrupt(netdev);
        if (i == TXDATA_INTERRUPT)
	 	handle_txdata_interrupt(netdev);	
	
	return IRQ_HANDLED;
}

static void liteeth_setup_slots(struct liteeth *priv)
{
	priv->num_rx_slots = ETHMAC_RX_SLOTS;
	priv->num_tx_slots = ETHMAC_TX_SLOTS;
	priv->slot_size = ETHMAC_SLOT_SIZE;
}

static int liteeth_init(struct litepcie_device *lpdev)
{
	struct net_device *netdev;
	struct pci_dev *pdev;
	struct liteeth *priv;
	int err;

	pdev = lpdev->dev;
	netdev = devm_alloc_etherdev(&pdev->dev, sizeof(*priv));
	if (!netdev)
		return -ENOMEM;

	SET_NETDEV_DEV(netdev, &pdev->dev);

	priv = netdev_priv(netdev);
	priv->netdev = netdev;

	lpdev->ethdev = priv;
	priv->lpdev = lpdev;

	priv->use_polling = 0;
	//irq = pci_irq_vector(lpdev->dev,IRQ1_INTERRUPT); //FIXME:Use ethernet IRQ connected to MSI
	//netdev->irq = irq;

	liteeth_setup_slots(priv);

	/* Rx slots */
	priv->rx_base = lpdev->host_dma_addr;
	priv->rx_base_dma = lpdev->dma_addr;

	/* Tx slots come after Rx slots */
	priv->tx_base = priv->rx_base + priv->num_rx_slots * priv->slot_size;
	priv->tx_base_dma = priv->rx_base_dma + priv->num_rx_slots * priv->slot_size;
	priv->tx_slot = 0;

	memcpy(netdev->dev_addr, mac_addr, ETH_ALEN);

	netdev->netdev_ops = &liteeth_netdev_ops;

	err = register_netdev(netdev);
	if (err) {
		dev_err(&pdev->dev, "Failed to register netdev %d\n", err);
		return err;		//FIXME:add post-clean if error occurs
	}

	netdev_info(netdev, "irq %d slots: tx %d rx %d size %d\n",
		    netdev->irq, priv->num_tx_slots, priv->num_rx_slots, priv->slot_size);

	return 0;
}
static int litepcie_pci_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int ret = 0;
	int irqs = 0;
	uint8_t rev_id;
	int i;
	char fpga_identifier[256];
	struct litepcie_device *litepcie_dev = NULL;

	dev_info(&dev->dev, "\e[1m[Probing device]\e[0m\n");

	litepcie_dev = devm_kzalloc(&dev->dev, sizeof(struct litepcie_device), GFP_KERNEL);
	if (!litepcie_dev) {
		ret = -ENOMEM;
		goto fail1;
	}

	pci_set_drvdata(dev, litepcie_dev);
	litepcie_dev->dev = dev;

	ret = pcim_enable_device(dev);
	if (ret != 0) {
		dev_err(&dev->dev, "Cannot enable device\n");
		goto fail1;
	}

	ret = -EIO;

	/* check device version */
	pci_read_config_byte(dev, PCI_REVISION_ID, &rev_id);
	if (rev_id != 0) {
		dev_err(&dev->dev, "Unsupported device version %d\n", rev_id);
		goto fail1;
	}

	/* check bar0 config */
	if (!(pci_resource_flags(dev, 0) & IORESOURCE_MEM)) {
		dev_err(&dev->dev, "Invalid BAR0 configuration\n");
		goto fail1;
	}

	if (pcim_iomap_regions(dev, BIT(0), LITEPCIE_NAME) < 0) {
		dev_err(&dev->dev, "Could not request regions\n");
		goto fail1;
	}

	litepcie_dev->bar0_addr = pcim_iomap_table(dev)[0];
	if (!litepcie_dev->bar0_addr) {
		dev_err(&dev->dev, "Could not map BAR0\n");
		goto fail1;
	}

	/* show identifier */
	for (i = 0; i < 256; i++)
		fpga_identifier[i] = litepcie_readl(litepcie_dev, CSR_IDENTIFIER_MEM_BASE + i*4);
	dev_info(&dev->dev, "Version %s\n", fpga_identifier);

	pci_set_master(dev);
	ret = pci_set_dma_mask(dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&dev->dev, "Failed to set DMA mask\n");
		goto fail1;
	};

	irqs = pci_alloc_irq_vectors(dev, 1, 32, PCI_IRQ_MSI);
	if (irqs < 0) {
		dev_err(&dev->dev, "Failed to enable MSI\n");
		ret = irqs;
		goto fail1;
	}
	dev_info(&dev->dev, "%d MSI IRQs allocated.\n", irqs);

	litepcie_dev->irqs = 0;
	for (i = 0; i < irqs; i++) {
		int irq = pci_irq_vector(dev, i);

		ret = request_irq(irq, litepcie_interrupt, IRQF_SHARED, LITEPCIE_NAME, litepcie_dev);
		if (ret < 0) {
			dev_err(&dev->dev, " Failed to allocate IRQ %d\n", dev->irq);
			while (--i >= 0) {
				irq = pci_irq_vector(dev, i);
				free_irq(irq, dev);
			}
			goto fail2;
		}
		litepcie_dev->irqs += 1;
	}
	
	litepcie_dev->host_dma_addr = dma_alloc_coherent(
				&dev->dev,
				LITEPCIE_DMA_BUF_SIZE,
				&litepcie_dev->dma_addr,
				GFP_DMA32);
	if (!litepcie_dev->host_dma_addr){
		ret = -ENOMEM;
		goto fail2;
	}

	liteeth_init(litepcie_dev);

	pci_info(dev,"dma addr 0x%x, host dma addr 0x%x, PAGE_SHIFT:%d, BUF_SIZE: %d\n",litepcie_dev->dma_addr,litepcie_dev->host_dma_addr,PAGE_SHIFT,LITEPCIE_DMA_BUF_SIZE);

	return 0;

fail2:
	pci_free_irq_vectors(dev);
fail1:
	return ret;
}

static void litepcie_pci_remove(struct pci_dev *dev)
{
	int i, irq;
	struct litepcie_device *litepcie_dev;

	litepcie_dev = pci_get_drvdata(dev);

	dev_info(&dev->dev, "\e[1m[Removing device]\e[0m\n");

	/* Disable all interrupts */
	litepcie_writel(litepcie_dev, CSR_PCIE_MSI_ENABLE_ADDR, 0);

	/* Free all interrupts */
	for (i = 0; i < litepcie_dev->irqs; i++) {
		irq = pci_irq_vector(dev, i);
		free_irq(irq, litepcie_dev);
	}
	dma_free_coherent(&dev->dev,LITEPCIE_DMA_BUF_SIZE,litepcie_dev->host_dma_addr,litepcie_dev->dma_addr);
	pci_free_irq_vectors(dev);
}

static const struct pci_device_id litepcie_pci_ids[] = {
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_S7_GEN2_X1), },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_S7_GEN2_X2), },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_S7_GEN2_X4), },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_S7_GEN2_X8), },

	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_US_GEN2_X1), },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_US_GEN2_X2), },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_US_GEN2_X4), },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_US_GEN2_X8), },

	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_US_GEN3_X1), },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_US_GEN3_X2), },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_US_GEN3_X4), },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_US_GEN3_X8), },

	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN2_X1),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN2_X2),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN2_X4),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN2_X8),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN2_X16), },

	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN3_X1),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN3_X2),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN3_X4),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN3_X8),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN3_X16), },

	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN4_X1),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN4_X2),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN4_X4),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN4_X8),  },

	{ 0, }
};
MODULE_DEVICE_TABLE(pci, litepcie_pci_ids);

static struct pci_driver litepcie_pci_driver = {
	.name = LITEPCIE_NAME,
	.id_table = litepcie_pci_ids,
	.probe = litepcie_pci_probe,
	.remove = litepcie_pci_remove,
};


static int __init litepcie_module_init(void)
{
	int ret;
	ret = pci_register_driver(&litepcie_pci_driver);
	if (ret < 0) {
		pr_err(" Error while registering PCI driver\n");
	}

	return ret;
}

static void __exit litepcie_module_exit(void)
{
	pci_unregister_driver(&litepcie_pci_driver);
}


module_init(litepcie_module_init);
module_exit(litepcie_module_exit);

MODULE_LICENSE("GPL");
