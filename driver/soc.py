    def add_ethernet_pcie(self, name="ethmac", phy=None, pcie_phy=None, phy_cd="eth", dynamic_ip=False,
                          software_debug=False,
                          nrxslots=32,
                          ntxslots=32,
                          with_timing_constraints=True,
                          max_pending_requests=8,
                          with_msi=True):
        # Imports
        from liteeth.mac import LiteEthMAC
        from liteeth.phy.model import LiteEthPHYModel
        data_width=128
        self.submodules.pcie_mem_bus_rx = SoCBusHandler(
            data_width=data_width
        )
        self.submodules.pcie_mem_bus_tx = SoCBusHandler(
            data_width=data_width
        )
        # MAC.
        self.check_if_exists(name)
        ethmac = LiteEthMAC(
            phy=phy,
            dw=data_width,
            interface="wishbone",
            endianness=self.cpu.endianness,
            nrxslots=nrxslots,
            ntxslots=ntxslots,
            timestamp=None,
            with_preamble_crc=not software_debug)
        # Use PHY's eth_tx/eth_rx clock domains.
        ethmac = ClockDomainsRenamer({
            "eth_tx": phy_cd + "_tx",
            "eth_rx": phy_cd + "_rx"})(ethmac)
        setattr(self.submodules, name, ethmac)
        # Compute Regions size and add it to the SoC.
        ethmac_region_rx = SoCRegion(origin=0, size=ethmac.rx_slots.read() * ethmac.slot_size.read(), cached=False)
        ethmac_region_tx = SoCRegion(origin=0, size=ethmac.tx_slots.read() * ethmac.slot_size.read(), cached=False)
        self.pcie_mem_bus_rx.add_region(name="io",region=SoCIORegion(0x00000000,0x100000000))
        self.pcie_mem_bus_tx.add_region(name="io",region=SoCIORegion(0x00000000,0x100000000))
        self.pcie_mem_bus_rx.add_slave(name='ethmac_rx', slave=ethmac.rx_bus, region=ethmac_region_rx)
        self.pcie_mem_bus_tx.add_slave(name='ethmac_tx', slave=ethmac.tx_bus, region=ethmac_region_tx)

        # Timing constraints
        if with_timing_constraints:
            eth_rx_clk = getattr(phy, "crg", phy).cd_eth_rx.clk
            eth_tx_clk = getattr(phy, "crg", phy).cd_eth_tx.clk
            if not isinstance(phy, LiteEthPHYModel) and not getattr(phy, "model", False):
                self.platform.add_period_constraint(eth_rx_clk, 1e9 / phy.rx_clk_freq)
                self.platform.add_period_constraint(eth_tx_clk, 1e9 / phy.tx_clk_freq)
                self.platform.add_false_path_constraints(self.crg.cd_sys.clk, eth_rx_clk, eth_tx_clk)

        # PCIe

        from litedram.common import LiteDRAMNativePort
        from litedram.core import LiteDRAMCore
        from litedram.frontend.wishbone import LiteDRAMWishbone2Native
        from litepcie.frontend.wishbone_dma import LitePCIe2WishboneDMA, LiteWishbone2PCIeDMANative, PCIeInterruptTest
        from litepcie.core import LitePCIeEndpoint, LitePCIeMSI
        from litepcie.frontend.dma import LitePCIeDMA
        from litepcie.frontend.wishbone import LitePCIeWishboneMaster

        name = "pcie"
        self.check_if_exists(f"{name}_endpoint")
        endpoint = LitePCIeEndpoint(pcie_phy, max_pending_requests=max_pending_requests, endianness=pcie_phy.endianness)
        setattr(self.submodules, f"{name}_endpoint", endpoint)

        # MMAP.
        self.check_if_exists(f"{name}_mmap")
        mmap = LitePCIeWishboneMaster(self.pcie_endpoint, base_address=self.mem_map["csr"])
        self.add_wb_master(mmap.wishbone)
        setattr(self.submodules, f"{name}_mmap", mmap)

        pcie_host_wb2pcie_dma = LiteWishbone2PCIeDMANative(endpoint, data_width)
        self.submodules.pcie_host_wb2pcie_dma = pcie_host_wb2pcie_dma
        self.pcie_mem_bus_rx.add_master("pcie_master_wb2pcie", pcie_host_wb2pcie_dma.bus_wr)
        pcie_host_pcie2wb_dma = LitePCIe2WishboneDMA(endpoint, data_width)
        self.submodules.pcie_host_pcie2wb_dma = pcie_host_pcie2wb_dma
        self.pcie_mem_bus_tx.add_master("pcie_master_pcie2wb", pcie_host_pcie2wb_dma.bus_rd)

        self.comb += [
            pcie_host_wb2pcie_dma.bus_addr.eq(ethmac_region_rx.origin + ethmac.interface.sram.writer.stat_fifo.source.slot * ethmac.slot_size.read()),
            pcie_host_wb2pcie_dma.host_addr_offset.eq(ethmac.interface.sram.writer.stat_fifo.source.slot * ethmac.slot_size.read()),
            pcie_host_wb2pcie_dma.length.eq(1536),
            pcie_host_wb2pcie_dma.start.eq(ethmac.interface.sram.writer.start_transfer),
            ethmac.interface.sram.writer.transfer_ready.eq(pcie_host_wb2pcie_dma.ready),
        ]
        '''
        self.submodules.pcie_mem_bus_interconnect = wishbone.InterconnectShared(
            masters=list(self.pcie_mem_bus.masters.values()),
            slaves=[(self.pcie_mem_bus.regions[n].decoder(self.pcie_mem_bus), s) for n, s in
                    self.pcie_mem_bus.slaves.items()],
            register=True)
        
        
        self.submodules.pcie_mem_bus_interconnect_rx = wishbone.InterconnectShared(
            masters=list(self.pcie_mem_bus_rx.masters.values()),
            slaves=[(self.pcie_mem_bus_rx.regions[n].decoder(self.pcie_mem_bus_rx), s) for n, s in
                    self.pcie_mem_bus_rx.slaves.items()],
            register=True)

        self.submodules.pcie_mem_bus_interconnect_tx = wishbone.InterconnectShared(
            masters=list(self.pcie_mem_bus_tx.masters.values()),
            slaves=[(self.pcie_mem_bus_tx.regions[n].decoder(self.pcie_mem_bus_tx), s) for n, s in
                    self.pcie_mem_bus_tx.slaves.items()],
            register=True)
        '''
        self.submodules.bus_interconnect_tx = wishbone.InterconnectPointToPoint(
            master=next(iter(self.pcie_mem_bus_tx.masters.values())),
            slave=next(iter(self.pcie_mem_bus_tx.slaves.values())))

        self.submodules.bus_interconnect_rx = wishbone.InterconnectPointToPoint(
            master=next(iter(self.pcie_mem_bus_rx.masters.values())),
            slave=next(iter(self.pcie_mem_bus_rx.slaves.values())))

        from litepcie.core import LitePCIeMSIMultiVector
        if with_msi:
            self.check_if_exists(f"{name}_msi")
            msi = LitePCIeMSIMultiVector()
            setattr(self.submodules, f"{name}_msi", msi)
            self.comb += msi.source.connect(pcie_phy.msi)
            self.msis = {}

            self.msis["ETHRX"] = ethmac.rx_pcie_irq
            self.msis["ETHTX"] = ethmac.tx_pcie_irq
            self.msis["TXDATA"] = pcie_host_pcie2wb_dma.irq

            for i, (k, v) in enumerate(sorted(self.msis.items())):
                self.comb += msi.irqs[i].eq(v)
                self.add_constant(k + "_INTERRUPT", i)

        # Timing constraints.
        self.platform.add_false_path_constraints(self.crg.cd_sys.clk, pcie_phy.cd_pcie.clk)

