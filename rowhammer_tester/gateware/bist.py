from migen import *

from litex.soc.interconnect.csr import AutoCSR, CSRStatus, CSRStorage

from litedram.frontend.bist import LiteDRAMBISTGenerator, LiteDRAMBISTChecker
from litedram.frontend.dma import LiteDRAMDMAReader, LiteDRAMDMAWriter

class BistWriter(Module, AutoCSR):
    def __init__(self, dram_port, w0_port, w1_port, w2_port, w3_port, adr_port):
        self.reset        = CSRStorage()
        self.start        = CSRStorage()
        self.done         = CSRStatus()

        self.count        = CSRStorage(size=(32*1))

        self.mem_base     = CSRStorage(size=32)
        self.mem_mask     = CSRStorage(size=32)
        self.data_mask    = CSRStorage(size=32) # patterns

        dma = LiteDRAMDMAWriter(dram_port, fifo_depth=1)
        self.submodules += dma

        cmd_counter = Signal(32)

        self.comb += [
            w0_port.adr.eq(cmd_counter & self.data_mask.storage),
            w1_port.adr.eq(cmd_counter & self.data_mask.storage),
            w2_port.adr.eq(cmd_counter & self.data_mask.storage),
            w3_port.adr.eq(cmd_counter & self.data_mask.storage),
            adr_port.adr.eq(cmd_counter & self.data_mask.storage),
        ]

        self.comb += [
            dma.sink.address.eq(self.mem_base.storage + adr_port.dat_r + (cmd_counter & self.mem_mask.storage)),
            dma.sink.data.eq(Cat(w0_port.dat_r, w1_port.dat_r, w2_port.dat_r, w3_port.dat_r)),
        ]

        fsm = FSM(reset_state="IDLE")
        self.submodules += fsm
        fsm.act("IDLE",
            If(self.start.storage,
                NextValue(cmd_counter, 0),
                NextState("WAIT"),
            )
        )
        fsm.act("WAIT",
            If(cmd_counter >= self.count.storage,
                NextState("DONE")
            ).Else(
                NextState("RUN")
            )
        )
        fsm.act("RUN",
            dma.sink.valid.eq(1),
            If(dma.sink.ready,
                NextValue(cmd_counter, cmd_counter + 1),
                NextState("WAIT")
            )
        )
        fsm.act("DONE",
            self.done.status.eq(1),
            If(self.reset.storage,
                NextState("IDLE"))
        )

class BistReader(Module, AutoCSR):
    def __init__(self, dram_port, w0_port, w1_port, w2_port, w3_port, adr_port):
        self.reset        = CSRStorage()
        self.start        = CSRStorage()
        self.done         = CSRStatus()

        self.count        = CSRStorage(size=32)
        self.pointer      = CSRStatus(size=32)

        self.mem_base     = CSRStorage(size=32)
        self.mem_mask     = CSRStorage(size=32)
        self.data_mask    = CSRStorage(size=32) # patterns

        dma = LiteDRAMDMAReader(dram_port, fifo_depth=1, fifo_buffered=False)
        self.submodules += dma

        cmd_counter = Signal(32)

        self.comb += [
            w0_port.adr.eq(cmd_counter & self.data_mask.storage),
            w1_port.adr.eq(cmd_counter & self.data_mask.storage),
            w2_port.adr.eq(cmd_counter & self.data_mask.storage),
            w3_port.adr.eq(cmd_counter & self.data_mask.storage),
            adr_port.adr.eq(cmd_counter & self.data_mask.storage),
        ]

        data_pattern = Signal(32 * 4)
        self.comb += [
            dma.sink.address.eq(self.mem_base.storage + adr_port.dat_r + (cmd_counter & self.mem_mask.storage)),
            data_pattern.eq(Cat(w0_port.dat_r, w1_port.dat_r, w2_port.dat_r, w3_port.dat_r)),
        ]

        fsm = FSM(reset_state="IDLE")
        self.submodules += fsm
        fsm.act("IDLE",
            If(self.start.storage,
                NextValue(cmd_counter, 0),
                NextValue(self.pointer.status, 0xdeadbeef),
                NextState("WAIT"),
            )
        )
        fsm.act("WAIT",
            If(cmd_counter >= self.count.storage,
                NextState("DONE")
            ).Else(
                NextState("WR_ADR")
            )
        )
        fsm.act("WR_ADR",
            dma.sink.valid.eq(1),
            If(dma.sink.ready,
                NextState("RD_DATA")
            )
        )
        fsm.act("RD_DATA",
            dma.source.ready.eq(1),
            If(dma.source.valid,
                NextValue(cmd_counter, cmd_counter + 1),
                If(dma.source.data != data_pattern,
                    NextValue(self.pointer.status, cmd_counter)
                ),
                NextState("WAIT")
            )
        )
        fsm.act("DONE",
            self.done.status.eq(1),
            If(self.reset.storage,
                NextState("IDLE"))
        )
