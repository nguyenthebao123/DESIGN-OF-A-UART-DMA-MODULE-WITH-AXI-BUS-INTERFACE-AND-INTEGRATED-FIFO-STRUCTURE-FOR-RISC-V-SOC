# UART-DMA Module with AXI Bus Interface for a RISC-V SoC

Design and integration of a small System-on-Chip on the **ZedBoard (Zynq XC7Z020)**, combining a self-designed 5-stage pipelined **RV32I RISC-V** core, an **AXI4 DMA controller**, and a **UART** peripheral with integrated TX/RX FIFOs — all connected through the AXI4-Lite / AXI4 bus protocol.

> Course project — Hardware/Software Co-design (HWSW Codesign), HCMC University of Technology and Education (HCMUTE)

📄 Paper: *"Design of a UART DMA Module with AXI Bus Interface and Integrated FIFO Structure for RISC-V SoC"* — Tạp chí Khoa học Giáo dục Kỹ thuật, HCMUTE
🎥 Demo video: https://www.youtube.com/watch?v=otC6dJ1Bdi8

---

## Overview

Most educational RISC-V core projects stop at a single, isolated CPU design. This project goes further by integrating **three cooperating hardware blocks into one unified SoC**, with a clearly defined data flow and handshaking protocol between them:

- A CPU that can talk to peripherals without stalling the whole system.
- A DMA engine that moves data between memory and UART **without CPU intervention per byte**.
- A UART peripheral with FIFO buffering that decouples the slow serial line from the fast system bus.

All three blocks were designed in **Verilog HDL**, verified independently with **RTL simulation in Vivado Simulator**, then wired together by hand in a single top-level module (`soc_top.v`) — without relying on Vivado's Block Design / IP Integrator automatic interconnect.

## System Architecture

```
 sys_clock ──▶ ┌─────────────┐  AXI4-Lite   ┌─────────────┐  internal    ┌─────────────┐
 reset_rtl ──▶ │  SoC RISC-V │◀────────────▶│    UART     │◀────signals─▶│     DMA     │
               │ (RV32I, 5-  │               │ (TX/RX FIFO,│              │ (TX/RX FSM) │
               │ stage pipe) │               │ AXI Register)│              └──────┬──────┘
               └─────────────┘               └─────────────┘                     │ AXI4
                                                                                   ▼
                                                                          ┌─────────────────┐
                                                                          │ System Memory    │
                                                                          │ (BRAM, sys_ram)  │
                                                                          └─────────────────┘
```

- **CPU ↔ UART**: connected directly through a 17-wire **AXI4-Lite** interface (`awaddr`, `awvalid`, `awready`, `wdata`, `wstrb`, `wvalid`, `wready`, `bresp`, `bvalid`, `bready`, `araddr`, `arvalid`, `arready`, `rdata`, `rresp`, `rvalid`, `rready`).
- **UART ↔ DMA**: connected through 14 dedicated coordination signals (`tx_dma_start`, `tx_addr`, `tx_len`, `tx_fifo_full`, `rx_dma_start`, `rx_addr`, `rx_len`, `rx_fifo_rd_data`, `rx_fifo_empty`, `tx_start_clear`, `tx_dma_busy`, `tx_done_pulse`, `dma_tx_fifo_wr_en/data`, `rx_start_clear`, `rx_dma_busy`, `rx_done_pulse`, `dma_rx_fifo_rd_en`) — a private contract, not routed through the AXI bus.
- **DMA ↔ Memory**: DMA acts as an **AXI4 master**, reading/writing a 4 KB dummy BRAM (`sys_ram`) that represents system memory.

## Module Breakdown

### 1. RISC-V Core (`riscv_soc_top`)
- Classic **5-stage pipeline**: IF → ID → EX → MEM → WB.
- Full **RV32I** base integer instruction set:

  | Group | Instructions |
  |---|---|
  | R-type | ADD, SUB, SLL, SLT, SLTU, XOR, SRL, SRA, OR, AND |
  | I-type | ADDI, XORI, ORI, ANDI, SLTI, SLTIU, SLLI, SRLI, SRAI |
  | Load/Store | LB, LH, LW, LBU, LHU / SB, SH, SW |
  | Branch/Jump | BEQ, BNE, BLT, BGE, BLTU, BGEU / JAL, JALR |
  | Upper Immediate | LUI, AUIPC |

- Sub-modules: `u_imem` (instruction ROM), `u_core` (pipeline datapath), `u_dmem` (1-cycle internal data RAM), `u_bridge` (AXI4-Lite master bridge).
- **Hazard handling implemented entirely in hardware**:
  - *Load-use hazard*: automatic 1-cycle stall.
  - *Branch/Jump hazard*: 2 mis-fetched instructions flushed when resolved at EX.
  - *AXI stall*: pipeline freezes while waiting for a peripheral response.
  - *Forwarding unit*: results from EX/MEM are forwarded straight to the ALU input, avoiding most stalls.
- Address decoding: addresses **below 16 KB** hit internal RAM in 1 cycle; addresses **above 16 KB** are routed out over AXI4-Lite to peripherals.

### 2. DMA Controller (`axi4_dma_master`)
- Two fully independent FSMs, **TX and RX**, each acting as an AXI4 master.
- CPU only needs to configure a source/destination address, a length, and set a start bit — the DMA does the rest.
- **TX FSM**: reads bytes from memory over AXI and pushes them into the UART TX FIFO until the requested length is sent.
- **RX FSM**: pulls bytes out of the UART RX FIFO and writes them to memory, waiting for write acknowledgment before continuing.

### 3. UART Peripheral (`uart_axi_top`)
Six cooperating sub-modules forming one IP block:

| Sub-module | Role |
|---|---|
| Baud Rate Generator | 16-bit counter generating a `tick_x16` pulse at 16× the baud rate |
| UART TX | 4-state FSM (IDLE → START → DATA → STOP), packs each byte into a 10-bit 8N1 frame |
| UART RX | Samples at mid-bit for reliability, validates the stop bit before accepting a byte |
| TX FIFO / RX FIFO | Synchronous FIFOs, `DATA_WIDTH=8`, `DEPTH=16` |
| AXI4-Lite Register | 11 memory-mapped 32-bit registers, with independent read/write FSMs and valid/ready handshaking |

**Register map:**

| Offset | Name | Type | Description |
|---|---|---|---|
| 0x00 | CTRL | R/W | bit0 TX_START · bit1 RX_START · bit2 SOFT_RST · bit3 TX_FIFO_CLR · bit4 RX_FIFO_CLR |
| 0x04 | STATUS | RO | bit0 TX_BUSY · bit1 RX_BUSY · bit2 TXFIFO_EMPTY · bit3 TXFIFO_FULL · bit4 RXFIFO_EMPTY · bit5 RXFIFO_FULL |
| 0x08 | BAUD_DIV | R/W | Baud divisor = f_CLK / (16 × BAUD) − 1 (default `651` → 9600 bps @ 100 MHz) |
| 0x0C | TX_ADDR | R/W | Source address in memory for DMA TX |
| 0x10 | TX_LEN | R/W | Number of bytes to transmit via DMA |
| 0x14 | RX_ADDR | R/W | Destination address in memory for DMA RX |
| 0x18 | RX_LEN | R/W | Number of bytes to receive via DMA |
| 0x1C | IRQ_EN | R/W | bit0 TX_DONE · bit1 RX_DONE · bit2 RX_OVF · bit3 RX_FERR |
| 0x20 | IRQ_STATUS | R/W | Sticky flags, write 1 to clear |
| 0x24 | TX_DATA | WO | Direct byte write into TX FIFO (polling mode, no DMA) |
| 0x28 | RX_DATA | RO | Direct byte read from RX FIFO |

## Simulation Results

All results below were captured from RTL simulation in **Vivado Simulator**.

**RISC-V core** — 18/18 instructions verified correctly, e.g.:

| Address | Machine code | Instruction | Result |
|---|---|---|---|
| 0x00 | 500093 | `addi x1, x0, 5` | x1 = 0x00000005 |
| 0x04 | 00A00113 | `addi x2, x0, 10` | x2 = 0x0000000A |
| 0x08 | 002081B3 | `add x3, x1, x2` | x3 = 0x0000000F |
| 0x0C | 40110233 | `sub x4, x2, x1` | x4 = 0x00000005 |
| 0x10 | 0020F2B3 | `and x5, x1, x2` | x5 = 0x00000000 |
| 0x14 | 0020E333 | `or x6, x1, x2` | x6 = 0x0000000F |
| 0x18 | 0020C3B3 | `xor x7, x1, x2` | x7 = 0x0000000F |

**UART loopback** — CPU writes `0x41` ('A') to `TX_DATA`; with TX looped back into RX, the same byte is read back from `RX_DATA` after ~500 clock cycles.

**DMA** — write phase captured moving 2 bytes (`0x55`, `0xA3`) from memory onto the AXI bus; read phase captured pulling received bytes back into memory.

**Full system integration** — the testbench streams the ASCII bytes of `"HELLO"` (plus a leading `'A'`) into `uart_rx_0` and observes a correct byte-for-byte echo on `uart_tx_0`:

| Test case | Byte sent | Byte received |
|---|---|---|
| Echo 1 byte | 0x41 ('A') | 0x41 ('A') |
| Echo 'H' | 0x48 | 0x48 |
| Echo 'E' | 0x45 | 0x45 |
| Echo 'L' | 0x4C | 0x4C |
| Echo 'L' | 0x4C | 0x4C |
| Echo 'O' | 0x4F | 0x4F |

## FPGA Resource Utilization (Zynq XC7Z020)

| Module | Slice LUTs | Slice Registers | Block RAM |
|---|---|---|---|
| `soc_top` (total) | 4231 | 2379 | 1 |
| `u_cpu` (riscv_soc_top) | 3725 | 1619 | 0 |
| `u_dma` (axi4_dma_master) | 194 | 147 | 0 |
| `u_uart` (uart_axi_top) | 312 | 612 | 0 |

- Total LUT/Register usage is **under 10%** of the target device, leaving significant headroom for future peripherals.
- Estimated **total on-chip power: 0.666 W** (Dynamic 0.553 W / 83%, Static 0.113 W / 17%) at an operating temperature of **32.7 °C** — well within safe limits for a single-core embedded SoC.

## Repository Contents

- Verilog RTL source for the RISC-V core, DMA controller, UART peripheral, and the `soc_top.v` top-level integration.
- Testbenches used to verify each block individually and the integrated system.
- Vivado simulation waveforms / screenshots referenced in the project report.
- Project report and presentation slides.

## Tools & Requirements

- **Vivado Design Suite** (Simulator + Synthesis), targeting **Zynq-7000 (XC7Z020, ZedBoard)**.
- Verilog HDL (IEEE 1364).
- No external IP cores — the AXI4-Lite bridge, DMA master, and UART registers are all custom RTL.

## How to Simulate

1. Open Vivado and create a new RTL project, adding all source files under the design sources.
2. Add the corresponding testbench as a simulation-only source.
3. Run **Behavioral Simulation** and inspect the waveform viewer for the signals of interest (e.g. `imem_addr`, `rd_data`, `uart_tx_line`, `tx_dma_busy`).
4. For full-system verification, use the top-level `soc_top` testbench, which drives `uart_rx_0` with a byte stream and checks the echoed output on `uart_tx_0`.

## Future Work

- Full integration test on real FPGA hardware (ZedBoard) with an external memory controller instead of the internal dummy BRAM.
- Performance optimization of the DMA engine to increase effective throughput.
- Addition of further AXI-based peripherals to the same SoC template (e.g. SPI, GPIO, timers).

## Authors

Course project for **Hardware/Software Co-design** at HCMC University of Technology and Education (HCMUTE):

- **Nguyễn Thế Bảo** — 23119050@hcmute.edu.vn
- Huỳnh Võ Gia Bảo
- Ngô Thiện Toàn
- Nguyễn Thúy Hiền

## References

1. A. Waterman and K. Asanović (Eds.), *The RISC-V Instruction Set Manual, Volume I: User-Level ISA*, RISC-V Foundation, 2019.
2. D. A. Patterson and J. L. Hennessy, *Computer Organization and Design RISC-V Edition*, 2nd ed., Morgan Kaufmann, 2020.
3. ARM Limited, *AMBA AXI and ACE Protocol Specification*, ARM IHI 0022, 2021.
4. Xilinx Inc., *Zynq-7000 SoC Technical Reference Manual*, UG585, 2021.
5. Xilinx Inc., *Vivado Design Suite User Guide: Synthesis*, UG901, 2022.
6. J. L. Hennessy and D. A. Patterson, *Computer Architecture: A Quantitative Approach*, 6th ed., Morgan Kaufmann, 2017.
7. Telecommunications Industry Association, *TIA/EIA-232-F*, 1997.
8. ARM Ltd., *AMBA AXI4-Stream Protocol Specification*, IHI0051A, 2010.
9. S. Palnitkar, *Verilog HDL: A Guide to Digital Design and Synthesis*, 2nd ed., Prentice Hall, 2003.
10. Xilinx Inc., *AXI UART 16550 v2.0 Product Guide*, PG143, 2017.
