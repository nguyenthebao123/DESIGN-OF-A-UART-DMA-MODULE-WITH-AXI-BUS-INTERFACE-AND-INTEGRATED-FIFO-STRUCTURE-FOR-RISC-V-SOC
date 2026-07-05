//=====================================================================
// soc_top.v
// Top-level module ket noi truc tiep:
//   - riscv_soc_top  : CPU RISC-V pipeline 5 tang + RAM/ROM + AXI4-Lite master
//   - uart_axi_top   : UART TX/RX + FIFO + thanh ghi AXI4-Lite slave
//   - axi4_dma_master: DMA AXI4 master (doc/ghi bo nho he thong)
//
// KHONG dung Vivado Block Design / IP Integrator / AXI Interconnect.
// Tat ca day noi duoc thuc hien bang Verilog thuan.
//
// Cong ben ngoai (board):
//   sys_clock   -- xung clock tu board (truoc Clocking Wizard)
//   reset_rtl   -- active-HIGH, tu nut/chan board
//   uart_rx_0   -- chan vat ly UART RX
//   uart_tx_0   -- chan vat ly UART TX
//
// Dia chi thanh ghi UART (offset tu base):
//   0x00 CTRL  | 0x04 STATUS | 0x08 BAUD_DIV | 0x0C TX_ADDR
//   0x10 TX_LEN| 0x14 RX_ADDR| 0x18 RX_LEN  | 0x1C IRQ_EN
//   0x20 IRQ_STATUS | 0x24 TX_DATA | 0x28 RX_DATA
//
// sel_periph: cpu_addr[31:14] != 0  => AXI ngoai vi
//   Vi du firmware: base UART = 0x10000000 (bit 28 = 1 => sel_periph = 1)
//=====================================================================
`timescale 1ns / 1ps

module soc_top #(
    parameter IMEM_FILE   = "soc_test.hex",  // file khoi tao ROM lenh
    parameter BAUD_DIV_DEFAULT = 32'd651,    // 100MHz / (16*9600) - 1
    parameter S_ADDR_WIDTH     = 32          // do rong dia chi AXI CPU
)(
    input  wire sys_clock,
    input  wire reset_rtl,    // active-HIGH
    input  wire uart_rx_0,
    output wire uart_tx_0
);

    //----------------------------------------------------------------
    // Clock & Reset
    // Dung thang sys_clock. Neu can PLL/MMCM, them Clocking Wizard
    // ben ngoai va truyen clk_out vao day.
    //----------------------------------------------------------------
    wire clk   = sys_clock;
    wire rst_n = ~reset_rtl;   // dao: active-LOW cho cac module ben trong

    //================================================================
    // Day AXI4-Lite: CPU (master) <-> UART (slave)
    // Noi thang, khong qua bat ky chuyen doi nao.
    //================================================================
    wire [S_ADDR_WIDTH-1:0] cpu_axi_awaddr;
    wire                    cpu_axi_awvalid;
    wire                    cpu_axi_awready;
    wire [31:0]             cpu_axi_wdata;
    wire [3:0]              cpu_axi_wstrb;
    wire                    cpu_axi_wvalid;
    wire                    cpu_axi_wready;
    wire [1:0]              cpu_axi_bresp;
    wire                    cpu_axi_bvalid;
    wire                    cpu_axi_bready;
    wire [S_ADDR_WIDTH-1:0] cpu_axi_araddr;
    wire                    cpu_axi_arvalid;
    wire                    cpu_axi_arready;
    wire [31:0]             cpu_axi_rdata;
    wire [1:0]              cpu_axi_rresp;
    wire                    cpu_axi_rvalid;
    wire                    cpu_axi_rready;

    //================================================================
    // Day AXI4 full: DMA (master) <-> BRAM (slave)
    // Phan nay du phong cho DMA -- neu khong co BRAM noi them,
    // cac tin hieu duoc float (arready=0, rvalid=0, awready=0,
    // wready=0, bvalid=0) de CPU/DMA khong bi treo.
    //================================================================
    wire [31:0] dma_m_axi_araddr;
    wire [7:0]  dma_m_axi_arlen;
    wire [2:0]  dma_m_axi_arsize;
    wire [1:0]  dma_m_axi_arburst;
    wire        dma_m_axi_arvalid;
    wire        dma_m_axi_arready;
    wire [31:0] dma_m_axi_rdata;
    wire [1:0]  dma_m_axi_rresp;
    wire        dma_m_axi_rlast;
    wire        dma_m_axi_rvalid;
    wire        dma_m_axi_rready;
    wire [31:0] dma_m_axi_awaddr;
    wire [7:0]  dma_m_axi_awlen;
    wire [2:0]  dma_m_axi_awsize;
    wire [1:0]  dma_m_axi_awburst;
    wire        dma_m_axi_awvalid;
    wire        dma_m_axi_awready;
    wire [31:0] dma_m_axi_wdata;
    wire [3:0]  dma_m_axi_wstrb;
    wire        dma_m_axi_wlast;
    wire        dma_m_axi_wvalid;
    wire        dma_m_axi_wready;
    wire [1:0]  dma_m_axi_bresp;
    wire        dma_m_axi_bvalid;
    wire        dma_m_axi_bready;

    // Tie-off BRAM slave (khong co BRAM that): DMA se timeout/stall
    // khi co lenh DMA that su. Thay bang BRAM thuc neu can.
    assign dma_m_axi_arready = 1'b0;
    assign dma_m_axi_rdata   = 32'h0;
    assign dma_m_axi_rresp   = 2'b00;
    assign dma_m_axi_rlast   = 1'b0;
    assign dma_m_axi_rvalid  = 1'b0;
    assign dma_m_axi_awready = 1'b0;
    assign dma_m_axi_wready  = 1'b0;
    assign dma_m_axi_bresp   = 2'b00;
    assign dma_m_axi_bvalid  = 1'b0;

    //================================================================
    // Day noi UART <-> DMA (tin hieu dieu phoi noi bo)
    //================================================================
    wire        tx_dma_start;
    wire [31:0] tx_addr;
    wire [31:0] tx_len;
    wire        tx_fifo_full;
    wire        rx_dma_start;
    wire [31:0] rx_addr;
    wire [31:0] rx_len;
    wire [7:0]  rx_fifo_rd_data;
    wire        rx_fifo_empty;
    wire        tx_start_clear;
    wire        tx_dma_busy;
    wire        tx_done_pulse;
    wire        dma_tx_fifo_wr_en;
    wire [7:0]  dma_tx_fifo_wr_data;
    wire        rx_start_clear;
    wire        rx_dma_busy;
    wire        rx_done_pulse;
    wire        dma_rx_fifo_rd_en;

    //================================================================
    // KHOI 1: CPU RISC-V SoC
    //================================================================
    riscv_soc_top #(
        .S_ADDR_WIDTH (S_ADDR_WIDTH),
        .IMEM_FILE    (IMEM_FILE)
    ) u_cpu (
        .clk            (clk),
        .rst_n          (rst_n),
        // AXI4-Lite master -> noi thang vao UART slave
        .m_axi_awaddr   (cpu_axi_awaddr),
        .m_axi_awvalid  (cpu_axi_awvalid),
        .m_axi_awready  (cpu_axi_awready),
        .m_axi_wdata    (cpu_axi_wdata),
        .m_axi_wstrb    (cpu_axi_wstrb),
        .m_axi_wvalid   (cpu_axi_wvalid),
        .m_axi_wready   (cpu_axi_wready),
        .m_axi_bresp    (cpu_axi_bresp),
        .m_axi_bvalid   (cpu_axi_bvalid),
        .m_axi_bready   (cpu_axi_bready),
        .m_axi_araddr   (cpu_axi_araddr),
        .m_axi_arvalid  (cpu_axi_arvalid),
        .m_axi_arready  (cpu_axi_arready),
        .m_axi_rdata    (cpu_axi_rdata),
        .m_axi_rresp    (cpu_axi_rresp),
        .m_axi_rvalid   (cpu_axi_rvalid),
        .m_axi_rready   (cpu_axi_rready)
    );

    //================================================================
    // KHOI 2: UART AXI
    //================================================================
    uart_axi_top #(
        .S_ADDR_WIDTH (6),
        .FIFO_DEPTH   (16)
    ) u_uart (
        .clk              (clk),
        .rst_n            (rst_n),
        // Cong vat ly
        .uart_tx_line     (uart_tx_0),
        .uart_rx_line     (uart_rx_0),
        .irq              (),        // khong dung ngat trong ban nay
        // AXI4-Lite slave: noi vao 6 bit thap cua CPU awaddr/araddr
        // (CPU gui dia chi 32-bit nhung UART chi can 6-bit offset)
        .s_axi_awaddr     (cpu_axi_awaddr[5:0]),
        .s_axi_awvalid    (cpu_axi_awvalid),
        .s_axi_awready    (cpu_axi_awready),
        .s_axi_wdata      (cpu_axi_wdata),
        .s_axi_wstrb      (cpu_axi_wstrb),
        .s_axi_wvalid     (cpu_axi_wvalid),
        .s_axi_wready     (cpu_axi_wready),
        .s_axi_bresp      (cpu_axi_bresp),
        .s_axi_bvalid     (cpu_axi_bvalid),
        .s_axi_bready     (cpu_axi_bready),
        .s_axi_araddr     (cpu_axi_araddr[5:0]),
        .s_axi_arvalid    (cpu_axi_arvalid),
        .s_axi_arready    (cpu_axi_arready),
        .s_axi_rdata      (cpu_axi_rdata),
        .s_axi_rresp      (cpu_axi_rresp),
        .s_axi_rvalid     (cpu_axi_rvalid),
        .s_axi_rready     (cpu_axi_rready),
        // Dieu phoi DMA (noi vao u_dma ben duoi)
        .tx_dma_start     (tx_dma_start),
        .tx_addr          (tx_addr),
        .tx_len           (tx_len),
        .tx_fifo_full     (tx_fifo_full),
        .rx_dma_start     (rx_dma_start),
        .rx_addr          (rx_addr),
        .rx_len           (rx_len),
        .rx_fifo_rd_data  (rx_fifo_rd_data),
        .rx_fifo_empty    (rx_fifo_empty),
        .tx_start_clear   (tx_start_clear),
        .tx_dma_busy      (tx_dma_busy),
        .tx_done_pulse    (tx_done_pulse),
        .dma_tx_fifo_wr_en   (dma_tx_fifo_wr_en),
        .dma_tx_fifo_wr_data (dma_tx_fifo_wr_data),
        .rx_start_clear   (rx_start_clear),
        .rx_dma_busy      (rx_dma_busy),
        .rx_done_pulse    (rx_done_pulse),
        .dma_rx_fifo_rd_en   (dma_rx_fifo_rd_en)
    );

    //================================================================
    // KHOI 3: DMA Master
    //================================================================
    axi4_dma_master #(
        .ADDR_WIDTH (32),
        .DATA_WIDTH (32)
    ) u_dma (
        .clk                  (clk),
        .rst_n                (rst_n),
        // Dieu phoi tu UART
        .tx_dma_start         (tx_dma_start),
        .tx_addr              (tx_addr),
        .tx_len               (tx_len),
        .tx_fifo_full         (tx_fifo_full),
        .rx_dma_start         (rx_dma_start),
        .rx_addr              (rx_addr),
        .rx_len               (rx_len),
        .rx_fifo_rd_data      (rx_fifo_rd_data),
        .rx_fifo_empty        (rx_fifo_empty),
        // Dieu phoi ra UART
        .tx_start_clear       (tx_start_clear),
        .tx_dma_busy          (tx_dma_busy),
        .tx_done_pulse        (tx_done_pulse),
        .dma_tx_fifo_wr_en    (dma_tx_fifo_wr_en),
        .dma_tx_fifo_wr_data  (dma_tx_fifo_wr_data),
        .rx_start_clear       (rx_start_clear),
        .rx_dma_busy          (rx_dma_busy),
        .rx_done_pulse        (rx_done_pulse),
        .dma_rx_fifo_rd_en    (dma_rx_fifo_rd_en),
        // AXI4 master -> BRAM (tie-off o tren neu khong co BRAM that)
        .m_axi_araddr         (dma_m_axi_araddr),
        .m_axi_arlen          (dma_m_axi_arlen),
        .m_axi_arsize         (dma_m_axi_arsize),
        .m_axi_arburst        (dma_m_axi_arburst),
        .m_axi_arvalid        (dma_m_axi_arvalid),
        .m_axi_arready        (dma_m_axi_arready),
        .m_axi_rdata          (dma_m_axi_rdata),
        .m_axi_rresp          (dma_m_axi_rresp),
        .m_axi_rlast          (dma_m_axi_rlast),
        .m_axi_rvalid         (dma_m_axi_rvalid),
        .m_axi_rready         (dma_m_axi_rready),
        .m_axi_awaddr         (dma_m_axi_awaddr),
        .m_axi_awlen          (dma_m_axi_awlen),
        .m_axi_awsize         (dma_m_axi_awsize),
        .m_axi_awburst        (dma_m_axi_awburst),
        .m_axi_awvalid        (dma_m_axi_awvalid),
        .m_axi_awready        (dma_m_axi_awready),
        .m_axi_wdata          (dma_m_axi_wdata),
        .m_axi_wstrb          (dma_m_axi_wstrb),
        .m_axi_wlast          (dma_m_axi_wlast),
        .m_axi_wvalid         (dma_m_axi_wvalid),
        .m_axi_wready         (dma_m_axi_wready),
        .m_axi_bresp          (dma_m_axi_bresp),
        .m_axi_bvalid         (dma_m_axi_bvalid),
        .m_axi_bready         (dma_m_axi_bready)
    );

endmodule
