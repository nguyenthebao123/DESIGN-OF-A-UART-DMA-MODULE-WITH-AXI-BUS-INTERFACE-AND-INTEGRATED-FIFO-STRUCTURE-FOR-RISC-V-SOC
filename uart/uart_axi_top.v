//=====================================================================
// uart_axi_top.v
// Khoi UART AXI tich hop FIFO - KHONG bao gom DMA master (phan DMA do
// nguoi khac trong nhom lam rieng, ket noi vao qua cac chan duoi day).
//
// Pham vi cua khoi nay:
//   - AXI4-Lite slave  : CPU cau hinh / doc trang thai / IRQ
//   - TX FIFO + RX FIFO
//   - UART TX/RX core + baud generator
//
// Khong co trong khoi nay (nguoi khac lam, ket noi qua cac chan o duoi):
//   - AXI4 master that su truy cap bo nho he thong
//
//---------------------------------------------------------------------
// HOP DONG GIAO TIEP VOI KHOI DMA (quan trong, doc ky truoc khi ghep)
//---------------------------------------------------------------------
// Chieu RA khoi DMA (khoi DMA can doc):
//   tx_dma_start   : muc '1' khi CPU yeu cau bat dau phat (tu CTRL.bit0).
//                    Khoi DMA thay '1' lan dau (dang ranh) thi chot
//                    tx_addr/tx_len, bat dau doc bo nho, ROI PHAI keo
//                    tx_start_clear len '1' it nhat 1 chu ky de tu xoa
//                    bit nay (tranh lap lai vo han).
//   tx_addr, tx_len: dia chi/so byte nguon, hop le khi tx_dma_start='1'.
//   tx_fifo_full   : khoi DMA KHONG duoc day them byte khi tin hieu nay '1'.
//
//   rx_dma_start, rx_addr, rx_len : tuong tu cho chieu nhan.
//   rx_fifo_rd_data, rx_fifo_empty: du lieu doc duoc CO HIEU LUC NGAY
//                    trong cung chu ky voi dma_rx_fifo_rd_en (FIFO kieu
//                    "look-ahead"), chi pop khi rx_fifo_empty='0'.
//
// Chieu VAO tu khoi DMA (khoi DMA dieu khien):
//   tx_start_clear      : xung 1 chu ky, bao da nhan lenh TX_START.
//   tx_dma_busy          : muc '1' trong suot luc dang phat DMA (-> STATUS).
//   tx_done_pulse        : xung 1 chu ky khi phat DMA xong (-> IRQ_STATUS).
//   dma_tx_fifo_wr_en/data: xung 1 chu ky de day 1 byte vao TX FIFO.
//
//   rx_start_clear, rx_dma_busy, rx_done_pulse : tuong tu chieu nhan.
//   dma_rx_fifo_rd_en    : xung 1 chu ky de lay (pop) 1 byte khoi RX FIFO.
//
// GHI CHU: file axi4_dma_master.v (da co san trong bo nay) dung dung
// het cac ten/ngu nghia tin hieu nhu tren - co the dua cho nguoi lam
// DMA xem nhu mau tham khao de ho tu viet khoi cua ho.
//=====================================================================
module uart_axi_top #(
    parameter S_ADDR_WIDTH = 6,    // dia chi thanh ghi (AXI4-Lite slave)
    parameter FIFO_DEPTH   = 16    // phai la luy thua cua 2
)(
    input  wire clk,
    input  wire rst_n,

    // Chan vat ly UART
    output wire uart_tx_line,
    input  wire uart_rx_line,

    output wire irq,

    // ============== AXI4-Lite slave: thanh ghi dieu khien ==============
    input  wire [S_ADDR_WIDTH-1:0] s_axi_awaddr,
    input  wire                    s_axi_awvalid,
    output wire                    s_axi_awready,
    input  wire [31:0]             s_axi_wdata,
    input  wire [3:0]              s_axi_wstrb,
    input  wire                    s_axi_wvalid,
    output wire                    s_axi_wready,
    output wire [1:0]              s_axi_bresp,
    output wire                    s_axi_bvalid,
    input  wire                    s_axi_bready,
    input  wire [S_ADDR_WIDTH-1:0] s_axi_araddr,
    input  wire                    s_axi_arvalid,
    output wire                    s_axi_arready,
    output wire [31:0]             s_axi_rdata,
    output wire [1:0]              s_axi_rresp,
    output wire                    s_axi_rvalid,
    input  wire                    s_axi_rready,

    // ============== Giao tiep voi khoi DMA (nguoi khac lam) =============
    // -> ra khoi DMA
    output wire        tx_dma_start,
    output wire [31:0] tx_addr,
    output wire [31:0] tx_len,
    output wire        tx_fifo_full,

    output wire        rx_dma_start,
    output wire [31:0] rx_addr,
    output wire [31:0] rx_len,
    output wire [7:0]  rx_fifo_rd_data,
    output wire        rx_fifo_empty,

    // <- vao tu khoi DMA
    input  wire         tx_start_clear,
    input  wire         tx_dma_busy,
    input  wire         tx_done_pulse,
    input  wire         dma_tx_fifo_wr_en,
    input  wire [7:0]   dma_tx_fifo_wr_data,

    input  wire         rx_start_clear,
    input  wire         rx_dma_busy,
    input  wire         rx_done_pulse,
    input  wire         dma_rx_fifo_rd_en
);

    // ---------------- thanh ghi ----------------
    wire [31:0] reg_ctrl, reg_status, reg_baud_div;
    wire [31:0] reg_tx_addr, reg_tx_len, reg_rx_addr, reg_rx_len;
    wire [31:0] reg_irq_en, reg_irq_status;

    assign tx_dma_start = reg_ctrl[0];
    assign rx_dma_start = reg_ctrl[1];
    wire   soft_reset    = reg_ctrl[2];
    wire   tx_fifo_clear = reg_ctrl[3];
    wire   rx_fifo_clear = reg_ctrl[4];

    assign tx_addr = reg_tx_addr;
    assign tx_len  = reg_tx_len;
    assign rx_addr = reg_rx_addr;
    assign rx_len  = reg_rx_len;

    wire sys_rst_n = rst_n & ~soft_reset;

    wire cpu_tx_push_en;
    wire [7:0] cpu_tx_push_data;
    wire cpu_rx_pop_en;

    // ---------------- TX FIFO ----------------
    wire        tx_fifo_empty;
    wire [4:0]  tx_fifo_count;
    wire        tx_fifo_rd_en;
    wire [7:0]  tx_fifo_rd_data;

    wire tx_fifo_wr_en   = dma_tx_fifo_wr_en | cpu_tx_push_en;
    wire [7:0] tx_fifo_wr_data = dma_tx_fifo_wr_en ? dma_tx_fifo_wr_data : cpu_tx_push_data;

    sync_fifo #(.DATA_WIDTH(8), .DEPTH(FIFO_DEPTH), .ADDR_WIDTH(4)) u_tx_fifo (
        .clk          (clk),
        .rst_n        (sys_rst_n),
        .clear        (tx_fifo_clear),
        .wr_en        (tx_fifo_wr_en),
        .wr_data      (tx_fifo_wr_data),
        .full         (tx_fifo_full),
        .rd_en        (tx_fifo_rd_en),
        .rd_data      (tx_fifo_rd_data),
        .empty        (tx_fifo_empty),
        .count        (tx_fifo_count),
        .almost_full  (),
        .almost_empty ()
    );

    // ---------------- UART TX core ----------------
    wire uart_tx_ready, uart_tx_busy;

    assign tx_fifo_rd_en = uart_tx_ready & ~tx_fifo_empty;

    uart_tx u_uart_tx (
        .clk      (clk),
        .rst_n    (sys_rst_n),
        .tick_x16 (tick_x16),
        .tx_valid (~tx_fifo_empty),
        .tx_data  (tx_fifo_rd_data),
        .tx_ready (uart_tx_ready),
        .tx_busy  (uart_tx_busy),
        .tx       (uart_tx_line)
    );

    // ---------------- RX FIFO ----------------
    wire        rx_fifo_full;
    wire [4:0]  rx_fifo_count;
    wire        rx_fifo_rd_en;

    assign rx_fifo_rd_en = dma_rx_fifo_rd_en | cpu_rx_pop_en;

    wire        uart_rx_valid, uart_rx_frame_err;
    wire [7:0]  uart_rx_data;
    wire        rx_overflow = uart_rx_valid & rx_fifo_full;

    sync_fifo #(.DATA_WIDTH(8), .DEPTH(FIFO_DEPTH), .ADDR_WIDTH(4)) u_rx_fifo (
        .clk          (clk),
        .rst_n        (sys_rst_n),
        .clear        (rx_fifo_clear),
        .wr_en        (uart_rx_valid & ~rx_fifo_full),
        .wr_data      (uart_rx_data),
        .full         (rx_fifo_full),
        .rd_en        (rx_fifo_rd_en),
        .rd_data      (rx_fifo_rd_data),
        .empty        (rx_fifo_empty),
        .count        (rx_fifo_count),
        .almost_full  (),
        .almost_empty ()
    );

    // ---------------- UART RX core ----------------
    uart_rx u_uart_rx (
        .clk          (clk),
        .rst_n        (sys_rst_n),
        .tick_x16     (tick_x16),
        .rx           (uart_rx_line),
        .rx_valid     (uart_rx_valid),
        .rx_data      (uart_rx_data),
        .rx_frame_err (uart_rx_frame_err)
    );

    // ---------------- Bo tao baud rate ----------------
    wire tick_x16;

    baud_gen #(.DIV_WIDTH(16)) u_baud (
        .clk      (clk),
        .rst_n    (sys_rst_n),
        .baud_div (reg_baud_div[15:0]),
        .tick_x16 (tick_x16)
    );

    // ---------------- AXI4-Lite register slave ----------------
    assign reg_status = {26'b0,
                          rx_fifo_full, rx_fifo_empty,
                          tx_fifo_full, tx_fifo_empty,
                          rx_dma_busy,  tx_dma_busy};

    wire [31:0] irq_status_set = {28'b0,
                                   uart_rx_frame_err, rx_overflow,
                                   rx_done_pulse,     tx_done_pulse};

    axi4lite_regs #(.ADDR_WIDTH(S_ADDR_WIDTH)) u_regs (
        .clk            (clk),
        .rst_n          (rst_n),

        .s_axi_awaddr   (s_axi_awaddr),
        .s_axi_awvalid  (s_axi_awvalid),
        .s_axi_awready  (s_axi_awready),
        .s_axi_wdata    (s_axi_wdata),
        .s_axi_wstrb    (s_axi_wstrb),
        .s_axi_wvalid   (s_axi_wvalid),
        .s_axi_wready   (s_axi_wready),
        .s_axi_bresp    (s_axi_bresp),
        .s_axi_bvalid   (s_axi_bvalid),
        .s_axi_bready   (s_axi_bready),
        .s_axi_araddr   (s_axi_araddr),
        .s_axi_arvalid  (s_axi_arvalid),
        .s_axi_arready  (s_axi_arready),
        .s_axi_rdata    (s_axi_rdata),
        .s_axi_rresp    (s_axi_rresp),
        .s_axi_rvalid   (s_axi_rvalid),
        .s_axi_rready   (s_axi_rready),

        .reg_ctrl       (reg_ctrl),
        .reg_status     (reg_status),
        .reg_baud_div   (reg_baud_div),
        .reg_tx_addr    (reg_tx_addr),
        .reg_tx_len     (reg_tx_len),
        .reg_rx_addr    (reg_rx_addr),
        .reg_rx_len     (reg_rx_len),
        .reg_irq_en     (reg_irq_en),
        .irq_status_set (irq_status_set),
        .reg_irq_status (reg_irq_status),

        .tx_start_clear (tx_start_clear),
        .rx_start_clear (rx_start_clear),

        .cpu_tx_push_en   (cpu_tx_push_en),
        .cpu_tx_push_data (cpu_tx_push_data),
        .cpu_rx_pop_en    (cpu_rx_pop_en),
        .cpu_rx_pop_data  (rx_fifo_rd_data)
    );

    assign irq = |(reg_irq_status & reg_irq_en);

endmodule
