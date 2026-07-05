//=====================================================================
// tb_uart_with_dma.v
// Test ghep THAT 2 khoi: uart_axi_top.v (khoi UART) + axi4_dma_master.v
// (khoi DMA), noi voi nhau dung theo "hop dong tin hieu". Co mo hinh
// bo nho AXI4 + BFM AXI4-Lite (dong vai CPU) de chay 2 kich ban giong
// cac test truoc.
//=====================================================================
`timescale 1ns/1ps

module tb_uart_with_dma;

    localparam ADDR_WIDTH   = 32;
    localparam S_ADDR_WIDTH = 6;

    reg clk, rst_n;

    // ---- AXI4-Lite slave (CPU -> khoi UART) ----
    reg  [S_ADDR_WIDTH-1:0] s_axi_awaddr;
    reg                     s_axi_awvalid;
    wire                    s_axi_awready;
    reg  [31:0]             s_axi_wdata;
    reg  [3:0]              s_axi_wstrb;
    reg                     s_axi_wvalid;
    wire                    s_axi_wready;
    wire [1:0]              s_axi_bresp;
    wire                    s_axi_bvalid;
    reg                     s_axi_bready;
    reg  [S_ADDR_WIDTH-1:0] s_axi_araddr;
    reg                     s_axi_arvalid;
    wire                    s_axi_arready;
    wire [31:0]             s_axi_rdata;
    wire [1:0]              s_axi_rresp;
    wire                    s_axi_rvalid;
    reg                     s_axi_rready;

    // ---- ranh gioi UART <-> DMA (day la dieu can kiem chung) ----
    wire        tx_dma_start, rx_dma_start;
    wire [31:0] tx_addr, tx_len, rx_addr, rx_len;
    wire        tx_fifo_full, rx_fifo_empty;
    wire [7:0]  rx_fifo_rd_data;

    wire        tx_start_clear, rx_start_clear;
    wire        tx_dma_busy, rx_dma_busy, tx_done_pulse, rx_done_pulse;
    wire        dma_tx_fifo_wr_en, dma_rx_fifo_rd_en;
    wire [7:0]  dma_tx_fifo_wr_data;

    // ---- AXI4 master (DMA -> bo nho he thong) ----
    wire [ADDR_WIDTH-1:0] m_axi_araddr;
    wire [7:0]  m_axi_arlen;
    wire [2:0]  m_axi_arsize;
    wire [1:0]  m_axi_arburst;
    wire        m_axi_arvalid;
    wire        m_axi_arready;
    wire [31:0] m_axi_rdata;
    wire [1:0]  m_axi_rresp;
    wire        m_axi_rlast;
    wire        m_axi_rvalid;
    wire        m_axi_rready;

    wire [ADDR_WIDTH-1:0] m_axi_awaddr;
    wire [7:0]  m_axi_awlen;
    wire [2:0]  m_axi_awsize;
    wire [1:0]  m_axi_awburst;
    wire        m_axi_awvalid;
    wire        m_axi_awready;
    wire [31:0] m_axi_wdata;
    wire [3:0]  m_axi_wstrb;
    wire        m_axi_wlast;
    wire        m_axi_wvalid;
    wire        m_axi_wready;
    wire [1:0]  m_axi_bresp;
    wire        m_axi_bvalid;
    wire        m_axi_bready;

    wire uart_line; // loopback tx -> rx
    wire irq;

    integer errors = 0;

    // =================================================================
    // KHOI 1: uart_axi_top (UART + FIFO + AXI4-Lite regs)
    // =================================================================
    uart_axi_top #(
        .S_ADDR_WIDTH (S_ADDR_WIDTH),
        .FIFO_DEPTH   (16)
    ) dut_uart (
        .clk            (clk),
        .rst_n          (rst_n),
        .uart_tx_line   (uart_line),
        .uart_rx_line   (uart_line),
        .irq            (irq),

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

        .tx_dma_start (tx_dma_start),
        .tx_addr      (tx_addr),
        .tx_len       (tx_len),
        .tx_fifo_full (tx_fifo_full),
        .rx_dma_start (rx_dma_start),
        .rx_addr      (rx_addr),
        .rx_len       (rx_len),
        .rx_fifo_rd_data (rx_fifo_rd_data),
        .rx_fifo_empty   (rx_fifo_empty),

        .tx_start_clear      (tx_start_clear),
        .tx_dma_busy         (tx_dma_busy),
        .tx_done_pulse       (tx_done_pulse),
        .dma_tx_fifo_wr_en   (dma_tx_fifo_wr_en),
        .dma_tx_fifo_wr_data (dma_tx_fifo_wr_data),

        .rx_start_clear   (rx_start_clear),
        .rx_dma_busy      (rx_dma_busy),
        .rx_done_pulse    (rx_done_pulse),
        .dma_rx_fifo_rd_en(dma_rx_fifo_rd_en)
    );

    // =================================================================
    // KHOI 2: axi4_dma_master (DMA)
    // =================================================================
    axi4_dma_master #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(32)
    ) dut_dma (
        .clk   (clk),
        .rst_n (rst_n),

        .tx_dma_start (tx_dma_start),
        .tx_addr      (tx_addr),
        .tx_len       (tx_len),
        .tx_fifo_full (tx_fifo_full),
        .rx_dma_start (rx_dma_start),
        .rx_addr      (rx_addr),
        .rx_len       (rx_len),
        .rx_fifo_rd_data (rx_fifo_rd_data),
        .rx_fifo_empty   (rx_fifo_empty),

        .tx_start_clear      (tx_start_clear),
        .tx_dma_busy         (tx_dma_busy),
        .tx_done_pulse       (tx_done_pulse),
        .dma_tx_fifo_wr_en   (dma_tx_fifo_wr_en),
        .dma_tx_fifo_wr_data (dma_tx_fifo_wr_data),

        .rx_start_clear   (rx_start_clear),
        .rx_dma_busy      (rx_dma_busy),
        .rx_done_pulse    (rx_done_pulse),
        .dma_rx_fifo_rd_en(dma_rx_fifo_rd_en),

        .m_axi_araddr  (m_axi_araddr),
        .m_axi_arlen   (m_axi_arlen),
        .m_axi_arsize  (m_axi_arsize),
        .m_axi_arburst (m_axi_arburst),
        .m_axi_arvalid (m_axi_arvalid),
        .m_axi_arready (m_axi_arready),
        .m_axi_rdata   (m_axi_rdata),
        .m_axi_rresp   (m_axi_rresp),
        .m_axi_rlast   (m_axi_rlast),
        .m_axi_rvalid  (m_axi_rvalid),
        .m_axi_rready  (m_axi_rready),

        .m_axi_awaddr  (m_axi_awaddr),
        .m_axi_awlen   (m_axi_awlen),
        .m_axi_awsize  (m_axi_awsize),
        .m_axi_awburst (m_axi_awburst),
        .m_axi_awvalid (m_axi_awvalid),
        .m_axi_awready (m_axi_awready),
        .m_axi_wdata   (m_axi_wdata),
        .m_axi_wstrb   (m_axi_wstrb),
        .m_axi_wlast   (m_axi_wlast),
        .m_axi_wvalid  (m_axi_wvalid),
        .m_axi_wready  (m_axi_wready),
        .m_axi_bresp   (m_axi_bresp),
        .m_axi_bvalid  (m_axi_bvalid),
        .m_axi_bready  (m_axi_bready)
    );

    // =================================================================
    // Mo hinh bo nho AXI4 (gia lap "bo nho he thong" cho khoi DMA dung)
    // =================================================================
    reg [7:0] mem [0:1023];

    reg        ar_pending;
    reg [31:0] ar_addr_q;

    assign m_axi_arready = ~ar_pending;
    assign m_axi_rvalid  = ar_pending;
    assign m_axi_rresp   = 2'b00;
    assign m_axi_rlast   = 1'b1;
    assign m_axi_rdata   = { mem[{ar_addr_q[31:2],2'b11}],
                              mem[{ar_addr_q[31:2],2'b10}],
                              mem[{ar_addr_q[31:2],2'b01}],
                              mem[{ar_addr_q[31:2],2'b00}] };

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ar_pending <= 1'b0;
            ar_addr_q  <= 32'h0;
        end else begin
            if (!ar_pending && m_axi_arvalid && m_axi_arready) begin
                ar_pending <= 1'b1;
                ar_addr_q  <= m_axi_araddr;
            end else if (ar_pending && m_axi_rvalid && m_axi_rready) begin
                ar_pending <= 1'b0;
            end
        end
    end

    reg        aw_pending, w_pending;
    reg [31:0] aw_addr_q;

    assign m_axi_awready = ~aw_pending;
    assign m_axi_wready  = aw_pending & ~w_pending;
    assign m_axi_bvalid  = w_pending;
    assign m_axi_bresp   = 2'b00;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            aw_pending <= 1'b0;
            w_pending  <= 1'b0;
            aw_addr_q  <= 32'h0;
        end else begin
            if (!aw_pending && m_axi_awvalid && m_axi_awready) begin
                aw_pending <= 1'b1;
                aw_addr_q  <= m_axi_awaddr;
            end else if (aw_pending && !w_pending && m_axi_wvalid && m_axi_wready) begin
                w_pending <= 1'b1;
                if (m_axi_wstrb[0]) mem[{aw_addr_q[31:2],2'b00}] <= m_axi_wdata[7:0];
                if (m_axi_wstrb[1]) mem[{aw_addr_q[31:2],2'b01}] <= m_axi_wdata[15:8];
                if (m_axi_wstrb[2]) mem[{aw_addr_q[31:2],2'b10}] <= m_axi_wdata[23:16];
                if (m_axi_wstrb[3]) mem[{aw_addr_q[31:2],2'b11}] <= m_axi_wdata[31:24];
            end else if (w_pending && m_axi_bvalid && m_axi_bready) begin
                aw_pending <= 1'b0;
                w_pending  <= 1'b0;
            end
        end
    end

    // =================================================================
    // Clock / reset
    // =================================================================
    initial clk = 1'b0;
    always #5 clk = ~clk;

    // =================================================================
    // AXI4-Lite BFM tasks (CPU -> khoi UART)
    // =================================================================
    task axi_write(input [S_ADDR_WIDTH-1:0] addr, input [31:0] data);
        begin
            s_axi_awaddr  = addr;
            s_axi_awvalid = 1'b1;
            s_axi_wdata   = data;
            s_axi_wstrb   = 4'hF;
            s_axi_wvalid  = 1'b1;
            s_axi_bready  = 1'b1;
            @(posedge clk);
            while (!(s_axi_awready && s_axi_wready)) @(posedge clk);
            s_axi_awvalid = 1'b0;
            s_axi_wvalid  = 1'b0;
            while (!s_axi_bvalid) @(posedge clk);
            @(posedge clk);
            s_axi_bready = 1'b0;
        end
    endtask

    task axi_read(input [S_ADDR_WIDTH-1:0] addr, output [31:0] data);
        begin
            s_axi_araddr  = addr;
            s_axi_arvalid = 1'b1;
            s_axi_rready  = 1'b1;
            @(posedge clk);
            while (!s_axi_arready) @(posedge clk);
            s_axi_arvalid = 1'b0;
            while (!s_axi_rvalid) @(posedge clk);
            data = s_axi_rdata;
            @(posedge clk);
            s_axi_rready = 1'b0;
        end
    endtask

    localparam A_CTRL     = 6'h00;
    localparam A_BAUD_DIV = 6'h08;
    localparam A_TX_ADDR  = 6'h0C;
    localparam A_TX_LEN   = 6'h10;
    localparam A_RX_ADDR  = 6'h14;
    localparam A_RX_LEN   = 6'h18;

    reg [31:0] rdval;
    integer i;
    reg [7:0] golden [0:7];

    initial begin
        rst_n         = 1'b0;
        s_axi_awaddr  = 0; s_axi_awvalid = 0;
        s_axi_wdata   = 0; s_axi_wstrb   = 0; s_axi_wvalid = 0;
        s_axi_bready  = 0;
        s_axi_araddr  = 0; s_axi_arvalid = 0;
        s_axi_rready  = 0;

        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        axi_write(A_BAUD_DIV, 32'd1);

        $display("[TB] ---- Test ghep UART + DMA thuc (qua bo nho AXI4) ----");
        for (i = 0; i < 8; i = i + 1) begin
            golden[i] = 8'h60 + i;   // '`'..'g'
            mem[i]    = golden[i];  // vung nguon @ 0x00..0x07
            mem[64+i] = 8'h00;      // vung dich  @ 0x40..0x47
        end

        axi_write(A_RX_ADDR, 32'd64);
        axi_write(A_RX_LEN,  32'd8);
        axi_write(A_CTRL,    32'h0000_0002); // RX_START

        axi_write(A_TX_ADDR, 32'd0);
        axi_write(A_TX_LEN,  32'd8);
        axi_write(A_CTRL,    32'h0000_0001); // TX_START

        repeat (6000) @(posedge clk);

        for (i = 0; i < 8; i = i + 1) begin
            if (mem[64+i] !== golden[i]) begin
                $display("[TB] LOI: mem[%0d] = 0x%02x, mong doi 0x%02x",
                          64+i, mem[64+i], golden[i]);
                errors = errors + 1;
            end else begin
                $display("[TB] OK: mem[%0d] = 0x%02x", 64+i, mem[64+i]);
            end
        end

        if (errors == 0)
            $display("[TB] ===== UART + DMA GHEP DUNG, TAT CA TEST PASS =====");
        else
            $display("[TB] ===== CO %0d LOI =====", errors);

        $finish;
    end

    initial begin
        #2000000;
        $display("[TB] TIMEOUT.");
        $finish;
    end

endmodule
