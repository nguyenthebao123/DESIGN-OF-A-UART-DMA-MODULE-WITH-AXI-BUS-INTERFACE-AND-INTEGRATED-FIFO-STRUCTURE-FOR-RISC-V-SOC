//=====================================================================
// tb_uart_axi_top.v
// Test rieng khoi cua BAN (UART + FIFO + AXI4-Lite), KHONG can code
// DMA thuc cua ban cung nhom. Testbench tu dong vai "khoi DMA gia":
//   - Thay tx_dma_start='1' thi tu day du lieu mau vao TX FIFO qua
//     dma_tx_fifo_wr_en/data (dung HET tx_len byte), bao tx_done_pulse.
//   - Thay rx_dma_start='1' thi tu lay du lieu ra khoi RX FIFO qua
//     dma_rx_fifo_rd_en (dung HET rx_len byte), bao rx_done_pulse.
// Neu test nay PASS => khoi cua ban dung, sau nay ghep voi DMA thuc
// cua ban cung nhom (theo dung hop dong tin hieu da ghi trong
// uart_axi_top.v) se khong can sua gi ben trong khoi UART nua.
//=====================================================================
`timescale 1ns/1ps

module tb_uart_axi_top;

    localparam S_ADDR_WIDTH = 6;

    reg clk, rst_n;

    // ---- AXI4-Lite slave (CPU) ----
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

    // ---- giao tiep voi "khoi DMA" (gia lap trong TB nay) ----
    wire        tx_dma_start;
    wire [31:0] tx_addr, tx_len;
    wire        tx_fifo_full;
    wire        rx_dma_start;
    wire [31:0] rx_addr, rx_len;
    wire [7:0]  rx_fifo_rd_data;
    wire        rx_fifo_empty;

    reg         tx_start_clear, rx_start_clear;
    reg         tx_dma_busy, rx_dma_busy;
    reg         tx_done_pulse, rx_done_pulse;
    reg         dma_tx_fifo_wr_en;
    reg  [7:0]  dma_tx_fifo_wr_data;
    reg         dma_rx_fifo_rd_en;

    wire uart_line;
    wire irq;

    integer errors = 0;

    // =================================================================
    // DUT - dung KHOI CUA BAN (khong co DMA master ben trong)
    // =================================================================
    uart_axi_top #(
        .S_ADDR_WIDTH (S_ADDR_WIDTH),
        .FIFO_DEPTH   (16)
    ) dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .uart_tx_line   (uart_line),
        .uart_rx_line   (uart_line),   // loopback de tu kiem tra
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

        .tx_dma_start        (tx_dma_start),
        .tx_addr             (tx_addr),
        .tx_len              (tx_len),
        .tx_fifo_full        (tx_fifo_full),
        .rx_dma_start        (rx_dma_start),
        .rx_addr             (rx_addr),
        .rx_len              (rx_len),
        .rx_fifo_rd_data     (rx_fifo_rd_data),
        .rx_fifo_empty       (rx_fifo_empty),

        .tx_start_clear      (tx_start_clear),
        .tx_dma_busy         (tx_dma_busy),
        .tx_done_pulse       (tx_done_pulse),
        .dma_tx_fifo_wr_en   (dma_tx_fifo_wr_en),
        .dma_tx_fifo_wr_data (dma_tx_fifo_wr_data),

        .rx_start_clear      (rx_start_clear),
        .rx_dma_busy         (rx_dma_busy),
        .rx_done_pulse       (rx_done_pulse),
        .dma_rx_fifo_rd_en   (dma_rx_fifo_rd_en)
    );

    // =================================================================
    // "KHOI DMA GIA" - mo phong dung hanh vi ma DMA thuc se lam, de
    // test khoi UART cua ban ma khong can cho code cua ban cung nhom.
    // =================================================================
    reg [7:0] tx_src   [0:31];   // du lieu mau de "phat" (thay cho bo nho)
    reg [7:0] rx_dst   [0:31];   // noi "thu" du lieu nhan duoc
    integer   tx_idx, rx_idx;
    integer   tx_remain, rx_remain;

    localparam FT_IDLE = 0, FT_PUSH = 1;
    reg [1:0] ft_state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ft_state            <= FT_IDLE;
            tx_start_clear      <= 1'b0;
            tx_dma_busy         <= 1'b0;
            tx_done_pulse       <= 1'b0;
            dma_tx_fifo_wr_en   <= 1'b0;
            dma_tx_fifo_wr_data <= 8'h0;
            tx_idx              <= 0;
            tx_remain           <= 0;
        end else begin
            tx_start_clear    <= 1'b0;
            tx_done_pulse     <= 1'b0;
            dma_tx_fifo_wr_en <= 1'b0;

            case (ft_state)
                FT_IDLE: begin
                    tx_dma_busy <= 1'b0;
                    if (tx_dma_start) begin
                        tx_start_clear <= 1'b1;   // bao da nhan lenh (tu xoa CTRL.bit0)
                        tx_idx         <= 0;
                        tx_remain      <= tx_len;
                        tx_dma_busy    <= (tx_len != 0);
                        if (tx_len != 0)
                            ft_state <= FT_PUSH;
                    end
                end
                FT_PUSH: begin
                    if (tx_remain == 0) begin
                        tx_dma_busy   <= 1'b0;
                        tx_done_pulse <= 1'b1;
                        ft_state      <= FT_IDLE;
                    end else if (!tx_fifo_full) begin
                        dma_tx_fifo_wr_en   <= 1'b1;
                        dma_tx_fifo_wr_data <= tx_src[tx_idx];
                        tx_idx              <= tx_idx + 1;
                        tx_remain           <= tx_remain - 1;
                    end
                end
                default: ft_state <= FT_IDLE;
            endcase
        end
    end

    localparam FR_IDLE = 0, FR_CHECK = 1, FR_WAIT = 2;
    reg [1:0] fr_state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fr_state          <= FR_IDLE;
            rx_start_clear    <= 1'b0;
            rx_dma_busy       <= 1'b0;
            rx_done_pulse     <= 1'b0;
            dma_rx_fifo_rd_en <= 1'b0;
            rx_idx            <= 0;
            rx_remain         <= 0;
        end else begin
            rx_start_clear    <= 1'b0;
            rx_done_pulse     <= 1'b0;
            dma_rx_fifo_rd_en <= 1'b0;

            case (fr_state)
                FR_IDLE: begin
                    rx_dma_busy <= 1'b0;
                    if (rx_dma_start) begin
                        rx_start_clear <= 1'b1;
                        rx_idx         <= 0;
                        rx_remain      <= rx_len;
                        rx_dma_busy    <= (rx_len != 0);
                        if (rx_len != 0)
                            fr_state <= FR_CHECK;
                    end
                end
                FR_CHECK: begin
                    if (rx_remain == 0) begin
                        rx_dma_busy   <= 1'b0;
                        rx_done_pulse <= 1'b1;
                        fr_state      <= FR_IDLE;
                    end else if (!rx_fifo_empty) begin
                        // Lay 1 byte, ROI CHO 1 chu ky de con tro doc cua
                        // FIFO kip cap nhat truoc khi kiem tra lai - neu
                        // pop lien tuc khong cho, se doc trung 1 phan tu.
                        rx_dst[rx_idx]    <= rx_fifo_rd_data;
                        dma_rx_fifo_rd_en <= 1'b1;
                        rx_idx            <= rx_idx + 1;
                        rx_remain         <= rx_remain - 1;
                        fr_state          <= FR_WAIT;
                    end
                end
                FR_WAIT: begin
                    fr_state <= FR_CHECK;
                end
                default: fr_state <= FR_IDLE;
            endcase
        end
    end

    // =================================================================
    // Clock / reset
    // =================================================================
    initial clk = 1'b0;
    always #5 clk = ~clk; // 100 MHz

    // =================================================================
    // AXI4-Lite BFM tasks (CPU -> peripheral)
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
    localparam A_STATUS   = 6'h04;
    localparam A_BAUD_DIV = 6'h08;
    localparam A_TX_ADDR  = 6'h0C;
    localparam A_TX_LEN   = 6'h10;
    localparam A_RX_ADDR  = 6'h14;
    localparam A_RX_LEN   = 6'h18;
    localparam A_TX_DATA  = 6'h24;
    localparam A_RX_DATA  = 6'h28;

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

        // baud_div nho de mo phong nhanh
        axi_write(A_BAUD_DIV, 32'd1);

        // -----------------------------------------------------------
        // TEST 1: polling - ghi 1 byte qua TX_DATA, doc lai qua RX_DATA
        // (khong dung toi "khoi DMA gia" o tren)
        // -----------------------------------------------------------
        $display("[TB] ---- TEST 1: polling TX_DATA/RX_DATA loopback ----");
        axi_write(A_TX_DATA, 32'h00000041); // 'A'
        repeat (500) @(posedge clk);

        axi_read(A_STATUS, rdval);
        if (rdval[4]) begin
            $display("[TB] LOI: RX FIFO rong, khong nhan duoc byte!");
            errors = errors + 1;
        end else begin
            axi_read(A_RX_DATA, rdval);
            if (rdval[7:0] !== 8'h41) begin
                $display("[TB] LOI: doc RX_DATA = 0x%02x, mong doi 0x41", rdval[7:0]);
                errors = errors + 1;
            end else begin
                $display("[TB] OK: nhan dung byte 0x%02x qua duong polling", rdval[7:0]);
            end
        end

        // -----------------------------------------------------------
        // TEST 2: duong "DMA" - dung khoi DMA gia lap o tren, KHONG
        // dung toi code DMA thuc cua ban cung nhom.
        // -----------------------------------------------------------
        $display("[TB] ---- TEST 2: giao tiep voi khoi DMA (gia lap) ----");
        for (i = 0; i < 8; i = i + 1) begin
            golden[i] = 8'h30 + i;
            tx_src[i] = golden[i];
            rx_dst[i] = 8'h00;
        end

        axi_write(A_RX_LEN, 32'd8);
        axi_write(A_CTRL,   32'h0000_0002); // bat RX_START truoc

        axi_write(A_TX_LEN, 32'd8);
        axi_write(A_CTRL,   32'h0000_0001); // bat TX_START

        repeat (6000) @(posedge clk);

        for (i = 0; i < 8; i = i + 1) begin
            if (rx_dst[i] !== golden[i]) begin
                $display("[TB] LOI: rx_dst[%0d] = 0x%02x, mong doi 0x%02x",
                          i, rx_dst[i], golden[i]);
                errors = errors + 1;
            end else begin
                $display("[TB] OK: rx_dst[%0d] = 0x%02x", i, rx_dst[i]);
            end
        end

        if (errors == 0)
            $display("[TB] ===== TAT CA TEST PASS (khoi cua ban dung) =====");
        else
            $display("[TB] ===== CO %0d LOI =====", errors);

        $finish;
    end

    initial begin
        #2000000;
        $display("[TB] TIMEOUT - mo phong qua lau, dung lai.");
        $finish;
    end

endmodule
