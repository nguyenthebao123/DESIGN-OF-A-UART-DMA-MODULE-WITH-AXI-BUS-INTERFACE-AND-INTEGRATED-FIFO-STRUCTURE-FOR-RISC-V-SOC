//=====================================================================
// axi4lite_regs.v
// Khoi thanh ghi dieu khien/trang thai, giao tiep AXI4-Lite slave.
// Bat tay AW/W/B va AR/R duoc cai bang FSM 2 trang thai voi ngo ra
// *ready* to hop (khong dang ky), tranh hoan toan loi tranh chap
// (race) thuong gap voi mau "aw_en" kinh dien khi ready la thanh
// ghi dong bo.
//
// Ban do thanh ghi (offset byte, 32-bit, word-aligned):
//   0x00 CTRL        bit0 TX_START(*) bit1 RX_START(*) bit2 SOFT_RST
//                     bit3 TX_FIFO_CLR bit4 RX_FIFO_CLR
//                     (*) tu xoa ve 0 sau khi DMA da nhan lenh
//   0x04 STATUS  (RO) bit0 TX_BUSY bit1 RX_BUSY bit2 TXFIFO_EMPTY
//                     bit3 TXFIFO_FULL bit4 RXFIFO_EMPTY bit5 RXFIFO_FULL
//   0x08 BAUD_DIV     he so chia bao tao baud_gen
//   0x0C TX_ADDR      dia chi nguon trong bo nho (DMA doc -> phat UART)
//   0x10 TX_LEN       so byte can phat
//   0x14 RX_ADDR      dia chi dich trong bo nho (DMA ghi <- nhan UART)
//   0x18 RX_LEN       so byte can nhan
//   0x1C IRQ_EN       bit0 TX_DONE bit1 RX_DONE bit2 RX_OVF bit3 RX_FERR
//   0x20 IRQ_STATUS   sticky, ghi 1 de xoa (write-1-to-clear)
//   0x24 TX_DATA (WO) ghi byte truc tiep vao TX FIFO (mode polling, khong DMA)
//   0x28 RX_DATA (RO) doc se lay 1 byte tu RX FIFO (mode polling)
//=====================================================================
module axi4lite_regs #(
    parameter ADDR_WIDTH = 6,
    parameter DATA_WIDTH = 32
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // AW
    input  wire [ADDR_WIDTH-1:0]    s_axi_awaddr,
    input  wire                     s_axi_awvalid,
    output wire                     s_axi_awready,

    // W
    input  wire [DATA_WIDTH-1:0]    s_axi_wdata,
    input  wire [DATA_WIDTH/8-1:0]  s_axi_wstrb,
    input  wire                     s_axi_wvalid,
    output wire                     s_axi_wready,

    // B
    output reg  [1:0]               s_axi_bresp,
    output reg                      s_axi_bvalid,
    input  wire                     s_axi_bready,

    // AR
    input  wire [ADDR_WIDTH-1:0]    s_axi_araddr,
    input  wire                     s_axi_arvalid,
    output wire                     s_axi_arready,

    // R
    output reg  [DATA_WIDTH-1:0]    s_axi_rdata,
    output reg  [1:0]               s_axi_rresp,
    output reg                      s_axi_rvalid,
    input  wire                     s_axi_rready,

    // ---- giao tiep voi phan loi (core) con lai ----
    output reg  [31:0]              reg_ctrl,
    input  wire [31:0]              reg_status,
    output reg  [31:0]              reg_baud_div,
    output reg  [31:0]              reg_tx_addr,
    output reg  [31:0]              reg_tx_len,
    output reg  [31:0]              reg_rx_addr,
    output reg  [31:0]              reg_rx_len,
    output reg  [31:0]              reg_irq_en,
    input  wire [31:0]              irq_status_set,
    output reg  [31:0]              reg_irq_status,

    input  wire                     tx_start_clear,   // DMA da nhan lenh TX_START
    input  wire                     rx_start_clear,   // DMA da nhan lenh RX_START

    output reg                      cpu_tx_push_en,   // CPU day 1 byte vao TX FIFO
    output reg  [7:0]               cpu_tx_push_data,
    output reg                      cpu_rx_pop_en,    // CPU lay 1 byte tu RX FIFO
    input  wire [7:0]               cpu_rx_pop_data
);

    localparam ADDR_LSB        = 2;
    localparam ADDR_CTRL       = 6'h00;
    localparam ADDR_STATUS     = 6'h04;
    localparam ADDR_BAUD_DIV   = 6'h08;
    localparam ADDR_TX_ADDR    = 6'h0C;
    localparam ADDR_TX_LEN     = 6'h10;
    localparam ADDR_RX_ADDR    = 6'h14;
    localparam ADDR_RX_LEN     = 6'h18;
    localparam ADDR_IRQ_EN     = 6'h1C;
    localparam ADDR_IRQ_STATUS = 6'h20;
    localparam ADDR_TX_DATA    = 6'h24;
    localparam ADDR_RX_DATA    = 6'h28;

    //================================================================
    // Kenh ghi AW/W/B : FSM 2 trang thai, ready la to hop
    //================================================================
    localparam W_IDLE = 1'b0, W_RESP = 1'b1;
    reg w_state;

    assign s_axi_awready = (w_state == W_IDLE);
    assign s_axi_wready  = (w_state == W_IDLE);

    wire write_en = (w_state == W_IDLE) && s_axi_awvalid && s_axi_wvalid;
    wire [ADDR_WIDTH-1:0] wsel = s_axi_awaddr; // hop le dung luc write_en='1'

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_state      <= W_IDLE;
            s_axi_bvalid <= 1'b0;
            s_axi_bresp  <= 2'b00;
        end else begin
            case (w_state)
                W_IDLE: begin
                    if (write_en) begin
                        s_axi_bvalid <= 1'b1;
                        s_axi_bresp  <= 2'b00;
                        w_state      <= W_RESP;
                    end
                end
                W_RESP: begin
                    if (s_axi_bready) begin
                        s_axi_bvalid <= 1'b0;
                        w_state      <= W_IDLE;
                    end
                end
                default: w_state <= W_IDLE;
            endcase
        end
    end

    //----------------------------------------------------------------
    // CTRL: tach rieng vi HW phai tu xoa bit TX_START/RX_START
    //----------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_ctrl <= 32'h0;
        end else if (write_en &&
                     wsel[ADDR_WIDTH-1:ADDR_LSB] == ADDR_CTRL[ADDR_WIDTH-1:ADDR_LSB]) begin
            reg_ctrl <= s_axi_wdata;
        end else begin
            if (tx_start_clear) reg_ctrl[0] <= 1'b0;
            if (rx_start_clear) reg_ctrl[1] <= 1'b0;
        end
    end

    //----------------------------------------------------------------
    // Cac thanh ghi con lai + xung day/lay byte truc tiep (polling mode)
    //----------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_baud_div     <= 32'h0;
            reg_tx_addr      <= 32'h0;
            reg_tx_len       <= 32'h0;
            reg_rx_addr      <= 32'h0;
            reg_rx_len       <= 32'h0;
            reg_irq_en       <= 32'h0;
            reg_irq_status   <= 32'h0;
            cpu_tx_push_en   <= 1'b0;
            cpu_tx_push_data <= 8'h0;
        end else begin
            cpu_tx_push_en <= 1'b0;
            // OR-in cac xung ngat tu phan loi (TX_DONE, RX_DONE, loi...)
            reg_irq_status <= reg_irq_status | irq_status_set;

            if (write_en) begin
                case (wsel[ADDR_WIDTH-1:ADDR_LSB])
                    ADDR_BAUD_DIV[ADDR_WIDTH-1:ADDR_LSB]:   reg_baud_div   <= s_axi_wdata;
                    ADDR_TX_ADDR[ADDR_WIDTH-1:ADDR_LSB]:    reg_tx_addr    <= s_axi_wdata;
                    ADDR_TX_LEN[ADDR_WIDTH-1:ADDR_LSB]:     reg_tx_len     <= s_axi_wdata;
                    ADDR_RX_ADDR[ADDR_WIDTH-1:ADDR_LSB]:    reg_rx_addr    <= s_axi_wdata;
                    ADDR_RX_LEN[ADDR_WIDTH-1:ADDR_LSB]:     reg_rx_len     <= s_axi_wdata;
                    ADDR_IRQ_EN[ADDR_WIDTH-1:ADDR_LSB]:     reg_irq_en     <= s_axi_wdata;
                    ADDR_IRQ_STATUS[ADDR_WIDTH-1:ADDR_LSB]:
                        reg_irq_status <= (reg_irq_status | irq_status_set) & ~s_axi_wdata;
                    ADDR_TX_DATA[ADDR_WIDTH-1:ADDR_LSB]: begin
                        cpu_tx_push_en   <= 1'b1;
                        cpu_tx_push_data <= s_axi_wdata[7:0];
                    end
                    default: ;
                endcase
            end
        end
    end

    //================================================================
    // Kenh doc AR/R : FSM 2 trang thai, ready la to hop
    //================================================================
    localparam R_IDLE = 1'b0, R_RESP = 1'b1;
    reg r_state;

    assign s_axi_arready = (r_state == R_IDLE);

    reg [31:0] rdata_mux;
    always @(*) begin
        case (s_axi_araddr[ADDR_WIDTH-1:ADDR_LSB])
            ADDR_CTRL[ADDR_WIDTH-1:ADDR_LSB]:       rdata_mux = reg_ctrl;
            ADDR_STATUS[ADDR_WIDTH-1:ADDR_LSB]:     rdata_mux = reg_status;
            ADDR_BAUD_DIV[ADDR_WIDTH-1:ADDR_LSB]:   rdata_mux = reg_baud_div;
            ADDR_TX_ADDR[ADDR_WIDTH-1:ADDR_LSB]:    rdata_mux = reg_tx_addr;
            ADDR_TX_LEN[ADDR_WIDTH-1:ADDR_LSB]:     rdata_mux = reg_tx_len;
            ADDR_RX_ADDR[ADDR_WIDTH-1:ADDR_LSB]:    rdata_mux = reg_rx_addr;
            ADDR_RX_LEN[ADDR_WIDTH-1:ADDR_LSB]:     rdata_mux = reg_rx_len;
            ADDR_IRQ_EN[ADDR_WIDTH-1:ADDR_LSB]:     rdata_mux = reg_irq_en;
            ADDR_IRQ_STATUS[ADDR_WIDTH-1:ADDR_LSB]: rdata_mux = reg_irq_status;
            ADDR_RX_DATA[ADDR_WIDTH-1:ADDR_LSB]:    rdata_mux = {24'h0, cpu_rx_pop_data};
            default:                                rdata_mux = 32'h0;
        endcase
    end

    wire read_en = (r_state == R_IDLE) && s_axi_arvalid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_state       <= R_IDLE;
            s_axi_rvalid  <= 1'b0;
            s_axi_rresp   <= 2'b00;
            s_axi_rdata   <= 32'h0;
            cpu_rx_pop_en <= 1'b0;
        end else begin
            cpu_rx_pop_en <= 1'b0;
            case (r_state)
                R_IDLE: begin
                    if (read_en) begin
                        s_axi_rdata  <= rdata_mux;
                        s_axi_rresp  <= 2'b00;
                        s_axi_rvalid <= 1'b1;
                        if (s_axi_araddr[ADDR_WIDTH-1:ADDR_LSB] == ADDR_RX_DATA[ADDR_WIDTH-1:ADDR_LSB])
                            cpu_rx_pop_en <= 1'b1;  // doc RX_DATA se pop RX FIFO
                        r_state <= R_RESP;
                    end
                end
                R_RESP: begin
                    if (s_axi_rready) begin
                        s_axi_rvalid <= 1'b0;
                        r_state      <= R_IDLE;
                    end
                end
                default: r_state <= R_IDLE;
            endcase
        end
    end

endmodule
