//=====================================================================
// axi4_dma_master.v
// Khoi DMA, ket noi truc tiep voi khoi UART (uart_axi_top.v) qua dung
// "hop dong tin hieu" da thong nhat (xem docs/interface_uart_dma.md):
//   - Kenh doc AXI4 (AR/R)  : doc bo nho he thong -> day vao TX FIFO
//   - Kenh ghi AXI4 (AW/W/B): lay tu RX FIFO -> ghi vao bo nho he thong
// Hai huong dung 2 kenh AXI4 doc lap nen chay song song tu nhien.
//=====================================================================
module axi4_dma_master #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
)(
    input  wire                      clk,
    input  wire                      rst_n,

    // ---- Tu khoi UART (uart_axi_top) ----
    input  wire                      tx_dma_start,
    input  wire [31:0]               tx_addr,
    input  wire [31:0]               tx_len,
    input  wire                      tx_fifo_full,

    input  wire                      rx_dma_start,
    input  wire [31:0]               rx_addr,
    input  wire [31:0]               rx_len,
    input  wire [7:0]                rx_fifo_rd_data,
    input  wire                      rx_fifo_empty,

    // ---- Ra khoi UART (uart_axi_top) ----
    output wire                      tx_start_clear,
    output reg                       tx_dma_busy,
    output reg                       tx_done_pulse,
    output reg                       dma_tx_fifo_wr_en,
    output reg  [7:0]                dma_tx_fifo_wr_data,

    output wire                      rx_start_clear,
    output reg                       rx_dma_busy,
    output reg                       rx_done_pulse,
    output reg                       dma_rx_fifo_rd_en,

    // ---- AXI4 master: ket noi bo nho he thong ----
    output reg  [ADDR_WIDTH-1:0]     m_axi_araddr,
    output reg  [7:0]                m_axi_arlen,
    output reg  [2:0]                m_axi_arsize,
    output reg  [1:0]                m_axi_arburst,
    output reg                       m_axi_arvalid,
    input  wire                      m_axi_arready,
    input  wire [DATA_WIDTH-1:0]     m_axi_rdata,
    input  wire [1:0]                m_axi_rresp,
    input  wire                      m_axi_rlast,
    input  wire                      m_axi_rvalid,
    output reg                       m_axi_rready,

    output reg  [ADDR_WIDTH-1:0]     m_axi_awaddr,
    output reg  [7:0]                m_axi_awlen,
    output reg  [2:0]                m_axi_awsize,
    output reg  [1:0]                m_axi_awburst,
    output reg                       m_axi_awvalid,
    input  wire                      m_axi_awready,
    output reg  [DATA_WIDTH-1:0]     m_axi_wdata,
    output reg  [DATA_WIDTH/8-1:0]   m_axi_wstrb,
    output reg                       m_axi_wlast,
    output reg                       m_axi_wvalid,
    input  wire                      m_axi_wready,
    input  wire [1:0]                m_axi_bresp,
    input  wire                      m_axi_bvalid,
    output reg                       m_axi_bready
);

    // tx_dma_start/rx_dma_start la muc tin hieu tu thanh ghi CTRL; chi
    // can chuyen tiep thang lai cho UART de UART tu xoa bit CTRL (xem
    // docs/interface_uart_dma.md, muc 5.1)
    assign tx_start_clear = tx_dma_start;
    assign rx_start_clear = rx_dma_start;

    //=================================================================
    // Dong doc: bo nho he thong -> TX FIFO (qua dma_tx_fifo_wr_en/data)
    //=================================================================
    localparam T_IDLE = 2'd0, T_AR = 2'd1, T_R = 2'd2;

    reg [1:0]  t_state;
    reg [31:0] t_addr;
    reg [31:0] t_remaining;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            t_state             <= T_IDLE;
            t_addr              <= 32'h0;
            t_remaining         <= 32'h0;
            tx_dma_busy         <= 1'b0;
            tx_done_pulse       <= 1'b0;
            m_axi_arvalid       <= 1'b0;
            m_axi_araddr        <= {ADDR_WIDTH{1'b0}};
            m_axi_arlen         <= 8'h0;
            m_axi_arsize        <= 3'b0;
            m_axi_arburst       <= 2'b0;
            m_axi_rready        <= 1'b0;
            dma_tx_fifo_wr_en   <= 1'b0;
            dma_tx_fifo_wr_data <= 8'h0;
        end else begin
            tx_done_pulse     <= 1'b0;
            dma_tx_fifo_wr_en <= 1'b0;

            case (t_state)
                T_IDLE: begin
                    tx_dma_busy <= 1'b0;
                    if (tx_dma_start && tx_len != 32'h0) begin
                        t_addr      <= tx_addr;
                        t_remaining <= tx_len;
                        tx_dma_busy <= 1'b1;
                        t_state     <= T_AR;
                    end
                end

                T_AR: begin
                    if (!m_axi_arvalid && !tx_fifo_full) begin
                        m_axi_araddr  <= t_addr;
                        m_axi_arlen   <= 8'h00;
                        m_axi_arsize  <= 3'b000;   // 1 byte / beat
                        m_axi_arburst <= 2'b01;    // INCR
                        m_axi_arvalid <= 1'b1;
                    end else if (m_axi_arvalid && m_axi_arready) begin
                        m_axi_arvalid <= 1'b0;
                        m_axi_rready  <= 1'b1;
                        t_state       <= T_R;
                    end
                end

                T_R: begin
                    if (m_axi_rvalid && m_axi_rready) begin
                        case (t_addr[1:0])
                            2'b00: dma_tx_fifo_wr_data <= m_axi_rdata[7:0];
                            2'b01: dma_tx_fifo_wr_data <= m_axi_rdata[15:8];
                            2'b10: dma_tx_fifo_wr_data <= m_axi_rdata[23:16];
                            2'b11: dma_tx_fifo_wr_data <= m_axi_rdata[31:24];
                        endcase
                        dma_tx_fifo_wr_en <= 1'b1;
                        m_axi_rready      <= 1'b0;
                        t_addr            <= t_addr + 1'b1;

                        if (t_remaining == 32'h1) begin
                            t_remaining   <= 32'h0;
                            tx_dma_busy   <= 1'b0;
                            tx_done_pulse <= 1'b1;
                            t_state       <= T_IDLE;
                        end else begin
                            t_remaining <= t_remaining - 1'b1;
                            t_state     <= T_AR;
                        end
                    end
                end

                default: t_state <= T_IDLE;
            endcase
        end
    end

    //=================================================================
    // Dong ghi: RX FIFO (qua dma_rx_fifo_rd_en) -> bo nho he thong
    //=================================================================
    localparam R_IDLE = 2'd0, R_AW = 2'd1, R_W = 2'd2, R_B = 2'd3;

    reg [1:0]  r_state;
    reg [31:0] r_addr;
    reg [31:0] r_remaining;
    reg [7:0]  r_byte_hold;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_state           <= R_IDLE;
            r_addr            <= 32'h0;
            r_remaining       <= 32'h0;
            rx_dma_busy       <= 1'b0;
            rx_done_pulse     <= 1'b0;
            m_axi_awvalid     <= 1'b0;
            m_axi_awaddr      <= {ADDR_WIDTH{1'b0}};
            m_axi_awlen       <= 8'h0;
            m_axi_awsize      <= 3'b0;
            m_axi_awburst     <= 2'b0;
            m_axi_wvalid      <= 1'b0;
            m_axi_wdata       <= {DATA_WIDTH{1'b0}};
            m_axi_wstrb       <= {(DATA_WIDTH/8){1'b0}};
            m_axi_wlast       <= 1'b0;
            m_axi_bready      <= 1'b0;
            dma_rx_fifo_rd_en <= 1'b0;
            r_byte_hold       <= 8'h0;
        end else begin
            rx_done_pulse     <= 1'b0;
            dma_rx_fifo_rd_en <= 1'b0;

            case (r_state)
                R_IDLE: begin
                    rx_dma_busy <= 1'b0;
                    if (rx_dma_start && rx_len != 32'h0) begin
                        r_addr      <= rx_addr;
                        r_remaining <= rx_len;
                        rx_dma_busy <= 1'b1;
                        r_state     <= R_AW;
                    end
                end

                R_AW: begin
                    if (!m_axi_awvalid && !rx_fifo_empty) begin
                        r_byte_hold       <= rx_fifo_rd_data;
                        dma_rx_fifo_rd_en <= 1'b1;
                        m_axi_awaddr      <= r_addr;
                        m_axi_awlen       <= 8'h00;
                        m_axi_awsize      <= 3'b000;
                        m_axi_awburst     <= 2'b01;
                        m_axi_awvalid     <= 1'b1;
                    end else if (m_axi_awvalid && m_axi_awready) begin
                        m_axi_awvalid <= 1'b0;
                        m_axi_wdata   <= {r_byte_hold, r_byte_hold, r_byte_hold, r_byte_hold};
                        case (r_addr[1:0])
                            2'b00: m_axi_wstrb <= 4'b0001;
                            2'b01: m_axi_wstrb <= 4'b0010;
                            2'b10: m_axi_wstrb <= 4'b0100;
                            2'b11: m_axi_wstrb <= 4'b1000;
                        endcase
                        m_axi_wlast  <= 1'b1;
                        m_axi_wvalid <= 1'b1;
                        r_state      <= R_W;
                    end
                end

                R_W: begin
                    if (m_axi_wvalid && m_axi_wready) begin
                        m_axi_wvalid <= 1'b0;
                        m_axi_wlast  <= 1'b0;
                        m_axi_bready <= 1'b1;
                        r_state      <= R_B;
                    end
                end

                R_B: begin
                    if (m_axi_bvalid && m_axi_bready) begin
                        m_axi_bready <= 1'b0;
                        r_addr       <= r_addr + 1'b1;

                        if (r_remaining == 32'h1) begin
                            r_remaining   <= 32'h0;
                            rx_dma_busy   <= 1'b0;
                            rx_done_pulse <= 1'b1;
                            r_state       <= R_IDLE;
                        end else begin
                            r_remaining <= r_remaining - 1'b1;
                            r_state     <= R_AW;
                        end
                    end
                end

                default: r_state <= R_IDLE;
            endcase
        end
    end

endmodule
