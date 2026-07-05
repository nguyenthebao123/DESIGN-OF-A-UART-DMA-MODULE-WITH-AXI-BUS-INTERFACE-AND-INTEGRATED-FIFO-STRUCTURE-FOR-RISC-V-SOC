//=====================================================================
// axi4lite_master_bridge.v
// Cau noi giua tang MEM cua CPU va 2 noi: RAM noi bo (truy cap 1 chu ky,
// nhanh) hoac thiet bi ngoai vi qua AXI4-Lite MASTER (vd khoi UART,
// nhieu chu ky vi co bat tay AW/W/B hoac AR/R).
//
// Ban do dia chi (don gian, co the doi sau):
//   addr[31:28] == 4'h1  -> ngoai vi (AXI4-Lite), dung addr[S_ADDR_WIDTH-1:0]
//                            lam dia chi thanh ghi.
//   con lai               -> RAM noi bo.
//
// Khi truy cap ngoai vi: keo cpu_stall='1' cho toi khi giao dich AXI4-Lite
// hoan tat - lam dong bang toan bo pipeline CPU (xem dmem_stall trong
// riscv_core.v).
//=====================================================================
module axi4lite_master_bridge #(
    parameter S_ADDR_WIDTH = 32
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---- tu tang MEM cua CPU ----
    input  wire [31:0] cpu_addr,
    input  wire [31:0] cpu_wdata,
    input  wire [3:0]  cpu_wstrb,
    input  wire        cpu_we,
    input  wire        cpu_re,
    output wire [31:0] cpu_rdata,
    output wire        cpu_stall,

    // ---- ra RAM noi bo ----
    output wire [31:0] ram_addr,
    output wire [31:0] ram_wdata,
    output wire [3:0]  ram_wstrb,
    output wire        ram_we,
    input  wire [31:0] ram_rdata,

    // ---- ra AXI4-Lite master (toi thiet bi ngoai vi) ----
    output reg  [S_ADDR_WIDTH-1:0] m_axi_awaddr,
    output reg                     m_axi_awvalid,
    input  wire                    m_axi_awready,
    output reg  [31:0]             m_axi_wdata,
    output reg  [3:0]              m_axi_wstrb,
    output reg                     m_axi_wvalid,
    input  wire                    m_axi_wready,
    input  wire [1:0]              m_axi_bresp,
    input  wire                    m_axi_bvalid,
    output reg                     m_axi_bready,

    output reg  [S_ADDR_WIDTH-1:0] m_axi_araddr,
    output reg                     m_axi_arvalid,
    input  wire                    m_axi_arready,
    input  wire [31:0]             m_axi_rdata,
    input  wire [1:0]              m_axi_rresp,
    input  wire                    m_axi_rvalid,
    output reg                     m_axi_rready
);

    // SUA: dieu kien cu (addr[31:28]==4'h1) chi dung voi dia chi gia dinh
    // 0x10000000 luc truoc. Dia chi THAT trong he thong (theo Address
    // Editor cua Vivado) la 0x43C00000 (UART) va 0xC0000000 (BRAM) - khong
    // khop dieu kien cu. Logic moi: bat ky dia chi nao NGOAI vung RAM noi
    // bo 16KB (0x0000-0x3FFF) deu duoc coi la ngoai vi, dua thang ra m_axi
    // (dung dia chi DAY DU 32-bit, khong cat cut) de crossbar cua Vivado
    // dinh tuyen dung theo Address Editor.
    wire sel_periph = (cpu_addr[31:14] != 18'b0);

    assign ram_addr  = cpu_addr;
    assign ram_wdata = cpu_wdata;
    assign ram_wstrb = cpu_wstrb;
    assign ram_we    = cpu_we && !sel_periph;

    localparam S_IDLE=3'd0, S_AW=3'd1, S_B=3'd2, S_AR=3'd3, S_R=3'd4, S_DONE=3'd5;
    reg [2:0]  state;
    reg [31:0] rdata_q;

    // "Da xong" cho tung kenh AW/W trong CHINH chu ky nay - tinh to hop
    // tu gia tri HIEN TAI cua thanh ghi valid (truoc khi NBA ap dung),
    // dung de tranh doc nham gia tri "cu" khi 2 kenh hoan tat CUNG luc.
    wire aw_complete = !m_axi_awvalid || m_axi_awready;
    wire w_complete  = !m_axi_wvalid  || m_axi_wready;

    // QUAN TRONG: sau khi giao dich xong, BAT BUOC co 1 chu ky o trang
    // thai S_DONE voi cpu_stall='0' truoc khi quay lai S_IDLE kiem tra
    // yeu cau moi. Neu thieu buoc nay, vi pipeline CPU can it nhat 1 chu
    // ky voi dmem_stall='0' moi tien duoc sang lenh ke tiep, tin hieu
    // cpu_we/cpu_re cua lenh CU se con giu nguyen dung luc state vua ve
    // S_IDLE, bi hieu nham la "yeu cau moi" -> lap lai chinh giao dich
    // vua xong, vo han, khong bao gio tien len duoc.
    wire periph_access_starting = sel_periph && (cpu_we || cpu_re) && (state == S_IDLE);
    assign cpu_stall = (state==S_AW) || (state==S_B) || (state==S_AR) || (state==S_R) ||
                        periph_access_starting;
    assign cpu_rdata = sel_periph ? rdata_q : ram_rdata;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_IDLE;
            m_axi_awaddr  <= {S_ADDR_WIDTH{1'b0}};
            m_axi_awvalid <= 1'b0;
            m_axi_wdata   <= 32'h0;
            m_axi_wstrb   <= 4'h0;
            m_axi_wvalid  <= 1'b0;
            m_axi_bready  <= 1'b0;
            m_axi_araddr  <= {S_ADDR_WIDTH{1'b0}};
            m_axi_arvalid <= 1'b0;
            m_axi_rready  <= 1'b0;
            rdata_q       <= 32'h0;
        end else begin
            case (state)
                S_IDLE: begin
                    if (sel_periph && cpu_we) begin
                        m_axi_awaddr  <= cpu_addr[S_ADDR_WIDTH-1:0];
                        m_axi_awvalid <= 1'b1;
                        m_axi_wdata   <= cpu_wdata;
                        m_axi_wstrb   <= cpu_wstrb;
                        m_axi_wvalid  <= 1'b1;
                        state         <= S_AW;
                    end else if (sel_periph && cpu_re) begin
                        m_axi_araddr  <= cpu_addr[S_ADDR_WIDTH-1:0];
                        m_axi_arvalid <= 1'b1;
                        state         <= S_AR;
                    end
                end

                S_AW: begin
                    if (m_axi_awvalid && m_axi_awready) m_axi_awvalid <= 1'b0;
                    if (m_axi_wvalid  && m_axi_wready)  m_axi_wvalid  <= 1'b0;
                    if (aw_complete && w_complete) begin
                        m_axi_bready <= 1'b1;
                        state        <= S_B;
                    end
                end

                S_B: begin
                    if (m_axi_bvalid && m_axi_bready) begin
                        m_axi_bready <= 1'b0;
                        state        <= S_DONE;
                    end
                end

                S_AR: begin
                    if (m_axi_arvalid && m_axi_arready) begin
                        m_axi_arvalid <= 1'b0;
                        m_axi_rready  <= 1'b1;
                        state         <= S_R;
                    end
                end

                S_R: begin
                    if (m_axi_rvalid && m_axi_rready) begin
                        rdata_q      <= m_axi_rdata;
                        m_axi_rready <= 1'b0;
                        state        <= S_DONE;
                    end
                end

                S_DONE: begin
                    // 1 chu ky "nghi" bat buoc, cpu_stall='0' o day de
                    // pipeline CPU kip tien sang lenh ke tiep.
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
