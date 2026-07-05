//=====================================================================
// tb_riscv_soc_top.v
// Testbench cho riscv_soc_top:
//   - Sinh clock/reset.
//   - Nap chuong trinh test (imem.hex) vao ROM lenh qua tham so IMEM_FILE.
//   - Lam AXI4-Lite SLAVE "gia lap" phia ngoai de tra loi cac giao dich
//     AW/W/B (ghi) va AR/R (doc) tu bridge cua SoC, mo phong 1 thanh ghi
//     ngoai vi don gian (echo lai gia tri da ghi lan cuoi).
//   - Tu dong kiem tra ket qua trong regfile / RAM sau khi chuong trinh
//     chay xong (CPU roi vao vong lap vo han tai dia chi cuoi) va bao
//     PASS/FAIL.
//
// Chuong trinh test (xem asm.py / imem.hex) thuc hien:
//   addi x1,x0,5        x2,x0,10
//   add  x3,x1,x2        -> x3 = 15
//   sub  x4,x2,x1        -> x4 = 5
//   and  x5,x1,x2        -> x5 = 0
//   or   x6,x1,x2        -> x6 = 15
//   xor  x7,x1,x2        -> x7 = 15
//   sw   x3,0(x0)        -> RAM[0] = 15
//   lw   x8,0(x0)        -> x8 = 15
//   beq  x3,x8,+8        -> nhay qua lenh addi x9 (x9 phai = 0, khong bi ghi)
//   jal  x10,+8          -> x10 = 0x30, nhay qua lenh addi x11 (x11 phai = 0)
//   lui  x12,4           -> x12 = 0x4000 (vung ngoai vi, addr[31:14] != 0)
//   sw   x3,0(x12)       -> ghi 15 ra AXI4-Lite toi "thanh ghi" ngoai vi
//   lw   x13,0(x12)      -> doc lai tu AXI4-Lite, ky vong = gia tri slave echo (0xCAFEBABE lan dau,
//                            hoac chinh gia tri 15 da ghi neu slave echo-back - xem AXI_SLAVE_MODE)
//   jal  x0,0            -> vong lap vo han tai dia chi 0x40 (bao hieu chuong trinh ket thuc)
//=====================================================================
`timescale 1ns/1ps

module tb_riscv_soc_top;

    // -----------------------------------------------------------------
    // Tham so
    // -----------------------------------------------------------------
    localparam S_ADDR_WIDTH = 32;
    localparam CLK_PERIOD   = 10;      // 100 MHz
    localparam TIMEOUT_CYC  = 2000;    // watchdog: so chu ky toi da truoc khi ep ket thuc

    // Dia chi cuoi chuong trinh: CPU nam trong vong lap "jal x0,0" tai day
    // -> dung de phat hien chuong trinh da chay xong.
    localparam [31:0] DONE_PC = 32'h0000_0040;

    // Gia tri AXI slave gia lap tra ve khi CPU DOC tu ngoai vi (truoc khi
    // co ghi nao). Neu muon slave "echo" lai du lieu vua ghi, xem bien
    // periph_reg duoc cap nhat trong khoi always cua slave ben duoi.
    localparam [31:0] AXI_SLAVE_INIT_RDATA = 32'hCAFE_BABE;

    // -----------------------------------------------------------------
    // Clock / reset
    // -----------------------------------------------------------------
    reg clk;
    reg rst_n;

    always #(CLK_PERIOD/2) clk = ~clk;

    // -----------------------------------------------------------------
    // Tin hieu AXI4-Lite giua SoC (master) va slave gia lap trong TB
    // -----------------------------------------------------------------
    wire [S_ADDR_WIDTH-1:0] m_axi_awaddr;
    wire                    m_axi_awvalid;
    reg                     m_axi_awready;
    wire [31:0]             m_axi_wdata;
    wire [3:0]               m_axi_wstrb;
    wire                    m_axi_wvalid;
    reg                     m_axi_wready;
    reg  [1:0]               m_axi_bresp;
    reg                     m_axi_bvalid;
    wire                    m_axi_bready;

    wire [S_ADDR_WIDTH-1:0] m_axi_araddr;
    wire                    m_axi_arvalid;
    reg                     m_axi_arready;
    reg  [31:0]             m_axi_rdata;
    reg  [1:0]               m_axi_rresp;
    reg                     m_axi_rvalid;
    wire                    m_axi_rready;

    // -----------------------------------------------------------------
    // DUT
    // -----------------------------------------------------------------
    riscv_soc_top #(
        .S_ADDR_WIDTH (S_ADDR_WIDTH),
        .IMEM_FILE    ("imem.hex")
    ) dut (
        .clk            (clk),
        .rst_n          (rst_n),

        .m_axi_awaddr   (m_axi_awaddr),
        .m_axi_awvalid  (m_axi_awvalid),
        .m_axi_awready  (m_axi_awready),
        .m_axi_wdata    (m_axi_wdata),
        .m_axi_wstrb    (m_axi_wstrb),
        .m_axi_wvalid   (m_axi_wvalid),
        .m_axi_wready   (m_axi_wready),
        .m_axi_bresp    (m_axi_bresp),
        .m_axi_bvalid   (m_axi_bvalid),
        .m_axi_bready   (m_axi_bready),

        .m_axi_araddr   (m_axi_araddr),
        .m_axi_arvalid  (m_axi_arvalid),
        .m_axi_arready  (m_axi_arready),
        .m_axi_rdata    (m_axi_rdata),
        .m_axi_rresp    (m_axi_rresp),
        .m_axi_rvalid   (m_axi_rvalid),
        .m_axi_rready   (m_axi_rready)
    );

    // -----------------------------------------------------------------
    // AXI4-Lite SLAVE gia lap (mo phong 1 thanh ghi ngoai vi 32-bit)
    // FSM don gian, moi kenh bat tay 1 chu ky (co the them do tre neu
    // muon kiem tra cpu_stall giu lau hon).
    // -----------------------------------------------------------------
    reg [31:0] periph_reg;
    integer    axi_write_count;
    integer    axi_read_count;

    localparam AS_IDLE=2'd0, AS_AW=2'd1, AS_W=2'd2, AS_B=2'd3;
    reg [1:0] awstate;
    localparam RS_IDLE=2'd0, RS_AR=2'd1, RS_R=2'd2;
    reg [1:0] arstate;

    // --- kenh ghi: AW/W/B ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_axi_awready   <= 1'b0;
            m_axi_wready    <= 1'b0;
            m_axi_bvalid    <= 1'b0;
            m_axi_bresp     <= 2'b00;
            periph_reg      <= 32'h0;
            axi_write_count <= 0;
            awstate         <= AS_IDLE;
        end else begin
            case (awstate)
                AS_IDLE: begin
                    m_axi_bvalid <= 1'b0;
                    if (m_axi_awvalid) begin
                        m_axi_awready <= 1'b1;
                        awstate       <= AS_AW;
                    end
                end
                AS_AW: begin
                    m_axi_awready <= 1'b0;
                    if (m_axi_wvalid) begin
                        m_axi_wready <= 1'b1;
                        awstate      <= AS_W;
                    end else begin
                        // W co the da valid cung luc voi AW; xu ly luon
                        m_axi_wready <= 1'b0;
                    end
                end
                AS_W: begin
                    m_axi_wready <= 1'b0;
                    // luu du lieu ghi vao "thanh ghi" ngoai vi (theo wstrb)
                    if (m_axi_wstrb[0]) periph_reg[7:0]   <= m_axi_wdata[7:0];
                    if (m_axi_wstrb[1]) periph_reg[15:8]  <= m_axi_wdata[15:8];
                    if (m_axi_wstrb[2]) periph_reg[23:16] <= m_axi_wdata[23:16];
                    if (m_axi_wstrb[3]) periph_reg[31:24] <= m_axi_wdata[31:24];
                    axi_write_count <= axi_write_count + 1;
                    m_axi_bvalid    <= 1'b1;
                    m_axi_bresp     <= 2'b00; // OKAY
                    awstate         <= AS_B;
                end
                AS_B: begin
                    if (m_axi_bvalid && m_axi_bready) begin
                        m_axi_bvalid <= 1'b0;
                        awstate      <= AS_IDLE;
                    end
                end
                default: awstate <= AS_IDLE;
            endcase
        end
    end

    // Bat W truoc AW (truong hop hiem, phong ngua deadlock don gian) -
    // trong thiet ke bridge hien tai AW/W duoc keo len cung luc nen
    // nhanh AS_AW se luon thay m_axi_wvalid='1' ngay chu ky ke tiep.

    // --- kenh doc: AR/R ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_axi_arready  <= 1'b0;
            m_axi_rvalid   <= 1'b0;
            m_axi_rdata    <= AXI_SLAVE_INIT_RDATA;
            m_axi_rresp    <= 2'b00;
            axi_read_count <= 0;
            arstate        <= RS_IDLE;
        end else begin
            case (arstate)
                RS_IDLE: begin
                    m_axi_rvalid <= 1'b0;
                    if (m_axi_arvalid) begin
                        m_axi_arready <= 1'b1;
                        arstate       <= RS_AR;
                    end
                end
                RS_AR: begin
                    m_axi_arready <= 1'b0;
                    // Echo lai gia tri da ghi gan nhat vao periph_reg;
                    // neu chua ghi lan nao thi tra ve AXI_SLAVE_INIT_RDATA.
                    m_axi_rdata    <= (axi_write_count > 0) ? periph_reg : AXI_SLAVE_INIT_RDATA;
                    m_axi_rresp    <= 2'b00; // OKAY
                    m_axi_rvalid   <= 1'b1;
                    axi_read_count <= axi_read_count + 1;
                    arstate        <= RS_R;
                end
                RS_R: begin
                    if (m_axi_rvalid && m_axi_rready) begin
                        m_axi_rvalid <= 1'b0;
                        arstate      <= RS_IDLE;
                    end
                end
                default: arstate <= RS_IDLE;
            endcase
        end
    end

    // -----------------------------------------------------------------
    // Waveform dump
    // -----------------------------------------------------------------
    initial begin
        $dumpfile("tb_riscv_soc_top.vcd");
        $dumpvars(0, tb_riscv_soc_top);
    end

    // -----------------------------------------------------------------
    // Theo doi PC de phat hien chuong trinh da toi vong lap ket thuc
    // (dut.u_core.pc == DONE_PC lien tuc trong vai chu ky).
    // -----------------------------------------------------------------
    integer done_streak;
    integer cycle_count;
    reg     program_done;

    initial begin
        cycle_count  = 0;
        done_streak  = 0;
        program_done = 1'b0;
    end

    // Luu y: PC KHONG dung yen tuyet doi tai DONE_PC - do tang IF van
    // tiep tuc fetch tuan tu (DONE_PC, DONE_PC+4, DONE_PC+8, ...) truoc
    // khi tang EX phat hien "jal x0,0" va flush PC ve lai DONE_PC (chu
    // ky lap = 3). Vi vay ta DEM SO LAN pc == DONE_PC xuat hien (khong
    // can lien tiep) thay vi doi dung yen.
    always @(posedge clk) begin
        if (rst_n) begin
            cycle_count <= cycle_count + 1;
            if (dut.u_core.pc == DONE_PC)
                done_streak <= done_streak + 1;

            if (done_streak == 4)
                program_done <= 1'b1;
        end
    end

    // (Tuy chon) in ra instruction trace don gian - bat/tat bang bien PRINT_TRACE
    localparam PRINT_TRACE = 0;
    always @(posedge clk) begin
        if (PRINT_TRACE && rst_n)
            $display("[%0t] PC=%08h IF/ID_instr=%08h", $time, dut.u_core.pc, dut.u_core.if_id_instr);
    end

    // -----------------------------------------------------------------
    // Task kiem tra 1 gia tri thanh ghi / bo nho, dem so loi
    // -----------------------------------------------------------------
    integer error_count;

    task check32;
        input [511:0] name;     // ten hien thi (chuoi, toi da ~64 ky tu)
        input [31:0]  actual;
        input [31:0]  expected;
        begin
            if (actual !== expected) begin
                $display("  [FAIL] %0s = 0x%08h (ky vong 0x%08h)", name, actual, expected);
                error_count = error_count + 1;
            end else begin
                $display("  [PASS] %0s = 0x%08h", name, actual);
            end
        end
    endtask

    // -----------------------------------------------------------------
    // Kich ban chinh
    // -----------------------------------------------------------------
    initial begin
        clk   = 1'b0;
        rst_n = 1'b0;
        error_count = 0;

        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        $display("========================================================");
        $display(" TESTBENCH: riscv_soc_top  -  bat dau mo phong tai t=%0t", $time);
        $display("========================================================");

        // Cho chuong trinh chay toi vong lap ket thuc, hoac het thoi gian timeout
        wait (program_done || cycle_count > TIMEOUT_CYC);

        if (!program_done) begin
            $display("[FAIL] TIMEOUT: chuong trinh khong toi duoc dia chi vong lap 0x%08h sau %0d chu ky",
                      DONE_PC, TIMEOUT_CYC);
            error_count = error_count + 1;
        end else begin
            $display("-> CPU da toi vong lap ket thuc (PC=0x%08h) sau %0d chu ky", DONE_PC, cycle_count);
        end

        // cho vai chu ky de moi ghi/doc cuoi cung on dinh hoan toan
        repeat (5) @(posedge clk);

        $display("--------------------------------------------------------");
        $display(" Kiem tra thanh ghi (regfile) qua duong dan hierarchical");
        $display("--------------------------------------------------------");
        check32("x1  (5)               ", dut.u_core.u_regfile.regs[1],  32'd5);
        check32("x2  (10)              ", dut.u_core.u_regfile.regs[2],  32'd10);
        check32("x3  (x1+x2=15)        ", dut.u_core.u_regfile.regs[3],  32'd15);
        check32("x4  (x2-x1=5)         ", dut.u_core.u_regfile.regs[4],  32'd5);
        check32("x5  (x1&x2=0)         ", dut.u_core.u_regfile.regs[5],  32'd0);
        check32("x6  (x1|x2=15)        ", dut.u_core.u_regfile.regs[6],  32'd15);
        check32("x7  (x1^x2=15)        ", dut.u_core.u_regfile.regs[7],  32'd15);
        check32("x8  (lw tu RAM =15)   ", dut.u_core.u_regfile.regs[8],  32'd15);
        check32("x9  (khong duoc ghi - beq phai nhay qua)", dut.u_core.u_regfile.regs[9], 32'd0);
        check32("x10 (jal link = 0x30) ", dut.u_core.u_regfile.regs[10], 32'h0000_0030);
        check32("x11 (khong duoc ghi - jal phai nhay qua)", dut.u_core.u_regfile.regs[11], 32'd0);
        check32("x12 (lui 4 = 0x4000)  ", dut.u_core.u_regfile.regs[12], 32'h0000_4000);
        check32("x13 (lw tu AXI periph)", dut.u_core.u_regfile.regs[13], 32'd15);

        $display("--------------------------------------------------------");
        $display(" Kiem tra RAM du lieu noi bo (dmem)");
        $display("--------------------------------------------------------");
        check32("dmem[word 0] (sw x3,0(x0)=15)", dut.u_dmem.mem[0], 32'd15);

        $display("--------------------------------------------------------");
        $display(" Kiem tra giao dich AXI4-Lite toi ngoai vi");
        $display("--------------------------------------------------------");
        if (axi_write_count < 1) begin
            $display("  [FAIL] Khong ghi nhan giao dich GHI AXI4-Lite nao (axi_write_count=%0d)", axi_write_count);
            error_count = error_count + 1;
        end else
            $display("  [PASS] So giao dich GHI AXI4-Lite = %0d", axi_write_count);

        if (axi_read_count < 1) begin
            $display("  [FAIL] Khong ghi nhan giao dich DOC AXI4-Lite nao (axi_read_count=%0d)", axi_read_count);
            error_count = error_count + 1;
        end else
            $display("  [PASS] So giao dich DOC AXI4-Lite = %0d", axi_read_count);

        check32("periph_reg (thanh ghi slave, phai =15)", periph_reg, 32'd15);

        $display("========================================================");
        if (error_count == 0)
            $display(" KET QUA TONG: PASS - tat ca %0d kiem tra deu dung", 13);
        else
            $display(" KET QUA TONG: FAIL - %0d loi", error_count);
        $display("========================================================");

        $finish;
    end

endmodule
