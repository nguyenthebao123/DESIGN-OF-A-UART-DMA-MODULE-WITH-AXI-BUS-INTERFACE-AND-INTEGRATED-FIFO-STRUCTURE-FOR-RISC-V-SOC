//=====================================================================
// riscv_core.v
// Loi RISC-V RV32I, pipeline 5 tang: IF - ID - EX - MEM - WB.
//
// Xu ly hazard:
//   - Load-use hazard: stall 1 chu ky (chen bubble), roi forwarding tu
//     MEM/WB se cung cap gia tri dung.
//   - Control hazard (branch/jump): giai quyet o tang EX, flush 2 lenh
//     (IF/ID va ID/EX) khi re nhanh/nhay duoc xac dinh la dung.
//   - Forwarding tu EX/MEM va MEM/WB ve dau vao ALU cua tang EX.
//
// Giao tiep bo nho:
//   - imem: cong rieng (Harvard), doc to hop 1 chu ky.
//   - dmem: cong rieng cho tang MEM, co tin hieu dmem_stall de ho tro
//     truy cap nhieu chu ky (vd qua AXI4-Lite toi ngoai vi).
//=====================================================================
module riscv_core #(
    parameter RESET_PC = 32'h0000_0000
)(
    input  wire        clk,
    input  wire        rst_n,

    output wire [31:0] imem_addr,
    input  wire [31:0] imem_rdata,

    output wire [31:0] dmem_addr,
    output wire [31:0] dmem_wdata,
    output wire [3:0]  dmem_wstrb,
    output wire        dmem_we,
    output wire        dmem_re,
    output wire [1:0]  dmem_size,
    output wire        dmem_unsigned,
    input  wire [31:0] dmem_rdata,
    input  wire        dmem_stall
);

    localparam ALU_ADD=4'd0, ALU_SUB=4'd1, ALU_SLL=4'd2, ALU_SLT=4'd3,
               ALU_SLTU=4'd4, ALU_XOR=4'd5, ALU_SRL=4'd6, ALU_SRA=4'd7,
               ALU_OR=4'd8, ALU_AND=4'd9;

    //================================================================
    // PC + IF
    //================================================================
    reg  [31:0] pc;
    wire        pc_stall;
    wire        pc_flush;
    wire [31:0] pc_redirect_target;

    assign imem_addr = pc;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)            pc <= RESET_PC;
        else if (dmem_stall)   pc <= pc;
        else if (pc_flush)     pc <= pc_redirect_target;
        else if (pc_stall)     pc <= pc;
        else                   pc <= pc + 32'd4;
    end

    //================================================================
    // IF/ID
    //================================================================
    reg [31:0] if_id_pc;
    reg [31:0] if_id_instr;

    wire if_id_write_en = !pc_stall && !dmem_stall;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            if_id_pc    <= 32'h0;
            if_id_instr <= 32'h0;
        end else if (dmem_stall) begin
            if_id_pc    <= if_id_pc;
            if_id_instr <= if_id_instr;
        end else if (pc_flush) begin
            if_id_pc    <= 32'h0;
            if_id_instr <= 32'h0;
        end else if (if_id_write_en) begin
            if_id_pc    <= pc;
            if_id_instr <= imem_rdata;
        end
    end

    //================================================================
    // ID
    //================================================================
    wire [4:0] id_rs1_addr = if_id_instr[19:15];
    wire [4:0] id_rs2_addr = if_id_instr[24:20];
    wire [4:0] id_rd_addr  = if_id_instr[11:7];

    wire [31:0] id_rs1_data, id_rs2_data;
    wire [31:0] id_imm;

    wire        id_reg_write;
    wire        id_mem_read, id_mem_write;
    wire [1:0]  id_mem_size;
    wire        id_mem_unsigned;
    wire [3:0]  id_alu_ctrl;
    wire [1:0]  id_alu_src_a_sel;
    wire        id_alu_src_b_sel;
    wire [1:0]  id_result_src;
    wire        id_branch, id_jump, id_jalr;

    control_unit u_ctrl (
        .instr         (if_id_instr),
        .reg_write     (id_reg_write),
        .mem_read      (id_mem_read),
        .mem_write     (id_mem_write),
        .mem_size      (id_mem_size),
        .mem_unsigned  (id_mem_unsigned),
        .alu_ctrl      (id_alu_ctrl),
        .alu_src_a_sel (id_alu_src_a_sel),
        .alu_src_b_sel (id_alu_src_b_sel),
        .result_src    (id_result_src),
        .branch        (id_branch),
        .jump          (id_jump),
        .jalr          (id_jalr)
    );

    imm_gen u_imm (
        .instr (if_id_instr),
        .imm   (id_imm)
    );

    wire        wb_reg_write;
    wire [4:0]  wb_rd_addr;
    wire [31:0] wb_write_data;

    regfile u_regfile (
        .clk      (clk),
        .rst_n    (rst_n),
        .we       (wb_reg_write),
        .rd_addr  (wb_rd_addr),
        .rd_data  (wb_write_data),
        .rs1_addr (id_rs1_addr),
        .rs2_addr (id_rs2_addr),
        .rs1_data (id_rs1_data),
        .rs2_data (id_rs2_data)
    );

    //================================================================
    // ID/EX (khai bao truoc de dung trong hazard detection)
    //================================================================
    reg [31:0] id_ex_pc;
    reg [31:0] id_ex_rs1_data, id_ex_rs2_data, id_ex_imm;
    reg [4:0]  id_ex_rs1_addr, id_ex_rs2_addr, id_ex_rd_addr;
    reg [2:0]  id_ex_funct3;
    reg        id_ex_reg_write, id_ex_mem_read, id_ex_mem_write;
    reg [1:0]  id_ex_mem_size;
    reg        id_ex_mem_unsigned;
    reg [3:0]  id_ex_alu_ctrl;
    reg [1:0]  id_ex_alu_src_a_sel;
    reg        id_ex_alu_src_b_sel;
    reg [1:0]  id_ex_result_src;
    reg        id_ex_branch, id_ex_jump, id_ex_jalr;

    //================================================================
    // Hazard detection (load-use)
    //================================================================
    wire load_use_hazard = id_ex_mem_read && (id_ex_rd_addr != 5'd0) &&
                           ((id_ex_rd_addr == id_rs1_addr) ||
                            (id_ex_rd_addr == id_rs2_addr));

    assign pc_stall = load_use_hazard;

    wire id_ex_bubble = load_use_hazard || pc_flush;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            id_ex_pc<=0; id_ex_rs1_data<=0; id_ex_rs2_data<=0; id_ex_imm<=0;
            id_ex_rs1_addr<=0; id_ex_rs2_addr<=0; id_ex_rd_addr<=0; id_ex_funct3<=0;
            id_ex_reg_write<=0; id_ex_mem_read<=0; id_ex_mem_write<=0;
            id_ex_mem_size<=0; id_ex_mem_unsigned<=0; id_ex_alu_ctrl<=ALU_ADD;
            id_ex_alu_src_a_sel<=0; id_ex_alu_src_b_sel<=0; id_ex_result_src<=0;
            id_ex_branch<=0; id_ex_jump<=0; id_ex_jalr<=0;
        end else if (dmem_stall) begin
            // giu nguyen
        end else if (id_ex_bubble) begin
            id_ex_pc <= if_id_pc;
            id_ex_rs1_data<=0; id_ex_rs2_data<=0; id_ex_imm<=0;
            id_ex_rs1_addr<=0; id_ex_rs2_addr<=0; id_ex_rd_addr<=0; id_ex_funct3<=0;
            id_ex_reg_write<=0; id_ex_mem_read<=0; id_ex_mem_write<=0;
            id_ex_mem_size<=0; id_ex_mem_unsigned<=0; id_ex_alu_ctrl<=ALU_ADD;
            id_ex_alu_src_a_sel<=0; id_ex_alu_src_b_sel<=0; id_ex_result_src<=0;
            id_ex_branch<=0; id_ex_jump<=0; id_ex_jalr<=0;
        end else begin
            id_ex_pc            <= if_id_pc;
            id_ex_rs1_data      <= id_rs1_data;
            id_ex_rs2_data      <= id_rs2_data;
            id_ex_imm           <= id_imm;
            id_ex_rs1_addr      <= id_rs1_addr;
            id_ex_rs2_addr      <= id_rs2_addr;
            id_ex_rd_addr       <= id_rd_addr;
            id_ex_funct3        <= if_id_instr[14:12];
            id_ex_reg_write     <= id_reg_write;
            id_ex_mem_read      <= id_mem_read;
            id_ex_mem_write     <= id_mem_write;
            id_ex_mem_size      <= id_mem_size;
            id_ex_mem_unsigned  <= id_mem_unsigned;
            id_ex_alu_ctrl       <= id_alu_ctrl;
            id_ex_alu_src_a_sel <= id_alu_src_a_sel;
            id_ex_alu_src_b_sel <= id_alu_src_b_sel;
            id_ex_result_src    <= id_result_src;
            id_ex_branch        <= id_branch;
            id_ex_jump          <= id_jump;
            id_ex_jalr           <= id_jalr;
        end
    end

    //================================================================
    // EX/MEM (khai bao truoc de dung trong forwarding cua EX)
    //================================================================
    reg [31:0] ex_mem_alu_result;
    reg [31:0] ex_mem_store_data;
    reg [31:0] ex_mem_pc_plus4;
    reg [4:0]  ex_mem_rd_addr;
    reg        ex_mem_reg_write;
    reg        ex_mem_mem_read, ex_mem_mem_write;
    reg [1:0]  ex_mem_mem_size;
    reg        ex_mem_mem_unsigned;
    reg [1:0]  ex_mem_result_src;

    //================================================================
    // EX
    //================================================================
    wire forward_a_from_exmem = ex_mem_reg_write && (ex_mem_rd_addr!=5'd0) &&
                                 (ex_mem_rd_addr == id_ex_rs1_addr);
    wire forward_b_from_exmem = ex_mem_reg_write && (ex_mem_rd_addr!=5'd0) &&
                                 (ex_mem_rd_addr == id_ex_rs2_addr);

    wire forward_a_from_memwb = wb_reg_write && (wb_rd_addr!=5'd0) &&
                                (wb_rd_addr == id_ex_rs1_addr) && !forward_a_from_exmem;
    wire forward_b_from_memwb = wb_reg_write && (wb_rd_addr!=5'd0) &&
                                (wb_rd_addr == id_ex_rs2_addr) && !forward_b_from_exmem;

    wire [31:0] forward_a = forward_a_from_exmem ? ex_mem_alu_result :
                             forward_a_from_memwb ? wb_write_data :
                             id_ex_rs1_data;
    wire [31:0] forward_b = forward_b_from_exmem ? ex_mem_alu_result :
                             forward_b_from_memwb ? wb_write_data :
                             id_ex_rs2_data;

    wire [31:0] alu_operand_a = (id_ex_alu_src_a_sel==2'b01) ? id_ex_pc :
                                 (id_ex_alu_src_a_sel==2'b10) ? 32'h0 :
                                 forward_a;
    wire [31:0] alu_operand_b = id_ex_alu_src_b_sel ? id_ex_imm : forward_b;

    wire [31:0] ex_alu_result;
    wire        ex_alu_zero;

    alu u_alu (
        .a        (alu_operand_a),
        .b        (alu_operand_b),
        .alu_ctrl (id_ex_alu_ctrl),
        .result   (ex_alu_result),
        .zero     (ex_alu_zero)
    );

    reg ex_branch_taken;
    always @(*) begin
        case (id_ex_funct3)
            3'b000:  ex_branch_taken = ex_alu_zero;        // BEQ
            3'b001:  ex_branch_taken = !ex_alu_zero;       // BNE
            3'b100:  ex_branch_taken = ex_alu_result[0];   // BLT
            3'b101:  ex_branch_taken = !ex_alu_result[0];  // BGE
            3'b110:  ex_branch_taken = ex_alu_result[0];   // BLTU
            3'b111:  ex_branch_taken = !ex_alu_result[0];  // BGEU
            default: ex_branch_taken = 1'b0;
        endcase
    end

    wire ex_pc_redirect = (id_ex_branch && ex_branch_taken) || id_ex_jump || id_ex_jalr;

    // QUAN TRONG: voi re nhanh (BEQ/BNE/...), ALU dang dung de SO SANH
    // (tra ve 0/1 hoac ket qua tru), KHONG PHAI dia chi dich - dia chi
    // dich phai tinh rieng = PC + offset. Voi JAL/JALR, ALU moi chinh
    // la dia chi dich (xem control_unit.v).
    wire [31:0] ex_branch_target = id_ex_pc + id_ex_imm;

    assign pc_flush           = ex_pc_redirect;
    assign pc_redirect_target = (id_ex_branch && ex_branch_taken) ? ex_branch_target
                                                                   : ex_alu_result;

    wire [31:0] ex_pc_plus4 = id_ex_pc + 32'd4;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ex_mem_alu_result<=0; ex_mem_store_data<=0; ex_mem_pc_plus4<=0;
            ex_mem_rd_addr<=0; ex_mem_reg_write<=0;
            ex_mem_mem_read<=0; ex_mem_mem_write<=0;
            ex_mem_mem_size<=0; ex_mem_mem_unsigned<=0; ex_mem_result_src<=0;
        end else if (dmem_stall) begin
            // giu nguyen
        end else begin
            ex_mem_alu_result   <= ex_alu_result;
            ex_mem_store_data   <= forward_b;
            ex_mem_pc_plus4     <= ex_pc_plus4;
            ex_mem_rd_addr      <= id_ex_rd_addr;
            ex_mem_reg_write    <= id_ex_reg_write;
            ex_mem_mem_read     <= id_ex_mem_read;
            ex_mem_mem_write    <= id_ex_mem_write;
            ex_mem_mem_size     <= id_ex_mem_size;
            ex_mem_mem_unsigned <= id_ex_mem_unsigned;
            ex_mem_result_src   <= id_ex_result_src;
        end
    end

    //================================================================
    // MEM
    //================================================================
    reg [31:0] mem_wdata_shifted;
    reg [3:0]  mem_wstrb_v;
    always @(*) begin
        case (ex_mem_mem_size)
            2'b00: begin // byte
                case (ex_mem_alu_result[1:0])
                    2'b00: begin mem_wdata_shifted = {24'h0, ex_mem_store_data[7:0]};       mem_wstrb_v=4'b0001; end
                    2'b01: begin mem_wdata_shifted = {16'h0, ex_mem_store_data[7:0], 8'h0};  mem_wstrb_v=4'b0010; end
                    2'b10: begin mem_wdata_shifted = {8'h0,  ex_mem_store_data[7:0], 16'h0}; mem_wstrb_v=4'b0100; end
                    2'b11: begin mem_wdata_shifted = {ex_mem_store_data[7:0], 24'h0};        mem_wstrb_v=4'b1000; end
                endcase
            end
            2'b01: begin // half
                if (ex_mem_alu_result[1] == 1'b0) begin
                    mem_wdata_shifted = {16'h0, ex_mem_store_data[15:0]};
                    mem_wstrb_v = 4'b0011;
                end else begin
                    mem_wdata_shifted = {ex_mem_store_data[15:0], 16'h0};
                    mem_wstrb_v = 4'b1100;
                end
            end
            default: begin
                mem_wdata_shifted = ex_mem_store_data;
                mem_wstrb_v = 4'b1111;
            end
        endcase
    end

    assign dmem_addr     = ex_mem_alu_result;
    assign dmem_wdata    = mem_wdata_shifted;
    assign dmem_wstrb    = mem_wstrb_v;
    assign dmem_we       = ex_mem_mem_write;
    assign dmem_re       = ex_mem_mem_read;
    assign dmem_size     = ex_mem_mem_size;
    assign dmem_unsigned = ex_mem_mem_unsigned;

    reg [31:0] mem_rdata_extended;
    always @(*) begin
        case (ex_mem_mem_size)
            2'b00: begin // byte
                case (ex_mem_alu_result[1:0])
                    2'b00: mem_rdata_extended = ex_mem_mem_unsigned ? {24'h0, dmem_rdata[7:0]}   : {{24{dmem_rdata[7]}},  dmem_rdata[7:0]};
                    2'b01: mem_rdata_extended = ex_mem_mem_unsigned ? {24'h0, dmem_rdata[15:8]}  : {{24{dmem_rdata[15]}}, dmem_rdata[15:8]};
                    2'b10: mem_rdata_extended = ex_mem_mem_unsigned ? {24'h0, dmem_rdata[23:16]} : {{24{dmem_rdata[23]}}, dmem_rdata[23:16]};
                    2'b11: mem_rdata_extended = ex_mem_mem_unsigned ? {24'h0, dmem_rdata[31:24]} : {{24{dmem_rdata[31]}}, dmem_rdata[31:24]};
                endcase
            end
            2'b01: begin // half
                if (ex_mem_alu_result[1] == 1'b0)
                    mem_rdata_extended = ex_mem_mem_unsigned ? {16'h0, dmem_rdata[15:0]}  : {{16{dmem_rdata[15]}}, dmem_rdata[15:0]};
                else
                    mem_rdata_extended = ex_mem_mem_unsigned ? {16'h0, dmem_rdata[31:16]} : {{16{dmem_rdata[31]}}, dmem_rdata[31:16]};
            end
            default: mem_rdata_extended = dmem_rdata;
        endcase
    end

    //================================================================
    // MEM/WB
    //================================================================
    reg [31:0] mem_wb_alu_result, mem_wb_mem_data, mem_wb_pc_plus4;
    reg [4:0]  mem_wb_rd_addr;
    reg        mem_wb_reg_write;
    reg [1:0]  mem_wb_result_src;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem_wb_alu_result<=0; mem_wb_mem_data<=0; mem_wb_pc_plus4<=0;
            mem_wb_rd_addr<=0; mem_wb_reg_write<=0; mem_wb_result_src<=0;
        end else if (dmem_stall) begin
            // MEM stage chua xong, WB chua nhan gia tri moi
        end else begin
            mem_wb_alu_result <= ex_mem_alu_result;
            mem_wb_mem_data   <= mem_rdata_extended;
            mem_wb_pc_plus4   <= ex_mem_pc_plus4;
            mem_wb_rd_addr    <= ex_mem_rd_addr;
            mem_wb_reg_write  <= ex_mem_reg_write;
            mem_wb_result_src <= ex_mem_result_src;
        end
    end

    //================================================================
    // WB
    //================================================================
    assign wb_write_data = (mem_wb_result_src == 2'b01) ? mem_wb_mem_data :
                            (mem_wb_result_src == 2'b10) ? mem_wb_pc_plus4 :
                            mem_wb_alu_result;
    assign wb_reg_write = mem_wb_reg_write;
    assign wb_rd_addr   = mem_wb_rd_addr;

endmodule
