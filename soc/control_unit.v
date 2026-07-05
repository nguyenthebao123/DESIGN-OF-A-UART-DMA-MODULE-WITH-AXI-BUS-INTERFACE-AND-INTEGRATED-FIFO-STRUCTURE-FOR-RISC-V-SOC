//=====================================================================
// control_unit.v
// Bo giai ma lenh chinh - day du RV32I (tru FENCE/ECALL/EBREAK, xu ly
// nhu NOP).
//=====================================================================
module control_unit (
    input  wire [31:0] instr,
    output reg          reg_write,
    output reg          mem_read,
    output reg          mem_write,
    output reg  [1:0]   mem_size,       // 00=byte 01=half 10=word
    output reg          mem_unsigned,
    output reg  [3:0]   alu_ctrl,
    output reg  [1:0]   alu_src_a_sel,  // 00=rs1 01=pc 10=zero
    output reg           alu_src_b_sel,  // 0=rs2  1=imm
    output reg  [1:0]   result_src,     // 00=alu 01=mem 10=pc+4
    output reg           branch,
    output reg           jump,           // JAL
    output reg           jalr
);
    wire [6:0] opcode   = instr[6:0];
    wire [2:0] funct3   = instr[14:12];
    wire       funct7b5 = instr[30];

    localparam ALU_ADD=4'd0, ALU_SUB=4'd1, ALU_SLL=4'd2, ALU_SLT=4'd3,
               ALU_SLTU=4'd4, ALU_XOR=4'd5, ALU_SRL=4'd6, ALU_SRA=4'd7,
               ALU_OR=4'd8, ALU_AND=4'd9;

    localparam OP_RTYPE  = 7'b0110011;
    localparam OP_ITYPE  = 7'b0010011;
    localparam OP_LOAD   = 7'b0000011;
    localparam OP_STORE  = 7'b0100011;
    localparam OP_BRANCH = 7'b1100011;
    localparam OP_JAL    = 7'b1101111;
    localparam OP_JALR   = 7'b1100111;
    localparam OP_LUI    = 7'b0110111;
    localparam OP_AUIPC  = 7'b0010111;

    always @(*) begin
        reg_write     = 1'b0;
        mem_read      = 1'b0;
        mem_write     = 1'b0;
        mem_size      = 2'b10;
        mem_unsigned  = 1'b0;
        alu_ctrl      = ALU_ADD;
        alu_src_a_sel = 2'b00;
        alu_src_b_sel = 1'b0;
        result_src    = 2'b00;
        branch        = 1'b0;
        jump          = 1'b0;
        jalr          = 1'b0;

        case (opcode)
            OP_RTYPE: begin
                reg_write = 1'b1;
                case (funct3)
                    3'b000: alu_ctrl = funct7b5 ? ALU_SUB : ALU_ADD;
                    3'b001: alu_ctrl = ALU_SLL;
                    3'b010: alu_ctrl = ALU_SLT;
                    3'b011: alu_ctrl = ALU_SLTU;
                    3'b100: alu_ctrl = ALU_XOR;
                    3'b101: alu_ctrl = funct7b5 ? ALU_SRA : ALU_SRL;
                    3'b110: alu_ctrl = ALU_OR;
                    3'b111: alu_ctrl = ALU_AND;
                endcase
            end

            OP_ITYPE: begin
                reg_write     = 1'b1;
                alu_src_b_sel = 1'b1;
                case (funct3)
                    3'b000: alu_ctrl = ALU_ADD;
                    3'b001: alu_ctrl = ALU_SLL;
                    3'b010: alu_ctrl = ALU_SLT;
                    3'b011: alu_ctrl = ALU_SLTU;
                    3'b100: alu_ctrl = ALU_XOR;
                    3'b101: alu_ctrl = funct7b5 ? ALU_SRA : ALU_SRL;
                    3'b110: alu_ctrl = ALU_OR;
                    3'b111: alu_ctrl = ALU_AND;
                endcase
            end

            OP_LOAD: begin
                reg_write     = 1'b1;
                mem_read      = 1'b1;
                alu_src_b_sel = 1'b1;
                alu_ctrl      = ALU_ADD;
                result_src    = 2'b01;
                case (funct3)
                    3'b000: begin mem_size = 2'b00; mem_unsigned = 1'b0; end
                    3'b001: begin mem_size = 2'b01; mem_unsigned = 1'b0; end
                    3'b010: begin mem_size = 2'b10; mem_unsigned = 1'b0; end
                    3'b100: begin mem_size = 2'b00; mem_unsigned = 1'b1; end
                    3'b101: begin mem_size = 2'b01; mem_unsigned = 1'b1; end
                    default: begin mem_size = 2'b10; mem_unsigned = 1'b0; end
                endcase
            end

            OP_STORE: begin
                mem_write     = 1'b1;
                alu_src_b_sel = 1'b1;
                alu_ctrl      = ALU_ADD;
                case (funct3)
                    3'b000: mem_size = 2'b00;
                    3'b001: mem_size = 2'b01;
                    default: mem_size = 2'b10;
                endcase
            end

            OP_BRANCH: begin
                branch = 1'b1;
                case (funct3)
                    3'b000, 3'b001: alu_ctrl = ALU_SUB;
                    3'b100, 3'b101: alu_ctrl = ALU_SLT;
                    3'b110, 3'b111: alu_ctrl = ALU_SLTU;
                    default:        alu_ctrl = ALU_SUB;
                endcase
            end

            OP_JAL: begin
                reg_write     = 1'b1;
                jump          = 1'b1;
                alu_src_a_sel = 2'b01;   // pc
                alu_src_b_sel = 1'b1;    // imm  -> alu_result = pc+imm = dich
                alu_ctrl      = ALU_ADD;
                result_src    = 2'b10;   // rd = pc+4
            end

            OP_JALR: begin
                reg_write     = 1'b1;
                jalr          = 1'b1;
                alu_src_a_sel = 2'b00;   // rs1
                alu_src_b_sel = 1'b1;    // imm -> alu_result = rs1+imm = dich
                alu_ctrl      = ALU_ADD;
                result_src    = 2'b10;   // rd = pc+4
            end

            OP_LUI: begin
                reg_write     = 1'b1;
                alu_src_a_sel = 2'b10;   // zero
                alu_src_b_sel = 1'b1;    // imm -> alu_result = imm
                alu_ctrl      = ALU_ADD;
            end

            OP_AUIPC: begin
                reg_write     = 1'b1;
                alu_src_a_sel = 2'b01;   // pc
                alu_src_b_sel = 1'b1;    // imm -> alu_result = pc+imm
                alu_ctrl      = ALU_ADD;
            end

            default: begin
                // FENCE, SYSTEM (ECALL/EBREAK), ma lenh khac -> NOP
            end
        endcase
    end
endmodule
