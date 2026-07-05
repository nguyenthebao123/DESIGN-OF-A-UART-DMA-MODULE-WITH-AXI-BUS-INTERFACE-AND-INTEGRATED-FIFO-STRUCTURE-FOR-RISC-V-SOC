//=====================================================================
// imm_gen.v
// Tao gia tri immediate da sign-extend, theo dung dinh dang RV32I
// (I/S/B/U/J-type).
//=====================================================================
module imm_gen (
    input  wire [31:0] instr,
    output reg  [31:0] imm
);
    wire [6:0] opcode = instr[6:0];

    always @(*) begin
        case (opcode)
            7'b0010011, // I-type ALU
            7'b0000011, // Load
            7'b1100111: // JALR
                imm = {{20{instr[31]}}, instr[31:20]};

            7'b0100011: // Store (S-type)
                imm = {{20{instr[31]}}, instr[31:25], instr[11:7]};

            7'b1100011: // Branch (B-type)
                imm = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};

            7'b0110111, // LUI
            7'b0010111: // AUIPC (U-type)
                imm = {instr[31:12], 12'b0};

            7'b1101111: // JAL (J-type)
                imm = {{11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};

            default:
                imm = 32'h0;
        endcase
    end
endmodule
