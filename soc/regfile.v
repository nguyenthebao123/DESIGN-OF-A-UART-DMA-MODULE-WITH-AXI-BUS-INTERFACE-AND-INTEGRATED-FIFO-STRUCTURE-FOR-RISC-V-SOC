//=====================================================================
// regfile.v
// Bo 32 thanh ghi 32-bit, x0 luon = 0. Doc 2 cong to hop, ghi 1 cong
// dong bo. Co "bypass" cho truong hop ghi va doc CUNG 1 thanh ghi
// trong CUNG 1 chu ky (WB ghi, ID doc) - neu khong co bypass nay se
// doc duoc gia tri CU.
//=====================================================================
module regfile (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        we,
    input  wire [4:0]  rd_addr,
    input  wire [31:0] rd_data,
    input  wire [4:0]  rs1_addr,
    input  wire [4:0]  rs2_addr,
    output wire [31:0] rs1_data,
    output wire [31:0] rs2_data
);
    reg [31:0] regs [1:31];
    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 1; i < 32; i = i + 1) regs[i] <= 32'h0;
        end else if (we && rd_addr != 5'd0) begin
            regs[rd_addr] <= rd_data;
        end
    end

    assign rs1_data = (rs1_addr == 5'd0) ? 32'h0 :
                       (we && (rd_addr == rs1_addr)) ? rd_data :
                       regs[rs1_addr];

    assign rs2_data = (rs2_addr == 5'd0) ? 32'h0 :
                       (we && (rd_addr == rs2_addr)) ? rd_data :
                       regs[rs2_addr];
endmodule
