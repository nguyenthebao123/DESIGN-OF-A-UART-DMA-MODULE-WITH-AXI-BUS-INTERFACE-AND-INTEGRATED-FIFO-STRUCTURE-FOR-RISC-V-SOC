//=====================================================================
// imem.v
// ROM chua chuong trinh, doc to hop 1 chu ky (Harvard, rieng voi dmem).
//=====================================================================
module imem #(
    parameter ADDR_WIDTH = 14,     // 2^14 byte = 16KB
    parameter INIT_FILE   = ""
)(
    input  wire        clk,
    input  wire [31:0] addr,
    output wire [31:0] rdata
);
    localparam DEPTH = (1 << (ADDR_WIDTH-2));

    reg [31:0] mem [0:DEPTH-1];

    initial begin
        if (INIT_FILE != "")
            $readmemh(INIT_FILE, mem);
    end

    assign rdata = mem[addr[ADDR_WIDTH-1:2]];
endmodule
