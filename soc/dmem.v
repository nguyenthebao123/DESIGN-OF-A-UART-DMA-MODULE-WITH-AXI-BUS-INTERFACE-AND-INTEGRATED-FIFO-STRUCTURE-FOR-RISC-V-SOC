//=====================================================================
// dmem.v
// RAM du lieu noi bo, doc to hop 1 chu ky / ghi dong bo, co byte-lane
// write strobe (giong quy uoc AXI wstrb da dung trong khoi UART/DMA).
//=====================================================================
module dmem #(
    parameter ADDR_WIDTH = 14    // 2^14 byte = 16KB
)(
    input  wire        clk,
    input  wire [31:0] addr,
    input  wire [31:0] wdata,
    input  wire [3:0]  wstrb,
    input  wire        we,
    output wire [31:0] rdata
);
    localparam DEPTH = (1 << (ADDR_WIDTH-2));

    reg [31:0] mem [0:DEPTH-1];

    wire [ADDR_WIDTH-3:0] word_addr = addr[ADDR_WIDTH-1:2];

    assign rdata = mem[word_addr];

    always @(posedge clk) begin
        if (we) begin
            if (wstrb[0]) mem[word_addr][7:0]   <= wdata[7:0];
            if (wstrb[1]) mem[word_addr][15:8]  <= wdata[15:8];
            if (wstrb[2]) mem[word_addr][23:16] <= wdata[23:16];
            if (wstrb[3]) mem[word_addr][31:24] <= wdata[31:24];
        end
    end
endmodule
