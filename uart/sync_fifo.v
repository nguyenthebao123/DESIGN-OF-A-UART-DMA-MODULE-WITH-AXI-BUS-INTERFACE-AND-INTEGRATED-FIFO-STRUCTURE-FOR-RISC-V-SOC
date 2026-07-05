//=====================================================================
// sync_fifo.v
// FIFO dong bo, tham so hoa, dung cho bo dem TX/RX cua UART.
// DEPTH phai la luy thua cua 2 (vi dung ky thuat con tro them 1 bit
// de phan biet "day"/"rong" ma khong can them logic rieng).
//=====================================================================
module sync_fifo #(
    parameter DATA_WIDTH = 8,
    parameter DEPTH      = 16,
    parameter ADDR_WIDTH = 4              // = log2(DEPTH), DEPTH=16 -> 4
)(
    input  wire                   clk,
    input  wire                   rst_n,
    input  wire                   clear,       // xoa FIFO dong bo (mem khong bi xoa, chi con tro)

    // Cong ghi
    input  wire                   wr_en,
    input  wire [DATA_WIDTH-1:0]  wr_data,
    output wire                   full,

    // Cong doc (du lieu ra to hop, hop le ngay khi !empty)
    input  wire                   rd_en,
    output wire [DATA_WIDTH-1:0]  rd_data,
    output wire                   empty,

    // Trang thai
    output wire [ADDR_WIDTH:0]    count,
    output wire                   almost_full,
    output wire                   almost_empty
);

    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    reg [ADDR_WIDTH:0]   wr_ptr;
    reg [ADDR_WIDTH:0]   rd_ptr;

    wire [ADDR_WIDTH-1:0] wr_addr = wr_ptr[ADDR_WIDTH-1:0];
    wire [ADDR_WIDTH-1:0] rd_addr = rd_ptr[ADDR_WIDTH-1:0];

    assign empty = (wr_ptr == rd_ptr);
    assign full  = (wr_ptr[ADDR_WIDTH] != rd_ptr[ADDR_WIDTH]) &&
                   (wr_ptr[ADDR_WIDTH-1:0] == rd_ptr[ADDR_WIDTH-1:0]);

    assign count        = wr_ptr - rd_ptr;
    assign almost_full   = (count >= (DEPTH - 1));
    assign almost_empty  = (count <= 1);

    assign rd_data = mem[rd_addr];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || clear) begin
            wr_ptr <= {(ADDR_WIDTH+1){1'b0}};
        end else if (wr_en && !full) begin
            mem[wr_addr] <= wr_data;
            wr_ptr       <= wr_ptr + 1'b1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || clear) begin
            rd_ptr <= {(ADDR_WIDTH+1){1'b0}};
        end else if (rd_en && !empty) begin
            rd_ptr <= rd_ptr + 1'b1;
        end
    end

endmodule
