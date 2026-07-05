//=====================================================================
// baud_gen.v
// Sinh xung "tick_x16" voi tan so = 16 x baud_rate, dung de lay mau
// (RX) va dinh thoi (TX). baud_div = f_clk / (16 x baud_rate) - 1.
//=====================================================================
module baud_gen #(
    parameter DIV_WIDTH = 16
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire [DIV_WIDTH-1:0]    baud_div,
    output reg                     tick_x16
);

    reg [DIV_WIDTH-1:0] cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt      <= {DIV_WIDTH{1'b0}};
            tick_x16 <= 1'b0;
        end else if (cnt == baud_div) begin
            cnt      <= {DIV_WIDTH{1'b0}};
            tick_x16 <= 1'b1;
        end else begin
            cnt      <= cnt + 1'b1;
            tick_x16 <= 1'b0;
        end
    end

endmodule
