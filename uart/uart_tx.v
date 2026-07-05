//=====================================================================
// uart_tx.v
// Loi phat UART, khung 8-N-1 (1 bit start, 8 bit data LSB truoc,
// 1 bit stop). Moi bit keo dai 16 xung tick_x16.
//=====================================================================
module uart_tx (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        tick_x16,

    input  wire        tx_valid,   // co du lieu cho o ngoai (vd: !fifo_empty)
    input  wire [7:0]  tx_data,    // du lieu hien tai dau FIFO (to hop)
    output wire         tx_ready,   // '1' khi loi dang IDLE, san sang nhan byte moi

    output reg          tx_busy,
    output reg          tx          // duong truyen noi tiep ra ngoai
);

    localparam S_IDLE  = 2'd0,
               S_START = 2'd1,
               S_DATA  = 2'd2,
               S_STOP  = 2'd3;

    reg [1:0] state;
    reg [3:0] tick_cnt;
    reg [2:0] bit_idx;
    reg [7:0] shift_reg;

    assign tx_ready = (state == S_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            tick_cnt  <= 4'h0;
            bit_idx   <= 3'h0;
            shift_reg <= 8'h00;
            tx        <= 1'b1;
            tx_busy   <= 1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    tx      <= 1'b1;
                    tx_busy <= 1'b0;
                    if (tx_valid) begin
                        shift_reg <= tx_data;
                        tx_busy   <= 1'b1;
                        tick_cnt  <= 4'h0;
                        state     <= S_START;
                    end
                end

                S_START: begin
                    tx <= 1'b0;
                    if (tick_x16) begin
                        if (tick_cnt == 4'd15) begin
                            tick_cnt <= 4'h0;
                            bit_idx  <= 3'h0;
                            state    <= S_DATA;
                        end else
                            tick_cnt <= tick_cnt + 1'b1;
                    end
                end

                S_DATA: begin
                    tx <= shift_reg[0];
                    if (tick_x16) begin
                        if (tick_cnt == 4'd15) begin
                            tick_cnt  <= 4'h0;
                            shift_reg <= shift_reg >> 1;
                            if (bit_idx == 3'd7)
                                state <= S_STOP;
                            else
                                bit_idx <= bit_idx + 1'b1;
                        end else
                            tick_cnt <= tick_cnt + 1'b1;
                    end
                end

                S_STOP: begin
                    tx <= 1'b1;
                    if (tick_x16) begin
                        if (tick_cnt == 4'd15) begin
                            tick_cnt <= 4'h0;
                            state    <= S_IDLE;
                        end else
                            tick_cnt <= tick_cnt + 1'b1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
