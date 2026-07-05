//=====================================================================
// uart_rx.v
// Loi nhan UART, lay mau giua bit (oversampling 16x) de chong nhieu.
// Co dong bo hoa 2 tang cho tin hieu rx (chong metastable).
//=====================================================================
module uart_rx (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        tick_x16,

    input  wire        rx,             // duong truyen noi tiep vao

    output reg          rx_valid,       // xung 1 chu ky khi co 1 byte moi
    output reg  [7:0]   rx_data,
    output reg          rx_frame_err    // bit stop khong dung muc '1'
);

    localparam S_IDLE  = 2'd0,
               S_START = 2'd1,
               S_DATA  = 2'd2,
               S_STOP  = 2'd3;

    reg [1:0] state;
    reg [3:0] tick_cnt;
    reg [2:0] bit_idx;
    reg [7:0] shift_reg;
    reg       rx_sync0, rx_sync1;

    // Dong bo hoa ngo vao rx (tin hieu khong dong bo voi clk)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync0 <= 1'b1;
            rx_sync1 <= 1'b1;
        end else begin
            rx_sync0 <= rx;
            rx_sync1 <= rx_sync0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            tick_cnt     <= 4'h0;
            bit_idx      <= 3'h0;
            shift_reg    <= 8'h00;
            rx_valid     <= 1'b0;
            rx_data      <= 8'h00;
            rx_frame_err <= 1'b0;
        end else begin
            rx_valid <= 1'b0; // mac dinh, chi len '1' dung 1 chu ky khi xong byte

            case (state)
                S_IDLE: begin
                    if (!rx_sync1) begin           // phat hien suon xuong (start bit)
                        state    <= S_START;
                        tick_cnt <= 4'h0;
                    end
                end

                S_START: begin
                    if (tick_x16) begin
                        if (tick_cnt == 4'd7) begin // giua bit start (8/16 tick)
                            if (!rx_sync1) begin
                                tick_cnt <= 4'h0;
                                bit_idx  <= 3'h0;
                                state    <= S_DATA;
                            end else begin
                                state <= S_IDLE;     // nhieu/glitch, khong phai start thuc
                            end
                        end else
                            tick_cnt <= tick_cnt + 1'b1;
                    end
                end

                S_DATA: begin
                    if (tick_x16) begin
                        if (tick_cnt == 4'd15) begin   // giua bit du lieu hien tai
                            tick_cnt           <= 4'h0;
                            shift_reg[bit_idx] <= rx_sync1;
                            if (bit_idx == 3'd7)
                                state <= S_STOP;
                            else
                                bit_idx <= bit_idx + 1'b1;
                        end else
                            tick_cnt <= tick_cnt + 1'b1;
                    end
                end

                S_STOP: begin
                    if (tick_x16) begin
                        if (tick_cnt == 4'd15) begin   // giua bit stop
                            tick_cnt     <= 4'h0;
                            rx_data      <= shift_reg;
                            rx_valid     <= 1'b1;
                            rx_frame_err <= !rx_sync1;
                            state        <= S_IDLE;
                        end else
                            tick_cnt <= tick_cnt + 1'b1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
