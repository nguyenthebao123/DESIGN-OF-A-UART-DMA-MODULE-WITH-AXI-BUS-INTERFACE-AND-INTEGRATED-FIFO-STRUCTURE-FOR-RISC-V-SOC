`timescale 1ps / 1ps
//=====================================================================
// tb_soc_top.v
// Testbench cho soc_top.v (noi truc tiep, khong Block Design).
// Port giong design_1_wrapper.v cu nen co the dung lai waveform
// da co san trong Vivado.
//=====================================================================
module tb_soc_top;

    // ----------------------------------------------------------------
    // Tham so
    // ----------------------------------------------------------------
    parameter SYS_CLK_PERIOD = 10000;   // 10 ns = 100 MHz (don vi ps)
    parameter UART_BAUD      = 9600;
    time      BIT_PERIOD;               // 64-bit, tranh tran so

    // ----------------------------------------------------------------
    // Tin hieu
    // ----------------------------------------------------------------
    reg  reset_rtl      = 1;
    reg  sys_clock      = 0;
    reg  uart_rx_0      = 1;   // idle
    wire uart_tx_0;

    // ----------------------------------------------------------------
    // DUT
    // ----------------------------------------------------------------
    soc_top #(
        .IMEM_FILE        ("soc_test.hex"),
        .BAUD_DIV_DEFAULT (32'd651),
        .S_ADDR_WIDTH     (32)
    ) dut (
        .sys_clock  (sys_clock),
        .reset_rtl  (reset_rtl),
        .uart_rx_0  (uart_rx_0),
        .uart_tx_0  (uart_tx_0)
    );

    // ----------------------------------------------------------------
    // Clock
    // ----------------------------------------------------------------
    always #(SYS_CLK_PERIOD/2) sys_clock = ~sys_clock;

    // ----------------------------------------------------------------
    // Bo dem nhan TX (chay doc lap, bat ki khi uart_tx_0 xuong 0)
    // ----------------------------------------------------------------
    reg [7:0] rx_queue [0:31];
    reg [31:0] rx_count = 0;
    reg [7:0]  rx_last  = 8'hXX;

    task wait_rx;
        input integer idx;
        input time    timeout_ps;
        time waited;
        begin
            waited = 0;
            while (rx_count <= idx && waited < timeout_ps) begin
                #100000;
                waited = waited + 100000;
            end
        end
    endtask

    always @(negedge uart_tx_0) begin : mon_tx
        integer i;
        reg [7:0] b;
        time hb, fb;
        hb = BIT_PERIOD / 2;
        fb = BIT_PERIOD;
        #(hb);                          // giua bit start
        b = 8'h00;
        for (i = 0; i < 8; i = i+1) begin
            #(fb);
            b[i] = uart_tx_0;
        end
        #(fb);                          // stop bit
        rx_queue[rx_count[4:0]] = b;
        rx_last  = b;
        $display("[%0t ps] UART_TX -> 0x%02X ('%0s')", $time, b,
                 (b >= 8'h20 && b < 8'h7F) ? b : 8'h3F);
        rx_count = rx_count + 1;
    end

    // ----------------------------------------------------------------
    // Task gui byte vao UART RX
    // ----------------------------------------------------------------
    task send_byte;
        input [7:0] data;
        integer i;
        begin
            uart_rx_0 = 0; #(BIT_PERIOD);
            for (i = 0; i < 8; i = i+1) begin
                uart_rx_0 = data[i]; #(BIT_PERIOD);
            end
            uart_rx_0 = 1; #(BIT_PERIOD);
            $display("[%0t ps] TB -> UART_RX: 0x%02X ('%0s')",
                     $time, data,
                     (data >= 8'h20 && data < 8'h7F) ? data : 8'h3F);
        end
    endtask

    // ----------------------------------------------------------------
    // VCD
    // ----------------------------------------------------------------
    initial begin
        $dumpfile("wave.vcd");
        $dumpvars(0, tb_soc_top);
    end

    // ----------------------------------------------------------------
    // Trinh tu test chinh
    // ----------------------------------------------------------------
    initial begin
        BIT_PERIOD = 64'd1_000_000_000_000 / UART_BAUD;

        $display("=========================================");
        $display(" TB soc_top  BIT_PERIOD=%0d ps baud=%0d",
                 BIT_PERIOD, UART_BAUD);
        $display("=========================================");

        // Watchdog 20 ms
        fork
            begin
                #(20_000_000_000);
                $display("[FAIL] Watchdog timeout!");
                $finish;
            end
            begin
                // Reset 200 ns
                reset_rtl = 1;
                #200000;
                reset_rtl = 0;
                $display("[%0t ps] Reset released", $time);

                // Doi firmware khoi dong (CPU ghi BAUD_DIV ~ 1.5 ms)
                #(BIT_PERIOD * 5);

                // ---- TEST 1: Echo mot byte ----
                $display("--- TEST 1: gui 0x41 (A) ---");
                send_byte(8'h41);
                wait_rx(0, BIT_PERIOD * 30);
                if (rx_count > 0) begin
                    if (rx_queue[0] == 8'h41)
                        $display("[PASS] Echo dung: 0x41");
                    else
                        $display("[FAIL] Echo sai: nhan 0x%02X", rx_queue[0]);
                end else
                    $display("[TIMEOUT] Khong nhan duoc byte echo");

                // ---- TEST 2: Chuoi HELLO ----
                $display("--- TEST 2: gui HELLO ---");
                send_byte(8'h48); // H
                send_byte(8'h45); // E
                send_byte(8'h4C); // L
                send_byte(8'h4C); // L
                send_byte(8'h4F); // O

                wait_rx(5, BIT_PERIOD * 30 * 5);
                begin : chk
                    reg [7:0] exp [0:5];
                    integer k;
                    exp[0]=8'h41; exp[1]=8'h48; exp[2]=8'h45;
                    exp[3]=8'h4C; exp[4]=8'h4C; exp[5]=8'h4F;
                    for (k = 1; k <= 5; k = k+1) begin
                        if (k < rx_count) begin
                            if (rx_queue[k[4:0]] == exp[k])
                                $display("[PASS] byte[%0d]=0x%02X", k, exp[k]);
                            else
                                $display("[FAIL] byte[%0d] mong 0x%02X nhan 0x%02X",
                                         k, exp[k], rx_queue[k[4:0]]);
                        end else
                            $display("[TIMEOUT] chua nhan byte %0d", k);
                    end
                end

                $display("=========================================");
                $display(" XONG tai %0t ps", $time);
                $display("=========================================");
                $finish;
            end
        join
    end

endmodule
