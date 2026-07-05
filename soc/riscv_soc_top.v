//=====================================================================
// riscv_soc_top.v
// SoC RISC-V toi gian: loi CPU 5 tang + ROM lenh + RAM du lieu + cau
// noi AXI4-Lite master - du de chay 1 chuong trinh dieu khien khoi
// UART (uart_axi_top.v) qua cong AXI4-Lite chuan.
//=====================================================================
module riscv_soc_top #(
    parameter S_ADDR_WIDTH = 32,
    parameter IMEM_FILE    = ""
)(
    input  wire clk,
    input  wire rst_n,

    output wire [S_ADDR_WIDTH-1:0] m_axi_awaddr,
    output wire                    m_axi_awvalid,
    input  wire                    m_axi_awready,
    output wire [31:0]             m_axi_wdata,
    output wire [3:0]              m_axi_wstrb,
    output wire                    m_axi_wvalid,
    input  wire                    m_axi_wready,
    input  wire [1:0]              m_axi_bresp,
    input  wire                    m_axi_bvalid,
    output wire                    m_axi_bready,

    output wire [S_ADDR_WIDTH-1:0] m_axi_araddr,
    output wire                    m_axi_arvalid,
    input  wire                    m_axi_arready,
    input  wire [31:0]             m_axi_rdata,
    input  wire [1:0]              m_axi_rresp,
    input  wire                    m_axi_rvalid,
    output wire                    m_axi_rready
);

    wire [31:0] imem_addr, imem_rdata;
    wire [31:0] dmem_addr, dmem_wdata, dmem_rdata_cpu;
    wire [3:0]  dmem_wstrb;
    wire        dmem_we, dmem_re, dmem_stall;
    wire [1:0]  dmem_size;
    wire        dmem_unsigned;

    riscv_core #(.RESET_PC(32'h0)) u_core (
        .clk           (clk),
        .rst_n         (rst_n),
        .imem_addr     (imem_addr),
        .imem_rdata    (imem_rdata),
        .dmem_addr     (dmem_addr),
        .dmem_wdata    (dmem_wdata),
        .dmem_wstrb    (dmem_wstrb),
        .dmem_we       (dmem_we),
        .dmem_re       (dmem_re),
        .dmem_size     (dmem_size),
        .dmem_unsigned (dmem_unsigned),
        .dmem_rdata    (dmem_rdata_cpu),
        .dmem_stall    (dmem_stall)
    );

    imem #(.ADDR_WIDTH(14), .INIT_FILE(IMEM_FILE)) u_imem (
        .clk   (clk),
        .addr  (imem_addr),
        .rdata (imem_rdata)
    );

    wire [31:0] ram_addr, ram_wdata, ram_rdata;
    wire [3:0]  ram_wstrb;
    wire        ram_we;

    dmem #(.ADDR_WIDTH(14)) u_dmem (
        .clk   (clk),
        .addr  (ram_addr),
        .wdata (ram_wdata),
        .wstrb (ram_wstrb),
        .we    (ram_we),
        .rdata (ram_rdata)
    );

    axi4lite_master_bridge #(.S_ADDR_WIDTH(S_ADDR_WIDTH)) u_bridge (
        .clk       (clk),
        .rst_n     (rst_n),
        .cpu_addr  (dmem_addr),
        .cpu_wdata (dmem_wdata),
        .cpu_wstrb (dmem_wstrb),
        .cpu_we    (dmem_we),
        .cpu_re    (dmem_re),
        .cpu_rdata (dmem_rdata_cpu),
        .cpu_stall (dmem_stall),

        .ram_addr  (ram_addr),
        .ram_wdata (ram_wdata),
        .ram_wstrb (ram_wstrb),
        .ram_we    (ram_we),
        .ram_rdata (ram_rdata),

        .m_axi_awaddr  (m_axi_awaddr),
        .m_axi_awvalid (m_axi_awvalid),
        .m_axi_awready (m_axi_awready),
        .m_axi_wdata   (m_axi_wdata),
        .m_axi_wstrb   (m_axi_wstrb),
        .m_axi_wvalid  (m_axi_wvalid),
        .m_axi_wready  (m_axi_wready),
        .m_axi_bresp   (m_axi_bresp),
        .m_axi_bvalid  (m_axi_bvalid),
        .m_axi_bready  (m_axi_bready),

        .m_axi_araddr  (m_axi_araddr),
        .m_axi_arvalid (m_axi_arvalid),
        .m_axi_arready (m_axi_arready),
        .m_axi_rdata   (m_axi_rdata),
        .m_axi_rresp   (m_axi_rresp),
        .m_axi_rvalid  (m_axi_rvalid),
        .m_axi_rready  (m_axi_rready)
    );

endmodule
