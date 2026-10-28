// 将帧缓存的mc_app接口转换为YUNTAI_DDR IP使用的PAXI接口，
// 不修改工具生成的DDR控制器源码。
module YUNTAI_DDR_BRIDGE (
    input  wire         I_sys_clk,
    input  wire         I_sys_rst_n,

    output wire         O_ddr_clk,
    output wire         O_init_calib_complete,

    input  wire [24:0]  I_mc_app_addr,
    input  wire [2:0]   I_mc_app_cmd,
    input  wire         I_mc_app_en,
    input  wire [127:0] I_mc_app_wdf_data,
    input  wire         I_mc_app_wdf_end,
    input  wire [15:0]  I_mc_app_wdf_mask,
    input  wire         I_mc_app_wdf_wren,
    output wire [127:0] O_mc_app_rd_data,
    output wire         O_mc_app_rd_data_end,
    output wire         O_mc_app_rd_data_valid,
    output wire         O_mc_app_rdy,
    output wire         O_mc_app_wdf_rdy,

    output wire [12:0] ddr_addr,
    output wire [1:0]  ddr_ba,
    output wire        ddr_cke,
    output wire        ddr_odt,
    output wire        ddr_cs_n,
    output wire        ddr_ras_n,
    output wire        ddr_cas_n,
    output wire        ddr_we_n,
    output wire        ddr_ck_p,
    output wire        ddr_ck_n,
    inout  wire [1:0]  ddr_dm,
    inout  wire [15:0] ddr_dq,
    inout  wire [1:0]  ddr_dqs_p,
    inout  wire [1:0]  ddr_dqs_n
);

    wire        S_ddrphy_rdy;
    wire [24:0] S_paxi_awaddr;
    wire        S_paxi_awvalid;
    wire        S_paxi_awready;
    wire [127:0] S_paxi_wdata;
    wire [15:0] S_paxi_wstrb;
    wire        S_paxi_wvalid;
    wire        S_paxi_wlast;
    wire        S_paxi_wready;
    wire [24:0] S_paxi_araddr;
    wire        S_paxi_arvalid;
    wire        S_paxi_arready;
    wire [127:0] S_paxi_rdata;
    wire        S_paxi_rlast;
    wire        S_paxi_rvalid;

    // 命令0表示写，命令最低位为1表示读，与mc_to_user_interface约定一致。
    assign S_paxi_awaddr  = I_mc_app_cmd[0] ? 25'd0 : I_mc_app_addr;
    assign S_paxi_awvalid = I_mc_app_cmd[0] ? 1'b0  : I_mc_app_en;
    assign S_paxi_wdata   = I_mc_app_wdf_data;
    assign S_paxi_wstrb   = I_mc_app_wdf_mask;
    assign S_paxi_wvalid  = I_mc_app_wdf_wren;
    assign S_paxi_wlast   = I_mc_app_wdf_end;
    assign S_paxi_araddr  = I_mc_app_cmd[0] ? I_mc_app_addr : 25'd0;
    assign S_paxi_arvalid = I_mc_app_cmd[0] ? I_mc_app_en   : 1'b0;

    assign O_ddr_clk             = S_dhi_clk;
    assign O_init_calib_complete = S_ddrphy_rdy;
    assign O_mc_app_rd_data      = S_paxi_rdata;
    assign O_mc_app_rd_data_end  = S_paxi_rlast;
    assign O_mc_app_rd_data_valid = S_paxi_rvalid;

    // 帧缓存命令和写数据同拍发出；DDR校准完成前禁止所有用户请求。
    assign O_mc_app_rdy     = S_ddrphy_rdy && (S_paxi_arready || S_paxi_awready);
    assign O_mc_app_wdf_rdy = S_ddrphy_rdy && S_paxi_awready;

    wire S_dhi_clk;

    YUNTAI_DDR U_DDR (
        .ref_clk      (I_sys_clk),
        .sys_rst_n    (I_sys_rst_n),
        .apb_pclk     (I_sys_clk),
        .apb_prst_n   (I_sys_rst_n),
        .apb_pready   (),
        .apb_psel     (1'b0),
        .apb_penable  (1'b0),
        .apb_pwrite   (1'b0),
        .apb_paddr    (16'd0),
        .apb_pwdata   (32'd0),
        .apb_prdata   (),
        .dhi_clk      (S_dhi_clk),
        .cfg_clk      (),
        .cfg_rst_n    (),
        .ddrphy_rst_n (),
        .ddrphy_rdy   (S_ddrphy_rdy),
        .paxi_awaddr  (S_paxi_awaddr),
        .paxi_awvalid (S_paxi_awvalid),
        .paxi_awready (S_paxi_awready),
        .paxi_wdata   (S_paxi_wdata),
        .paxi_wstrb   (S_paxi_wstrb),
        .paxi_wvalid  (S_paxi_wvalid),
        .paxi_wlast   (S_paxi_wlast),
        .paxi_wready  (S_paxi_wready),
        .paxi_araddr  (S_paxi_araddr),
        .paxi_arvalid (S_paxi_arvalid),
        .paxi_arready (S_paxi_arready),
        .paxi_rdata   (S_paxi_rdata),
        .paxi_rlast   (S_paxi_rlast),
        .paxi_rvalid  (S_paxi_rvalid),
        .paxi_rready  (1'b1),
        .ddr_ck_p     (ddr_ck_p),
        .ddr_ck_n     (ddr_ck_n),
        .ddr_cke      (ddr_cke),
        .ddr_odt      (ddr_odt),
        .ddr_cs_n     (ddr_cs_n),
        .ddr_ras_n    (ddr_ras_n),
        .ddr_cas_n    (ddr_cas_n),
        .ddr_we_n     (ddr_we_n),
        .ddr_ba       (ddr_ba),
        .ddr_addr     (ddr_addr),
        .ddr_dqs_p    (ddr_dqs_p),
        .ddr_dqs_n    (ddr_dqs_n),
        .ddr_dq       (ddr_dq),
        .ddr_dm       (ddr_dm)
    );

endmodule
