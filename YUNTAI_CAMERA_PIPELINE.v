// SC520CS接收流水线：MIPI CSI-2 RAW10经过D-PHY、解包和ISP，
// 转换为帧缓存使用的RGB888/128位数据流。
module YUNTAI_CAMERA_PIPELINE (
    input  wire        I_lp_clk,
    input  wire        I_rst_n,

    inout  wire        IO_rx_clk_pad_n,
    inout  wire        IO_rx_clk_pad_p,
    inout  wire [3:0]  IO_rx_data_pad_n,
    inout  wire [3:0]  IO_rx_data_pad_p,

    output wire        O_video_clk,
    output wire        O_frame_start,
    output wire        O_valid,
    output wire [127:0] O_data,
    output wire        O_lane_error
);

    wire        S_hs_valid;
    wire [15:0] S_hs_data;
    wire [1:0]  S_lane_error;

    wire        S_csi_frame_start;
    wire        S_csi_frame_end;
    wire        S_csi_valid;
    wire [31:0] S_csi_data;

    wire        S_raw10_frame_start;
    wire        S_raw10_frame_end;
    wire        S_raw10_valid;
    wire [39:0] S_raw10_data;

    wire        S_axis_valid;
    wire [39:0] S_axis_data;
    wire        S_axis_user;
    wire        S_axis_last;

    wire        S_isp_valid;
    wire [127:0] S_isp_data;
    wire        S_isp_user;
    wire        S_isp_last;

    // SC520CS使用1280x720 RAW10双通道，DPHY0对应开发板摄像头专用接口。
    mipi_dphy_rx_ph1p_mipiio_wrapper #(
        .DPHY_RX_LOCATION ("DPHY0"),
        .HS_EQUALIZER     ("3dB"),
        .HS_VGA_GAIN      ("8dB"),
        .LANE_NUM         (2),
        .BYTE_NUM         (1)
    ) U_DPHY_RX (
        .I_lp_clk              (I_lp_clk),
        .I_rst                 (~I_rst_n),
        .I_clk_lane_in_delay   (6'd0),
        .I_data_lane0_in_delay (6'd0),
        .I_data_lane1_in_delay (6'd0),
        .I_data_lane2_in_delay (6'd0),
        .I_data_lane3_in_delay (6'd0),
        .I_lane_invert         (2'b00),
        .O_hs_rx_clk           (O_video_clk),
        .O_hs_rx_valid         (S_hs_valid),
        .O_hs_rx_data          (S_hs_data),
        .O_lp_rx_lane0_p       (),
        .O_lp_rx_lane0_n       (),
        .I_lp_tx_en            (1'b0),
        .I_lp_tx_lane0_p       (1'b1),
        .I_lp_tx_lane0_n       (1'b1),
        .O_lane_match_error    (),
        .O_lane_error          (S_lane_error),
        .IO_rx_clk_pad_n       (IO_rx_clk_pad_n),
        .IO_rx_clk_pad_p       (IO_rx_clk_pad_p),
        .IO_rx_data_pad_n      (IO_rx_data_pad_n),
        .IO_rx_data_pad_p      (IO_rx_data_pad_p)
    );

    csi_unpacket_2lane U_CSI_UNPACK (
        .I_clk             (O_video_clk),
        .I_rst_n           (I_rst_n),
        .I_hs_valid        (S_hs_valid),
        .I_hs_data         (S_hs_data),
        .O_csi_frame_start (S_csi_frame_start),
        .O_csi_frame_end   (S_csi_frame_end),
        .O_csi_valid       (S_csi_valid),
        .O_csi_hs          (),
        .O_csi_data        (S_csi_data),
        .O_data_length     ()
    );

    raw10_unpacket_2lane U_RAW10_UNPACK (
        .I_clk                (O_video_clk),
        .I_rst_n              (I_rst_n),
        .I_csi_frame_start    (S_csi_frame_start),
        .I_csi_frame_end      (S_csi_frame_end),
        .I_csi_valid          (S_csi_valid),
        .I_csi_data           (S_csi_data),
        .I_camera_black_level (10'd0),
        .O_raw10_frame_start  (S_raw10_frame_start),
        .O_raw10_frame_end    (S_raw10_frame_end),
        .O_raw10_valid        (S_raw10_valid),
        .O_raw10_data         (S_raw10_data)
    );

    // 每拍打包4个RAW10像素；分辨率必须与摄像头寄存器表和HDMI时序一致。
    uial2axis #(
        .IMG_WIDTH        (1280),
        .IMG_HEIGHT       (720),
        .INPUT_DATA_WIDTH (40)
    ) U_RAW10_TO_AXIS (
        .I_native_clk (O_video_clk),
        .I_rst_n      (I_rst_n),
        .I_data       (S_raw10_data),
        .I_data_valid (S_raw10_valid),
        .I_data_start (S_raw10_frame_start),
        .I_data_end   (S_raw10_frame_end),
        .axis_tvalid  (S_axis_valid),
        .axis_tdata   (S_axis_data),
        .axis_tuser   (S_axis_user),
        .axis_tlast   (S_axis_last)
    );

    // ISP完成BGGR去马赛克和白平衡，再把RGB888像素打包为128位数据。
    isp_top U_ISP (
        .axi4s_video_aclk (O_video_clk),
        .I_rst_n          (I_rst_n),
        .I_tlast          (S_axis_last),
        .I_tuser          (S_axis_user),
        .I_tdata          (S_axis_data),
        .I_tvalid         (S_axis_valid),
        .I_tdest          (10'd0),
        .O_tready         (1'b1),
        .O_tdata          (S_isp_data),
        .O_tlast          (S_isp_last),
        .O_tuser          (S_isp_user),
        .O_tvalid         (S_isp_valid),
        .I_tready         ()
    );

    assign O_frame_start = S_isp_user;
    assign O_valid       = S_isp_valid;
    assign O_data        = S_isp_data;
    assign O_lane_error  = |S_lane_error;

endmodule
