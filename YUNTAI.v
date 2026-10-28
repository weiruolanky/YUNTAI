// 云台系统顶层：SC520CS采集 -> MIPI/ISP -> DDR帧缓存 -> 图像处理 -> HDMI。
// 三色识别结果同时送往目标框叠加和双轴舵机控制。
module YUNTAI (
    input  wire        I_sys_clk,
    input  wire        I_rst_n,

    output wire        O_cam_scl,
    inout  wire        IO_cam_sda,
    output wire        O_cam_mclk,
    output wire        O_cam_rst_n,
    input  wire [1:0]  I_button,
    // 板载按键低电平有效：K1=D5，K2=A9，K3=B9，K4=C7。
    input  wire [3:0]  I_key,

    output wire        O_screen_pwm,
    output wire        O_status,
    // 云台PWM：水平轴接J2-1/C18，俯仰轴接J2-2/D15。
    output wire        O_servo_pan_pwm,
    output wire        O_servo_tilt_pwm,
    output wire        O_tmds_ch0_p,
    output wire        O_tmds_ch1_p,
    output wire        O_tmds_ch2_p,
    output wire        O_tmds_clk_p,

    // SC520CS使用DPHY0专用MIPI管脚，不能改成普通LVCMOS约束。
    inout  wire        IO_rx_clk_pad_n,
    inout  wire        IO_rx_clk_pad_p,
    inout  wire [3:0]  IO_rx_data_pad_n,
    inout  wire [3:0]  IO_rx_data_pad_p,

    // 板载DDR2 x16接口。
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

    localparam integer CAM_PWDNB_LOW_CYCLES = 23_810;   // 23.809524MHz下至少延时1ms
    localparam integer CAM_I2C_START_CYCLES = 142_857;  // 上电至少6ms后启动I2C

    wire S_clk_100m;
    wire S_clk_cam;
    wire S_clk_pixel;
    wire S_clk_serial;
    wire S_pll_lock;
    wire S_logic_rst_n;

    // 使用已知的100MHz主时钟生成16位毫秒时间戳，约65.536秒回绕一次。
    reg [16:0] S_latency_ms_divider;
    reg [15:0] S_latency_time_ms;
    reg [15:0] S_latency_time_gray;
    wire [15:0] S_latency_time_ms_next;

    reg  [17:0] S_cam_power_cnt;
    reg         S_cam_pwdnb;
    reg         S_cam_i2c_ready;
    wire        S_cam_cfg_done;
    wire        S_cam_ae_cfg_done;
    wire        S_cam_i2c_error;
    wire [15:0] S_cam_sensor_id;
    wire        S_cam_sensor_id_ok;
    wire        S_ae_req;
    wire [15:0] S_ae;
    wire [15:0] S_ag;
    wire        S_auto_exposure_enable;
    wire        S_sample_average_update;

    wire        S_camera_clk;
    wire        S_camera_frame_start;
    wire        S_camera_valid;
    wire [127:0] S_camera_data;
    wire        S_camera_lane_error;

    wire        S_ddr_clk;
    wire        S_ddr_init_done;
    wire        S_video_in_wr_busy;
    wire        S_video_out_rd_busy;
    wire [1:0]  S_video_out_rp;
    wire [15:0] S_latency_time_gray_ddr;
    wire [15:0] S_display_time_gray;
    wire        S_display_frame_valid;
    wire [11:0] S_latency_ms;
    wire        S_latency_valid;
    wire        S_vi_ddr_wr_en;
    wire [24:0] S_vi_ddr_wr_addr;
    wire [127:0] S_vi_ddr_wr_data;
    wire        S_vo_ddr_rd_en;
    wire [24:0] S_vo_ddr_rd_addr;
    wire        S_ddr_user_ready;
    wire        S_ddr_user_rd_valid;
    wire [127:0] S_ddr_user_rd_data;
    wire        S_ddr_user_wr_en;
    wire        S_ddr_user_rd_en;
    wire [24:0] S_ddr_user_addr;
    wire [127:0] S_ddr_user_wr_data;

    wire [24:0] S_mc_app_addr;
    wire [2:0]  S_mc_app_cmd;
    wire        S_mc_app_en;
    wire [127:0] S_mc_app_wdf_data;
    wire        S_mc_app_wdf_end;
    wire [31:0] S_mc_app_wdf_mask;
    wire        S_mc_app_wdf_wren;
    wire [127:0] S_mc_app_rd_data;
    wire        S_mc_app_rd_data_end;
    wire        S_mc_app_rd_data_valid;
    wire        S_mc_app_rdy;
    wire        S_mc_app_wdf_rdy;

    wire        S_hdmi_vsync;
    wire        S_hdmi_hsync;
    wire        S_hdmi_de;
    wire        S_hdmi_user;
    wire        S_hdmi_last;
    wire        S_video_out_vsync;
    wire        S_hdmi_window_rd_en;
    wire [23:0] S_video_rd_data;
    wire        S_hdmi_out_vsync;
    wire        S_hdmi_out_hsync;
    wire        S_hdmi_out_de;
    wire [23:0] S_hdmi_out_data;
    wire        S_display_vsync;
    wire        S_display_de;
    wire [23:0] S_display_data;
    wire [1:0]  S_display_mode;
    wire        S_target_valid;
    wire        S_target_result_update;
    wire        S_target_measurement_valid;
    wire [1:0]  S_target_color;
    wire [11:0] S_target_x_min;
    wire [11:0] S_target_x_max;
    wire [10:0] S_target_y_min;
    wire [10:0] S_target_y_max;
    wire [11:0] S_target_center_x;
    wire [10:0] S_target_center_y;
    wire [19:0] S_target_pixels;
    wire        S_target_overlay_enable;
    wire [23:0] S_target_data;
    wire [23:0] S_logo_data;
    wire        S_rgb_monitor_enable;
    wire [7:0]  S_sample_average_r;
    wire [7:0]  S_sample_average_g;
    wire [7:0]  S_sample_average_b;
    wire [23:0] S_monitor_data;
    wire [23:0] S_mode_notice_data;
    wire [19:0] S_motion_pixels;
    wire        S_motion_valid;
    wire        S_motion_detected;

    assign S_logic_rst_n = I_rst_n && S_pll_lock;
    assign S_latency_time_ms_next = S_latency_time_ms + 1'b1;

    always @(posedge S_clk_100m or negedge S_logic_rst_n) begin
        if (!S_logic_rst_n) begin
            S_latency_ms_divider <= 17'd0;
            S_latency_time_ms <= 16'd0;
            S_latency_time_gray <= 16'd0;
        end else if (S_latency_ms_divider == 17'd99_999) begin
            S_latency_ms_divider <= 17'd0;
            S_latency_time_ms <= S_latency_time_ms_next;
            S_latency_time_gray <= S_latency_time_ms_next ^
                                   (S_latency_time_ms_next >> 1);
        end else begin
            S_latency_ms_divider <= S_latency_ms_divider + 1'b1;
        end
    end
    assign O_screen_pwm  = 1'b1;
    // 摄像头配置和DDR训练均成功后点亮状态灯。
    assign O_status = S_cam_cfg_done && S_cam_sensor_id_ok &&
                      !S_cam_i2c_error && S_ddr_init_done;

    // PLL输出：100MHz MIPI低功耗时钟、23.809524MHz摄像头MCLK、
    // 75MHz HDMI像素时钟和375MHz TMDS串行时钟。
    YUNTAI_PLL U_PLL (
        .refclk   (I_sys_clk),
        .reset    (~I_rst_n),
        .clk0_out (S_clk_100m),
        .clk1_out (S_clk_cam),
        .clk4_out (S_clk_pixel),
        .clk5_out (S_clk_serial),
        .lock     (S_pll_lock)
    );

    // 舵机使用板载50MHz时钟；只接收握手确认的新测量，不使用显示保持帧。
    // 帧差静止测试期间关闭舵机PWM，避免云台运动干扰画面差分。
    YUNTAI_SERVO_FIXED U_SERVO_FIXED (
        .I_clk_50m           (I_sys_clk),
        .I_rst_n             (I_rst_n),
        .I_enable            (S_target_overlay_enable),
        .I_pixel_clk         (S_clk_pixel),
        .I_result_update     (S_target_result_update),
        .I_measurement_valid (S_target_measurement_valid),
        .I_target_color      (S_target_color),
        .I_center_x          (S_target_center_x),
        .I_center_y          (S_target_center_y),
        .O_pan_pwm           (O_servo_pan_pwm),
        .O_tilt_pwm          (O_servo_tilt_pwm)
    );

    // PLL锁定并满足模拟上电延时后，才释放摄像头并启动I2C。
    always @(posedge S_clk_cam or negedge S_logic_rst_n) begin
        if (!S_logic_rst_n) begin
            S_cam_power_cnt <= 18'd0;
            S_cam_pwdnb     <= 1'b0;
            S_cam_i2c_ready <= 1'b0;
        end else begin
            if (S_cam_power_cnt < CAM_I2C_START_CYCLES)
                S_cam_power_cnt <= S_cam_power_cnt + 1'b1;
            if (S_cam_power_cnt >= CAM_PWDNB_LOW_CYCLES)
                S_cam_pwdnb <= 1'b1;
            if (S_cam_power_cnt >= CAM_I2C_START_CYCLES - 1)
                S_cam_i2c_ready <= 1'b1;
        end
    end

    assign O_cam_mclk  = S_clk_cam;
    assign O_cam_rst_n = S_cam_pwdnb;

    // SC520CS工作在1280x720@60、RAW10、双通道828Mbps；K4长按控制自动曝光。
    ae_set U_AE_SET (
        .I_clk          (S_clk_cam),
        .I_rst          (~S_logic_rst_n),
        .I_btn          ({I_button, 2'b11}),
        .I_auto_enable  (S_auto_exposure_enable),
        .I_average_update(S_sample_average_update),
        .I_average_r    (S_sample_average_r),
        .I_average_g    (S_sample_average_g),
        .I_average_b    (S_sample_average_b),
        .I_ae_cfg_done  (S_cam_ae_cfg_done),
        .I_cam_cfg_done (S_cam_cfg_done),
        .O_ae_req       (S_ae_req),
        .O_ae           (S_ae),
        .O_ag           (S_ag)
    );

    uicfgcs520 #(
        .CLK_DIV (23809524 / 100000 - 1)
    ) U_SC520_CONFIG (
        .I_clk          (S_clk_cam),
        .I_rst_n        (S_cam_i2c_ready),
        .I_ae_req       (S_ae_req),
        .I_ae           (S_ae),
        .I_ag           (S_ag),
        .O_cam_scl      (O_cam_scl),
        .IO_cam_sda     (IO_cam_sda),
        .O_cfg_done     (S_cam_cfg_done),
        .O_ae_cfg_done  (S_cam_ae_cfg_done),
        .O_sensor_id    (S_cam_sensor_id),
        .O_sensor_id_ok (S_cam_sensor_id_ok),
        .O_iic_error    (S_cam_i2c_error)
    );

    YUNTAI_CAMERA_PIPELINE U_CAMERA_PIPELINE (
        .I_lp_clk          (S_clk_100m),
        .I_rst_n           (S_logic_rst_n),
        .IO_rx_clk_pad_n   (IO_rx_clk_pad_n),
        .IO_rx_clk_pad_p   (IO_rx_clk_pad_p),
        .IO_rx_data_pad_n  (IO_rx_data_pad_n),
        .IO_rx_data_pad_p  (IO_rx_data_pad_p),
        .O_video_clk       (S_camera_clk),
        .O_frame_start     (S_camera_frame_start),
        .O_valid           (S_camera_valid),
        .O_data            (S_camera_data),
        .O_lane_error      (S_camera_lane_error)
    );

    // 四帧DDR缓存隔离MIPI写入时钟域和75MHz HDMI读出时钟域。
    video_in U_VIDEO_IN (
        .I_rst_n              (S_logic_rst_n),
        .I_camera_clk         (S_camera_clk),
        .I_camera_frame_start (S_camera_frame_start),
        .I_camera_valid       (S_camera_valid),
        .I_camera_data        (S_camera_data),
        .I_mipi_rx_error      (S_camera_lane_error),
        .I_ddr_clk            (S_ddr_clk),
        .I_display_pause      (1'b0),
        .I_video_out_rd_busy  (S_video_out_rd_busy),
        .O_video_in_wr_busy   (S_video_in_wr_busy),
        .O_video_out_rp       (S_video_out_rp),
        .I_latency_time_gray  (S_latency_time_gray),
        .O_latency_time_gray  (S_latency_time_gray_ddr),
        .O_display_time_gray  (S_display_time_gray),
        .O_display_frame_valid(S_display_frame_valid),
        .O_ddr_user_wr_en     (S_vi_ddr_wr_en),
        .O_ddr_user_addr      (S_vi_ddr_wr_addr),
        .O_ddr_user_wr_data   (S_vi_ddr_wr_data),
        .I_ddr_user_ready     (S_ddr_user_ready)
    );

    assign S_ddr_user_wr_en   = S_vi_ddr_wr_en;
    assign S_ddr_user_rd_en   = S_vo_ddr_rd_en;
    assign S_ddr_user_addr    = S_vi_ddr_wr_en ? S_vi_ddr_wr_addr :
                                S_vo_ddr_rd_en ? S_vo_ddr_rd_addr : 25'd0;
    assign S_ddr_user_wr_data = S_vi_ddr_wr_data;

    assign S_video_out_vsync = ~S_hdmi_vsync;

    video_out U_VIDEO_OUT (
        .I_rst_n             (S_logic_rst_n),
        .I_ddr_clk           (S_ddr_clk),
        .O_video_out_rd_busy (S_video_out_rd_busy),
        .I_video_in_wr_busy  (S_video_in_wr_busy),
        .I_video_out_rp      (S_video_out_rp),
        .I_latency_time_gray (S_latency_time_gray_ddr),
        .I_display_time_gray  (S_display_time_gray),
        .I_display_frame_valid(S_display_frame_valid),
        .O_ddr_user_rd_en    (S_vo_ddr_rd_en),
        .O_ddr_user_addr     (S_vo_ddr_rd_addr),
        .I_ddr_user_ready    (S_ddr_user_ready),
        .I_ddr_user_rd_valid (S_ddr_user_rd_valid),
        .I_ddr_user_rd_data  (S_ddr_user_rd_data),
        .I_dsi_clk           (S_clk_pixel),
        .I_video_vsync       (S_video_out_vsync),
        .I_video_rd_en       (S_hdmi_window_rd_en),
        .O_vdieo_data        (S_video_rd_data),
        .O_latency_ms       (S_latency_ms),
        .O_latency_valid    (S_latency_valid)
    );

    mc_to_user_interface U_MC_TO_USER (
        .I_clk                  (S_ddr_clk),
        .I_rst_n                (S_logic_rst_n),
        .I_ddr_user_wr_en       (S_ddr_user_wr_en),
        .I_ddr_user_rd_en       (S_ddr_user_rd_en),
        .I_ddr_user_addr        (S_ddr_user_addr),
        .I_ddr_user_wr_data     (S_ddr_user_wr_data),
        .O_ddr_user_ready       (S_ddr_user_ready),
        .O_ddr_user_rd_valid    (S_ddr_user_rd_valid),
        .O_ddr_user_rd_data     (S_ddr_user_rd_data),
        .O_mc_app_en            (S_mc_app_en),
        .O_mc_app_addr          (S_mc_app_addr),
        .O_mc_app_cmd           (S_mc_app_cmd),
        .I_mc_app_rdy           (S_mc_app_rdy),
        .O_mc_app_wdf_wren      (S_mc_app_wdf_wren),
        .O_mc_app_wdf_data      (S_mc_app_wdf_data),
        .O_mc_app_wdf_end       (S_mc_app_wdf_end),
        .O_mc_app_wdf_mask      (S_mc_app_wdf_mask),
        .I_mc_app_wdf_rdy       (S_mc_app_wdf_rdy),
        .I_mc_app_rd_data       (S_mc_app_rd_data),
        .I_mc_app_rd_data_end   (S_mc_app_rd_data_end),
        .I_mc_app_rd_data_valid (S_mc_app_rd_data_valid)
    );

    YUNTAI_DDR_BRIDGE U_DDR_BRIDGE (
    // DDR控制器IP使用100MHz参考时钟。
        .I_sys_clk              (S_clk_100m),
        .I_sys_rst_n            (S_logic_rst_n),
        .O_ddr_clk              (S_ddr_clk),
        .O_init_calib_complete  (S_ddr_init_done),
        .I_mc_app_addr          (S_mc_app_addr),
        .I_mc_app_cmd           (S_mc_app_cmd),
        .I_mc_app_en            (S_mc_app_en),
        .I_mc_app_wdf_data      (S_mc_app_wdf_data),
        .I_mc_app_wdf_end       (S_mc_app_wdf_end),
        .I_mc_app_wdf_mask      (S_mc_app_wdf_mask[15:0]),
        .I_mc_app_wdf_wren      (S_mc_app_wdf_wren),
        .O_mc_app_rd_data       (S_mc_app_rd_data),
        .O_mc_app_rd_data_end   (S_mc_app_rd_data_end),
        .O_mc_app_rd_data_valid (S_mc_app_rd_data_valid),
        .O_mc_app_rdy           (S_mc_app_rdy),
        .O_mc_app_wdf_rdy       (S_mc_app_wdf_rdy),
        .ddr_addr               (ddr_addr),
        .ddr_ba                 (ddr_ba),
        .ddr_cke                (ddr_cke),
        .ddr_odt                (ddr_odt),
        .ddr_cs_n               (ddr_cs_n),
        .ddr_ras_n              (ddr_ras_n),
        .ddr_cas_n              (ddr_cas_n),
        .ddr_we_n               (ddr_we_n),
        .ddr_ck_p               (ddr_ck_p),
        .ddr_ck_n               (ddr_ck_n),
        .ddr_dm                 (ddr_dm),
        .ddr_dq                 (ddr_dq),
        .ddr_dqs_p              (ddr_dqs_p),
        .ddr_dqs_n              (ddr_dqs_n)
    );

    // 生成1280x720扫描时序，并使DDR读出数据与DE/同步信号对齐。
    uivtc #(
        .H_ActiveSize (1280),
        .H_FrameSize  (1650),
        .H_SyncStart  (1390),
        .H_SyncEnd    (1430),
        .V_ActiveSize (720),
        .V_FrameSize  (750),
        .V_SyncStart  (725),
        .V_SyncEnd    (730)
    ) U_HDMI_TIMING (
        .I_vtc_rstn     (S_logic_rst_n),
        .I_vtc_clk      (S_clk_pixel),
        .O_vtc_vs       (S_hdmi_vsync),
        .O_vtc_hs       (S_hdmi_hsync),
        .O_vtc_de_valid (S_hdmi_de),
        .O_vtc_user     (S_hdmi_user),
        .O_vtc_last     (S_hdmi_last)
    );

    hdmi_mixer #(
        .H_OFFSET   (0),
        .V_OFFSET   (0),
        .IMG_WIDTH  (1280),
        .IMG_HEIGHT (720),
        .DEBUG_MODE (0)
    ) U_HDMI_MIXER (
        .I_clk           (S_clk_pixel),
        .I_rst_n         (S_logic_rst_n),
        .I_video_vsync   (S_hdmi_vsync),
        .I_video_hsync   (S_hdmi_hsync),
        .I_video_de      (S_hdmi_de),
        .I_video_user    (S_hdmi_user),
        .I_video_last    (S_hdmi_last),
        .I_debug_status  (4'd0),
        .O_video_rd_en   (S_hdmi_window_rd_en),
        .I_video_rd_data (S_video_rd_data),
        .O_hdmi_vsync    (S_hdmi_out_vsync),
        .O_hdmi_hsync    (S_hdmi_out_hsync),
        .O_hdmi_de       (S_hdmi_out_de),
        .O_hdmi_data     (S_hdmi_out_data)
    );

    // K1短按切换二值化，K1长按切换Sobel，K2切换灰度；各模式共用对齐流水线。
    YUNTAI_BINARY_DISPLAY U_BINARY_DISPLAY (
        .I_clk          (S_clk_pixel),
        .I_rst_n        (S_logic_rst_n),
        .I_key1_n       (I_key[0]),
        .I_key2_n       (I_key[1]),
        .I_vsync        (S_hdmi_out_vsync),
        .I_de           (S_hdmi_out_de),
        .I_rgb          (S_hdmi_out_data),
        .O_vsync        (S_display_vsync),
        .O_de           (S_display_de),
        .O_rgb          (S_display_data),
        .O_display_mode (S_display_mode)
    );

    // 在叠加文字和图框前，用原始RGB数据检测红、蓝、紫目标。
    YUNTAI_FLOWER_DETECT U_FLOWER_DETECT (
        .I_clk           (S_clk_pixel),
        .I_rst_n         (S_logic_rst_n),
        .I_vsync         (S_hdmi_out_vsync),
        .I_de            (S_hdmi_out_de),
        .I_rgb           (S_hdmi_out_data),
        .O_target_valid  (S_target_valid),
        .O_x_min         (S_target_x_min),
        .O_x_max         (S_target_x_max),
        .O_y_min         (S_target_y_min),
        .O_y_max         (S_target_y_max),
        .O_center_x      (S_target_center_x),
        .O_center_y      (S_target_center_y),
        .O_target_pixels (S_target_pixels),
        .O_target_color  (S_target_color),
        .O_result_update (S_target_result_update),
        .O_measurement_valid (S_target_measurement_valid)
    );

    // 固定云台调试用帧差统计，阈值可在模块参数 DIFF_THRESHOLD 中调整。
    YUNTAI_FRAME_DIFF U_FRAME_DIFF (
        .I_clk          (S_clk_pixel),
        .I_rst_n        (S_logic_rst_n),
        .I_vsync        (S_hdmi_out_vsync),
        .I_de           (S_hdmi_out_de),
        .I_rgb          (S_hdmi_out_data),
        .I_target_valid (S_target_valid),
        .O_motion_pixels(S_motion_pixels),
        .O_result_valid (S_motion_valid),
        .O_motion_detected(S_motion_detected)
    );

    // K3控制智能跟踪、目标框、中心标记和识别状态显示。
    YUNTAI_TARGET_OVERLAY U_TARGET_OVERLAY (
        .I_clk              (S_clk_pixel),
        .I_rst_n            (S_logic_rst_n),
        .I_key3_n           (I_key[2]),
        .I_vsync            (S_display_vsync),
        .I_de               (S_display_de),
        .I_rgb              (S_display_data),
        .I_target_valid     (S_target_valid),
        .I_target_color     (S_target_color),
        .I_x_min            (S_target_x_min),
        .I_x_max            (S_target_x_max),
        .I_y_min            (S_target_y_min),
        .I_y_max            (S_target_y_max),
        .I_center_x         (S_target_center_x),
        .I_center_y         (S_target_center_y),
        .I_latency_ms       (S_latency_ms),
        .I_latency_valid    (S_latency_valid),
        .I_motion_pixels    (S_motion_pixels),
        .I_motion_valid     (S_motion_valid),
        .I_motion_detected  (S_motion_detected),
        .O_rgb              (S_target_data),
        .O_overlay_enable   (S_target_overlay_enable)
    );

    // Logo采用透明背景叠加，位于图像处理和目标框之后。
    YUNTAI_LOGO_OVERLAY U_LOGO_OVERLAY (
        .I_clk          (S_clk_pixel),
        .I_rst_n        (S_logic_rst_n),
        .I_vsync        (S_display_vsync),
        .I_de           (S_display_de),
        .I_rgb          (S_target_data),
        .O_rgb          (S_logo_data)
    );

    // K4短按显示RGB采样值，长按2秒切换自动曝光。
    YUNTAI_RGB_MONITOR U_RGB_MONITOR (
        .I_clk             (S_clk_pixel),
        .I_rst_n           (S_logic_rst_n),
        .I_key4_n          (I_key[3]),
        .I_source_vsync    (S_hdmi_out_vsync),
        .I_source_de       (S_hdmi_out_de),
        .I_source_rgb      (S_hdmi_out_data),
        .I_display_vsync   (S_display_vsync),
        .I_display_de      (S_display_de),
        .I_display_rgb     (S_logo_data),
        .O_rgb             (S_monitor_data),
        .O_monitor_enable  (S_rgb_monitor_enable),
        .O_auto_exposure_enable (S_auto_exposure_enable),
        .O_average_update  (S_sample_average_update),
        .O_average_r       (S_sample_average_r),
        .O_average_g       (S_sample_average_g),
        .O_average_b       (S_sample_average_b)
    );

    // 按键改变模式后，在画面中央短暂显示英文状态提示。
    YUNTAI_MODE_NOTICE U_MODE_NOTICE (
        .I_clk                  (S_clk_pixel),
        .I_rst_n                (S_logic_rst_n),
        .I_vsync                (S_display_vsync),
        .I_de                   (S_display_de),
        .I_rgb                  (S_monitor_data),
        .I_display_mode         (S_display_mode),
        .I_tracking_enable      (S_target_overlay_enable),
        .I_rgb_monitor_enable   (S_rgb_monitor_enable),
        .I_auto_exposure_enable (S_auto_exposure_enable),
        .O_rgb                  (S_mode_notice_data)
    );

    hdmi_tx U_HDMI_TX (
        .I_pixel_clk        (S_clk_pixel),
        .I_serial_clk       (S_clk_serial),
        .I_rst              (~S_logic_rst_n),
        .I_key_in           (1'b0),
        .I_edid_read_trig   (1'b0),
        .O_edid_read_valid  (),
        .O_edid_read_data   (),
        .I_video_rgb_enable (1'b1),
        .I_video_in_vs      (S_display_vsync),
        .I_video_in_de      (S_display_de),
        .I_video_in_user    (1'b0),
        .I_video_in_valid   (1'b0),
        .I_video_in_last    (1'b0),
        .O_video_in_ready   (),
        .I_video_in_data    (S_mode_notice_data),
        .I_audio_valid      (1'b0),
        .I_audio_left_data  (24'd0),
        .I_audio_right_data (24'd0),
        .I_i2s_BCLK         (1'b0),
        .I_i2s_LRCK         (1'b0),
        .I_i2s_DOUT         (1'b0),
        .O_ddc_scl          (),
        .IO_ddc_sda         (),
        .O_hdmi_clk_p       (),
        .O_hdmi_tx_p        (),
        .O_tmds_ch0_p       (O_tmds_ch0_p),
        .O_tmds_ch1_p       (O_tmds_ch1_p),
        .O_tmds_ch2_p       (O_tmds_ch2_p),
        .O_tmds_clk_p       (O_tmds_clk_p)
    );

endmodule
