// 帧差统计：保存上一帧亮度，并统计当前帧变化超过阈值的像素数量。
// 该模块只输出帧级统计值，不参与舵机控制，适合先固定云台进行抗噪测试。
module YUNTAI_FRAME_DIFF #(
    parameter integer IMAGE_WIDTH  = 1280,
    parameter integer IMAGE_HEIGHT = 720,
    parameter [7:0]  DIFF_THRESHOLD = 8'd12,
    // 根据实测静止约150~170、运动约500~700设置滞回门限。
    parameter [19:0] MOTION_ON_THRESHOLD  = 20'd400,
    parameter [19:0] MOTION_OFF_THRESHOLD = 20'd300
) (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_vsync,
    input  wire        I_de,
    input  wire [23:0] I_rgb,
    input  wire        I_target_valid,
    output reg  [19:0] O_motion_pixels,
    output reg         O_result_valid,
    output reg         O_motion_detected
);

    // 采用 8x8 像素块采样，避免为完整 720p 帧消耗大量片上存储器。
    localparam integer SAMPLE_WIDTH  = (IMAGE_WIDTH + 15) / 16;
    localparam integer SAMPLE_HEIGHT = (IMAGE_HEIGHT + 15) / 16;
    localparam integer SAMPLE_PIXELS = SAMPLE_WIDTH * SAMPLE_HEIGHT;
    localparam integer ADDR_WIDTH = 12;

    // 1 字节/像素的亮度缓存；综合器应将其映射为片上 RAM。
    // 提示安路综合器优先使用片上块 RAM，避免展开为大量 LUT。
    (* ram_style = "block" *) reg [7:0] S_previous_luma [0:SAMPLE_PIXELS-1];
    reg [10:0] S_x;
    reg [9:0]  S_y;
    reg [19:0] S_motion_count;
    reg [1:0]  S_frame_index;
    reg [1:0]  S_high_frames;
    reg [1:0]  S_low_frames;
    reg [7:0]  S_target_loss_frames;
    reg        S_vsync_d;
    reg [7:0]  S_old_luma_reg;
    reg [7:0]  S_sample_luma;
    reg        S_sample_pending;

    wire [9:0] S_luma_sum =
        {2'b00, I_rgb[23:16] >> 2} +
        {2'b00, I_rgb[15:8]  >> 1} +
        {2'b00, I_rgb[7:0]   >> 4};
    wire [7:0] S_luma = S_luma_sum[7:0];
    // 当前像素所属 8x8 采样块地址，确保每个块跨行时仍与自身比较。
    // SAMPLE_WIDTH=160，使用移位相加替代乘法器。
    wire [ADDR_WIDTH-1:0] S_sample_addr =
        (((S_y >> 4) << 6) + ((S_y >> 4) << 4)) + (S_x >> 4);
    wire [8:0] S_luma_delta = (S_sample_luma >= S_old_luma_reg) ?
                               ({1'b0, S_sample_luma} - {1'b0, S_old_luma_reg}) :
                               ({1'b0, S_old_luma_reg} - {1'b0, S_sample_luma});
    wire S_motion_pixel = S_sample_pending && (S_luma_delta > DIFF_THRESHOLD);

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_x             <= 11'd0;
            S_y             <= 10'd0;
            S_motion_count  <= 20'd0;
            S_frame_index   <= 2'd0;
            S_vsync_d       <= 1'b0;
            O_motion_pixels <= 20'd0;
            O_result_valid  <= 1'b0;
            O_motion_detected <= 1'b0;
            S_high_frames   <= 2'd0;
            S_low_frames    <= 2'd0;
            S_target_loss_frames <= 8'd0;
            S_old_luma_reg  <= 8'd0;
            S_sample_luma   <= 8'd0;
            S_sample_pending<= 1'b0;
        end else begin
            S_vsync_d <= I_vsync;

            // 帧消隐结束时锁存上一帧结果，并开始新一帧统计。
            if (S_vsync_d && !I_vsync) begin
                O_motion_pixels <= (S_frame_index != 0) ? S_motion_count : 20'd0;
                O_result_valid  <= (S_frame_index != 0);
                // 使用上下两个门限和连续三帧确认，抑制静止噪声导致的误报。
                if (S_frame_index == 0) begin
                    O_motion_detected <= 1'b0;
                    S_high_frames <= 2'd0;
                    S_low_frames  <= 2'd0;
                end else if (!I_target_valid) begin
                    // 目标暂时丢失时不再累计帧差，连续约2秒无目标后清零状态。
                    S_high_frames <= 2'd0;
                    S_low_frames  <= 2'd0;
                    if (S_target_loss_frames < 8'd120)
                        S_target_loss_frames <= S_target_loss_frames + 1'b1;
                    else
                        O_motion_detected <= 1'b0;
                end else if (!O_motion_detected) begin
                    S_target_loss_frames <= 8'd0;
                    S_low_frames <= 2'd0;
                    if (S_motion_count > MOTION_ON_THRESHOLD) begin
                        if (S_high_frames == 2'd2) begin
                            O_motion_detected <= 1'b1;
                            S_high_frames <= 2'd0;
                        end else begin
                            S_high_frames <= S_high_frames + 1'b1;
                        end
                    end else begin
                        S_high_frames <= 2'd0;
                    end
                end else begin
                    S_high_frames <= 2'd0;
                    if (S_motion_count < MOTION_OFF_THRESHOLD) begin
                        if (S_low_frames == 2'd2) begin
                            O_motion_detected <= 1'b0;
                            S_low_frames <= 2'd0;
                        end else begin
                            S_low_frames <= S_low_frames + 1'b1;
                        end
                    end else begin
                        S_low_frames <= 2'd0;
                    end
                end
                S_frame_index   <= (S_frame_index == 2'd3) ? 2'd3 : S_frame_index + 1'b1;
                S_motion_count  <= 20'd0;
                S_x             <= 11'd0;
                S_y             <= 10'd0;
                S_sample_pending<= 1'b0;
            end else if (I_de) begin
                // 每个 8x8 块只在左上角采样一次，统计结果按块计数。
                if ((S_x[3:0] == 4'd0) && (S_y[3:0] == 4'd0)) begin
                    if (S_sample_pending && I_target_valid &&
                        (S_frame_index != 0) &&
                        S_motion_pixel && (S_motion_count <= 20'hffffb))
                        S_motion_count <= S_motion_count + 20'd4;
                    S_old_luma_reg   <= S_previous_luma[S_sample_addr];
                    S_sample_luma    <= S_luma;
                    S_sample_pending <= 1'b1;
                    S_previous_luma[S_sample_addr] <= S_luma;
                end
                if (S_x < IMAGE_WIDTH - 1)
                    S_x <= S_x + 1'b1;
                else begin
                    S_x <= 11'd0;
                    if (S_y < IMAGE_HEIGHT - 1)
                        S_y <= S_y + 1'b1;
                end
            end
        end
    end
endmodule
