// 双轴智能跟踪：水平轴接J2-1/C18，俯仰轴接J2-2/D15。
module YUNTAI_SERVO_FIXED (
    input  wire I_clk_50m,
    input  wire I_rst_n,
    input  wire I_enable,
    input  wire I_pixel_clk,
    input  wire I_result_update,
    input  wire I_measurement_valid,
    input  wire [1:0] I_target_color,
    input  wire [11:0] I_center_x,
    input  wire [10:0] I_center_y,
    output reg  O_pan_pwm,
    output reg  O_tilt_pwm
);
    // 50MHz下1us=50个时钟；PWM周期20ms。
    localparam integer SERVO_PULSE_US = 1500;
    localparam [19:0] PERIOD_CYCLES = 20'd1_000_000; // PWM周期20ms
    localparam [19:0] START_PULSE = SERVO_PULSE_US * 50;
    // 俯仰轴按0~120度控制，约1833.5us对应90度；上电后主动转到并保持该位置。
    localparam [19:0] TILT_START_PULSE = 20'd91_675;
    // 水平轴500~2500us约对应180度，最终安全范围以机械限位为准。
    localparam [19:0] MIN_PULSE = 20'd25_000;
    localparam [19:0] MAX_PULSE = 20'd125_000;
    // 俯仰轴833~2167us约对应120度，避免支架碰撞。
    localparam [19:0] TILT_MIN_PULSE = 20'd41_650;
    localparam [19:0] TILT_MAX_PULSE = 20'd108_350;
    // 画面中心为(640,360)；误差落在死区内时不调整PWM。
    // 相邻帧跳变超过SAMPLE_JUMP时视为不可靠测量，避免误检造成大幅转动。
    localparam [11:0] CENTER_X = 12'd640;
    localparam [10:0] CENTER_Y = 11'd360;
    localparam [11:0] DEADBAND_X = 12'd24;
    localparam [10:0] DEADBAND_Y = 11'd16;
    localparam [11:0] SAMPLE_JUMP_X = 12'd96;
    localparam [10:0] SAMPLE_JUMP_Y = 11'd64;
    localparam [22:0] RESULT_TIMEOUT = 23'd5_000_000; // 100ms无新结果即停止修正
    localparam [2:0] CONTROL_PERIODS = 3'd3; // 每3个PWM周期，即60ms修正一次
    // 纯比例控制：步进=|像素误差|*4个时钟，最大500个时钟=10us。
    localparam [19:0] MAX_STEP = 20'd500;
    // 当前实物安装方向：水平轴取反，俯仰轴不取反。
    localparam PAN_REVERSE = 1'b1;
    localparam TILT_REVERSE = 1'b0;

    // 坐标邮箱采用请求/应答翻转握手，源端在应答前保持整组数据稳定。
    reg [23:0] S_mailbox;
    reg S_request;
    reg S_acknowledge;
    reg S_ack_meta;
    reg S_ack_sync;
    reg S_request_meta;
    reg S_request_sync;
    reg S_enable_meta;
    reg S_enable_sync;
    reg [19:0] S_counter;
    reg [19:0] S_command_pulse;
    reg [19:0] S_active_pulse;
    reg [2:0] S_control_periods;
    reg [1:0] S_valid_frames;
    reg [11:0] S_previous_x;
    reg [11:0] S_filtered_x;
    reg [10:0] S_previous_y;
    reg [10:0] S_filtered_y;
    reg [22:0] S_result_age;
    reg [19:0] S_tilt_command_pulse;
    reg [19:0] S_tilt_active_pulse;

    wire S_new_result = (S_request_sync != S_acknowledge);
    wire S_mailbox_valid = S_mailbox[23];
    wire [11:0] S_mailbox_x = S_mailbox[22:11];
    wire [10:0] S_mailbox_y = S_mailbox[10:0];
    wire [11:0] S_sample_delta = (S_mailbox_x >= S_previous_x) ?
        S_mailbox_x - S_previous_x : S_previous_x - S_mailbox_x;
    wire [11:0] S_filter_delta = (S_mailbox_x >= S_filtered_x) ?
        S_mailbox_x - S_filtered_x : S_filtered_x - S_mailbox_x;
    wire [10:0] S_sample_delta_y = (S_mailbox_y >= S_previous_y) ?
        S_mailbox_y - S_previous_y : S_previous_y - S_mailbox_y;
    wire [10:0] S_filter_delta_y = (S_mailbox_y >= S_filtered_y) ?
        S_mailbox_y - S_filtered_y : S_filtered_y - S_mailbox_y;
    // 坐标滤波每次向新测量移动|差值|/4；差值为1~3像素时至少移动1像素。
    wire [11:0] S_filter_step = (S_filter_delta < 12'd4) ?
        ((S_filter_delta != 0) ? 12'd1 : 12'd0) : (S_filter_delta >> 2);
    wire [10:0] S_filter_step_y = (S_filter_delta_y < 11'd4) ?
        ((S_filter_delta_y != 0) ? 11'd1 : 11'd0) : (S_filter_delta_y >> 2);
    wire [11:0] S_error_abs = (S_filtered_x >= CENTER_X) ?
        S_filtered_x - CENTER_X : CENTER_X - S_filtered_x;
    wire [19:0] S_proportional_step = {6'd0, S_error_abs, 2'b00};
    wire [19:0] S_step = (S_proportional_step > MAX_STEP) ?
        MAX_STEP : S_proportional_step;
    wire S_increase = (S_filtered_x > CENTER_X) ^ PAN_REVERSE;
    wire [10:0] S_error_abs_y = (S_filtered_y >= CENTER_Y) ?
        S_filtered_y - CENTER_Y : CENTER_Y - S_filtered_y;
    wire [19:0] S_proportional_step_y = {9'd0, S_error_abs_y} << 2;
    wire [19:0] S_step_y = (S_proportional_step_y > MAX_STEP) ?
        MAX_STEP : S_proportional_step_y;
    wire S_increase_y = (S_filtered_y > CENTER_Y) ^ TILT_REVERSE;
    wire S_control_ready = S_enable_sync && (S_valid_frames == 2'd3) &&
        (S_result_age < RESULT_TIMEOUT) && !S_new_result &&
        (S_control_periods == CONTROL_PERIODS - 1'b1);
    wire S_correct_x = S_control_ready && (S_error_abs > DEADBAND_X);
    wire S_correct_y = S_control_ready && (S_error_abs_y > DEADBAND_Y);
    wire [19:0] S_increased_pulse =
        (S_command_pulse >= MAX_PULSE - S_step) ? MAX_PULSE :
        S_command_pulse + S_step;
    wire [19:0] S_decreased_pulse =
        (S_command_pulse <= MIN_PULSE + S_step) ? MIN_PULSE :
        S_command_pulse - S_step;
    wire [19:0] S_next_pulse = !S_correct_x ? S_command_pulse :
        (S_increase ? S_increased_pulse : S_decreased_pulse);
    wire [19:0] S_tilt_increased_pulse =
        (S_tilt_command_pulse >= TILT_MAX_PULSE - S_step_y) ? TILT_MAX_PULSE :
        S_tilt_command_pulse + S_step_y;
    wire [19:0] S_tilt_decreased_pulse =
        (S_tilt_command_pulse <= TILT_MIN_PULSE + S_step_y) ? TILT_MIN_PULSE :
        S_tilt_command_pulse - S_step_y;
    wire [19:0] S_next_tilt_pulse = !S_correct_y ? S_tilt_command_pulse :
        (S_increase_y ? S_tilt_increased_pulse : S_tilt_decreased_pulse);

    always @(posedge I_pixel_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_mailbox <= 24'd0;
            S_request <= 1'b0;
            S_ack_meta <= 1'b0;
            S_ack_sync <= 1'b0;
        end else begin
            S_ack_meta <= S_acknowledge;
            S_ack_sync <= S_ack_meta;
            if (I_result_update && (S_request == S_ack_sync)) begin
                S_mailbox <= {I_enable && I_measurement_valid &&
                    (I_target_color != 2'd0) &&
                    (I_center_x < 12'd1280) && (I_center_y < 11'd720),
                    I_center_x, I_center_y};
                S_request <= ~S_request;
            end
        end
    end

    always @(posedge I_clk_50m or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_enable_meta <= 1'b0;
            S_enable_sync <= 1'b0;
            S_request_meta <= 1'b0;
            S_request_sync <= 1'b0;
        end else begin
            S_enable_meta <= I_enable;
            S_enable_sync <= S_enable_meta;
            S_request_meta <= S_request;
            S_request_sync <= S_request_meta;
        end
    end

            // 每帧结果只接收一次；无效测量立即取消跟踪资格。
    always @(posedge I_clk_50m or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_acknowledge <= 1'b0;
            S_valid_frames <= 2'd0;
            S_previous_x <= CENTER_X;
            S_filtered_x <= CENTER_X;
            S_previous_y <= CENTER_Y;
            S_filtered_y <= CENTER_Y;
            S_result_age <= RESULT_TIMEOUT;
        end else begin
            if (S_new_result)
                S_acknowledge <= S_request_sync;

            if (!S_enable_sync) begin
                S_valid_frames <= 2'd0;
                S_result_age <= RESULT_TIMEOUT;
            end else if (S_new_result) begin
                S_result_age <= 23'd0;
                if (!S_mailbox_valid) begin
                    S_valid_frames <= 2'd0;
                end else begin
                    S_previous_x <= S_mailbox_x;
                    S_previous_y <= S_mailbox_y;
                    if ((S_valid_frames == 0) ||
                        (S_sample_delta > SAMPLE_JUMP_X) ||
                        (S_sample_delta_y > SAMPLE_JUMP_Y)) begin
                        S_valid_frames <= 2'd1;
                        S_filtered_x <= S_mailbox_x;
                        S_filtered_y <= S_mailbox_y;
                    end else begin
                        if (S_valid_frames != 2'd3)
                            S_valid_frames <= S_valid_frames + 1'b1;
                        if (S_mailbox_x >= S_filtered_x)
                            S_filtered_x <= S_filtered_x + S_filter_step;
                        else
                            S_filtered_x <= S_filtered_x - S_filter_step;
                        if (S_mailbox_y >= S_filtered_y)
                            S_filtered_y <= S_filtered_y + S_filter_step_y;
                        else
                            S_filtered_y <= S_filtered_y - S_filter_step_y;
                    end
                end
            end else if (S_result_age < RESULT_TIMEOUT) begin
                S_result_age <= S_result_age + 1'b1;
            end else begin
                S_valid_frames <= 2'd0;
            end
        end
    end

            // 只在20ms周期边界更新脉宽；K3只控制跟踪，俯仰轴始终输出PWM保持位置。
    always @(posedge I_clk_50m or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_counter <= 20'd0;
            S_command_pulse <= START_PULSE;
            S_active_pulse <= START_PULSE;
            S_tilt_command_pulse <= TILT_START_PULSE;
            S_tilt_active_pulse <= TILT_START_PULSE;
            S_control_periods <= 3'd0;
            O_pan_pwm <= 1'b0;
            O_tilt_pwm <= 1'b0;
        end else begin
            if (!S_enable_sync)
                S_control_periods <= 3'd0;

            if (S_counter == PERIOD_CYCLES - 1'b1) begin
                S_counter <= 20'd0;
                S_command_pulse <= S_next_pulse;
                S_active_pulse <= S_next_pulse;
                S_tilt_command_pulse <= S_next_tilt_pulse;
                S_tilt_active_pulse <= S_next_tilt_pulse;
                O_pan_pwm <= S_enable_sync;
                // 即使K3尚未开启，也输出90度初始脉宽，避免舵机停留在机械0度。
                O_tilt_pwm <= 1'b1;
                if (S_enable_sync) begin
                    if (S_control_periods == CONTROL_PERIODS - 1'b1)
                        S_control_periods <= 3'd0;
                    else
                        S_control_periods <= S_control_periods + 1'b1;
                end
            end else begin
                S_counter <= S_counter + 1'b1;
                if (S_counter == S_active_pulse - 1'b1)
                    O_pan_pwm <= 1'b0;
                if (S_counter == S_tilt_active_pulse - 1'b1)
                    O_tilt_pwm <= 1'b0;
            end
        end
    end
endmodule
