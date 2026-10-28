// 红、蓝、紫目标检测：RGB阈值 -> 3x3多数滤波 -> 16x8局部密度筛选
// -> 固定数量连通区域跟踪 -> 选择像素数最多的有效目标。
// 不使用整帧标记图和搜索队列，以控制FPGA资源与布局布线时间。
module YUNTAI_FLOWER_DETECT #(
    parameter integer IMAGE_WIDTH  = 1280,
    parameter integer IMAGE_HEIGHT = 720,

    // 红色条件：RGB分别落入范围，且R-G、R-B达到最小差值。
    // 当前参数按实测中心值R=203、G=100、B=77标定。
    parameter [7:0] RED_MIN        = 8'd165,
    parameter [7:0] RED_MAX        = 8'd240,
    parameter [7:0] GREEN_MIN      = 8'd65,
    parameter [7:0] GREEN_MAX      = 8'd135,
    parameter [7:0] BLUE_MIN       = 8'd45,
    parameter [7:0] BLUE_MAX       = 8'd110,
    parameter [8:0] RED_GREEN_GAP  = 9'd50,
    parameter [8:0] RED_BLUE_GAP   = 9'd65,

    // 蓝色条件：RGB范围+B-G、G-R、B-R差值；实测中心值约48/61/71。
    parameter [7:0] BLUE_FLOWER_R_MIN = 8'd30,
    parameter [7:0] BLUE_FLOWER_R_MAX = 8'd66,
    parameter [7:0] BLUE_FLOWER_G_MIN = 8'd43,
    parameter [7:0] BLUE_FLOWER_G_MAX = 8'd80,
    parameter [7:0] BLUE_FLOWER_B_MIN = 8'd52,
    parameter [7:0] BLUE_FLOWER_B_MAX = 8'd95,
    parameter [8:0] BLUE_BG_GAP       = 9'd5,
    parameter [8:0] BLUE_GR_GAP       = 9'd5,
    parameter [8:0] BLUE_BR_GAP       = 9'd12,

    // 黄色条件：RGB范围及红、绿通道相对蓝通道的最小差值；实测中心值为75/82/55。
    parameter [7:0] YELLOW_FLOWER_R_MIN = 8'd55,
    parameter [7:0] YELLOW_FLOWER_R_MAX = 8'd100,
    parameter [7:0] YELLOW_FLOWER_G_MIN = 8'd62,
    parameter [7:0] YELLOW_FLOWER_G_MAX = 8'd104,
    parameter [7:0] YELLOW_FLOWER_B_MIN = 8'd36,
    parameter [7:0] YELLOW_FLOWER_B_MAX = 8'd84,
    parameter [8:0] YELLOW_RB_GAP       = 9'd15,
    parameter [8:0] YELLOW_GB_GAP       = 9'd25,

    // 抗噪阈值：3x3窗口至少5点命中；16x8单元至少12点才参与连通域。
    // 前者去除孤立噪点，后者防止少量同色像素形成虚假目标。
    parameter [3:0] MASK_NEIGHBOR_MIN = 4'd5,
    parameter [7:0] MIN_CELL_PIXELS   = 8'd12,

    // 最终目标必须同时满足最小像素数、宽度和高度；多个目标取像素数最大者。
    parameter [19:0] MIN_TARGET_PIXELS = 20'd700,
    parameter [11:0] MIN_TARGET_WIDTH  = 12'd20,
    parameter [10:0] MIN_TARGET_HEIGHT = 11'd20,

    // 目标框稳定参数：边界变化不超过死区时保持原值，超出时取新旧坐标平均；
    // 连续漏检不超过LOST_FRAME_HOLD帧时继续显示旧框，但旧框不驱动舵机。
    parameter [11:0] BOX_DEADBAND_X  = 12'd16,
    parameter [10:0] BOX_DEADBAND_Y  = 11'd8,
    parameter [2:0]  LOST_FRAME_HOLD = 3'd3,

    // 以下候选匹配参数保留接口兼容；三种颜色当前均采用即时发布和统一平滑。
    parameter [3:0]  COLOR_CONFIRM_FRAMES = 4'd3, // 有效范围1~15
    parameter [2:0]  COLOR_LOST_FRAME_HOLD = 3'd6,
    parameter [11:0] COLOR_DEADBAND_X = 12'd24,
    parameter [10:0] COLOR_DEADBAND_Y = 11'd12,
    parameter [11:0] LOCK_MATCH_X = 12'd96,
    parameter [10:0] LOCK_MATCH_Y = 11'd64,
    parameter [11:0] CONFIRM_MATCH_X = 12'd48,
    parameter [10:0] CONFIRM_MATCH_Y = 11'd32,
    // RED_ONLY=1仅用于单独调试红色；正常三色识别必须保持为0。
    parameter RED_ONLY = 1'b0
) (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_vsync,
    input  wire        I_de,
    input  wire [23:0] I_rgb,

    output reg         O_target_valid,
    output reg  [11:0] O_x_min,
    output reg  [11:0] O_x_max,
    output reg  [10:0] O_y_min,
    output reg  [10:0] O_y_max,
    output reg  [11:0] O_center_x,
    output reg  [10:0] O_center_y,
    output reg  [19:0] O_target_pixels,
    // 颜色编码：0无目标，1红色，2蓝色，3黄色。
    output reg  [1:0]  O_target_color,
    // 舵机只使用帧完成时的新测量，不使用为显示稳定而保留的旧目标框。
    output reg         O_result_update,
    output reg         O_measurement_valid
);

    localparam integer CELL_COLUMNS = (IMAGE_WIDTH + 15) / 16;
    localparam integer TRACK_COUNT  = 12;
    localparam [1:0] COLOR_RED = 2'd1;
    localparam [1:0] COLOR_BLUE = 2'd2;
    localparam [1:0] COLOR_YELLOW = 2'd3;

    localparam [2:0] PS_IDLE     = 3'd0;
    localparam [2:0] PS_CLEAN    = 3'd1;
    localparam [2:0] PS_SCAN     = 3'd2;
    localparam [2:0] PS_FINALIZE = 3'd3;
    localparam [2:0] PS_PUBLISH  = 3'd4;

    wire [7:0] S_red;
    wire [7:0] S_green;
    wire [7:0] S_blue;
    wire [2:0] S_raw_mask;

    assign S_red   = I_rgb[23:16];
    assign S_green = I_rgb[15:8];
    assign S_blue  = I_rgb[7:0];

    assign S_raw_mask[0] = I_de &&
                        (S_red >= RED_MIN) &&
                        (S_red <= RED_MAX) &&
                        (S_green >= GREEN_MIN) &&
                        (S_green <= GREEN_MAX) &&
                        (S_blue >= BLUE_MIN) &&
                        (S_blue <= BLUE_MAX) &&
                        ({1'b0, S_red} >= ({1'b0, S_green} + RED_GREEN_GAP)) &&
                        ({1'b0, S_red} >= ({1'b0, S_blue}  + RED_BLUE_GAP));

    assign S_raw_mask[1] = !RED_ONLY && I_de &&
        (S_red >= BLUE_FLOWER_R_MIN) && (S_red <= BLUE_FLOWER_R_MAX) &&
        (S_green >= BLUE_FLOWER_G_MIN) && (S_green <= BLUE_FLOWER_G_MAX) &&
        (S_blue >= BLUE_FLOWER_B_MIN) && (S_blue <= BLUE_FLOWER_B_MAX) &&
        ({1'b0, S_blue} >= ({1'b0, S_green} + BLUE_BG_GAP)) &&
        ({1'b0, S_green} >= ({1'b0, S_red} + BLUE_GR_GAP)) &&
        ({1'b0, S_blue} >= ({1'b0, S_red} + BLUE_BR_GAP));

    assign S_raw_mask[2] = !RED_ONLY && I_de &&
        (S_red >= YELLOW_FLOWER_R_MIN) && (S_red <= YELLOW_FLOWER_R_MAX) &&
        (S_green >= YELLOW_FLOWER_G_MIN) && (S_green <= YELLOW_FLOWER_G_MAX) &&
        (S_blue >= YELLOW_FLOWER_B_MIN) && (S_blue <= YELLOW_FLOWER_B_MAX) &&
        ({1'b0, S_red} >= ({1'b0, S_blue} + YELLOW_RB_GAP)) &&
        ({1'b0, S_green} >= ({1'b0, S_blue} + YELLOW_GB_GAP));

    reg [11:0] S_x;
    reg [10:0] S_y;
    // 直接维护中心在x-1处的16像素统计列，避免关键路径上再做坐标减法和移位。
    reg [6:0]  S_filter_cell_column;
    reg        S_de_d;
    reg        S_vsync_d;

    // 三种颜色分别滤波，防止不同颜色像素被合并为同一目标。
    reg [2:0] S_line_1 [0:IMAGE_WIDTH-1];
    reg [2:0] S_line_2 [0:IMAGE_WIDTH-1];
    reg [2:0] S_top_left;
    reg [2:0] S_top_mid;
    reg [2:0] S_middle_left;
    reg [2:0] S_middle_mid;
    reg [2:0] S_bottom_left;
    reg [2:0] S_bottom_mid;

    wire [2:0] S_previous_1;
    wire [2:0] S_previous_2;
    wire [2:0] S_filtered_mask;

    assign S_previous_1 = S_line_1[S_x];
    assign S_previous_2 = S_line_2[S_x];
    function F_majority;
        input [8:0] I_window;
        reg [3:0] S_sum;
        integer S_bit;
        begin
            S_sum = 4'd0;
            for (S_bit = 0; S_bit < 9; S_bit = S_bit + 1)
                S_sum = S_sum + {3'd0, I_window[S_bit]};
            F_majority = (S_sum >= MASK_NEIGHBOR_MIN);
        end
    endfunction

    genvar S_color_bit;
    generate
        for (S_color_bit = 0; S_color_bit < 3; S_color_bit = S_color_bit + 1) begin: G_MASK_FILTER
            assign S_filtered_mask[S_color_bit] = F_majority({
                S_top_left[S_color_bit], S_top_mid[S_color_bit], S_previous_2[S_color_bit],
                S_middle_left[S_color_bit], S_middle_mid[S_color_bit], S_previous_1[S_color_bit],
                S_bottom_left[S_color_bit], S_bottom_mid[S_color_bit], S_raw_mask[S_color_bit]});
        end
    endgenerate

    // 每16列设置一个计数器，每处理完8行复用一次。
    reg [7:0] S_cell_count [0:CELL_COLUMNS-1];
    reg [7:0] S_blue_cell_count [0:CELL_COLUMNS-1];
    reg [7:0] S_yellow_cell_count [0:CELL_COLUMNS-1];
    integer S_reset_index;

    // 使用12个固定区域跟踪槽，避免大规模连通域标记存储器。
    reg        S_track_active [0:TRACK_COUNT-1];
    reg [1:0]  S_track_color [0:TRACK_COUNT-1];
    reg [6:0]  S_track_last_row [0:TRACK_COUNT-1];
    reg [6:0]  S_track_row_x_min [0:TRACK_COUNT-1];
    reg [6:0]  S_track_row_x_max [0:TRACK_COUNT-1];
    reg [6:0]  S_track_box_x_min [0:TRACK_COUNT-1];
    reg [6:0]  S_track_box_x_max [0:TRACK_COUNT-1];
    reg [6:0]  S_track_box_y_min [0:TRACK_COUNT-1];
    reg [6:0]  S_track_box_y_max [0:TRACK_COUNT-1];
    reg [19:0] S_track_pixels [0:TRACK_COUNT-1];

    reg [2:0] S_process_state;
    reg [6:0] S_process_column;
    reg [6:0] S_process_row;
    reg [3:0] S_process_track;
    reg [1:0] S_process_color;
    wire [7:0] S_process_pixels;
    assign S_process_pixels = (S_process_color == COLOR_RED) ?
        S_cell_count[S_process_column] :
        (S_process_color == COLOR_BLUE) ? S_blue_cell_count[S_process_column] :
        S_yellow_cell_count[S_process_column];

    reg       S_match_found;
    reg [3:0] S_match_index;
    reg       S_free_found;
    reg [3:0] S_free_index;
    integer S_search_index;

    // 当前单元与同一行相邻或与上一单元行重叠时，归入同一连通区域。
    always @(*) begin
        S_match_found = 1'b0;
        S_match_index = 4'd0;
        S_free_found = 1'b0;
        S_free_index = 4'd0;

        for (S_search_index = 0;
             S_search_index < TRACK_COUNT;
             S_search_index = S_search_index + 1) begin
            if (!S_free_found && !S_track_active[S_search_index]) begin
                S_free_found = 1'b1;
                S_free_index = S_search_index;
            end

            if (!S_match_found && S_track_active[S_search_index] &&
                (S_track_color[S_search_index] == S_process_color)) begin
                if ((S_track_last_row[S_search_index] == S_process_row) &&
                    (S_process_column <=
                     (S_track_row_x_max[S_search_index] + 1'b1))) begin
                    S_match_found = 1'b1;
                    S_match_index = S_search_index;
                end else if (((S_track_last_row[S_search_index] + 1'b1) ==
                              S_process_row) &&
                             ((S_process_column + 1'b1) >=
                              S_track_row_x_min[S_search_index]) &&
                             (S_process_column <=
                              (S_track_row_x_max[S_search_index] + 1'b1))) begin
                    S_match_found = 1'b1;
                    S_match_index = S_search_index;
                end
            end
        end
    end

    reg [19:0] S_best_pixels;
    reg [1:0]  S_best_color;
    reg        S_best_locked;
    reg [6:0]  S_best_x_min;
    reg [6:0]  S_best_x_max;
    reg [6:0]  S_best_y_min;
    reg [6:0]  S_best_y_max;
    reg [2:0]  S_missed_frames;

    // 候选缓存只保存一个区域的边界和帧数，不缓存整帧图像。
    reg [3:0]  S_pending_frames;
    reg [1:0]  S_pending_color;
    reg [11:0] S_pending_x_min;
    reg [11:0] S_pending_x_max;
    reg [10:0] S_pending_y_min;
    reg [10:0] S_pending_y_max;
    wire       S_pending_matches;
    wire       S_candidate_confirmed;
    wire       S_publish_accept;
    wire [2:0] S_hold_frames;
    wire [11:0] S_deadband_x;
    wire [10:0] S_deadband_y;
    wire [1:0] S_smooth_shift;

    wire [11:0] S_raw_x_min;
    wire [11:0] S_raw_x_max;
    wire [10:0] S_raw_y_min;
    wire [10:0] S_raw_y_max;
    wire [11:0] S_x_min_delta;
    wire [11:0] S_x_max_delta;
    wire [10:0] S_y_min_delta;
    wire [10:0] S_y_max_delta;
    wire [11:0] S_stable_x_min;
    wire [11:0] S_stable_x_max;
    wire [10:0] S_stable_y_min;
    wire [10:0] S_stable_y_max;

    assign S_raw_x_min = {1'b0, S_best_x_min, 4'b0};
    assign S_raw_x_max = {1'b0, S_best_x_max, 4'b0} + 12'd15;
    assign S_raw_y_min = {1'b0, S_best_y_min, 3'b0};
    assign S_raw_y_max = {1'b0, S_best_y_max, 3'b0} + 11'd7;

    assign S_x_min_delta = (S_raw_x_min >= O_x_min) ?
                           (S_raw_x_min - O_x_min) : (O_x_min - S_raw_x_min);
    assign S_x_max_delta = (S_raw_x_max >= O_x_max) ?
                           (S_raw_x_max - O_x_max) : (O_x_max - S_raw_x_max);
    assign S_y_min_delta = (S_raw_y_min >= O_y_min) ?
                           (S_raw_y_min - O_y_min) : (O_y_min - S_raw_y_min);
    assign S_y_max_delta = (S_raw_y_max >= O_y_max) ?
                           (S_raw_y_max - O_y_max) : (O_y_max - S_raw_y_max);

    // 三种颜色共用死区、二分之一平滑和丢失保持，保证跟踪响应一致。
    assign S_deadband_x = BOX_DEADBAND_X;
    assign S_deadband_y = BOX_DEADBAND_Y;
    assign S_smooth_shift = 2'd1;
    assign S_hold_frames = LOST_FRAME_HOLD;

    assign S_stable_x_min = (S_x_min_delta <= S_deadband_x) ? O_x_min :
        (({1'b0, O_x_min} + {1'b0, S_raw_x_min}) >> 1);
    assign S_stable_x_max = (S_x_max_delta <= S_deadband_x) ? O_x_max :
        (({1'b0, O_x_max} + {1'b0, S_raw_x_max}) >> 1);
    assign S_stable_y_min = (S_y_min_delta <= S_deadband_y) ? O_y_min :
        (({1'b0, O_y_min} + {1'b0, S_raw_y_min}) >> 1);
    assign S_stable_y_max = (S_y_max_delta <= S_deadband_y) ? O_y_max :
        (({1'b0, O_y_max} + {1'b0, S_raw_y_max}) >> 1);

    // 候选匹配比较四条边界而不是只比较中心，扩展位宽防止边缘坐标溢出。
    assign S_pending_matches = (S_pending_frames != 0) &&
        (S_pending_color == S_best_color) &&
        ({1'b0, S_raw_x_min} <= ({1'b0, S_pending_x_min} + {1'b0, CONFIRM_MATCH_X})) &&
        ({1'b0, S_pending_x_min} <= ({1'b0, S_raw_x_min} + {1'b0, CONFIRM_MATCH_X})) &&
        ({1'b0, S_raw_x_max} <= ({1'b0, S_pending_x_max} + {1'b0, CONFIRM_MATCH_X})) &&
        ({1'b0, S_pending_x_max} <= ({1'b0, S_raw_x_max} + {1'b0, CONFIRM_MATCH_X})) &&
        ({1'b0, S_raw_y_min} <= ({1'b0, S_pending_y_min} + {1'b0, CONFIRM_MATCH_Y})) &&
        ({1'b0, S_pending_y_min} <= ({1'b0, S_raw_y_min} + {1'b0, CONFIRM_MATCH_Y})) &&
        ({1'b0, S_raw_y_max} <= ({1'b0, S_pending_y_max} + {1'b0, CONFIRM_MATCH_Y})) &&
        ({1'b0, S_pending_y_max} <= ({1'b0, S_raw_y_max} + {1'b0, CONFIRM_MATCH_Y}));
    assign S_candidate_confirmed = (COLOR_CONFIRM_FRAMES <= 4'd1) ||
        (S_pending_matches &&
         (({1'b0, S_pending_frames} + 5'd1) >= {1'b0, COLOR_CONFIRM_FRAMES}));
    // 三种颜色都立即接收当前帧中像素数最多的合格区域。
    assign S_publish_accept = (S_best_pixels != 0);

    wire [11:0] S_process_width;
    wire [10:0] S_process_height;
    wire        S_process_qualifies;
    wire        S_process_locked;
    wire        S_process_take;
    wire [11:0] S_process_x_min;
    wire [11:0] S_process_x_max;
    wire [10:0] S_process_y_min;
    wire [10:0] S_process_y_max;

    assign S_process_x_min = {1'b0, S_track_box_x_min[S_process_track], 4'b0};
    assign S_process_x_max = {1'b0, S_track_box_x_max[S_process_track], 4'b0} + 12'd15;
    assign S_process_y_min = {1'b0, S_track_box_y_min[S_process_track], 3'b0};
    assign S_process_y_max = {1'b0, S_track_box_y_max[S_process_track], 3'b0} + 11'd7;
    assign S_process_locked = 1'b0;
    // 红、蓝、紫统一按合格区域的像素数选取最大目标。
    assign S_process_take = S_process_qualifies &&
        (S_track_pixels[S_process_track] > S_best_pixels);

    assign S_process_width =
        ({1'b0, S_track_box_x_max[S_process_track], 4'b0} + 12'd16) -
         {1'b0, S_track_box_x_min[S_process_track], 4'b0};
    assign S_process_height =
        ({1'b0, S_track_box_y_max[S_process_track], 3'b0} + 11'd8) -
         {1'b0, S_track_box_y_min[S_process_track], 3'b0};
    assign S_process_qualifies =
        S_track_active[S_process_track] &&
        (S_track_pixels[S_process_track] >= MIN_TARGET_PIXELS) &&
        (S_process_width >= MIN_TARGET_WIDTH) &&
        (S_process_height >= MIN_TARGET_HEIGHT);

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_x <= 12'd0;
            S_y <= 11'd0;
            S_filter_cell_column <= 7'd0;
            S_de_d <= 1'b0;
            S_vsync_d <= 1'b0;

            S_top_left <= 1'b0;
            S_top_mid <= 1'b0;
            S_middle_left <= 1'b0;
            S_middle_mid <= 1'b0;
            S_bottom_left <= 1'b0;
            S_bottom_mid <= 1'b0;

            for (S_reset_index = 0;
                 S_reset_index < CELL_COLUMNS;
                 S_reset_index = S_reset_index + 1) begin
                S_cell_count[S_reset_index] <= 8'd0;
                S_blue_cell_count[S_reset_index] <= 8'd0;
                S_yellow_cell_count[S_reset_index] <= 8'd0;
            end

            for (S_reset_index = 0;
                 S_reset_index < TRACK_COUNT;
                 S_reset_index = S_reset_index + 1) begin
                S_track_active[S_reset_index] <= 1'b0;
                S_track_color[S_reset_index] <= 2'd0;
                S_track_last_row[S_reset_index] <= 7'd0;
                S_track_row_x_min[S_reset_index] <= 7'd0;
                S_track_row_x_max[S_reset_index] <= 7'd0;
                S_track_box_x_min[S_reset_index] <= 7'd0;
                S_track_box_x_max[S_reset_index] <= 7'd0;
                S_track_box_y_min[S_reset_index] <= 7'd0;
                S_track_box_y_max[S_reset_index] <= 7'd0;
                S_track_pixels[S_reset_index] <= 20'd0;
            end

            S_process_state <= PS_IDLE;
            S_process_column <= 7'd0;
            S_process_row <= 7'd0;
            S_process_track <= 4'd0;
            S_process_color <= COLOR_RED;

            S_best_pixels <= 20'd0;
            S_best_color <= 2'd0;
            S_best_locked <= 1'b0;
            S_best_x_min <= 7'd0;
            S_best_x_max <= 7'd0;
            S_best_y_min <= 7'd0;
            S_best_y_max <= 7'd0;
            S_missed_frames <= 3'd0;
            S_pending_frames <= 4'd0;
            S_pending_color <= 2'd0;
            S_pending_x_min <= 12'd0;
            S_pending_x_max <= 12'd0;
            S_pending_y_min <= 11'd0;
            S_pending_y_max <= 11'd0;

            O_target_valid <= 1'b0;
            O_x_min <= 12'd0;
            O_x_max <= 12'd0;
            O_y_min <= 11'd0;
            O_y_max <= 11'd0;
            O_center_x <= IMAGE_WIDTH / 2;
            O_center_y <= IMAGE_HEIGHT / 2;
            O_target_pixels <= 20'd0;
            O_target_color <= 2'd0;
            O_result_update <= 1'b0;
            O_measurement_valid <= 1'b0;
        end else begin
            O_result_update <= 1'b0;
            O_measurement_valid <= 1'b0;
            S_de_d <= I_de;
            S_vsync_d <= I_vsync;

            if (I_vsync) begin
                S_x <= 12'd0;
                S_y <= 11'd0;
                S_filter_cell_column <= 7'd0;
            end else if (I_de) begin
                if (S_x < IMAGE_WIDTH - 1)
                    S_x <= S_x + 1'b1;
                // x=16时窗口中心仍为15；非阻塞更新后，x=17开始进入下一统计列。
                if ((S_x != 0) && (S_x[3:0] == 4'd0))
                    S_filter_cell_column <= S_filter_cell_column + 1'b1;
            end else begin
                S_x <= 12'd0;
                S_filter_cell_column <= 7'd0;
                if (S_de_d && (S_y < IMAGE_HEIGHT))
                    S_y <= S_y + 1'b1;
            end

            // 移动3x3二值窗口，滤波结果对应中心坐标(x-1,y-1)。
            if (I_de) begin
                S_line_2[S_x] <= S_line_1[S_x];
                S_line_1[S_x] <= S_raw_mask;

                if (S_x == 0) begin
                    S_top_left <= 1'b0;
                    S_top_mid <= S_previous_2;
                    S_middle_left <= 1'b0;
                    S_middle_mid <= S_previous_1;
                    S_bottom_left <= 1'b0;
                    S_bottom_mid <= S_raw_mask;
                end else begin
                    S_top_left <= S_top_mid;
                    S_top_mid <= S_previous_2;
                    S_middle_left <= S_middle_mid;
                    S_middle_mid <= S_previous_1;
                    S_bottom_left <= S_bottom_mid;
                    S_bottom_mid <= S_raw_mask;
                end

                if ((S_x >= 2) && (S_y >= 2)) begin
                    if (S_filtered_mask[0] &&
                        (S_cell_count[S_filter_cell_column] != 8'hff))
                        S_cell_count[S_filter_cell_column] <=
                            S_cell_count[S_filter_cell_column] + 1'b1;
                    if (S_filtered_mask[1] &&
                        (S_blue_cell_count[S_filter_cell_column] != 8'hff))
                        S_blue_cell_count[S_filter_cell_column] <=
                            S_blue_cell_count[S_filter_cell_column] + 1'b1;
                    if (S_filtered_mask[2] &&
                        (S_yellow_cell_count[S_filter_cell_column] != 8'hff))
                        S_yellow_cell_count[S_filter_cell_column] <=
                            S_yellow_cell_count[S_filter_cell_column] + 1'b1;
                end
            end

            // 每8行利用水平消隐处理80列密度计数，并清理已结束区域。
            if (S_de_d && !I_de && (S_y >= 2) &&
                ((((S_y - 1'b1) & 11'd7) == 11'd7) ||
                 (S_y == IMAGE_HEIGHT - 1))) begin
                S_process_state <= PS_CLEAN;
                S_process_track <= 4'd0;
                S_process_column <= 7'd0;
                S_process_color <= COLOR_RED;
                S_process_row <= (S_y - 1'b1) >> 3;
            end else begin
                case (S_process_state)
                    PS_CLEAN: begin
                        // 未延伸到上一单元行的区域已经结束，若更大则记录为当前最佳目标。
                        if (S_track_active[S_process_track] &&
                            ((S_track_last_row[S_process_track] + 1'b1) <
                             S_process_row)) begin
                            if (S_process_take) begin
                                S_best_pixels <=
                                    S_track_pixels[S_process_track];
                                S_best_color <= S_track_color[S_process_track];
                                S_best_locked <= S_process_locked;
                                S_best_x_min <=
                                    S_track_box_x_min[S_process_track];
                                S_best_x_max <=
                                    S_track_box_x_max[S_process_track];
                                S_best_y_min <=
                                    S_track_box_y_min[S_process_track];
                                S_best_y_max <=
                                    S_track_box_y_max[S_process_track];
                            end
                            S_track_active[S_process_track] <= 1'b0;
                        end

                        if (S_process_track == TRACK_COUNT - 1) begin
                            S_process_track <= 4'd0;
                            S_process_state <= PS_SCAN;
                        end else begin
                            S_process_track <= S_process_track + 1'b1;
                        end
                    end

                    PS_SCAN: begin
                        if (S_process_pixels >= MIN_CELL_PIXELS) begin
                            if (S_match_found) begin
                                S_track_pixels[S_match_index] <=
                                    S_track_pixels[S_match_index] +
                                    S_process_pixels;

                                if (S_track_last_row[S_match_index] ==
                                    S_process_row) begin
                                    S_track_row_x_max[S_match_index] <=
                                        S_process_column;
                                end else begin
                                    S_track_last_row[S_match_index] <=
                                        S_process_row;
                                    S_track_row_x_min[S_match_index] <=
                                        S_process_column;
                                    S_track_row_x_max[S_match_index] <=
                                        S_process_column;
                                end

                                if (S_process_column <
                                    S_track_box_x_min[S_match_index])
                                    S_track_box_x_min[S_match_index] <=
                                        S_process_column;
                                if (S_process_column >
                                    S_track_box_x_max[S_match_index])
                                    S_track_box_x_max[S_match_index] <=
                                        S_process_column;
                                S_track_box_y_max[S_match_index] <=
                                    S_process_row;
                            end else if (S_free_found) begin
                                S_track_active[S_free_index] <= 1'b1;
                                S_track_color[S_free_index] <= S_process_color;
                                S_track_last_row[S_free_index] <=
                                    S_process_row;
                                S_track_row_x_min[S_free_index] <=
                                    S_process_column;
                                S_track_row_x_max[S_free_index] <=
                                    S_process_column;
                                S_track_box_x_min[S_free_index] <=
                                    S_process_column;
                                S_track_box_x_max[S_free_index] <=
                                    S_process_column;
                                S_track_box_y_min[S_free_index] <=
                                    S_process_row;
                                S_track_box_y_max[S_free_index] <=
                                    S_process_row;
                                S_track_pixels[S_free_index] <=
                                    S_process_pixels;
                            end
                        end

                        // 三种颜色共扫描240拍，再用12拍清理，均在720p行消隐内完成。
                        case (S_process_color)
                            COLOR_RED: S_cell_count[S_process_column] <= 8'd0;
                            COLOR_BLUE: S_blue_cell_count[S_process_column] <= 8'd0;
                            COLOR_YELLOW: S_yellow_cell_count[S_process_column] <= 8'd0;
                            default: begin end
                        endcase
                        if (S_process_color != COLOR_YELLOW) begin
                            S_process_color <= S_process_color + 1'b1;
                        end else begin
                            S_process_color <= COLOR_RED;
                            if (S_process_column == CELL_COLUMNS - 1) begin
                                S_process_column <= 7'd0;
                                S_process_state <= PS_IDLE;
                            end else begin
                                S_process_column <= S_process_column + 1'b1;
                            end
                        end
                    end

                    PS_FINALIZE: begin
                        // 帧消隐期间结束并评估本帧剩余的所有区域。
                        if (S_process_take) begin
                            S_best_pixels <=
                                S_track_pixels[S_process_track];
                            S_best_color <= S_track_color[S_process_track];
                            S_best_locked <= S_process_locked;
                            S_best_x_min <=
                                S_track_box_x_min[S_process_track];
                            S_best_x_max <=
                                S_track_box_x_max[S_process_track];
                            S_best_y_min <=
                                S_track_box_y_min[S_process_track];
                            S_best_y_max <=
                                S_track_box_y_max[S_process_track];
                        end
                        S_track_active[S_process_track] <= 1'b0;

                        if (S_process_track == TRACK_COUNT - 1) begin
                            S_process_track <= 4'd0;
                            S_process_state <= PS_PUBLISH;
                        end else begin
                            S_process_track <= S_process_track + 1'b1;
                        end
                    end

                    PS_PUBLISH: begin
                        O_result_update <= 1'b1;
                        O_measurement_valid <= S_publish_accept;
                        // 保留候选帧状态供兼容使用；当前即时发布路径会在此处清零。
                        if ((S_best_pixels == 0) || S_publish_accept) begin
                            S_pending_frames <= 4'd0;
                            S_pending_color <= 2'd0;
                        end else begin
                            if (S_pending_matches) begin
                                if (S_pending_frames != 4'hf)
                                    S_pending_frames <= S_pending_frames + 1'b1;
                            end else begin
                                S_pending_frames <= 4'd1;
                            end
                            S_pending_color <= S_best_color;
                            S_pending_x_min <= S_raw_x_min;
                            S_pending_x_max <= S_raw_x_max;
                            S_pending_y_min <= S_raw_y_min;
                            S_pending_y_max <= S_raw_y_max;
                        end

                        if (S_publish_accept) begin
                            O_target_valid <= 1'b1;
                            O_target_pixels <= S_best_pixels;
                            O_target_color <= S_best_color;
                            S_missed_frames <= 3'd0;

                            // 首次捕获或颜色改变时直接采用测量框，其余情况使用统一平滑。
                            if (!O_target_valid ||
                                (O_target_color != S_best_color)) begin
                                O_x_min <= S_raw_x_min;
                                O_x_max <= S_raw_x_max;
                                O_y_min <= S_raw_y_min;
                                O_y_max <= S_raw_y_max;
                                O_center_x <= (S_raw_x_min + S_raw_x_max) >> 1;
                                O_center_y <= (S_raw_y_min + S_raw_y_max) >> 1;
                            end else begin
                                O_x_min <= S_stable_x_min;
                                O_x_max <= S_stable_x_max;
                                O_y_min <= S_stable_y_min;
                                O_y_max <= S_stable_y_max;
                                O_center_x <=
                                    (S_stable_x_min + S_stable_x_max) >> 1;
                                O_center_y <=
                                    (S_stable_y_min + S_stable_y_max) >> 1;
                            end
                        end else if (O_target_valid &&
                                     (S_missed_frames < S_hold_frames)) begin
                            // 短暂曝光变化导致漏检时保留旧框，避免画面闪烁和舵机跳变。
                            S_missed_frames <= S_missed_frames + 1'b1;
                        end else begin
                            O_target_valid <= 1'b0;
                            O_target_pixels <= 20'd0;
                            O_target_color <= 2'd0;
                            S_missed_frames <= 3'd0;
                            O_x_min <= 12'd0;
                            O_x_max <= 12'd0;
                            O_y_min <= 11'd0;
                            O_y_max <= 11'd0;
                            O_center_x <= IMAGE_WIDTH / 2;
                            O_center_y <= IMAGE_HEIGHT / 2;
                        end

                        S_best_pixels <= 20'd0;
                        S_best_color <= 2'd0;
                        S_best_locked <= 1'b0;
                        S_process_state <= PS_IDLE;
                    end

                    default: begin
                        S_process_state <= PS_IDLE;
                    end
                endcase
            end

            if (I_vsync && !S_vsync_d) begin
                S_process_state <= PS_FINALIZE;
                S_process_track <= 4'd0;
            end
        end
    end

endmodule
