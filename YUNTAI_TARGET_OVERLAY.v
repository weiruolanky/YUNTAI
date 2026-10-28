// K3切换智能跟踪显示：叠加目标框、中心标记、识别颜色和中心坐标。
module YUNTAI_TARGET_OVERLAY #(
    parameter integer IMAGE_WIDTH  = 1280,
    parameter integer IMAGE_HEIGHT = 720
) (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_key3_n,
    input  wire        I_vsync,
    input  wire        I_de,
    input  wire [23:0] I_rgb,
    input  wire        I_target_valid,
    input  wire [1:0]  I_target_color, // 1红色，2蓝色，3黄色
    input  wire [11:0] I_x_min,
    input  wire [11:0] I_x_max,
    input  wire [10:0] I_y_min,
    input  wire [10:0] I_y_max,
    input  wire [11:0] I_center_x,
    input  wire [10:0] I_center_y,
    input  wire [11:0] I_latency_ms,
    input  wire        I_latency_valid,
    input  wire [19:0] I_motion_pixels,
    input  wire        I_motion_valid,
    input  wire        I_motion_detected,
    output wire [23:0] O_rgb,
    output reg         O_overlay_enable
);

    localparam [20:0] DEBOUNCE_LIMIT = 21'd1_499_999; // 75MHz下消抖20ms
    localparam [11:0] TEXT_X = 12'd1024;
    localparam [10:0] TEXT_Y = 11'd24;
    localparam [23:0] NO_TARGET_COLOR = 24'hff3030;
    localparam integer LABEL_CHARS = 17;
    localparam integer LABEL_WIDTH = LABEL_CHARS * 16;

    reg        S_key_meta;
    reg        S_key_sync;
    reg        S_key_stable;
    reg [20:0] S_key_count;
    wire       S_key_press;

    reg [11:0] S_x;
    reg [10:0] S_y;
    reg        S_de_d;

    // 在有效像素通路之外用12拍Double-Dabble把中心坐标转成BCD，避免除法器。
    reg        S_vsync_d;
    reg [3:0]  S_decimal_steps;
    reg [11:0] S_binary_x;
    reg [11:0] S_binary_y;
    reg [15:0] S_decimal_x_work;
    reg [15:0] S_decimal_y_work;
    reg [15:0] S_decimal_x;
    reg [15:0] S_decimal_y;
    reg [11:0] S_binary_latency;
    reg [15:0] S_decimal_latency_work;
    reg [15:0] S_decimal_latency;
    reg        S_coordinate_valid;
    reg        S_latency_valid;
    reg [15:0] S_decimal_x_adjusted;
    reg [15:0] S_decimal_y_adjusted;
    reg [15:0] S_decimal_latency_adjusted;
    wire [11:0] S_latency_ms;
    integer S_digit_index;

    wire S_box_pixel;
    wire S_center_pixel;
    wire S_status_background;
    wire S_text_window;
    wire [11:0] S_text_dx;
    wire [10:0] S_text_dy;
    wire [3:0] S_char_index;
    wire [2:0] S_glyph_x;
    wire [2:0] S_glyph_y;
    reg  [7:0] S_char_code;
    wire [4:0] S_glyph_row;
    wire       S_text_pixel;
    wire       S_coordinate_row;
    wire       S_latency_row;
    wire       S_motion_row;
    wire [23:0] S_status_color;
    wire [23:0] S_target_box_color;
    wire        S_label_window;
    wire        S_label_background;
    wire [11:0] S_label_x;
    wire [10:0] S_label_y;
    wire [11:0] S_label_dx;
    wire [10:0] S_label_dy;
    wire [4:0]  S_label_char_index;
    wire [2:0]  S_label_glyph_x;
    wire [2:0]  S_label_glyph_y;
    reg  [7:0]  S_label_char_code;
    wire [4:0]  S_label_glyph_row;
    wire        S_label_text_pixel;

    assign S_key_press = S_key_stable && !S_key_sync &&
                         (S_key_count == DEBOUNCE_LIMIT);

    // 标签格式：RED(x,y)、BLUE(x,y) 或 YELLOW(x,y)。坐标使用当前帧中心坐标。
    always @(*) begin
        S_label_char_code = 8'h20;
        case (I_target_color)
            2'd1: begin
                case (S_label_char_index)
                    5'd0:  S_label_char_code = "R";
                    5'd1:  S_label_char_code = "E";
                    5'd2:  S_label_char_code = "D";
                    5'd3:  S_label_char_code = "(";
                    5'd4:  S_label_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_x[15:12]}) : "-";
                    5'd5:  S_label_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_x[11:8]})  : "-";
                    5'd6:  S_label_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_x[7:4]})   : "-";
                    5'd7:  S_label_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_x[3:0]})   : "-";
                    5'd8:  S_label_char_code = ",";
                    5'd9:  S_label_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_y[15:12]}) : "-";
                    5'd10: S_label_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_y[11:8]})  : "-";
                    5'd11: S_label_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_y[7:4]})   : "-";
                    5'd12: S_label_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_y[3:0]})   : "-";
                    5'd13: S_label_char_code = ")";
                    default: S_label_char_code = 8'h20;
                endcase
            end
            2'd2: begin
                case (S_label_char_index)
                    5'd0:  S_label_char_code = "B";
                    5'd1:  S_label_char_code = "L";
                    5'd2:  S_label_char_code = "U";
                    5'd3:  S_label_char_code = "E";
                    5'd4:  S_label_char_code = "(";
                    5'd5:  S_label_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_x[15:12]}) : "-";
                    5'd6:  S_label_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_x[11:8]})  : "-";
                    5'd7:  S_label_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_x[7:4]})   : "-";
                    5'd8:  S_label_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_x[3:0]})   : "-";
                    5'd9:  S_label_char_code = ",";
                    5'd10: S_label_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_y[15:12]}) : "-";
                    5'd11: S_label_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_y[11:8]})  : "-";
                    5'd12: S_label_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_y[7:4]})   : "-";
                    5'd13: S_label_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_y[3:0]})   : "-";
                    5'd14: S_label_char_code = ")";
                    default: S_label_char_code = 8'h20;
                endcase
            end
            2'd3: begin
                case (S_label_char_index)
                    5'd0:  S_label_char_code = "Y";
                    5'd1:  S_label_char_code = "E";
                    5'd2:  S_label_char_code = "L";
                    5'd3:  S_label_char_code = "L";
                    5'd4:  S_label_char_code = "O";
                    5'd5:  S_label_char_code = "W";
                    5'd6:  S_label_char_code = "(";
                    5'd7:  S_label_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_x[15:12]}) : "-";
                    5'd8:  S_label_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_x[11:8]})  : "-";
                    5'd9:  S_label_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_x[7:4]})   : "-";
                    5'd10: S_label_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_x[3:0]})   : "-";
                    5'd11: S_label_char_code = ",";
                    5'd12: S_label_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_y[15:12]}) : "-";
                    5'd13: S_label_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_y[11:8]})  : "-";
                    5'd14: S_label_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_y[7:4]})   : "-";
                    5'd15: S_label_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_y[3:0]})   : "-";
                    5'd16: S_label_char_code = ")";
                    default: S_label_char_code = 8'h20;
                endcase
            end
            default: S_label_char_code = 8'h20;
        endcase
    end

    always @(*) begin
        S_decimal_x_adjusted = S_decimal_x_work;
        S_decimal_y_adjusted = S_decimal_y_work;
        S_decimal_latency_adjusted = S_decimal_latency_work;
        for (S_digit_index = 0; S_digit_index < 4;
             S_digit_index = S_digit_index + 1) begin
            if (S_decimal_x_work[S_digit_index*4 +: 4] >= 4'd5)
                S_decimal_x_adjusted[S_digit_index*4 +: 4] =
                    S_decimal_x_work[S_digit_index*4 +: 4] + 4'd3;
            if (S_decimal_y_work[S_digit_index*4 +: 4] >= 4'd5)
                S_decimal_y_adjusted[S_digit_index*4 +: 4] =
                    S_decimal_y_work[S_digit_index*4 +: 4] + 4'd3;
            if (S_decimal_latency_work[S_digit_index*4 +: 4] >= 4'd5)
                S_decimal_latency_adjusted[S_digit_index*4 +: 4] =
                    S_decimal_latency_work[S_digit_index*4 +: 4] + 4'd3;
        end
    end

    // 延迟值由DDR显示帧切换时测得，直接显示，不再按帧率估算。
    assign S_latency_ms = I_latency_ms;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_vsync_d <= 1'b0;
            S_decimal_steps <= 4'd0;
            S_binary_x <= 12'd0;
            S_binary_y <= 12'd0;
            S_decimal_x_work <= 16'd0;
            S_decimal_y_work <= 16'd0;
            S_decimal_x <= 16'd0;
            S_decimal_y <= 16'd0;
            S_binary_latency <= 12'd0;
            S_decimal_latency_work <= 16'd0;
            S_decimal_latency <= 16'd0;
            S_coordinate_valid <= 1'b0;
            S_latency_valid <= 1'b0;
        end else begin
            S_vsync_d <= I_vsync;
            // 检测在帧同步附近结束，于VS下降沿锁存一帧坐标。
            if (S_vsync_d && !I_vsync) begin
                S_binary_x <= I_center_x;
                S_binary_y <= {1'b0, I_center_y};
                S_binary_latency <= S_latency_ms;
                S_decimal_x_work <= 16'd0;
                S_decimal_y_work <= 16'd0;
                S_decimal_latency_work <= 16'd0;
                S_decimal_steps <= 4'd12;
                S_coordinate_valid <= I_target_valid;
                S_latency_valid <= I_latency_valid;
            end else if (S_decimal_steps != 0) begin
                S_binary_x <= {S_binary_x[10:0], 1'b0};
                S_binary_y <= {S_binary_y[10:0], 1'b0};
                S_decimal_x_work <= {S_decimal_x_adjusted[14:0], S_binary_x[11]};
                S_decimal_y_work <= {S_decimal_y_adjusted[14:0], S_binary_y[11]};
                S_binary_latency <= {S_binary_latency[10:0], 1'b0};
                S_decimal_latency_work <= {S_decimal_latency_adjusted[14:0], S_binary_latency[11]};
                S_decimal_steps <= S_decimal_steps - 1'b1;
                if (S_decimal_steps == 4'd1) begin
                    S_decimal_x <= {S_decimal_x_adjusted[14:0], S_binary_x[11]};
                    S_decimal_y <= {S_decimal_y_adjusted[14:0], S_binary_y[11]};
                    S_decimal_latency <= {S_decimal_latency_adjusted[14:0], S_binary_latency[11]};
                end
            end
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_key_meta       <= 1'b1;
            S_key_sync       <= 1'b1;
            S_key_stable     <= 1'b1;
            S_key_count      <= 21'd0;
            O_overlay_enable <= 1'b0;
        end else begin
            S_key_meta <= I_key3_n;
            S_key_sync <= S_key_meta;

            if (S_key_sync == S_key_stable) begin
                S_key_count <= 21'd0;
            end else if (S_key_count == DEBOUNCE_LIMIT) begin
                S_key_stable <= S_key_sync;
                S_key_count  <= 21'd0;
            end else begin
                S_key_count <= S_key_count + 1'b1;
            end

            if (S_key_press)
                O_overlay_enable <= ~O_overlay_enable;
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_x    <= 12'd0;
            S_y    <= 11'd0;
            S_de_d <= 1'b0;
        end else begin
            S_de_d <= I_de;
            if (I_vsync) begin
                S_x <= 12'd0;
                S_y <= 11'd0;
            end else if (I_de) begin
                if (S_x < IMAGE_WIDTH - 1)
                    S_x <= S_x + 1'b1;
            end else begin
                S_x <= 12'd0;
                if (S_de_d && (S_y < IMAGE_HEIGHT))
                    S_y <= S_y + 1'b1;
            end
        end
    end

    // 目标框线宽3像素；比较式扩展位宽，防止画面边缘加减法溢出。
    assign S_box_pixel = O_overlay_enable && I_target_valid && I_de &&
        ((((S_x >= I_x_min) && (S_x <= I_x_max)) &&
          (((S_y >= I_y_min) && ({1'b0, S_y} < ({1'b0, I_y_min} + 12'd3))) ||
           ((S_y <= I_y_max) && (({1'b0, S_y} + 12'd3) > {1'b0, I_y_max})))) ||
         (((S_y >= I_y_min) && (S_y <= I_y_max)) &&
          (((S_x >= I_x_min) && ({1'b0, S_x} < ({1'b0, I_x_min} + 13'd3))) ||
           ((S_x <= I_x_max) && (({1'b0, S_x} + 13'd3) > {1'b0, I_x_max})))));

    // 绘制17x17中心十字，使用扩展加法避免坐标减法下溢。
    assign S_center_pixel = O_overlay_enable && I_target_valid && I_de &&
        (((S_y >= I_center_y) && (S_y < (I_center_y + 2'd2)) &&
          (({1'b0, S_x} + 13'd8) >= {1'b0, I_center_x}) &&
          ({1'b0, S_x} <= ({1'b0, I_center_x} + 13'd8))) ||
         ((S_x >= I_center_x) && (S_x < (I_center_x + 2'd2)) &&
          (({1'b0, S_y} + 12'd8) >= {1'b0, I_center_y}) &&
          ({1'b0, S_y} <= ({1'b0, I_center_y} + 12'd8))));

    // 每字符占16x16单元，5x7字模放大2倍；二次幂尺寸避免像素通路除法。
    assign S_status_background = O_overlay_enable && I_de &&
                                 (S_x >= TEXT_X - 12'd8) &&
                                 (S_x < TEXT_X + 12'd232) &&
                                 (S_y >= TEXT_Y - 11'd8) &&
                                 (S_y < TEXT_Y + 11'd70);
    assign S_text_window = O_overlay_enable && I_de &&
                           (S_x >= TEXT_X) && (S_x < TEXT_X + 12'd224) &&
                           (S_y >= TEXT_Y) && (S_y < TEXT_Y + 11'd62) &&
                           (S_text_dy[3:0] < 4'd14);
    assign S_text_dx    = S_x - TEXT_X;
    assign S_text_dy    = S_y - TEXT_Y;
    assign S_char_index = S_text_dx[7:4];
    assign S_glyph_x    = S_text_dx[3:1];
    assign S_glyph_y    = S_text_dy[3:1];
    assign S_coordinate_row = S_text_dy[4];
    assign S_latency_row = (S_text_dy[5:4] == 2'b10);
    assign S_motion_row = (S_text_dy[5:4] == 2'b11);

    // 标签贴在识别框上方；靠近右边缘时向左收，顶部空间不足时放到框下方。
    assign S_label_x = (I_x_min > (IMAGE_WIDTH - LABEL_WIDTH)) ?
                       (IMAGE_WIDTH - LABEL_WIDTH) : I_x_min;
    assign S_label_y = (I_y_min >= 11'd16) ? (I_y_min - 11'd16) :
                       ((I_y_max <= (IMAGE_HEIGHT - 16)) ?
                        (I_y_max + 11'd1) : 11'd0);
    assign S_label_dx = S_x - S_label_x;
    assign S_label_dy = S_y - S_label_y;
    assign S_label_char_index = S_label_dx[8:4];
    assign S_label_glyph_x = S_label_dx[3:1];
    assign S_label_glyph_y = S_label_dy[3:1];
    assign S_label_window = O_overlay_enable && I_target_valid && I_de &&
                            (S_x >= S_label_x) &&
                            (S_x < (S_label_x + LABEL_WIDTH)) &&
                            (S_y >= S_label_y) &&
                            (S_y < (S_label_y + 11'd16)) &&
                            (S_label_dy[3:0] < 4'd14);
    assign S_label_background = O_overlay_enable && I_target_valid && I_de &&
                                (S_x >= S_label_x) &&
                                (S_x < (S_label_x + LABEL_WIDTH)) &&
                                (S_y >= S_label_y) &&
                                (S_y < (S_label_y + 11'd16));

    // 标签采用同一套5x7字模，字符单元为16x16，保证标签随目标框整块移动。
    assign S_label_glyph_row = F_glyph_row(S_label_char_code, S_label_glyph_y);
    assign S_label_text_pixel = S_label_window &&
                                (S_label_glyph_x < 3'd5) &&
                                S_label_glyph_row[4-S_label_glyph_x];

    always @(*) begin
        S_char_code = 8'h20;
        if (S_motion_row) begin
            case (S_char_index)
                4'd0: S_char_code = "M";
                4'd1: S_char_code = ":";
                4'd2: S_char_code = I_motion_valid ? (8'h30 + {4'd0,I_motion_pixels[19:16]}) : "-";
                4'd3: S_char_code = I_motion_valid ? (8'h30 + {4'd0,I_motion_pixels[15:12]}) : "-";
                4'd4: S_char_code = I_motion_valid ? (8'h30 + {4'd0,I_motion_pixels[11:8]}) : "-";
                4'd5: S_char_code = I_motion_valid ? (8'h30 + {4'd0,I_motion_pixels[7:4]}) : "-";
                4'd6: S_char_code = I_motion_valid ? (8'h30 + {4'd0,I_motion_pixels[3:0]}) : "-";
                4'd8: S_char_code = "D";
                4'd9: S_char_code = ":";
                4'd10: S_char_code = I_motion_detected ? "1" : "0";
                default: S_char_code = 8'h20;
            endcase
        end else if (S_latency_row) begin
            case (S_char_index)
                4'd0: S_char_code = "L";
                4'd1: S_char_code = "A";
                4'd2: S_char_code = "T";
                4'd3: S_char_code = ":";
                4'd4: S_char_code = S_latency_valid ? (8'h30 + {4'd0,S_decimal_latency[15:12]}) : "-";
                4'd5: S_char_code = S_latency_valid ? (8'h30 + {4'd0,S_decimal_latency[11:8]}) : "-";
                4'd6: S_char_code = S_latency_valid ? (8'h30 + {4'd0,S_decimal_latency[7:4]}) : "-";
                4'd7: S_char_code = S_latency_valid ? (8'h30 + {4'd0,S_decimal_latency[3:0]}) : "-";
                4'd8: S_char_code = "m";
                4'd9: S_char_code = "s";
                default: S_char_code = 8'h20;
            endcase
        end else if (S_coordinate_row) begin
            case (S_char_index)
                4'd0: S_char_code = "X";
                4'd1, 4'd8: S_char_code = ":";
                4'd2: S_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_x[15:12]}) : "-";
                4'd3: S_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_x[11:8]}) : "-";
                4'd4: S_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_x[7:4]}) : "-";
                4'd5: S_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_x[3:0]}) : "-";
                4'd7: S_char_code = "Y";
                4'd9: S_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_y[15:12]}) : "-";
                4'd10: S_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_y[11:8]}) : "-";
                4'd11: S_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_y[7:4]}) : "-";
                4'd12: S_char_code = S_coordinate_valid ? (8'h30 + {4'd0,S_decimal_y[3:0]}) : "-";
                default: S_char_code = 8'h20;
            endcase
        end else if (I_target_valid) begin
            case (S_char_index)
                4'd0: S_char_code = "s";
                4'd1: S_char_code = "u";
                4'd2: S_char_code = "c";
                4'd3: S_char_code = "c";
                4'd4: S_char_code = "e";
                4'd5: S_char_code = "s";
                4'd6: S_char_code = "s";
                4'd8: begin
                    case (I_target_color)
                        2'd1: S_char_code = "R";
                        2'd2: S_char_code = "B";
                        2'd3: S_char_code = "Y";
                        default: S_char_code = 8'h20;
                    endcase
                end
                4'd9: begin
                    case (I_target_color)
                        2'd1: S_char_code = "e";
                        2'd2: S_char_code = "l";
                        2'd3: S_char_code = "e";
                        default: S_char_code = 8'h20;
                    endcase
                end
                4'd10: begin
                    case (I_target_color)
                        2'd1: S_char_code = "d";
                        2'd2: S_char_code = "u";
                        2'd3: S_char_code = "l";
                        default: S_char_code = 8'h20;
                    endcase
                end
                4'd11: begin
                    case (I_target_color)
                        2'd2: S_char_code = "e";
                        2'd3: S_char_code = "l";
                        default: S_char_code = 8'h20;
                    endcase
                end
                4'd12: if (I_target_color == 2'd3) S_char_code = "o";
                4'd13: if (I_target_color == 2'd3) S_char_code = "w";
                default: S_char_code = 8'h20;
            endcase
        end else begin
            case (S_char_index)
                4'd0: S_char_code = "N";
                4'd1: S_char_code = "O";
                4'd2: S_char_code = 8'h20;
                4'd3: S_char_code = "T";
                4'd4: S_char_code = "A";
                4'd5: S_char_code = "R";
                4'd6: S_char_code = "G";
                4'd7: S_char_code = "E";
                4'd8: S_char_code = "T";
                default: S_char_code = 8'h20;
            endcase
        end
    end

    function [4:0] F_glyph_row;
        input [7:0] I_char;
        input [2:0] I_row;
        reg [34:0] S_digit_bitmap;
        begin
            F_glyph_row = 5'b00000;
            S_digit_bitmap = 35'd0;
            case (I_char)
                "0","1","2","3","4","5","6","7","8","9": begin
                    case (I_char)
                        "0": S_digit_bitmap = {5'b01110,5'b10001,5'b10011,5'b10101,5'b11001,5'b10001,5'b01110};
                        "1": S_digit_bitmap = {5'b00100,5'b01100,5'b00100,5'b00100,5'b00100,5'b00100,5'b01110};
                        "2": S_digit_bitmap = {5'b01110,5'b10001,5'b00001,5'b00010,5'b00100,5'b01000,5'b11111};
                        "3": S_digit_bitmap = {5'b11110,5'b00001,5'b00001,5'b01110,5'b00001,5'b00001,5'b11110};
                        "4": S_digit_bitmap = {5'b00010,5'b00110,5'b01010,5'b10010,5'b11111,5'b00010,5'b00010};
                        "5": S_digit_bitmap = {5'b11111,5'b10000,5'b10000,5'b11110,5'b00001,5'b00001,5'b11110};
                        "6": S_digit_bitmap = {5'b01110,5'b10000,5'b10000,5'b11110,5'b10001,5'b10001,5'b01110};
                        "7": S_digit_bitmap = {5'b11111,5'b00001,5'b00010,5'b00100,5'b01000,5'b01000,5'b01000};
                        "8": S_digit_bitmap = {5'b01110,5'b10001,5'b10001,5'b01110,5'b10001,5'b10001,5'b01110};
                        "9": S_digit_bitmap = {5'b01110,5'b10001,5'b10001,5'b01111,5'b00001,5'b00001,5'b01110};
                        default: S_digit_bitmap = 35'd0;
                    endcase
                    case (I_row)
                        0: F_glyph_row = S_digit_bitmap[34:30];
                        1: F_glyph_row = S_digit_bitmap[29:25];
                        2: F_glyph_row = S_digit_bitmap[24:20];
                        3: F_glyph_row = S_digit_bitmap[19:15];
                        4: F_glyph_row = S_digit_bitmap[14:10];
                        5: F_glyph_row = S_digit_bitmap[9:5];
                        6: F_glyph_row = S_digit_bitmap[4:0];
                        default: F_glyph_row = 5'd0;
                    endcase
                end
                "X": case (I_row)
                    0,1,5,6: F_glyph_row=5'b10001;
                    2,4: F_glyph_row=5'b01010;
                    3: F_glyph_row=5'b00100;
                    default: F_glyph_row=5'b00000;
                endcase
                "Y": case (I_row)
                    0,1: F_glyph_row=5'b10001;
                    2: F_glyph_row=5'b01010;
                    3,4,5,6: F_glyph_row=5'b00100;
                    default: F_glyph_row=5'b00000;
                endcase
                ":": case (I_row)
                    2,5: F_glyph_row=5'b00100;
                    default: F_glyph_row=5'b00000;
                endcase
                "-": case (I_row)
                    3: F_glyph_row=5'b01110;
                    default: F_glyph_row=5'b00000;
                endcase
                "s": case (I_row)
                    0: F_glyph_row=5'b00000; 1: F_glyph_row=5'b00000;
                    2: F_glyph_row=5'b01111; 3: F_glyph_row=5'b10000;
                    4: F_glyph_row=5'b01110; 5: F_glyph_row=5'b00001;
                    6: F_glyph_row=5'b11110; default: F_glyph_row=5'b00000;
                endcase
                "u": case (I_row)
                    0: F_glyph_row=5'b00000; 1: F_glyph_row=5'b00000;
                    2: F_glyph_row=5'b10001; 3: F_glyph_row=5'b10001;
                    4: F_glyph_row=5'b10001; 5: F_glyph_row=5'b10011;
                    6: F_glyph_row=5'b01101; default: F_glyph_row=5'b00000;
                endcase
                "c": case (I_row)
                    0: F_glyph_row=5'b00000; 1: F_glyph_row=5'b00000;
                    2: F_glyph_row=5'b01110; 3: F_glyph_row=5'b10001;
                    4: F_glyph_row=5'b10000; 5: F_glyph_row=5'b10001;
                    6: F_glyph_row=5'b01110; default: F_glyph_row=5'b00000;
                endcase
                "e": case (I_row)
                    0: F_glyph_row=5'b00000; 1: F_glyph_row=5'b00000;
                    2: F_glyph_row=5'b01110; 3: F_glyph_row=5'b10001;
                    4: F_glyph_row=5'b11111; 5: F_glyph_row=5'b10000;
                    6: F_glyph_row=5'b01110; default: F_glyph_row=5'b00000;
                endcase
                "d": case (I_row)
                    0: F_glyph_row=5'b00001; 1: F_glyph_row=5'b00001;
                    2: F_glyph_row=5'b01101; 3: F_glyph_row=5'b10011;
                    4: F_glyph_row=5'b10001; 5: F_glyph_row=5'b10001;
                    6: F_glyph_row=5'b01111; default: F_glyph_row=5'b00000;
                endcase
                "B": case (I_row)
                    0: F_glyph_row=5'b11110; 1: F_glyph_row=5'b10001;
                    2: F_glyph_row=5'b10001; 3: F_glyph_row=5'b11110;
                    4: F_glyph_row=5'b10001; 5: F_glyph_row=5'b10001;
                    6: F_glyph_row=5'b11110; default: F_glyph_row=5'b00000;
                endcase
                "D": case (I_row)
                    0: F_glyph_row=5'b11110; 1: F_glyph_row=5'b10001;
                    2: F_glyph_row=5'b10001; 3: F_glyph_row=5'b10001;
                    4: F_glyph_row=5'b10001; 5: F_glyph_row=5'b10001;
                    6: F_glyph_row=5'b11110; default: F_glyph_row=5'b00000;
                endcase
                "l": case (I_row)
                    0: F_glyph_row=5'b01100; 1: F_glyph_row=5'b00100;
                    2: F_glyph_row=5'b00100; 3: F_glyph_row=5'b00100;
                    4: F_glyph_row=5'b00100; 5: F_glyph_row=5'b00100;
                    6: F_glyph_row=5'b01110; default: F_glyph_row=5'b00000;
                endcase
                "m": case (I_row)
                    0,1: F_glyph_row=5'b00000;
                    2: F_glyph_row=5'b11011; 3: F_glyph_row=5'b10101;
                    4: F_glyph_row=5'b10101; 5: F_glyph_row=5'b10101;
                    6: F_glyph_row=5'b10101; default: F_glyph_row=5'b00000;
                endcase
                "o": case (I_row)
                    0,1: F_glyph_row=5'b00000;
                    2: F_glyph_row=5'b01110; 3: F_glyph_row=5'b10001;
                    4: F_glyph_row=5'b10001; 5: F_glyph_row=5'b10001;
                    6: F_glyph_row=5'b01110; default: F_glyph_row=5'b00000;
                endcase
                "w": case (I_row)
                    0,1: F_glyph_row=5'b00000;
                    2: F_glyph_row=5'b10001; 3: F_glyph_row=5'b10001;
                    4: F_glyph_row=5'b10101; 5: F_glyph_row=5'b10101;
                    6: F_glyph_row=5'b01010; default: F_glyph_row=5'b00000;
                endcase
                "P": case (I_row)
                    0: F_glyph_row=5'b11110; 1: F_glyph_row=5'b10001;
                    2: F_glyph_row=5'b10001; 3: F_glyph_row=5'b11110;
                    4: F_glyph_row=5'b10000; 5: F_glyph_row=5'b10000;
                    6: F_glyph_row=5'b10000; default: F_glyph_row=5'b00000;
                endcase
                "r": case (I_row)
                    0: F_glyph_row=5'b00000; 1: F_glyph_row=5'b00000;
                    2: F_glyph_row=5'b10110; 3: F_glyph_row=5'b11001;
                    4: F_glyph_row=5'b10000; 5: F_glyph_row=5'b10000;
                    6: F_glyph_row=5'b10000; default: F_glyph_row=5'b00000;
                endcase
                "p": case (I_row)
                    0: F_glyph_row=5'b00000; 1: F_glyph_row=5'b11110;
                    2: F_glyph_row=5'b10001; 3: F_glyph_row=5'b11110;
                    4: F_glyph_row=5'b10000; 5: F_glyph_row=5'b10000;
                    6: F_glyph_row=5'b10000; default: F_glyph_row=5'b00000;
                endcase
                "A": case (I_row)
                    0: F_glyph_row=5'b01110; 1: F_glyph_row=5'b10001;
                    2: F_glyph_row=5'b10001; 3: F_glyph_row=5'b11111;
                    4: F_glyph_row=5'b10001; 5: F_glyph_row=5'b10001;
                    6: F_glyph_row=5'b10001; default: F_glyph_row=5'b00000;
                endcase
                "E": case (I_row)
                    0: F_glyph_row=5'b11111; 1: F_glyph_row=5'b10000;
                    2: F_glyph_row=5'b10000; 3: F_glyph_row=5'b11110;
                    4: F_glyph_row=5'b10000; 5: F_glyph_row=5'b10000;
                    6: F_glyph_row=5'b11111; default: F_glyph_row=5'b00000;
                endcase
                "F": case (I_row)
                    0: F_glyph_row=5'b11111; 1: F_glyph_row=5'b10000;
                    2: F_glyph_row=5'b10000; 3: F_glyph_row=5'b11110;
                    4: F_glyph_row=5'b10000; 5: F_glyph_row=5'b10000;
                    6: F_glyph_row=5'b10000; default: F_glyph_row=5'b00000;
                endcase
                "G": case (I_row)
                    0: F_glyph_row=5'b01110; 1: F_glyph_row=5'b10001;
                    2: F_glyph_row=5'b10000; 3: F_glyph_row=5'b10111;
                    4: F_glyph_row=5'b10001; 5: F_glyph_row=5'b10001;
                    6: F_glyph_row=5'b01110; default: F_glyph_row=5'b00000;
                endcase
                "L": case (I_row)
                    0: F_glyph_row=5'b10000; 1: F_glyph_row=5'b10000;
                    2: F_glyph_row=5'b10000; 3: F_glyph_row=5'b10000;
                    4: F_glyph_row=5'b10000; 5: F_glyph_row=5'b10000;
                    6: F_glyph_row=5'b11111; default: F_glyph_row=5'b00000;
                endcase
                "M": case (I_row)
                    0: F_glyph_row=5'b10001; 1: F_glyph_row=5'b11011;
                    2: F_glyph_row=5'b10101; 3: F_glyph_row=5'b10101;
                    4: F_glyph_row=5'b10001; 5: F_glyph_row=5'b10001;
                    6: F_glyph_row=5'b10001; default: F_glyph_row=5'b00000;
                endcase
                "N": case (I_row)
                    0: F_glyph_row=5'b10001; 1: F_glyph_row=5'b11001;
                    2: F_glyph_row=5'b11001; 3: F_glyph_row=5'b10101;
                    4: F_glyph_row=5'b10011; 5: F_glyph_row=5'b10011;
                    6: F_glyph_row=5'b10001; default: F_glyph_row=5'b00000;
                endcase
                "O": case (I_row)
                    0: F_glyph_row=5'b01110; 1: F_glyph_row=5'b10001;
                    2: F_glyph_row=5'b10001; 3: F_glyph_row=5'b10001;
                    4: F_glyph_row=5'b10001; 5: F_glyph_row=5'b10001;
                    6: F_glyph_row=5'b01110; default: F_glyph_row=5'b00000;
                endcase
                "R": case (I_row)
                    0: F_glyph_row=5'b11110; 1: F_glyph_row=5'b10001;
                    2: F_glyph_row=5'b10001; 3: F_glyph_row=5'b11110;
                    4: F_glyph_row=5'b10100; 5: F_glyph_row=5'b10010;
                    6: F_glyph_row=5'b10001; default: F_glyph_row=5'b00000;
                endcase
                "T": case (I_row)
                    0: F_glyph_row=5'b11111; 1: F_glyph_row=5'b00100;
                    2: F_glyph_row=5'b00100; 3: F_glyph_row=5'b00100;
                    4: F_glyph_row=5'b00100; 5: F_glyph_row=5'b00100;
                    6: F_glyph_row=5'b00100; default: F_glyph_row=5'b00000;
                endcase
                "U": case (I_row)
                    0,1,2,3,4,5: F_glyph_row=5'b10001;
                    6: F_glyph_row=5'b01110;
                    default: F_glyph_row=5'b00000;
                endcase
                "(": case (I_row)
                    0,1,2,3,4,5,6: F_glyph_row=5'b01000;
                    default: F_glyph_row=5'b00000;
                endcase
                ")": case (I_row)
                    0,1,2,3,4,5,6: F_glyph_row=5'b00010;
                    default: F_glyph_row=5'b00000;
                endcase
                ",": case (I_row)
                    6: F_glyph_row=5'b00100;
                    default: F_glyph_row=5'b00000;
                endcase
                "W": case (I_row)
                    0: F_glyph_row=5'b10001; 1: F_glyph_row=5'b10001;
                    2: F_glyph_row=5'b10001; 3: F_glyph_row=5'b10101;
                    4: F_glyph_row=5'b10101; 5: F_glyph_row=5'b10101;
                    6: F_glyph_row=5'b01010; default: F_glyph_row=5'b00000;
                endcase
                default: F_glyph_row = 5'b00000;
            endcase
        end
    endfunction

    assign S_glyph_row = F_glyph_row(S_char_code, S_glyph_y);
    assign S_text_pixel = S_text_window && (S_glyph_x < 3'd5) &&
                          S_glyph_row[4-S_glyph_x];
    // 识别框和状态文字使用目标对应的颜色，避免蓝/黄目标仍显示成红框。
    assign S_target_box_color = (I_target_color == 2'd1) ? 24'hff3030 :
                                (I_target_color == 2'd2) ? 24'h3080ff :
                                (I_target_color == 2'd3) ? 24'hffff00 :
                                24'h00ff40;
    assign S_status_color = I_target_valid ? S_target_box_color : NO_TARGET_COLOR;

    assign O_rgb = !I_de ? 24'h000000 :
                   S_label_text_pixel ? 24'hffffff :
                   S_text_pixel ? S_status_color :
                   S_label_background ? 24'h000000 :
                   S_status_background ? 24'h000000 :
                   (S_box_pixel || S_center_pixel) ? S_target_box_color : I_rgb;

endmodule
