// 统计画面中央128x128区域的RGB平均值，并在右下角以0~255十进制显示。
// K4短按切换监视框，长按2秒切换自动曝光；黄色指示点表示自动曝光开启。
module YUNTAI_RGB_MONITOR #(
    parameter integer IMAGE_WIDTH  = 1280,
    parameter integer IMAGE_HEIGHT = 720,
    // 采样窗口左上角为(SAMPLE_X,SAMPLE_Y)，尺寸必须保持为128才能使用右移14位求平均。
    parameter [11:0] SAMPLE_X = 12'd576,
    parameter [10:0] SAMPLE_Y = 11'd296,
    parameter integer SAMPLE_SIZE = 128
) (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_key4_n,

    input  wire        I_source_vsync,
    input  wire        I_source_de,
    input  wire [23:0] I_source_rgb,

    input  wire        I_display_vsync,
    input  wire        I_display_de,
    input  wire [23:0] I_display_rgb,

    output wire [23:0] O_rgb,
    output reg         O_monitor_enable,
    output reg         O_auto_exposure_enable,
    output reg         O_average_update,
    output reg  [7:0]  O_average_r,
    output reg  [7:0]  O_average_g,
    output reg  [7:0]  O_average_b
);

    localparam [20:0] DEBOUNCE_LIMIT = 21'd1_499_999; // 75MHz下消抖20ms
    localparam [27:0] LONG_PRESS_CYCLES = 28'd150_000_000; // 75MHz下长按2s
    localparam [11:0] PANEL_X = 12'd1096;
    localparam [10:0] PANEL_Y = 11'd638;

    reg        S_key_meta;
    reg        S_key_sync;
    reg        S_key_stable;
    reg [20:0] S_key_count;
    reg [27:0] S_key_hold_count;
    reg        S_long_press_done;
    wire       S_key_press;
    wire       S_key_release;

    reg [11:0] S_source_x;
    reg [10:0] S_source_y;
    reg        S_source_de_d;
    reg        S_source_vsync_d;
    reg [21:0] S_sum_r;
    reg [21:0] S_sum_g;
    reg [21:0] S_sum_b;
    wire       S_sample_pixel;

    reg [11:0] S_display_x;
    reg [10:0] S_display_y;
    reg        S_display_de_d;
    wire       S_sample_border;
    wire       S_panel_background;
    wire       S_ae_indicator;

    wire [11:0] S_text_dx;
    wire [10:0] S_text_dy;
    wire [3:0]  S_char_index;
    wire [2:0]  S_glyph_x;
    reg  [2:0]  S_glyph_y;
    reg         S_line_active;
    reg  [1:0]  S_line_index;
    reg  [7:0]  S_line_value;
    reg  [7:0]  S_char_code;
    wire [3:0]  S_hundreds;
    wire [3:0]  S_tens;
    wire [3:0]  S_ones;
    wire [4:0]  S_glyph_row;
    wire        S_text_pixel;
    reg  [23:0] S_text_color;

    assign S_key_press = S_key_stable && !S_key_sync &&
                         (S_key_count == DEBOUNCE_LIMIT);
    assign S_key_release = !S_key_stable && S_key_sync &&
                           (S_key_count == DEBOUNCE_LIMIT);

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_key_meta       <= 1'b1;
            S_key_sync       <= 1'b1;
            S_key_stable     <= 1'b1;
            S_key_count      <= 21'd0;
            S_key_hold_count <= 28'd0;
            S_long_press_done <= 1'b0;
            O_monitor_enable <= 1'b0;
            O_auto_exposure_enable <= 1'b0;
        end else begin
            S_key_meta <= I_key4_n;
            S_key_sync <= S_key_meta;
            if (S_key_sync == S_key_stable) begin
                S_key_count <= 21'd0;
            end else if (S_key_count == DEBOUNCE_LIMIT) begin
                S_key_stable <= S_key_sync;
                S_key_count  <= 21'd0;
            end else begin
                S_key_count <= S_key_count + 1'b1;
            end

            if (S_key_press) begin
                S_key_hold_count <= 28'd0;
                S_long_press_done <= 1'b0;
            end else if (!S_key_stable && !S_key_sync && !S_long_press_done) begin
                if (S_key_hold_count >= LONG_PRESS_CYCLES - 1'b1) begin
                    S_long_press_done <= 1'b1;
                    O_auto_exposure_enable <= ~O_auto_exposure_enable;
                end else begin
                    S_key_hold_count <= S_key_hold_count + 1'b1;
                end
            end

            if (S_key_release) begin
                if (!S_long_press_done)
                    O_monitor_enable <= ~O_monitor_enable;
                S_key_hold_count <= 28'd0;
                S_long_press_done <= 1'b0;
            end
        end
    end

    // 一帧内分别累加RGB；128x128=16384，平均值可直接右移14位得到。
    assign S_sample_pixel = I_source_de &&
                            (S_source_x >= SAMPLE_X) &&
                            (S_source_x < SAMPLE_X + SAMPLE_SIZE) &&
                            (S_source_y >= SAMPLE_Y) &&
                            (S_source_y < SAMPLE_Y + SAMPLE_SIZE);

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_source_x       <= 12'd0;
            S_source_y       <= 11'd0;
            S_source_de_d    <= 1'b0;
            S_source_vsync_d <= 1'b0;
            S_sum_r          <= 22'd0;
            S_sum_g          <= 22'd0;
            S_sum_b          <= 22'd0;
            O_average_r      <= 8'd0;
            O_average_g      <= 8'd0;
            O_average_b      <= 8'd0;
            O_average_update <= 1'b0;
        end else begin
            S_source_de_d    <= I_source_de;
            S_source_vsync_d <= I_source_vsync;

            if (I_source_vsync) begin
                S_source_x <= 12'd0;
                S_source_y <= 11'd0;
            end else if (I_source_de) begin
                if (S_source_x < IMAGE_WIDTH - 1)
                    S_source_x <= S_source_x + 1'b1;
            end else begin
                S_source_x <= 12'd0;
                if (S_source_de_d && (S_source_y < IMAGE_HEIGHT))
                    S_source_y <= S_source_y + 1'b1;
            end

            if (S_sample_pixel) begin
                S_sum_r <= S_sum_r + I_source_rgb[23:16];
                S_sum_g <= S_sum_g + I_source_rgb[15:8];
                S_sum_b <= S_sum_b + I_source_rgb[7:0];
            end

            if (I_source_vsync && !S_source_vsync_d) begin
                O_average_r <= S_sum_r[21:14];
                O_average_g <= S_sum_g[21:14];
                O_average_b <= S_sum_b[21:14];
                O_average_update <= ~O_average_update;
                S_sum_r     <= 22'd0;
                S_sum_g     <= 22'd0;
                S_sum_b     <= 22'd0;
            end
        end
    end

    // 显示坐标独立计数，补偿图像处理流水线相对原始采样流的延时。
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_display_x    <= 12'd0;
            S_display_y    <= 11'd0;
            S_display_de_d <= 1'b0;
        end else begin
            S_display_de_d <= I_display_de;
            if (I_display_vsync) begin
                S_display_x <= 12'd0;
                S_display_y <= 11'd0;
            end else if (I_display_de) begin
                if (S_display_x < IMAGE_WIDTH - 1)
                    S_display_x <= S_display_x + 1'b1;
            end else begin
                S_display_x <= 12'd0;
                if (S_display_de_d && (S_display_y < IMAGE_HEIGHT))
                    S_display_y <= S_display_y + 1'b1;
            end
        end
    end

    // 用2像素黄色边框标出实际128x128采样区域。
    assign S_sample_border = O_monitor_enable && I_display_de &&
        ((((S_display_x >= SAMPLE_X) &&
           (S_display_x < SAMPLE_X + SAMPLE_SIZE)) &&
          (((S_display_y >= SAMPLE_Y) && (S_display_y < SAMPLE_Y + 2)) ||
           ((S_display_y >= SAMPLE_Y + SAMPLE_SIZE - 2) &&
            (S_display_y < SAMPLE_Y + SAMPLE_SIZE)))) ||
         (((S_display_y >= SAMPLE_Y) &&
           (S_display_y < SAMPLE_Y + SAMPLE_SIZE)) &&
          (((S_display_x >= SAMPLE_X) && (S_display_x < SAMPLE_X + 2)) ||
           ((S_display_x >= SAMPLE_X + SAMPLE_SIZE - 2) &&
            (S_display_x < SAMPLE_X + SAMPLE_SIZE)))));

    // 右下角面板分三行显示R、G、B平均值。
    assign S_panel_background = O_monitor_enable && I_display_de &&
                                (S_display_x >= PANEL_X - 12'd8) &&
                                (S_display_x < PANEL_X + 12'd100) &&
                                (S_display_y >= PANEL_Y - 11'd8) &&
                                (S_display_y < PANEL_Y + 11'd62);
    assign S_ae_indicator = O_monitor_enable && I_display_de &&
                            (S_display_x >= PANEL_X + 12'd88) &&
                            (S_display_x < PANEL_X + 12'd96) &&
                            (S_display_y >= PANEL_Y + 11'd2) &&
                            (S_display_y < PANEL_Y + 11'd10);
    assign S_text_dx    = S_display_x - PANEL_X;
    assign S_text_dy    = S_display_y - PANEL_Y;
    assign S_char_index = S_text_dx[7:4];
    assign S_glyph_x    = S_text_dx[3:1];

    always @(*) begin
        S_line_active = 1'b0;
        S_line_index  = 2'd0;
        S_line_value  = O_average_r;
        S_glyph_y     = 3'd0;
        S_text_color  = 24'hff4040;

        if ((S_text_dy < 11'd14) && (S_text_dx < 12'd80)) begin
            S_line_active = 1'b1;
            S_line_index  = 2'd0;
            S_line_value  = O_average_r;
            S_glyph_y     = S_text_dy[3:1];
            S_text_color  = 24'hff4040;
        end else if ((S_text_dy >= 11'd20) && (S_text_dy < 11'd34) &&
                     (S_text_dx < 12'd80)) begin
            S_line_active = 1'b1;
            S_line_index  = 2'd1;
            S_line_value  = O_average_g;
            S_glyph_y     = (S_text_dy - 11'd20) >> 1;
            S_text_color  = 24'h40ff40;
        end else if ((S_text_dy >= 11'd40) && (S_text_dy < 11'd54) &&
                     (S_text_dx < 12'd80)) begin
            S_line_active = 1'b1;
            S_line_index  = 2'd2;
            S_line_value  = O_average_b;
            S_glyph_y     = (S_text_dy - 11'd40) >> 1;
            S_text_color  = 24'h4080ff;
        end
    end

    assign S_hundreds = S_line_value / 8'd100;
    assign S_tens     = (S_line_value % 8'd100) / 8'd10;
    assign S_ones     = S_line_value % 8'd10;

    always @(*) begin
        S_char_code = 8'h20;
        case (S_char_index)
            4'd0: begin
                case (S_line_index)
                    2'd0: S_char_code = "R";
                    2'd1: S_char_code = "G";
                    default: S_char_code = "B";
                endcase
            end
            4'd1: S_char_code = ":";
            4'd2: S_char_code = "0" + S_hundreds;
            4'd3: S_char_code = "0" + S_tens;
            4'd4: S_char_code = "0" + S_ones;
            default: S_char_code = 8'h20;
        endcase
    end

    function [4:0] F_glyph_row;
        input [7:0] I_char;
        input [2:0] I_row;
        begin
            F_glyph_row = 5'b00000;
            case (I_char)
                "0": case (I_row)
                    0:F_glyph_row=5'b01110; 1:F_glyph_row=5'b10001;
                    2:F_glyph_row=5'b10011; 3:F_glyph_row=5'b10101;
                    4:F_glyph_row=5'b11001; 5:F_glyph_row=5'b10001;
                    6:F_glyph_row=5'b01110; default:F_glyph_row=5'b00000;
                endcase
                "1": case (I_row)
                    0:F_glyph_row=5'b00100; 1:F_glyph_row=5'b01100;
                    2:F_glyph_row=5'b00100; 3:F_glyph_row=5'b00100;
                    4:F_glyph_row=5'b00100; 5:F_glyph_row=5'b00100;
                    6:F_glyph_row=5'b01110; default:F_glyph_row=5'b00000;
                endcase
                "2": case (I_row)
                    0:F_glyph_row=5'b01110; 1:F_glyph_row=5'b10001;
                    2:F_glyph_row=5'b00001; 3:F_glyph_row=5'b00010;
                    4:F_glyph_row=5'b00100; 5:F_glyph_row=5'b01000;
                    6:F_glyph_row=5'b11111; default:F_glyph_row=5'b00000;
                endcase
                "3": case (I_row)
                    0:F_glyph_row=5'b11110; 1:F_glyph_row=5'b00001;
                    2:F_glyph_row=5'b00001; 3:F_glyph_row=5'b01110;
                    4:F_glyph_row=5'b00001; 5:F_glyph_row=5'b00001;
                    6:F_glyph_row=5'b11110; default:F_glyph_row=5'b00000;
                endcase
                "4": case (I_row)
                    0:F_glyph_row=5'b00010; 1:F_glyph_row=5'b00110;
                    2:F_glyph_row=5'b01010; 3:F_glyph_row=5'b10010;
                    4:F_glyph_row=5'b11111; 5:F_glyph_row=5'b00010;
                    6:F_glyph_row=5'b00010; default:F_glyph_row=5'b00000;
                endcase
                "5": case (I_row)
                    0:F_glyph_row=5'b11111; 1:F_glyph_row=5'b10000;
                    2:F_glyph_row=5'b10000; 3:F_glyph_row=5'b11110;
                    4:F_glyph_row=5'b00001; 5:F_glyph_row=5'b00001;
                    6:F_glyph_row=5'b11110; default:F_glyph_row=5'b00000;
                endcase
                "6": case (I_row)
                    0:F_glyph_row=5'b01110; 1:F_glyph_row=5'b10000;
                    2:F_glyph_row=5'b10000; 3:F_glyph_row=5'b11110;
                    4:F_glyph_row=5'b10001; 5:F_glyph_row=5'b10001;
                    6:F_glyph_row=5'b01110; default:F_glyph_row=5'b00000;
                endcase
                "7": case (I_row)
                    0:F_glyph_row=5'b11111; 1:F_glyph_row=5'b00001;
                    2:F_glyph_row=5'b00010; 3:F_glyph_row=5'b00100;
                    4:F_glyph_row=5'b01000; 5:F_glyph_row=5'b01000;
                    6:F_glyph_row=5'b01000; default:F_glyph_row=5'b00000;
                endcase
                "8": case (I_row)
                    0:F_glyph_row=5'b01110; 1:F_glyph_row=5'b10001;
                    2:F_glyph_row=5'b10001; 3:F_glyph_row=5'b01110;
                    4:F_glyph_row=5'b10001; 5:F_glyph_row=5'b10001;
                    6:F_glyph_row=5'b01110; default:F_glyph_row=5'b00000;
                endcase
                "9": case (I_row)
                    0:F_glyph_row=5'b01110; 1:F_glyph_row=5'b10001;
                    2:F_glyph_row=5'b10001; 3:F_glyph_row=5'b01111;
                    4:F_glyph_row=5'b00001; 5:F_glyph_row=5'b00001;
                    6:F_glyph_row=5'b01110; default:F_glyph_row=5'b00000;
                endcase
                "R": case (I_row)
                    0:F_glyph_row=5'b11110; 1:F_glyph_row=5'b10001;
                    2:F_glyph_row=5'b10001; 3:F_glyph_row=5'b11110;
                    4:F_glyph_row=5'b10100; 5:F_glyph_row=5'b10010;
                    6:F_glyph_row=5'b10001; default:F_glyph_row=5'b00000;
                endcase
                "G": case (I_row)
                    0:F_glyph_row=5'b01110; 1:F_glyph_row=5'b10001;
                    2:F_glyph_row=5'b10000; 3:F_glyph_row=5'b10111;
                    4:F_glyph_row=5'b10001; 5:F_glyph_row=5'b10001;
                    6:F_glyph_row=5'b01110; default:F_glyph_row=5'b00000;
                endcase
                "B": case (I_row)
                    0:F_glyph_row=5'b11110; 1:F_glyph_row=5'b10001;
                    2:F_glyph_row=5'b10001; 3:F_glyph_row=5'b11110;
                    4:F_glyph_row=5'b10001; 5:F_glyph_row=5'b10001;
                    6:F_glyph_row=5'b11110; default:F_glyph_row=5'b00000;
                endcase
                ":": case (I_row)
                    1:F_glyph_row=5'b00100; 2:F_glyph_row=5'b00100;
                    4:F_glyph_row=5'b00100; 5:F_glyph_row=5'b00100;
                    default:F_glyph_row=5'b00000;
                endcase
                default: F_glyph_row = 5'b00000;
            endcase
        end
    endfunction

    assign S_glyph_row = F_glyph_row(S_char_code, S_glyph_y);
    assign S_text_pixel = O_monitor_enable && S_line_active &&
                          (S_glyph_x < 3'd5) && S_glyph_row[4-S_glyph_x];

    assign O_rgb = !I_display_de ? 24'h000000 :
                   S_text_pixel ? S_text_color :
                   S_ae_indicator ? (O_auto_exposure_enable ? 24'h00ff00 : 24'h404040) :
                   S_panel_background ? 24'h000000 :
                   S_sample_border ? 24'hffff00 : I_display_rgb;

endmodule
