// K1~K4状态改变后，在画面中央显示约DISPLAY_FRAMES帧的英文模式提示。
// 本模块只叠加最终显示像素，不参与识别和舵机控制。
module YUNTAI_MODE_NOTICE #(
    parameter integer IMAGE_WIDTH  = 1280,
    parameter integer IMAGE_HEIGHT = 720,
    parameter [6:0] DISPLAY_FRAMES = 7'd60 // 以帧同步计时，60fps下约显示1秒
) (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_vsync,
    input  wire        I_de,
    input  wire [23:0] I_rgb,
    input  wire [1:0]  I_display_mode,
    input  wire        I_tracking_enable,
    input  wire        I_rgb_monitor_enable,
    input  wire        I_auto_exposure_enable,
    output wire [23:0] O_rgb
);

    localparam [3:0] MSG_NONE      = 4'd0;
    localparam [3:0] MSG_COLOR     = 4'd1;
    localparam [3:0] MSG_BINARY    = 4'd2;
    localparam [3:0] MSG_GRAY      = 4'd3;
    localparam [3:0] MSG_SOBEL     = 4'd4;
    localparam [3:0] MSG_TRACK_ON  = 4'd5;
    localparam [3:0] MSG_TRACK_OFF = 4'd6;
    localparam [3:0] MSG_RGB_ON    = 4'd7;
    localparam [3:0] MSG_RGB_OFF   = 4'd8;
    localparam [3:0] MSG_AE_ON     = 4'd9;
    localparam [3:0] MSG_AE_OFF    = 4'd10;

    reg [1:0] S_display_mode_d;
    reg       S_tracking_enable_d;
    reg       S_rgb_monitor_enable_d;
    reg       S_auto_exposure_enable_d;
    reg [3:0] S_message;
    reg [6:0] S_frames_left;

    reg [11:0] S_x;
    reg [10:0] S_y;
    reg        S_de_d;
    reg        S_vsync_d;

    reg [4:0]  S_message_length;
    reg [11:0] S_text_left;
    wire [11:0] S_text_width;
    wire [11:0] S_text_dx;
    wire [10:0] S_text_dy;
    wire [4:0]  S_char_index;
    wire [4:0]  S_cell_x;
    wire [2:0]  S_glyph_x;
    wire [2:0]  S_glyph_y;
    wire [7:0]  S_char_code;
    wire [34:0] S_glyph_bitmap;
    reg  [4:0]  S_glyph_row;
    wire        S_notice_active;
    wire        S_panel_pixel;
    wire        S_text_window;
    wire        S_glyph_window;
    wire        S_text_pixel;

    function [7:0] F_message_char;
        input [3:0] I_message;
        input [4:0] I_index;
        begin
            F_message_char = " ";
            case (I_message)
                MSG_COLOR: case (I_index)
                    0:F_message_char="C"; 1:F_message_char="O";
                    2:F_message_char="L"; 3:F_message_char="O";
                    4:F_message_char="R"; 6:F_message_char="M";
                    7:F_message_char="O"; 8:F_message_char="D";
                    9:F_message_char="E"; default:F_message_char=" ";
                endcase
                MSG_BINARY: case (I_index)
                    0:F_message_char="B"; 1:F_message_char="I";
                    2:F_message_char="N"; 3:F_message_char="A";
                    4:F_message_char="R"; 5:F_message_char="Y";
                    7:F_message_char="M"; 8:F_message_char="O";
                    9:F_message_char="D"; 10:F_message_char="E";
                    default:F_message_char=" ";
                endcase
                MSG_GRAY: case (I_index)
                    0:F_message_char="G"; 1:F_message_char="R";
                    2:F_message_char="A"; 3:F_message_char="Y";
                    4:F_message_char="S"; 5:F_message_char="C";
                    6:F_message_char="A"; 7:F_message_char="L";
                    8:F_message_char="E"; 10:F_message_char="M";
                    11:F_message_char="O"; 12:F_message_char="D";
                    13:F_message_char="E"; default:F_message_char=" ";
                endcase
                MSG_SOBEL: case (I_index)
                    0:F_message_char="S"; 1:F_message_char="O";
                    2:F_message_char="B"; 3:F_message_char="E";
                    4:F_message_char="L"; 6:F_message_char="E";
                    7:F_message_char="D"; 8:F_message_char="G";
                    9:F_message_char="E"; default:F_message_char=" ";
                endcase
                MSG_TRACK_ON: case (I_index)
                    0:F_message_char="S"; 1:F_message_char="M";
                    2:F_message_char="A"; 3:F_message_char="R";
                    4:F_message_char="T"; 6:F_message_char="T";
                    7:F_message_char="R"; 8:F_message_char="A";
                    9:F_message_char="C"; 10:F_message_char="K";
                    11:F_message_char="I"; 12:F_message_char="N";
                    13:F_message_char="G"; 15:F_message_char="O";
                    16:F_message_char="N"; default:F_message_char=" ";
                endcase
                MSG_TRACK_OFF: case (I_index)
                    0:F_message_char="S"; 1:F_message_char="M";
                    2:F_message_char="A"; 3:F_message_char="R";
                    4:F_message_char="T"; 6:F_message_char="T";
                    7:F_message_char="R"; 8:F_message_char="A";
                    9:F_message_char="C"; 10:F_message_char="K";
                    11:F_message_char="I"; 12:F_message_char="N";
                    13:F_message_char="G"; 15:F_message_char="O";
                    16:F_message_char="F"; 17:F_message_char="F";
                    default:F_message_char=" ";
                endcase
                MSG_RGB_ON: case (I_index)
                    0:F_message_char="R"; 1:F_message_char="G";
                    2:F_message_char="B"; 4:F_message_char="M";
                    5:F_message_char="O"; 6:F_message_char="N";
                    7:F_message_char="I"; 8:F_message_char="T";
                    9:F_message_char="O"; 10:F_message_char="R";
                    12:F_message_char="O"; 13:F_message_char="N";
                    default:F_message_char=" ";
                endcase
                MSG_RGB_OFF: case (I_index)
                    0:F_message_char="R"; 1:F_message_char="G";
                    2:F_message_char="B"; 4:F_message_char="M";
                    5:F_message_char="O"; 6:F_message_char="N";
                    7:F_message_char="I"; 8:F_message_char="T";
                    9:F_message_char="O"; 10:F_message_char="R";
                    12:F_message_char="O"; 13:F_message_char="F";
                    14:F_message_char="F"; default:F_message_char=" ";
                endcase
                MSG_AE_ON: case (I_index)
                    0:F_message_char="A"; 1:F_message_char="U";
                    2:F_message_char="T"; 3:F_message_char="O";
                    5:F_message_char="E"; 6:F_message_char="X";
                    7:F_message_char="P"; 8:F_message_char="O";
                    9:F_message_char="S"; 10:F_message_char="U";
                    11:F_message_char="R"; 12:F_message_char="E";
                    14:F_message_char="O"; 15:F_message_char="N";
                    default:F_message_char=" ";
                endcase
                MSG_AE_OFF: case (I_index)
                    0:F_message_char="A"; 1:F_message_char="U";
                    2:F_message_char="T"; 3:F_message_char="O";
                    5:F_message_char="E"; 6:F_message_char="X";
                    7:F_message_char="P"; 8:F_message_char="O";
                    9:F_message_char="S"; 10:F_message_char="U";
                    11:F_message_char="R"; 12:F_message_char="E";
                    14:F_message_char="O"; 15:F_message_char="F";
                    16:F_message_char="F"; default:F_message_char=" ";
                endcase
                default: F_message_char = " ";
            endcase
        end
    endfunction

    function [34:0] F_glyph_bitmap;
        input [7:0] I_char;
        begin
            case (I_char)
                "A":F_glyph_bitmap={5'b01110,5'b10001,5'b10001,5'b11111,5'b10001,5'b10001,5'b10001};
                "B":F_glyph_bitmap={5'b11110,5'b10001,5'b10001,5'b11110,5'b10001,5'b10001,5'b11110};
                "C":F_glyph_bitmap={5'b01110,5'b10001,5'b10000,5'b10000,5'b10000,5'b10001,5'b01110};
                "D":F_glyph_bitmap={5'b11110,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b11110};
                "E":F_glyph_bitmap={5'b11111,5'b10000,5'b10000,5'b11110,5'b10000,5'b10000,5'b11111};
                "F":F_glyph_bitmap={5'b11111,5'b10000,5'b10000,5'b11110,5'b10000,5'b10000,5'b10000};
                "G":F_glyph_bitmap={5'b01110,5'b10001,5'b10000,5'b10111,5'b10001,5'b10001,5'b01110};
                "H":F_glyph_bitmap={5'b10001,5'b10001,5'b10001,5'b11111,5'b10001,5'b10001,5'b10001};
                "I":F_glyph_bitmap={5'b01110,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100,5'b01110};
                "K":F_glyph_bitmap={5'b10001,5'b10010,5'b10100,5'b11000,5'b10100,5'b10010,5'b10001};
                "L":F_glyph_bitmap={5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b11111};
                "M":F_glyph_bitmap={5'b10001,5'b11011,5'b10101,5'b10101,5'b10001,5'b10001,5'b10001};
                "N":F_glyph_bitmap={5'b10001,5'b11001,5'b10101,5'b10011,5'b10001,5'b10001,5'b10001};
                "O":F_glyph_bitmap={5'b01110,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01110};
                "P":F_glyph_bitmap={5'b11110,5'b10001,5'b10001,5'b11110,5'b10000,5'b10000,5'b10000};
                "R":F_glyph_bitmap={5'b11110,5'b10001,5'b10001,5'b11110,5'b10100,5'b10010,5'b10001};
                "S":F_glyph_bitmap={5'b01111,5'b10000,5'b10000,5'b01110,5'b00001,5'b00001,5'b11110};
                "T":F_glyph_bitmap={5'b11111,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100};
                "U":F_glyph_bitmap={5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01110};
                "X":F_glyph_bitmap={5'b10001,5'b10001,5'b01010,5'b00100,5'b01010,5'b10001,5'b10001};
                "Y":F_glyph_bitmap={5'b10001,5'b10001,5'b01010,5'b00100,5'b00100,5'b00100,5'b00100};
                default:F_glyph_bitmap=35'd0;
            endcase
        end
    endfunction

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_display_mode_d <= 2'b00;
            S_tracking_enable_d <= 1'b0;
            S_rgb_monitor_enable_d <= 1'b0;
            S_auto_exposure_enable_d <= 1'b0;
            S_message <= MSG_NONE;
            S_frames_left <= 7'd0;
        end else begin
            S_display_mode_d <= I_display_mode;
            S_tracking_enable_d <= I_tracking_enable;
            S_rgb_monitor_enable_d <= I_rgb_monitor_enable;
            S_auto_exposure_enable_d <= I_auto_exposure_enable;

            if (I_display_mode != S_display_mode_d) begin
                case (I_display_mode)
                    2'b00:S_message <= MSG_COLOR;
                    2'b01:S_message <= MSG_BINARY;
                    2'b10:S_message <= MSG_GRAY;
                    default:S_message <= MSG_SOBEL;
                endcase
                S_frames_left <= DISPLAY_FRAMES;
            end else if (I_tracking_enable != S_tracking_enable_d) begin
                S_message <= I_tracking_enable ? MSG_TRACK_ON : MSG_TRACK_OFF;
                S_frames_left <= DISPLAY_FRAMES;
            end else if (I_rgb_monitor_enable != S_rgb_monitor_enable_d) begin
                S_message <= I_rgb_monitor_enable ? MSG_RGB_ON : MSG_RGB_OFF;
                S_frames_left <= DISPLAY_FRAMES;
            end else if (I_auto_exposure_enable != S_auto_exposure_enable_d) begin
                S_message <= I_auto_exposure_enable ? MSG_AE_ON : MSG_AE_OFF;
                S_frames_left <= DISPLAY_FRAMES;
            end else if (I_vsync && !S_vsync_d && (S_frames_left != 0)) begin
                S_frames_left <= S_frames_left - 1'b1;
            end
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_x <= 12'd0;
            S_y <= 11'd0;
            S_de_d <= 1'b0;
            S_vsync_d <= 1'b0;
        end else begin
            S_de_d <= I_de;
            S_vsync_d <= I_vsync;
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

    always @(*) begin
        case (S_message)
            MSG_COLOR, MSG_SOBEL: begin S_message_length=5'd10; S_text_left=12'd480; end
            MSG_BINARY: begin S_message_length=5'd11; S_text_left=12'd464; end
            MSG_GRAY, MSG_RGB_ON: begin S_message_length=5'd14; S_text_left=12'd416; end
            MSG_RGB_OFF: begin S_message_length=5'd15; S_text_left=12'd400; end
            MSG_AE_ON: begin S_message_length=5'd16; S_text_left=12'd384; end
            MSG_TRACK_ON, MSG_AE_OFF: begin S_message_length=5'd17; S_text_left=12'd368; end
            default: begin S_message_length=5'd18; S_text_left=12'd352; end
        endcase
    end

    assign S_text_width = {S_message_length, 5'd0};
    assign S_notice_active = (S_frames_left != 0) && (S_message != MSG_NONE);
    assign S_panel_pixel = S_notice_active && I_de &&
                           (S_x >= S_text_left - 12'd16) &&
                           (S_x < S_text_left + S_text_width + 12'd16) &&
                           (S_y >= 11'd334) && (S_y < 11'd386);
    assign S_text_window = S_notice_active && I_de &&
                           (S_x >= S_text_left) &&
                           (S_x < S_text_left + S_text_width) &&
                           (S_y >= 11'd346) && (S_y < 11'd374);
    assign S_text_dx = S_x - S_text_left;
    assign S_text_dy = S_y - 11'd346;
    assign S_char_index = S_text_dx[9:5];
    assign S_cell_x = S_text_dx[4:0];
    assign S_glyph_window = S_text_window &&
                            (S_cell_x >= 5'd6) && (S_cell_x < 5'd26);
    assign S_glyph_x = (S_cell_x - 5'd6) >> 2;
    assign S_glyph_y = S_text_dy[4:2];
    assign S_char_code = F_message_char(S_message, S_char_index);
    assign S_glyph_bitmap = F_glyph_bitmap(S_char_code);

    always @(*) begin
        case (S_glyph_y)
            3'd0:S_glyph_row=S_glyph_bitmap[34:30];
            3'd1:S_glyph_row=S_glyph_bitmap[29:25];
            3'd2:S_glyph_row=S_glyph_bitmap[24:20];
            3'd3:S_glyph_row=S_glyph_bitmap[19:15];
            3'd4:S_glyph_row=S_glyph_bitmap[14:10];
            3'd5:S_glyph_row=S_glyph_bitmap[9:5];
            default:S_glyph_row=S_glyph_bitmap[4:0];
        endcase
    end

    assign S_text_pixel = S_glyph_window && S_glyph_row[4-S_glyph_x];
    assign O_rgb = !I_de ? 24'h000000 :
                   S_text_pixel ? 24'hffffff :
                   S_panel_pixel ? 24'h101010 : I_rgb;

endmodule
