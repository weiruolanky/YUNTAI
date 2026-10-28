// 在1280x720有效画面左上角绘制安路Logo；仅红蓝图形覆盖画面，白底透明。
module YUNTAI_LOGO_OVERLAY (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_vsync,
    input  wire        I_de,
    input  wire [23:0] I_rgb,
    output wire [23:0] O_rgb
);

    // 原始315x196图形按横纵坐标各除以2，显示尺寸约158x98。
    localparam [11:0] LOGO_X = 12'd24;
    localparam [10:0] LOGO_Y = 11'd18;
    localparam [11:0] LOGO_W = 12'd158;
    localparam [10:0] LOGO_H = 11'd98;

    localparam [23:0] LOGO_BLUE = 24'h31405f;
    localparam [23:0] LOGO_RED  = 24'hed1b25;

    reg [11:0] S_x;
    reg [10:0] S_y;
    reg        S_de_d;

    wire S_logo_window;
    wire [11:0] S_logo_x;
    wire [10:0] S_logo_y;
    wire [17:0] S_x_196;
    wire [17:0] S_y_102;
    wire [17:0] S_y_103;
    wire        S_blue_outer;
    wire        S_blue_gap;
    wire        S_red_band;
    wire        S_red_shape;
    reg  [23:0] S_logo_rgb;

    // 根据已对齐的DE和VS恢复有效画面坐标，VS后的第一条有效行为第0行。
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_x           <= 12'd0;
            S_y           <= 11'd0;
            S_de_d        <= 1'b0;
        end else begin
            S_de_d <= I_de;

            if (I_vsync) begin
                S_x           <= 12'd0;
                S_y           <= 11'd0;
            end else if (I_de) begin
    // 组合坐标表示下一拍将输出的有效像素位置。
                if (S_x < 12'd1279)
                    S_x <= S_x + 1'b1;
            end else begin
                S_x <= 12'd0;
            // 每条有效行结束后，在行消隐期间行坐标加1。
                if (S_de_d && (S_y < 11'd720))
                    S_y <= S_y + 1'b1;
            end
        end
    end

    assign S_logo_window = I_de &&
                           (S_x >= LOGO_X) &&
                           (S_x < (LOGO_X + LOGO_W)) &&
                           (S_y >= LOGO_Y) &&
                           (S_y < (LOGO_Y + LOGO_H));
    // 显示坐标乘2映射回原图坐标，实现整体缩小而不是裁剪。
    assign S_logo_x = (S_x - LOGO_X) << 1;
    assign S_logo_y = (S_y - LOGO_Y) << 1;

    // 用常数乘法描述315x196轮廓直线，不使用图像ROM和可变除法器。
    assign S_x_196 = 18'd196 * S_logo_x;
    assign S_y_102 = 18'd102 * S_logo_y;
    assign S_y_103 = 18'd103 * S_logo_y;

    assign S_blue_outer =
        ((S_x_196 + S_y_102) >= 18'd19992) &&
        (S_x_196 <= (18'd19992 + S_y_103));

    // 透明镂空包含原Logo底部开口。
    assign S_blue_gap =
        (S_logo_y >= 11'd88) &&
        ((18'd108 * S_logo_x) <= (18'd5716 + 18'd59 * S_logo_y)) &&
        (((S_logo_y < 11'd159) &&
          ((18'd71 * S_logo_x + 18'd37 * S_logo_y) >= 18'd10427)) ||
         ((S_logo_y >= 11'd159) &&
          ((18'd37 * S_logo_x) >= (18'd964 + 18'd20 * S_logo_y))));

    assign S_red_band =
        (S_logo_y < 11'd12) ||
        ((S_logo_y >= 11'd23)  && (S_logo_y < 11'd35)) ||
        ((S_logo_y >= 11'd46)  && (S_logo_y < 11'd58)) ||
        ((S_logo_y >= 11'd69)  && (S_logo_y < 11'd81)) ||
        ((S_logo_y >= 11'd92)  && (S_logo_y < 11'd104)) ||
        ((S_logo_y >= 11'd115) && (S_logo_y < 11'd127)) ||
        ((S_logo_y >= 11'd138) && (S_logo_y < 11'd150)) ||
        ((S_logo_y >= 11'd161) && (S_logo_y < 11'd173)) ||
        ((S_logo_y >= 11'd184) && (S_logo_y < 11'd196));

    assign S_red_shape = S_red_band &&
        (S_x_196 >= (18'd21952 + 18'd100 * S_logo_y)) &&
        ((S_x_196 + S_y_103) <= 18'd61740);

    always @(*) begin
        S_logo_rgb = 24'h000000;
        if (S_logo_window) begin
            if (S_blue_outer && !S_blue_gap)
                S_logo_rgb = LOGO_BLUE;
            if (S_red_shape)
                S_logo_rgb = LOGO_RED;
        end
    end

    // 掩码为0时透传摄像头像素，Logo白色空隙不会遮挡画面。
    assign O_rgb = (S_logo_rgb == 24'h000000) ? I_rgb : S_logo_rgb;

endmodule
