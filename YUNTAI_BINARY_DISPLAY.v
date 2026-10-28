// 彩图、二值化、灰度和Sobel四种显示模式选择。
// K1短按切换二值化，K1长按2秒切换Sobel，K2切换灰度；重复操作返回彩图。
module YUNTAI_BINARY_DISPLAY (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_key1_n,
    input  wire        I_key2_n,
    input  wire        I_vsync,
    input  wire        I_de,
    input  wire [23:0] I_rgb,
    output wire        O_vsync,
    output wire        O_de,
    output wire [23:0] O_rgb,
    output reg  [1:0]  O_display_mode
);

    // 按键时间参数均按75MHz像素时钟计算。
    localparam [20:0] DEBOUNCE_LIMIT = 21'd1_499_999;
    localparam [27:0] LONG_PRESS_LIMIT = 28'd149_999_999;
    // 亮度和L=R+2G+B，范围0~1020；L<=THRESHOLD_LEVEL输出黑色，否则输出白色。
    // 该阈值不是单通道0~255灰度值，现场调节时应在0~1020范围内选择。
    localparam [9:0]  THRESHOLD_LEVEL = 10'd350;

    reg        S_key1_meta;
    reg        S_key1_sync;
    reg        S_key1_stable;
    reg [20:0] S_key1_count;
    reg        S_key2_meta;
    reg        S_key2_sync;
    reg        S_key2_stable;
    reg [20:0] S_key2_count;
    reg [27:0] S_key1_hold_count;
    reg        S_key1_long_handled;
    wire       S_key1_release;
    wire       S_key1_long_press;
    wire       S_key2_press;
    wire [23:0] S_rgb_aligned;
    wire        S_de_aligned;
    wire [7:0]  S_edge_level;
    wire [9:0] S_light_level;
    wire [7:0] S_gray_level;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_key1_meta   <= 1'b1;
            S_key1_sync   <= 1'b1;
            S_key1_stable <= 1'b1;
            S_key1_count  <= 21'd0;
            S_key2_meta   <= 1'b1;
            S_key2_sync   <= 1'b1;
            S_key2_stable <= 1'b1;
            S_key2_count  <= 21'd0;
        end else begin
            S_key1_meta <= I_key1_n;
            S_key1_sync <= S_key1_meta;
            S_key2_meta <= I_key2_n;
            S_key2_sync <= S_key2_meta;

            if (S_key1_sync == S_key1_stable) begin
                S_key1_count <= 21'd0;
            end else if (S_key1_count == DEBOUNCE_LIMIT) begin
                S_key1_stable <= S_key1_sync;
                S_key1_count  <= 21'd0;
            end else begin
                S_key1_count <= S_key1_count + 1'b1;
            end

            if (S_key2_sync == S_key2_stable) begin
                S_key2_count <= 21'd0;
            end else if (S_key2_count == DEBOUNCE_LIMIT) begin
                S_key2_stable <= S_key2_sync;
                S_key2_count  <= 21'd0;
            end else begin
                S_key2_count <= S_key2_count + 1'b1;
            end
        end
    end

    // 在消抖状态翻转的同一拍产生单周期按键事件。
    assign S_key1_release = !S_key1_stable && S_key1_sync &&
                            (S_key1_count == DEBOUNCE_LIMIT);
    assign S_key2_press = S_key2_stable && !S_key2_sync &&
                         (S_key2_count == DEBOUNCE_LIMIT);
    assign S_key1_long_press = !S_key1_stable && !S_key1_long_handled &&
                              (S_key1_hold_count == LONG_PRESS_LIMIT);

    // 模式编码：00彩图，01二值化，10灰度，11 Sobel。
    // K1短按在松开时生效，长按处理完成后屏蔽本次短按事件。
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_key1_hold_count   <= 28'd0;
            S_key1_long_handled <= 1'b0;
            O_display_mode     <= 2'b00;
        end else begin
            if (S_key1_stable) begin
                S_key1_hold_count   <= 28'd0;
                S_key1_long_handled <= 1'b0;
            end else if (!S_key1_long_handled) begin
                if (S_key1_long_press)
                    S_key1_long_handled <= 1'b1;
                else
                    S_key1_hold_count <= S_key1_hold_count + 1'b1;
            end

            // 长按到时与松开同拍时，优先按长按处理。
            if (S_key1_long_press) begin
                if (O_display_mode == 2'b11)
                    O_display_mode <= 2'b00;
                else
                    O_display_mode <= 2'b11;
            end else if (S_key2_press) begin
                if (O_display_mode == 2'b10)
                    O_display_mode <= 2'b00;
                else
                    O_display_mode <= 2'b10;
            end else if (S_key1_release && !S_key1_long_handled) begin
                if (O_display_mode == 2'b01)
                    O_display_mode <= 2'b00;
                else
                    O_display_mode <= 2'b01;
            end
        end
    end

    // Sobel流水线持续工作，使四种模式的RGB、DE和VS延时保持一致。
    YUNTAI_SOBEL_EDGE U_SOBEL_EDGE (
        .I_clk   (I_clk),
        .I_rst_n (I_rst_n),
        .I_vsync (I_vsync),
        .I_de    (I_de),
        .I_rgb   (I_rgb),
        .O_vsync (O_vsync),
        .O_de    (S_de_aligned),
        .O_rgb   (S_rgb_aligned),
        .O_edge  (S_edge_level)
    );
    assign O_de = S_de_aligned;

    // L=R+2G+B，最大值1020，10位不会溢出。
    assign S_light_level = {2'b00, S_rgb_aligned[23:16]} +
                           {1'b0, S_rgb_aligned[15:8], 1'b0} +
                           {2'b00, S_rgb_aligned[7:0]};
    // 灰度近似公式：Gray=(R+2G+B)/4。
    assign S_gray_level = S_light_level[9:2];

    // 消隐区固定为黑色；Sobel模式输出连续的8位边缘强度。
    assign O_rgb = !S_de_aligned ? 24'h000000 :
                   (O_display_mode == 2'b00) ? S_rgb_aligned :
                   (O_display_mode == 2'b01) ?
                       (S_light_level <= THRESHOLD_LEVEL ? 24'h000000 : 24'hffffff) :
                   (O_display_mode == 2'b10) ?
                       {S_gray_level, S_gray_level, S_gray_level} :
                       {S_edge_level, S_edge_level, S_edge_level};

endmodule
