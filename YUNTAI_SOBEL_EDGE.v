// 1280x720流式Sobel边缘检测：两行BRAM与移位寄存器组成3x3窗口。
// RGB、DE和VS经过相同的五级流水线，保证原图与边缘结果时序对齐。
module YUNTAI_SOBEL_EDGE #(
    parameter integer IMAGE_WIDTH = 1280,
    parameter integer IMAGE_HEIGHT = 720,
    // 梯度幅值M=|Gx|+|Gy|；M不超过此阈值时输出黑色。
    // 调大可抑制纹理和噪点，调小可保留较弱轮廓。
    parameter [10:0] EDGE_NOISE_FLOOR = 11'd16,
    // 边缘亮度=(|Gx|+|Gy|)>>EDGE_SHIFT，超过255时饱和；数值越小越亮。
    parameter integer EDGE_SHIFT = 1
) (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_vsync,
    input  wire        I_de,
    input  wire [23:0] I_rgb,
    output wire        O_vsync,
    output wire        O_de,
    output wire [23:0] O_rgb,
    output reg  [7:0]  O_edge
);

    wire [9:0] S_light_level;
    wire [7:0] S_gray_level;
    reg  [11:0] S_x;
    reg  [10:0] S_y;
    reg         S_de_previous;

    // 每个地址保存前两行像素；前两行会覆盖旧数据，因此BRAM无需整体复位。
    reg [15:0] S_line_history [0:IMAGE_WIDTH-1]; /* fehdl force_ram=1, ram_style="bram_fast" */
    reg [15:0] S_history_read;
    reg [11:0] S_write_x;
    reg [7:0]  S_gray_d;

    reg [23:0] S_rgb_pipe [0:4];
    reg [4:0]  S_de_pipe;
    reg [4:0]  S_vsync_pipe;
    reg [3:0]  S_window_valid;
    integer S_pipe_index;

    // 三行各保留两个水平方向历史像素，和当前像素组成3x3窗口。
    reg [7:0] S_top_left;
    reg [7:0] S_top_middle;
    reg [7:0] S_middle_left;
    reg [7:0] S_middle_middle;
    reg [7:0] S_bottom_left;
    reg [7:0] S_bottom_middle;

    reg [9:0]  S_sum_left;
    reg [9:0]  S_sum_right;
    reg [9:0]  S_sum_top;
    reg [9:0]  S_sum_bottom;
    // Gx、Gy范围为-1020~1020，使用11位补码。
    reg [10:0] S_gx;
    reg [10:0] S_gy;
    wire [10:0] S_abs_gx;
    wire [10:0] S_abs_gy;
    reg  [10:0] S_magnitude;
    wire [10:0] S_edge_scaled;

    assign S_light_level = {2'b00, I_rgb[23:16]} +
                           {1'b0, I_rgb[15:8], 1'b0} +
                           {2'b00, I_rgb[7:0]};
    assign S_gray_level = S_light_level[9:2];

    // BRAM读延时一拍，下一拍回写；行消隐隔开相邻两行的首尾访问。
    always @(posedge I_clk) begin
        if (I_rst_n && I_de)
            S_history_read <= S_line_history[S_x];
        if (I_rst_n && S_de_pipe[0])
            S_line_history[S_write_x] <= {S_history_read[7:0], S_gray_d};
    end

    // 统计有效区坐标，并同步延迟原始RGB、DE和VS。
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_x             <= 12'd0;
            S_y             <= 11'd0;
            S_de_previous   <= 1'b0;
            S_write_x       <= 12'd0;
            S_gray_d        <= 8'd0;
            S_de_pipe       <= 5'd0;
            S_vsync_pipe    <= 5'd0;
            S_window_valid  <= 4'd0;
            for (S_pipe_index = 0; S_pipe_index < 5; S_pipe_index = S_pipe_index + 1)
                S_rgb_pipe[S_pipe_index] <= 24'd0;
        end else begin
            S_de_previous <= I_de;
            S_write_x     <= S_x;
            S_gray_d      <= S_gray_level;
            S_rgb_pipe[0] <= I_rgb;
            for (S_pipe_index = 1; S_pipe_index < 5; S_pipe_index = S_pipe_index + 1)
                S_rgb_pipe[S_pipe_index] <= S_rgb_pipe[S_pipe_index-1];
            S_de_pipe    <= {S_de_pipe[3:0], I_de};
            S_vsync_pipe <= {S_vsync_pipe[3:0], I_vsync};
            S_window_valid <= {S_window_valid[2:0],
                (I_de && !I_vsync && (S_x >= 12'd2) && (S_y >= 11'd2) &&
                 (S_y < IMAGE_HEIGHT))};

            if (I_vsync) begin
                S_x <= 12'd0;
                S_y <= 11'd0;
            end else if (I_de) begin
                if (S_x < IMAGE_WIDTH - 1)
                    S_x <= S_x + 1'b1;
            end else begin
                S_x <= 12'd0;
                if (S_de_previous && (S_y < IMAGE_HEIGHT))
                    S_y <= S_y + 1'b1;
            end
        end
    end

    // Sobel公式：Gx=右侧加权和-左侧加权和，Gy=下侧加权和-上侧加权和。
    // 四个加权和最大为1020；窗口不完整的前两行、前两列固定输出黑色。
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_top_left       <= 8'd0;
            S_top_middle     <= 8'd0;
            S_middle_left    <= 8'd0;
            S_middle_middle  <= 8'd0;
            S_bottom_left    <= 8'd0;
            S_bottom_middle  <= 8'd0;
            S_sum_left       <= 10'd0;
            S_sum_right      <= 10'd0;
            S_sum_top        <= 10'd0;
            S_sum_bottom     <= 10'd0;
        end else if (S_de_pipe[0]) begin
            S_top_left      <= S_top_middle;
            S_top_middle    <= S_history_read[15:8];
            S_middle_left   <= S_middle_middle;
            S_middle_middle <= S_history_read[7:0];
            S_bottom_left   <= S_bottom_middle;
            S_bottom_middle <= S_gray_d;

            S_sum_left <= {2'b00, S_top_left} +
                          {1'b0, S_middle_left, 1'b0} +
                          {2'b00, S_bottom_left};
            S_sum_right <= {2'b00, S_history_read[15:8]} +
                           {1'b0, S_history_read[7:0], 1'b0} +
                           {2'b00, S_gray_d};
            S_sum_top <= {2'b00, S_top_left} +
                         {1'b0, S_top_middle, 1'b0} +
                         {2'b00, S_history_read[15:8]};
            S_sum_bottom <= {2'b00, S_bottom_left} +
                            {1'b0, S_bottom_middle, 1'b0} +
                            {2'b00, S_gray_d};
        end else begin
            // 每行结束时清空横向移位寄存器，防止跨行串接。
            S_top_left       <= 8'd0;
            S_top_middle     <= 8'd0;
            S_middle_left    <= 8'd0;
            S_middle_middle  <= 8'd0;
            S_bottom_left    <= 8'd0;
            S_bottom_middle  <= 8'd0;
            S_sum_left       <= 10'd0;
            S_sum_right      <= 10'd0;
            S_sum_top        <= 10'd0;
            S_sum_bottom     <= 10'd0;
        end
    end

    assign S_abs_gx = S_gx[10] ? (~S_gx + 11'd1) : S_gx;
    assign S_abs_gy = S_gy[10] ? (~S_gy + 11'd1) : S_gy;
    assign S_edge_scaled = S_magnitude >> EDGE_SHIFT;

    // 流水计算梯度、幅值、噪声门限和8位饱和，弱边缘为灰色、强边缘为白色。
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_gx        <= 11'd0;
            S_gy        <= 11'd0;
            S_magnitude <= 11'd0;
            O_edge      <= 8'd0;
        end else begin
            S_gx <= {1'b0, S_sum_right} - {1'b0, S_sum_left};
            S_gy <= {1'b0, S_sum_bottom} - {1'b0, S_sum_top};
            S_magnitude <= S_abs_gx + S_abs_gy;

            if (!S_window_valid[3] || (S_magnitude <= EDGE_NOISE_FLOOR))
                O_edge <= 8'd0;
            else if (S_edge_scaled > 11'd255)
                O_edge <= 8'hff;
            else
                O_edge <= S_edge_scaled[7:0];
        end
    end

    assign O_rgb   = S_rgb_pipe[4];
    assign O_de    = S_de_pipe[4];
    assign O_vsync = S_vsync_pipe[4];

endmodule
