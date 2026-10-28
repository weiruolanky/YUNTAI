`timescale 1ns / 1ps

// CEA-861 1280x720p60时序发生器；标准像素时钟为74.25MHz，本工程当前使用75MHz。
module YUNTAI_VIDEO_TIMING_720P (
    input  wire        I_pixel_clk,
    input  wire        I_rst_n,
    output reg  [10:0] O_x,
    output reg  [9:0]  O_y,
    output wire        O_de,
    output wire        O_hsync,
    output wire        O_vsync,
    output wire        O_frame_start
);
    localparam H_ACTIVE = 1280, H_FRONT = 110, H_SYNC = 40, H_BACK = 220, H_TOTAL = 1650;
    localparam V_ACTIVE = 720,  V_FRONT = 5,   V_SYNC = 5,  V_BACK = 20,  V_TOTAL = 750;
    reg [10:0] S_hcount;
    reg [9:0]  S_vcount;

    always @(posedge I_pixel_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_hcount <= 0;
            S_vcount <= 0;
        end else if (S_hcount == H_TOTAL - 1) begin
            S_hcount <= 0;
            if (S_vcount == V_TOTAL - 1)
                S_vcount <= 0;
            else
                S_vcount <= S_vcount + 1'b1;
        end else begin
            S_hcount <= S_hcount + 1'b1;
        end
    end

    assign O_de = (S_hcount < H_ACTIVE) && (S_vcount < V_ACTIVE);
    assign O_hsync = (S_hcount >= H_ACTIVE + H_FRONT) &&
                     (S_hcount < H_ACTIVE + H_FRONT + H_SYNC);
    assign O_vsync = (S_vcount >= V_ACTIVE + V_FRONT) &&
                     (S_vcount < V_ACTIVE + V_FRONT + V_SYNC);
    assign O_frame_start = (S_hcount == 0) && (S_vcount == 0);

    always @(*) begin
        O_x = S_hcount;
        O_y = S_vcount;
    end
endmodule
