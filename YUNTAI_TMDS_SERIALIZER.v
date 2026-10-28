`timescale 1ns / 1ps

// PH1P硬件串行器：每个像素时钟发送一个10位TMDS码字。
module YUNTAI_TMDS_SERIALIZER (
    input  wire       I_pixel_clk,
    input  wire       I_serial_clk,
    input  wire       I_rst_n,
    input  wire [9:0] I_symbol,
    output wire       O_tmds_p
);
    PH1P_LOGIC_HR_OSERDES #(
        .ODDRMODE("DDRX5")
    ) U_OSERDES (
        .sclk(I_serial_clk),
        .pclk(I_pixel_clk),
        .rst(~I_rst_n),
        .idata(I_symbol),
        .odata(O_tmds_p)
    );
endmodule
