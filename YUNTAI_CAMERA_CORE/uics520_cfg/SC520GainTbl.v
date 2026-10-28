`timescale 1ns / 1ps






module SC520GainTbl (
    input  wire [15:0] I_AG,
    output reg  [23:0] O
);
    reg [7:0] step;
    reg [7:0] fine_gain;

    always @(*) begin
        if (I_AG <= 16)
            step = 8'd0;
        else if (I_AG >= 176)
            step = 8'd160;
        else
            step = I_AG - 16;

        if (step < 32) begin
            fine_gain = 8'h80 + (step << 2);
            O = {8'h00, 8'h00, fine_gain};
        end else if (step < 64) begin
            fine_gain = 8'h80 + ((step - 32) << 2);
            O = {8'h08, 8'h00, fine_gain};
        end else if (step < 96) begin
            fine_gain = 8'h80 + ((step - 64) << 2);
            O = {8'h09, 8'h00, fine_gain};
        end else if (step < 128) begin
            fine_gain = 8'h80 + ((step - 96) << 2);
            O = {8'h0b, 8'h00, fine_gain};
        end else if (step < 160) begin
            fine_gain = 8'h80 + ((step - 128) << 2);
            O = {8'h0f, 8'h00, fine_gain};
        end else begin
            fine_gain = 8'h80;
            O = {8'h1f, 8'h00, fine_gain};
        end
    end
endmodule
