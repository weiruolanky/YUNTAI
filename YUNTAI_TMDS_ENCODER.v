`timescale 1ns / 1ps

// 单通道TMDS编码器：消隐期发送控制码，每次控制码发送时清零直流偏差累计值。
module YUNTAI_TMDS_ENCODER (
    input  wire       I_pixel_clk,
    input  wire       I_rst_n,
    input  wire [7:0] I_data,
    input  wire       I_de,
    input  wire       I_ctrl0,
    input  wire       I_ctrl1,
    output reg  [9:0] O_symbol
);
    integer S_i;
    integer S_ones;
    integer S_balance;
    reg [8:0] S_qm;
    reg signed [4:0] S_disparity;

    always @(*) begin
        S_ones = 0;
        for (S_i = 0; S_i < 8; S_i = S_i + 1)
            S_ones = S_ones + I_data[S_i];

        S_qm[0] = I_data[0];
        if ((S_ones > 4) || ((S_ones == 4) && !I_data[0])) begin
            for (S_i = 1; S_i < 8; S_i = S_i + 1)
                S_qm[S_i] = ~(S_qm[S_i-1] ^ I_data[S_i]);
            S_qm[8] = 1'b0;
        end else begin
            for (S_i = 1; S_i < 8; S_i = S_i + 1)
                S_qm[S_i] = S_qm[S_i-1] ^ I_data[S_i];
            S_qm[8] = 1'b1;
        end
        S_balance = 0;
        for (S_i = 0; S_i < 8; S_i = S_i + 1)
            S_balance = S_balance + (S_qm[S_i] ? 1 : -1);
    end

    always @(posedge I_pixel_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_symbol <= 10'b1101010100;
            S_disparity <= 0;
        end else if (!I_de) begin
            S_disparity <= 0;
            case ({I_ctrl1, I_ctrl0})
                2'b00: O_symbol <= 10'b1101010100;
                2'b01: O_symbol <= 10'b0010101011;
                2'b10: O_symbol <= 10'b0101010100;
                default: O_symbol <= 10'b1010101011;
            endcase
        end else if ((S_disparity == 0) || (S_balance == 0)) begin
            O_symbol <= S_qm[8] ? {2'b01, ~S_qm[7:0]} : {2'b10, S_qm[7:0]};
            S_disparity <= S_qm[8] ? (S_disparity - S_balance) :
                                      (S_disparity + S_balance);
        end else if (((S_disparity > 0) && (S_balance > 0)) ||
                     ((S_disparity < 0) && (S_balance < 0))) begin
            O_symbol <= {1'b1, S_qm[8], ~S_qm[7:0]};
            S_disparity <= S_disparity + (S_qm[8] ? 1 : -1) - S_balance;
        end else begin
            O_symbol <= {1'b0, S_qm[8], S_qm[7:0]};
            S_disparity <= S_disparity + S_balance + (S_qm[8] ? -1 : 1);
        end
    end
endmodule
