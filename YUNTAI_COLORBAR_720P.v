`timescale 1ns / 1ps

module YUNTAI_COLORBAR_720P (
    input  wire [10:0] I_x,
    input  wire [9:0]  I_y,
    input  wire        I_de,
    output reg  [7:0]  O_red,
    output reg  [7:0]  O_green,
    output reg  [7:0]  O_blue
);
    always @(*) begin
        if (!I_de) begin
            O_red = 8'h00;
            O_green = 8'h00;
            O_blue = 8'h00;
        end else begin
            case (I_x[10:8])
                3'd0: begin O_red = 8'hff; O_green = 8'hff; O_blue = 8'hff; end
                3'd1: begin O_red = 8'hff; O_green = 8'hff; O_blue = 8'h00; end
                3'd2: begin O_red = 8'h00; O_green = 8'hff; O_blue = 8'hff; end
                3'd3: begin O_red = 8'h00; O_green = 8'hff; O_blue = 8'h00; end
                3'd4: begin O_red = 8'hff; O_green = 8'h00; O_blue = 8'hff; end
                3'd5: begin O_red = 8'hff; O_green = 8'h00; O_blue = 8'h00; end
                default: begin O_red = 8'h00; O_green = 8'h00; O_blue = 8'hff; end
            endcase
        end
    end
endmodule
