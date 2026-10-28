`timescale 1ns / 1ps

// 只写I2C主机：8位器件地址、16位寄存器地址；SDA采用开漏输出，逻辑高由外部上拉产生。
module YUNTAI_I2C_MASTER #(
    parameter integer CLK_HZ = 24_000_000,
    parameter integer I2C_HZ = 100_000
) (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_start,
    input  wire [7:0]  I_device_addr,
    input  wire [15:0] I_register_addr,
    input  wire [7:0]  I_write_data,
    output reg         O_busy,
    output reg         O_done,
    output reg         O_nack,
    output reg         O_scl,
    inout  wire        IO_sda
);
    localparam integer HALF_PERIOD = CLK_HZ / (I2C_HZ * 2);
    localparam integer DIV_WIDTH = $clog2(HALF_PERIOD);
    localparam [3:0] ST_IDLE  = 4'd0, ST_START = 4'd1, ST_BIT_LOW = 4'd2,
                     ST_BIT_HIGH = 4'd3, ST_ACK_LOW = 4'd4, ST_ACK_HIGH = 4'd5,
                     ST_STOP_LOW = 4'd6, ST_STOP_HIGH = 4'd7, ST_DONE = 4'd8;

    reg [DIV_WIDTH-1:0] S_divider;
    reg                 S_tick;
    reg [3:0]           S_state;
    reg [1:0]           S_byte_index;
    reg [2:0]           S_bit_index;
    reg [7:0]           S_byte_data;
    reg                 S_sda_drive_low;
    wire                S_sda_in = IO_sda;

    assign IO_sda = S_sda_drive_low ? 1'b0 : 1'bz;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_divider <= {DIV_WIDTH{1'b0}};
            S_tick <= 1'b0;
        end else if (S_divider == HALF_PERIOD - 1) begin
            S_divider <= {DIV_WIDTH{1'b0}};
            S_tick <= 1'b1;
        end else begin
            S_divider <= S_divider + 1'b1;
            S_tick <= 1'b0;
        end
    end

    always @(*) begin
        case (S_byte_index)
            2'd0: S_byte_data = I_device_addr;
            2'd1: S_byte_data = I_register_addr[15:8];
            2'd2: S_byte_data = I_register_addr[7:0];
            default: S_byte_data = I_write_data;
        endcase
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_state <= ST_IDLE;
            S_byte_index <= 2'd0;
            S_bit_index <= 3'd7;
            S_sda_drive_low <= 1'b0;
            O_scl <= 1'b1;
            O_busy <= 1'b0;
            O_done <= 1'b0;
            O_nack <= 1'b0;
        end else begin
            O_done <= 1'b0;
            if (S_tick) begin
                case (S_state)
                    ST_IDLE: begin
                        O_scl <= 1'b1;
                        S_sda_drive_low <= 1'b0;
                        O_busy <= 1'b0;
                        O_nack <= 1'b0;
                        if (I_start) begin
                            O_busy <= 1'b1;
                            S_state <= ST_START;
                        end
                    end
                    ST_START: begin
                        S_sda_drive_low <= 1'b1;
                        S_byte_index <= 2'd0;
                        S_bit_index <= 3'd7;
                        S_state <= ST_BIT_LOW;
                    end
                    ST_BIT_LOW: begin
                        O_scl <= 1'b0;
                        S_sda_drive_low <= ~S_byte_data[S_bit_index];
                        S_state <= ST_BIT_HIGH;
                    end
                    ST_BIT_HIGH: begin
                        O_scl <= 1'b1;
                        if (S_bit_index == 0)
                            S_state <= ST_ACK_LOW;
                        else begin
                            S_bit_index <= S_bit_index - 1'b1;
                            S_state <= ST_BIT_LOW;
                        end
                    end
                    ST_ACK_LOW: begin
                        O_scl <= 1'b0;
                        S_sda_drive_low <= 1'b0;
                        S_state <= ST_ACK_HIGH;
                    end
                    ST_ACK_HIGH: begin
                        O_scl <= 1'b1;
                        if (S_sda_in)
                            O_nack <= 1'b1;
                        if (S_byte_index == 2'd3)
                            S_state <= ST_STOP_LOW;
                        else begin
                            S_byte_index <= S_byte_index + 1'b1;
                            S_bit_index <= 3'd7;
                            S_state <= ST_BIT_LOW;
                        end
                    end
                    ST_STOP_LOW: begin
                        O_scl <= 1'b0;
                        S_sda_drive_low <= 1'b1;
                        S_state <= ST_STOP_HIGH;
                    end
                    ST_STOP_HIGH: begin
                        O_scl <= 1'b1;
                        S_sda_drive_low <= 1'b0;
                        S_state <= ST_DONE;
                    end
                    ST_DONE: begin
                        O_done <= 1'b1;
                        O_busy <= 1'b0;
                        S_state <= ST_IDLE;
                    end
                    default: S_state <= ST_IDLE;
                endcase
            end
        end
    end
endmodule
