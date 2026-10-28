`timescale 1ns / 1ps

// SC520CS配置控制器：1280x720、RAW10、双通道；寄存器表和I2C时序均由本模块管理。
module YUNTAI_SC520_CONFIG #(
    parameter integer CLK_HZ = 25_000_000
) (
    input wire I_clk, input wire I_rst_n, output wire O_cam_scl,
    inout wire IO_cam_sda, output reg O_cam_rst_n, output wire O_cam_mclk,
    output reg O_cfg_done, output reg O_i2c_error
);
    localparam integer POWER_WAIT = CLK_HZ / 1000;
    localparam integer RESET_WAIT = (CLK_HZ / 1000) * 5;
    localparam [7:0] TABLE_LAST = 8'd164;
    localparam [7:0] SENSOR_WRITE = 8'h6c;
    localparam [2:0] ST_POWER = 3'd0, ST_RESET_WAIT = 3'd1,
                     ST_START = 3'd2, ST_BUSY = 3'd3, ST_DONE = 3'd4,
                     ST_SOFT_RESET_WAIT = 3'd5, ST_FAULT = 3'd6;
    reg [7:0] S_reg_index;
    reg [31:0] S_wait_count;
    reg S_i2c_start;
    reg [2:0] S_state;
    wire [23:0] S_reg_word;
    wire S_i2c_busy, S_i2c_done, S_i2c_nack;

    function [23:0] F_sensor_reg;
        input [7:0] I_index;
        begin
            case (I_index)                8'd0: F_sensor_reg = 24'h010301;
                8'd1: F_sensor_reg = 24'h36e980;
                8'd2: F_sensor_reg = 24'h37f980;
                8'd3: F_sensor_reg = 24'h36ea17;
                8'd4: F_sensor_reg = 24'h36eb0c;
                8'd5: F_sensor_reg = 24'h36ec4a;
                8'd6: F_sensor_reg = 24'h36ed08;
                8'd7: F_sensor_reg = 24'h36e901;
                8'd8: F_sensor_reg = 24'h37f923;
                8'd9: F_sensor_reg = 24'h300a20;
                8'd10: F_sensor_reg = 24'h301f89;
                8'd11: F_sensor_reg = 24'h320000;
                8'd12: F_sensor_reg = 24'h320100;
                8'd13: F_sensor_reg = 24'h320200;
                8'd14: F_sensor_reg = 24'h3203fc;
                8'd15: F_sensor_reg = 24'h32040a;
                8'd16: F_sensor_reg = 24'h320527;
                8'd17: F_sensor_reg = 24'h320606;
                8'd18: F_sensor_reg = 24'h3207ab;
                8'd19: F_sensor_reg = 24'h320805;
                8'd20: F_sensor_reg = 24'h320900;
                8'd21: F_sensor_reg = 24'h320a02;
                8'd22: F_sensor_reg = 24'h320bd0;
                8'd23: F_sensor_reg = 24'h320e03;
                8'd24: F_sensor_reg = 24'h320fe8;
                8'd25: F_sensor_reg = 24'h321000;
                8'd26: F_sensor_reg = 24'h32110a;
                8'd27: F_sensor_reg = 24'h321200;
                8'd28: F_sensor_reg = 24'h321304;
                8'd29: F_sensor_reg = 24'h321531;
                8'd30: F_sensor_reg = 24'h322081;
                8'd31: F_sensor_reg = 24'h322200;
                8'd32: F_sensor_reg = 24'h322492;
                8'd33: F_sensor_reg = 24'h322e03;
                8'd34: F_sensor_reg = 24'h322fe4;
                8'd35: F_sensor_reg = 24'h323000;
                8'd36: F_sensor_reg = 24'h323104;
                8'd37: F_sensor_reg = 24'h330108;
                8'd38: F_sensor_reg = 24'h330660;
                8'd39: F_sensor_reg = 24'h330810;
                8'd40: F_sensor_reg = 24'h330980;
                8'd41: F_sensor_reg = 24'h330bd0;
                8'd42: F_sensor_reg = 24'h331415;
                8'd43: F_sensor_reg = 24'h331f71;
                8'd44: F_sensor_reg = 24'h333310;
                8'd45: F_sensor_reg = 24'h333440;
                8'd46: F_sensor_reg = 24'h336456;
                8'd47: F_sensor_reg = 24'h336ccf;
                8'd48: F_sensor_reg = 24'h339009;
                8'd49: F_sensor_reg = 24'h33910f;
                8'd50: F_sensor_reg = 24'h33921f;
                8'd51: F_sensor_reg = 24'h33930e;
                8'd52: F_sensor_reg = 24'h339415;
                8'd53: F_sensor_reg = 24'h339519;
                8'd54: F_sensor_reg = 24'h33ad1c;
                8'd55: F_sensor_reg = 24'h33af3f;
                8'd56: F_sensor_reg = 24'h33b330;
                8'd57: F_sensor_reg = 24'h33b590;
                8'd58: F_sensor_reg = 24'h349f1e;
                8'd59: F_sensor_reg = 24'h34a609;
                8'd60: F_sensor_reg = 24'h34a70b;
                8'd61: F_sensor_reg = 24'h34a820;
                8'd62: F_sensor_reg = 24'h34a920;
                8'd63: F_sensor_reg = 24'h34f80f;
                8'd64: F_sensor_reg = 24'h34f910;
                8'd65: F_sensor_reg = 24'h363088;
                8'd66: F_sensor_reg = 24'h363334;
                8'd67: F_sensor_reg = 24'h3635c0;
                8'd68: F_sensor_reg = 24'h363752;
                8'd69: F_sensor_reg = 24'h3638c1;
                8'd70: F_sensor_reg = 24'h363a89;
                8'd71: F_sensor_reg = 24'h36704a;
                8'd72: F_sensor_reg = 24'h367448;
                8'd73: F_sensor_reg = 24'h367528;
                8'd74: F_sensor_reg = 24'h367618;
                8'd75: F_sensor_reg = 24'h367c0f;
                8'd76: F_sensor_reg = 24'h367d1f;
                8'd77: F_sensor_reg = 24'h369043;
                8'd78: F_sensor_reg = 24'h369143;
                8'd79: F_sensor_reg = 24'h369243;
                8'd80: F_sensor_reg = 24'h369889;
                8'd81: F_sensor_reg = 24'h369992;
                8'd82: F_sensor_reg = 24'h369aa5;
                8'd83: F_sensor_reg = 24'h369bca;
                8'd84: F_sensor_reg = 24'h369c0b;
                8'd85: F_sensor_reg = 24'h369d0f;
                8'd86: F_sensor_reg = 24'h36a209;
                8'd87: F_sensor_reg = 24'h36a30b;
                8'd88: F_sensor_reg = 24'h36a40f;
                8'd89: F_sensor_reg = 24'h36b138;
                8'd90: F_sensor_reg = 24'h36b2c1;
                8'd91: F_sensor_reg = 24'h370f01;
                8'd92: F_sensor_reg = 24'h3724b1;
                8'd93: F_sensor_reg = 24'h377102;
                8'd94: F_sensor_reg = 24'h377203;
                8'd95: F_sensor_reg = 24'h377303;
                8'd96: F_sensor_reg = 24'h377a0b;
                8'd97: F_sensor_reg = 24'h377b0f;
                8'd98: F_sensor_reg = 24'h390104;
                8'd99: F_sensor_reg = 24'h39058d;
                8'd100: F_sensor_reg = 24'h391d01;
                8'd101: F_sensor_reg = 24'h392623;
                8'd102: F_sensor_reg = 24'h3c0401;
                8'd103: F_sensor_reg = 24'h3e0000;
                8'd104: F_sensor_reg = 24'h3e017c;
                8'd105: F_sensor_reg = 24'h3e0270;
                8'd106: F_sensor_reg = 24'h3e0900;
                8'd107: F_sensor_reg = 24'h440113;
                8'd108: F_sensor_reg = 24'h440203;
                8'd109: F_sensor_reg = 24'h44030b;
                8'd110: F_sensor_reg = 24'h440421;
                8'd111: F_sensor_reg = 24'h440529;
                8'd112: F_sensor_reg = 24'h44070c;
                8'd113: F_sensor_reg = 24'h440c34;
                8'd114: F_sensor_reg = 24'h440d34;
                8'd115: F_sensor_reg = 24'h440e29;
                8'd116: F_sensor_reg = 24'h440f3f;
                8'd117: F_sensor_reg = 24'h441201;
                8'd118: F_sensor_reg = 24'h442401;
                8'd119: F_sensor_reg = 24'h442d00;
                8'd120: F_sensor_reg = 24'h442e00;
                8'd121: F_sensor_reg = 24'h450928;
                8'd122: F_sensor_reg = 24'h450d19;
                8'd123: F_sensor_reg = 24'h451d28;
                8'd124: F_sensor_reg = 24'h452605;
                8'd125: F_sensor_reg = 24'h48190d;
                8'd126: F_sensor_reg = 24'h481b07;
                8'd127: F_sensor_reg = 24'h481d1c;
                8'd128: F_sensor_reg = 24'h481f06;
                8'd129: F_sensor_reg = 24'h48210c;
                8'd130: F_sensor_reg = 24'h482307;
                8'd131: F_sensor_reg = 24'h482506;
                8'd132: F_sensor_reg = 24'h482706;
                8'd133: F_sensor_reg = 24'h48290b;
                8'd134: F_sensor_reg = 24'h50004e;
                8'd135: F_sensor_reg = 24'h500700;
                8'd136: F_sensor_reg = 24'h550e00;
                8'd137: F_sensor_reg = 24'h550f8a;
                8'd138: F_sensor_reg = 24'h578066;
                8'd139: F_sensor_reg = 24'h578708;
                8'd140: F_sensor_reg = 24'h578804;
                8'd141: F_sensor_reg = 24'h578901;
                8'd142: F_sensor_reg = 24'h578d40;
                8'd143: F_sensor_reg = 24'h579008;
                8'd144: F_sensor_reg = 24'h579104;
                8'd145: F_sensor_reg = 24'h579201;
                8'd146: F_sensor_reg = 24'h579946;
                8'd147: F_sensor_reg = 24'h57aaeb;
                8'd148: F_sensor_reg = 24'h590104;
                8'd149: F_sensor_reg = 24'h59f000;
                8'd150: F_sensor_reg = 24'h5ae0fe;
                8'd151: F_sensor_reg = 24'h5ae140;
                8'd152: F_sensor_reg = 24'h5ae238;
                8'd153: F_sensor_reg = 24'h5ae330;
                8'd154: F_sensor_reg = 24'h5ae40c;
                8'd155: F_sensor_reg = 24'h5ae538;
                8'd156: F_sensor_reg = 24'h5ae630;
                8'd157: F_sensor_reg = 24'h5ae728;
                8'd158: F_sensor_reg = 24'h5ae83f;
                8'd159: F_sensor_reg = 24'h5ae934;
                8'd160: F_sensor_reg = 24'h5aea2c;
                8'd161: F_sensor_reg = 24'h5aeb3f;
                8'd162: F_sensor_reg = 24'h5aec34;
                8'd163: F_sensor_reg = 24'h5aed2c;
                8'd164: F_sensor_reg = 24'h010001;
                default: F_sensor_reg = 24'h000000;
            endcase
        end
    endfunction

    assign S_reg_word = F_sensor_reg(S_reg_index);
    assign O_cam_mclk = I_clk;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_cam_rst_n <= 1'b0;
            O_cfg_done <= 1'b0;
            O_i2c_error <= 1'b0;
            S_reg_index <= 8'd0;
            S_wait_count <= 0;
            S_i2c_start <= 1'b0;
            S_state <= ST_POWER;
        end else begin
            case (S_state)
                ST_POWER: if (S_wait_count == POWER_WAIT - 1) begin
                    O_cam_rst_n <= 1'b1; S_wait_count <= 0; S_state <= ST_RESET_WAIT;
                end else S_wait_count <= S_wait_count + 1'b1;
                ST_RESET_WAIT: if (S_wait_count == RESET_WAIT - 1) begin
                    S_wait_count <= 0; S_state <= ST_START;
                end else S_wait_count <= S_wait_count + 1'b1;
                ST_START: begin S_i2c_start <= 1'b1; S_state <= ST_BUSY; end
                ST_BUSY: if (S_i2c_busy) begin S_i2c_start <= 1'b0; S_state <= ST_DONE; end
                ST_DONE: if (S_i2c_done) begin
                    if (S_i2c_nack) begin O_i2c_error <= 1'b1; S_state <= ST_FAULT; end
                    else if (S_reg_index == TABLE_LAST) begin O_cfg_done <= 1'b1; S_state <= ST_DONE; end
                    else if (S_reg_index == 0) begin S_wait_count <= 0; S_state <= ST_SOFT_RESET_WAIT; end
                    else begin S_reg_index <= S_reg_index + 1'b1; S_state <= ST_START; end
                end
                ST_SOFT_RESET_WAIT: if (S_wait_count == RESET_WAIT - 1) begin
                    S_reg_index <= 8'd1; S_wait_count <= 0; S_state <= ST_START;
                end else S_wait_count <= S_wait_count + 1'b1;
                default: begin S_i2c_start <= 1'b0; S_state <= ST_FAULT; end
            endcase
        end
    end

    YUNTAI_I2C_MASTER #(.CLK_HZ(CLK_HZ), .I2C_HZ(100_000)) U_I2C_MASTER (
        .I_clk(I_clk), .I_rst_n(I_rst_n && O_cam_rst_n), .I_start(S_i2c_start),
        .I_device_addr(SENSOR_WRITE), .I_register_addr(S_reg_word[23:8]),
        .I_write_data(S_reg_word[7:0]), .O_busy(S_i2c_busy),
        .O_done(S_i2c_done), .O_nack(S_i2c_nack), .O_scl(O_cam_scl), .IO_sda(IO_cam_sda)
    );
endmodule
