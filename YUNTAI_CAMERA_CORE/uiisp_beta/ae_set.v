/*
 * 原始模块来源：米联客电子科技有限公司参考例程，原始版权方保留相关权利。
 * 本工程在原有手动曝光控制基础上增加画面平均亮度自动曝光。
 * 自动模式和手动模式共用曝光、增益寄存器及摄像头更新请求接口。
 */

module ae_set
(
input wire I_clk,
input wire I_rst,
input wire [3:0]I_btn,
input wire I_auto_enable,
input wire I_average_update,
input wire [7:0] I_average_r,
input wire [7:0] I_average_g,
input wire [7:0] I_average_b,
input wire I_ae_cfg_done,
input wire I_cam_cfg_done,
output wire O_ae_req,
output wire [15:0] O_ae,
output wire [15:0] O_ag
);
	

reg 	[19:0] 	rc_ae_tick = 0; 
reg 			r_ae_tick = 0; 
reg 	[4:0] 	rc_ag_tick = 0; 
reg 			r_ag_tick = 0; 

// 自动曝光每100ms调整一次，目标亮度区间为100~124。
// 亮度公式：Y=(R+2G+B)/4；区间内不调整，避免曝光在临界值附近来回振荡。
localparam [22:0] AUTO_UPDATE_CYCLES = 23'd2_380_952;
localparam [7:0] AUTO_BRIGHT_LOW = 8'd100;
localparam [7:0] AUTO_BRIGHT_HIGH = 8'd124;
reg S_auto_enable_meta, S_auto_enable_sync;
reg S_average_update_meta, S_average_update_sync, S_average_update_seen;
reg S_average_valid;
reg [7:0] S_average_r_sync;
reg [7:0] S_average_g_sync;
reg [7:0] S_average_b_sync;
reg [22:0] S_auto_update_count;
wire [9:0] S_luma_sum = {2'b00, S_average_r_sync} +
                        {1'b0, S_average_g_sync, 1'b0} +
                        {2'b00, S_average_b_sync};
wire [7:0] S_luma = S_luma_sum[9:2];
wire S_auto_update = S_auto_enable_sync && S_average_valid &&
                     I_cam_cfg_done && I_ae_cfg_done &&
                     (S_auto_update_count == AUTO_UPDATE_CYCLES - 1'b1);

always @(posedge I_clk or posedge I_rst) begin
	if (I_rst) begin
		S_auto_enable_meta <= 1'b0;
		S_auto_enable_sync <= 1'b0;
		S_average_update_meta <= 1'b0;
		S_average_update_sync <= 1'b0;
		S_average_update_seen <= 1'b0;
		S_average_valid <= 1'b0;
		S_average_r_sync <= 8'd0;
		S_average_g_sync <= 8'd0;
		S_average_b_sync <= 8'd0;
	end else begin
		S_auto_enable_meta <= I_auto_enable;
		S_auto_enable_sync <= S_auto_enable_meta;
		S_average_update_meta <= I_average_update;
		S_average_update_sync <= S_average_update_meta;
		if (S_average_update_sync != S_average_update_seen) begin
			S_average_update_seen <= S_average_update_sync;
			S_average_valid <= 1'b1;
			S_average_r_sync <= I_average_r;
			S_average_g_sync <= I_average_g;
			S_average_b_sync <= I_average_b;
		end
	end
end

always @(posedge I_clk or posedge I_rst) begin
	if (I_rst)
		S_auto_update_count <= 23'd0;
	else if (!S_auto_enable_sync || !S_average_valid ||
			 !I_cam_cfg_done || !I_ae_cfg_done ||
			 (S_auto_update_count == AUTO_UPDATE_CYCLES - 1'b1))
		S_auto_update_count <= 23'd0;
	else
		S_auto_update_count <= S_auto_update_count + 1'b1;
end
	
always @(posedge I_clk or posedge I_rst) begin
	if(I_rst) begin
		rc_ae_tick <= 0; 
		r_ae_tick <= 0; 
		rc_ag_tick <= 0; 
		r_ag_tick <= 0; 
	end else begin
	// 手动曝光调节脉冲周期为1ms。
		rc_ae_tick <= rc_ae_tick + 1; 
		r_ae_tick <= 0; 
		if(rc_ae_tick >= 99999) begin
			rc_ae_tick <= 0; 
			r_ae_tick <= 1; 
		end 
	// 手动增益调节脉冲周期为32ms。
		rc_ag_tick <= rc_ag_tick + r_ae_tick; 
		r_ag_tick <= (&rc_ag_tick) && r_ae_tick; 
	end
end
	
// 关闭自动曝光后，板载按键的手动曝光和增益调节才生效。
reg 	[3:0] 	r_button_0 = 0, r_button_1 = 0; 
// 原厂720p60模式VTS=1000，曝光以半行计，最大值为2*VTS-9=1991。
// 默认使用最大安全曝光1991；增益控制144映射为0x3e09=0x0f、
// 0x3e06=0x00、0x3e07=0x80，即16倍模拟增益、1倍数字增益。
reg 	[15:0] 	r_ae_set = 1991, r_ag_set = 144; 
always @(posedge I_clk or posedge I_rst) begin
	if (I_rst) begin
		r_button_0 <= 4'd0;
		r_button_1 <= 4'd0;
		r_ae_set <= 16'd1991;
		r_ag_set <= 16'd144;
	end else begin
		r_button_0 <= ~I_btn;
		r_button_1 <= r_button_0;

		// 过亮时先降低增益再缩短曝光；过暗时先增加曝光再提高增益。
		// 曝光限制为16~1991，增益控制值限制为16~176，防止寄存器越界。
		if (S_auto_enable_sync) begin
			if (S_auto_update) begin
				if (S_luma > AUTO_BRIGHT_HIGH) begin
					if (r_ag_set > 16'd16)
						r_ag_set <= (r_ag_set > 16'd19) ? r_ag_set - 16'd4 : 16'd16;
					else if (r_ae_set > 16'd16)
						r_ae_set <= (r_ae_set > 16'd80) ? r_ae_set - 16'd64 : 16'd16;
				end else if (S_luma < AUTO_BRIGHT_LOW) begin
					if (r_ae_set < 16'd1991)
						r_ae_set <= (r_ae_set < 16'd1927) ? r_ae_set + 16'd64 : 16'd1991;
					else if (r_ag_set < 16'd176)
						r_ag_set <= (r_ag_set < 16'd173) ? r_ag_set + 16'd4 : 16'd176;
				end
			end
		end else begin
			if (r_button_1[0]) begin
				r_ae_set <= (r_ae_set > 16'd2) ? r_ae_set - r_ae_tick : r_ae_set;
			end else if (r_button_1[1]) begin
				r_ae_set <= (r_ae_set < 16'd1991) ? r_ae_set + r_ae_tick : r_ae_set;
			end
			if (r_button_1[2]) begin
				r_ag_set <= (r_ag_set > 16'd16) ? r_ag_set - r_ag_tick : r_ag_set;
			end else if (r_button_1[3]) begin
				r_ag_set <= (r_ag_set < 16'd176) ? r_ag_set + r_ag_tick : r_ag_set;
			end
		end
	end
end
	
// 曝光或增益发生变化时，向SC520CS配置模块发送一次成组更新请求。
reg 	[15:0] 	r_ae ; 
reg 	[15:0] 	r_ag ; 
reg 			r_ae_req = 0; //synthesis keep
assign O_ae = r_ae;
assign O_ag = r_ag;
assign O_ae_req = r_ae_req;

always @(posedge I_clk or posedge I_rst) begin
	if(I_rst) begin
		r_ae_req <= 0; 
		r_ae     <= 1990; 
		// 复位值故意与目标增益不同，使传感器初始化完成后至少触发一次成组写入，
		// 保证默认曝光和增益参数真正写入SC520CS。
		r_ag     <= 17; 
	end else if(r_ae_req) begin
		r_ae_req <= 0; 
	end else if(((r_ae != r_ae_set) || (r_ag != r_ag_set))&&(I_cam_cfg_done==1'b1)&&(I_ae_cfg_done==1'b1)) begin
		r_ae     <= r_ae_set; 
		r_ag     <= r_ag_set; 
		r_ae_req <= 1; 
	end
end


endmodule


	
