# 板载50MHz晶振与SC520CS MIPI通道时钟约束。
create_clock -name {sys_clk_50m} -period 20.000 -waveform {0.000 10.000} [get_ports {I_sys_clk}]
create_clock -name {mipi_rx_ck_pad} -period 2.415 -waveform {0.000 1.208} [get_nets {IO_rx_clk_pad_p}]

derive_clocks

# 以下时钟由工程内配置的YUNTAI_PLL产生。
rename_clock -name {pll_clk_100m} [get_clocks {U_PLL/ph1p_phy_pll_wrapper_70638bf6a3e5_Inst/u_PH1P_PHY_PLL.clkc[0]}]
rename_clock -name {pll_clk_24m}  [get_clocks {U_PLL/ph1p_phy_pll_wrapper_70638bf6a3e5_Inst/u_PH1P_PHY_PLL.clkc[1]}]
rename_clock -name {HDMI_PIXEL_CLK} [get_clocks {U_PLL/ph1p_phy_pll_wrapper_70638bf6a3e5_Inst/u_PH1P_PHY_PLL.clkc[4]}]
rename_clock -name {HDMI_SERIAL_CLK} [get_clocks {U_PLL/ph1p_phy_pll_wrapper_70638bf6a3e5_Inst/u_PH1P_PHY_PLL.clkc[5]}]

# MIPI字节时钟由PH1P DPHY0硬核接收器产生。
rename_clock -name {MIPI_RX_BYTE_CLK} [get_clocks {U_CAMERA_PIPELINE/U_DPHY_RX/u_ph1p_mipiio_rx_wrapper/u_PH1P_LOGIC_DPHY_MIPI_RX.o_fabric_div4_8_clk}]

# MIPI字节时钟、板载系统时钟、HDMI时钟和DDR用户时钟彼此异步。
# 视频与DDR之间通过双时钟FIFO跨域；自动曝光统计和舵机坐标通过翻转握手跨域，
# 多位数据在接收端应答前保持不变，因此不对这些跨域数据路径进行同步时序收敛。
set_clock_groups -asynchronous \
    -group [get_clocks {mipi_rx_ck_pad MIPI_RX_BYTE_CLK}] \
    -group [get_clocks {sys_clk_50m}] \
    -group [get_clocks {pll_clk_100m pll_clk_24m}] \
    -group [get_clocks {HDMI_PIXEL_CLK HDMI_SERIAL_CLK}] \
    -group [get_clocks {U_DDR_BRIDGE/U_DDR/ddr_clk U_DDR_BRIDGE/U_DDR/usr_clk}]
