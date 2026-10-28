# FPGA 视觉伺服追踪云台

本项目基于安路科技 **HX1P35A** FPGA 开发板和 **SC520CS** MIPI 摄像头，实现从图像采集、DDR2 帧缓存、图像处理、彩色目标识别到双轴舵机控制的完整视觉闭环。系统脱离上位机运行，摄像头画面通过 HDMI 以 1280×720 分辨率实时显示。

项目面向 2026 年全国大学生嵌入式芯片与系统设计竞赛 FPGA 创新设计赛道安路赛题二开发。

## 主要功能

- SC520CS 摄像头 I2C 初始化与传感器 ID 检查。
- 双通道 MIPI RAW10 图像接收、解包和 ISP 处理。
- 板载 DDR2 帧缓存及跨时钟域图像传输。
- 1280×720 HDMI 实时显示，像素时钟为 75 MHz，刷新率约为 60.6 帧/秒。
- 彩图、二值化、灰度和 Sobel 边缘检测四种显示模式。
- 红色、蓝色和黄色花朵识别，输出目标框、中心坐标及颜色标签。
- 基于颜色范围、通道差值、邻域滤波、分块密度和候选区域面积的抗干扰检测。
- 帧差运动检测，并在画面中显示运动统计信息。
- 中心 128×128 区域 RGB 均值监视和自动曝光调节。
- 两路 50 Hz PWM 双轴云台控制，使目标逐步回到画面中心。
- 安路 Logo、模式提示、识别结果、坐标和链路延迟等 OSD 信息叠加。

## 系统数据流

```text
SC520CS
   |
   v
MIPI D-PHY -> RAW10 解包/ISP -> DDR2 帧缓存 -> 720p 显示时序
                                              |
                                              v
                图像模式处理 -> 三色识别 -> OSD -> HDMI
                                     |
                                     v
                              坐标滤波与比例控制
                                     |
                                     v
                            水平/俯仰两路舵机 PWM
```

图像链路和控制链路并行工作。识别模块输出外接框中心点，控制模块计算该点与画面中心 `(640, 360)` 的偏差，再通过死区、跳变抑制、低通滤波和步进限幅调整云台。

## 硬件组成

| 设备 | 用途 |
| --- | --- |
| HX1P35A 开发板 | 主控制平台，核心器件为 PH1P35MDG324 |
| SC520CS 摄像头 | 1280×720、RAW10、MIPI 图像输入 |
| HDMI 显示器 | 实时画面和 OSD 信息显示 |
| 双轴云台及两个舵机 | 水平和俯仰方向调整 |
| 独立舵机电源 | 为舵机提供足够的瞬时电流 |

## 接线说明

| 信号 | 开发板连接 | FPGA 管脚 |
| --- | --- | --- |
| 水平舵机 PWM | J2 第 1 脚 | C18 |
| 俯仰舵机 PWM | J2 第 2 脚 | D15 |
| K1 | 板载按键 | D5 |
| K2 | 板载按键 | A9 |
| K3 | 板载按键 | B9 |
| K4 | 板载按键 | C7 |

舵机应使用独立稳压电源供电，舵机电源负极必须与开发板 GND 共地。FPGA 扩展口只输出 3.3 V PWM 控制信号，不应直接为舵机供电。

## 按键操作

系统上电后默认显示彩色摄像头画面，板载按键均为低电平有效。

| 按键操作 | 功能 |
| --- | --- |
| K1 短按 | 在彩图与二值化图像之间切换 |
| K1 长按约 2 秒 | 在彩图与 Sobel 边缘检测之间切换 |
| K2 短按 | 在彩图与灰度图像之间切换 |
| K3 短按 | 开启或关闭智能识别、目标框显示及云台跟踪 |
| K4 短按 | 开启或关闭中心区域 RGB 均值显示 |
| K4 长按约 2 秒 | 开启或关闭自动曝光 |

模式改变后，屏幕中央会短暂显示对应的英文模式名称。

## 目标识别

当前工程识别红、蓝、黄三类目标，颜色编码如下：

| 编码 | 目标颜色 | 调试参考 RGB |
| --- | --- | --- |
| 1 | 红色 | `(203, 100, 77)` |
| 2 | 蓝色 | `(48, 61, 71)` |
| 3 | 黄色 | `(75, 82, 55)` |

这些 RGB 数值是现场调试参考值，不是单点等值判定。实际检测同时使用各通道上下限和通道差值，以减少灰色、白色、绿色背景及孤立亮点的干扰。

识别流水线包括：

1. RGB 范围及通道差值判定。
2. 3×3 同色邻域多数滤波。
3. 16×8 分块命中密度统计。
4. 候选区域合并、面积及宽高筛选。
5. 多候选目标中优先选择有效像素数最多的区域。
6. 连续多帧确认、丢失保持和坐标死区，降低识别框抖动。

颜色阈值集中在 [`YUNTAI_FLOWER_DETECT.v`](YUNTAI_FLOWER_DETECT.v) 顶部参数区，光照环境或实物颜色改变时应重新测量并调整。

## 云台控制

云台模块输出周期为 20 ms 的 50 Hz PWM：

- 水平轴脉宽范围：500～2500 μs，约对应 180° 行程。
- 俯仰轴脉宽范围：833～2167 μs，按支架安全范围限制为约 120°。
- 画面中心：`(640, 360)`。
- 水平死区：24 像素。
- 俯仰死区：16 像素。
- 控制周期：60 ms。
- 单次最大脉宽变化：10 μs。
- 连续 3 帧有效后才开始修正，100 ms 无新结果时停止继续修正。

控制器采用轻量比例步进方案。坐标误差越大，单次修正越大；误差进入中心死区后保持当前位置。相邻测量跳变过大时重新确认，防止误检造成舵机突然大幅转动。

安装方向、初始位置、死区、控制周期和 PWM 行程可在 [`YUNTAI_SERVO_FIXED.v`](YUNTAI_SERVO_FIXED.v) 中调整。修改行程前应先确认舵机型号及机械限位，避免云台碰撞。

## 工程结构

| 文件或目录 | 作用 |
| --- | --- |
| [`YUNTAI.v`](YUNTAI.v) | 工程顶层及各功能模块连接 |
| [`YUNTAI.al`](YUNTAI.al) | 安路 TD 工程文件 |
| [`YUNTAI.adc`](YUNTAI.adc) | FPGA 管脚约束 |
| [`YUNTAI.sdc`](YUNTAI.sdc) | 时钟及跨时钟域约束 |
| [`YUNTAI_SC520_CONFIG.v`](YUNTAI_SC520_CONFIG.v) | SC520CS 寄存器配置 |
| [`YUNTAI_I2C_MASTER.v`](YUNTAI_I2C_MASTER.v) | 摄像头 I2C 主机 |
| [`YUNTAI_CAMERA_PIPELINE.v`](YUNTAI_CAMERA_PIPELINE.v) | 摄像头数据处理流水线 |
| [`YUNTAI_DDR_BRIDGE.v`](YUNTAI_DDR_BRIDGE.v) | 图像流与 DDR2 控制器桥接 |
| [`YUNTAI_VIDEO_TIMING_720P.v`](YUNTAI_VIDEO_TIMING_720P.v) | 720p 显示时序 |
| [`YUNTAI_BINARY_DISPLAY.v`](YUNTAI_BINARY_DISPLAY.v) | 彩图、二值化和灰度模式切换 |
| [`YUNTAI_SOBEL_EDGE.v`](YUNTAI_SOBEL_EDGE.v) | Sobel 边缘检测 |
| [`YUNTAI_FLOWER_DETECT.v`](YUNTAI_FLOWER_DETECT.v) | 红、蓝、黄目标检测与稳定化 |
| [`YUNTAI_FRAME_DIFF.v`](YUNTAI_FRAME_DIFF.v) | 帧差运动检测 |
| [`YUNTAI_TARGET_OVERLAY.v`](YUNTAI_TARGET_OVERLAY.v) | 目标框、标签、坐标及状态叠加 |
| [`YUNTAI_RGB_MONITOR.v`](YUNTAI_RGB_MONITOR.v) | RGB 均值监视和自动曝光按键控制 |
| [`YUNTAI_SERVO_FIXED.v`](YUNTAI_SERVO_FIXED.v) | 双轴 PWM 与视觉闭环控制 |
| [`YUNTAI_LOGO_OVERLAY.v`](YUNTAI_LOGO_OVERLAY.v) | 透明 Logo 叠加 |
| [`YUNTAI_MODE_NOTICE.v`](YUNTAI_MODE_NOTICE.v) | 模式切换文字提示 |
| `YUNTAI_CAMERA_CORE/` | MIPI、ISP、帧缓存相关模块 |
| `al_ip/` | PLL、DDR2 等安路 IP 文件 |
| `YUNTAI_Runs/` | 已生成的综合、布局布线及位流结果 |

## 编译与烧录

### 环境

- 安路 Tang Dynasty（TD）FPGA 开发工具。
- HX1P35A 开发板对应器件和 IP 支持包。
- 安路下载器及驱动。

### 步骤

1. 克隆仓库：

   ```bash
   git clone https://github.com/weiruolanky/YUNTAI.git
   ```

2. 在 TD 中打开 `YUNTAI.al`。
3. 确认顶层模块为 `YUNTAI`，器件、约束和已有 IP 均已正确载入。
4. 点击 `Run` 完成综合、布局布线和位流生成。
5. 使用 TD 下载工具将生成的 `YUNTAI.bit` 下载到开发板。

仓库中已有位流时，可在 `YUNTAI_Runs/best_result/YUNTAI.bit` 或 `YUNTAI_Runs/phy_1/YUNTAI.bit` 找到。重新编译后应优先使用本次运行新生成的文件。

## 常用调参位置

| 参数 | 文件 |
| --- | --- |
| 红、蓝、黄 RGB 阈值及稳定帧数 | `YUNTAI_FLOWER_DETECT.v` |
| 二值化阈值 | `YUNTAI_BINARY_DISPLAY.v` |
| Sobel 边缘阈值 | `YUNTAI_SOBEL_EDGE.v` |
| 帧差像素阈值及运动门限 | `YUNTAI_FRAME_DIFF.v` |
| 舵机方向、死区、行程、速度及初始位置 | `YUNTAI_SERVO_FIXED.v` |
| SC520CS 曝光、增益及输出格式 | `YUNTAI_SC520_CONFIG.v` |

调整颜色参数时，建议先使用 K4 打开 RGB 均值显示，在固定光照下记录目标和背景数据，再缩小阈值范围。调整舵机参数时应先断开机械负载或限制行程，确认方向正确后再开启闭环跟踪。

## 使用注意事项

- 摄像头 MIPI 信号使用器件专用 D-PHY 管脚，不应改成普通 LVCMOS 管脚。
- 舵机必须独立供电并与开发板共地，否则可能不动作或无规律转动。
- 不同摄像头白平衡、光照和目标材质会改变 RGB 分布，比赛现场需要重新标定阈值。
- PWM 行程必须结合实际舵机和云台机械结构设置，不要直接扩大到超出安全范围。
- 工程包含安路器件相关 IP，迁移到其他 FPGA 平台时需要替换 MIPI、PLL、DDR2 和 HDMI 物理层实现。

## 当前状态

摄像头采集、HDMI 图传、四种图像模式、三色目标识别、帧差检测、自动曝光、OSD 叠加和双轴视觉跟踪均已完成上板验证。
