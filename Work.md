# 上位机开发工作说明

本文档说明当前 SCARA_F103 固件与上位机的职责边界、通信协议、轨迹规划要求、限位要求和错误处理。

## 0.29.0 GRBL-SCARA 流式链路同步

当前正式运动链路已经从旧的“上位机完整规划、MCU 执行点轨迹”切换为 GRBL-style 四层流式架构：

```text
UI 懒生成真实 G-code
↓
character-counting 串口发送窗口
↓
MCU 32-block look-ahead planner
↓
MCU 16 个 5 ms step segment + TIM2 step-event 输出
```

上位机现在负责：

```text
生成真实 G0/G1/G2/G3/$J/M3/M4/M5 G-code。
维持最多 64 条待发送命令和 224 bytes 未确认窗口。
用独立串口 QThread 收发，不在 UI 主线程等待 ACK。
显示预览轨迹、反馈位置、planner/segment/RX 诊断。
在发送前做用户层限位和可达性提示。
```

MCU 现在负责：

```text
G-code parser 与实时字符处理。
32 block reverse/forward look-ahead。
五连杆 SCARA segment 端点逆解。
16 个 timed segment 预取。
TIM2 可变周期 step-event 调度。
TIM1/TIM4 one-pulse STEP 输出。
STOP/ESTOP/hold/resume/reset/Jog Cancel、回零、激光安全和状态回传。
```

后续维护时，旧的 BinaryTraj、host-timed 点轨迹、实验文本按钮和 UI 模式切换只能作为历史参考，不得重新接入正式运动入口。下面 0.24.x 到 0.26.x 内容保留为演进记录；若与本节冲突，以本节为准。

## 0.24.5 上位机仿真视野

主 UI 仿真图只在首次初始化时自动适配一次工作区。之后点动、前进/后退、轨迹预览、清空轨迹和运动刷新都不得自动改变坐标轴比例或范围；视野缩放和平移只由鼠标滚轮、左键拖动控制。

## 0.24.4 上位机显示与停止语义

主 UI 右侧只保留轨迹仿真视图，坐标纸铺满绘图区，不再显示图注；鼠标滚轮和拖动只改变真实坐标轴范围，不自动添加平滑或美化轨迹。速度/加速度由 `SCARA_UI/V_monitor.py` 独立窗口显示，数据只来自状态帧 `MPos:x,y` 换算后的真实反馈坐标，按相邻反馈帧的实际时间差解算，不做低通滤波，横轴采用 10 s 滚动窗口。

安全按钮拆分为两个语义：

```text
停止：清除上位机待发送队列，发送 STOP。
急停：保留上位机待发送队列，等待 ACK 的当前点会放回队首，发送 ESTOP。
```

固件协议本次未变化；`STOP` 和 `ESTOP` 的串口命令名称保持不变。

## 1. 角色分工

当前课程设计版采用：

```text
上位机规划，MCU 执行。
```

上位机负责：

```text
轨迹生成。
直线、圆弧、点云离散。
五连杆并联 SCARA 正逆运动学。
速度、加速度、拐角速度规划。
整条轨迹的限位和可达性检查。
逐条 G-code 发送和 ACK 校验。
UI 状态显示和错误处理。
```

下位机负责：

```text
串口接收。
G-code 基本解析。
G-code 环形规划缓冲。
XY 到双轴脉冲转换。
脉冲输出。
STOP/ESTOP。
回零状态机。
状态回传。
```

## 1.1 五连杆坐标和构型约定

当前机械模型按五连杆并联 SCARA 处理，项目参数为：

```text
base_distance = 150 mm
active_link   = 160 mm
passive_link  = 200 mm
```

UI 面向操作者，使用左电机为 X=0 的坐标；固件 G-code 和状态帧使用双电机中点为 X=0 的坐标。上位机发送和接收时必须做固定换算：

```text
X_mcu = X_ui - base_distance / 2
X_ui  = X_mcu + base_distance / 2
```

正逆解构型统一为对称非交叉并联构型：左臂 `+`、右臂 `-` 分支。正解存在两个圆交点时，取 Y 更高的交点作为末端位置。这个约定与固件 `APP_SCARA_IK_LEFT_ELBOW_SIGN=1`、`APP_SCARA_IK_RIGHT_ELBOW_SIGN=-1` 保持一致。

默认软件零点为 UI `X=75, Y=220`，对应 MCU `X=0, Y=220`。直接串口调试时，中心点必须按 MCU 坐标发送，例如：

```text
G1 X0.000 Y220.000 F900 ;ID=MANUAL LIM=1
```

步进系统没有绝对位置反馈，烧录、复位或 `ZERO` 只改变软件中的脉冲零点解释。真实运动前必须确认实体五连杆已经处于对称非交叉零点附近，否则软件坐标和机械姿态会错位。

## 2. 串口协议

串口参数：

```text
115200 8N1
```

上位机每次发送一行：

```text
G1 X120.050 Y80.010 F1200 ;ID=0123 LIM=1
```

下位机成功接收、解析并入队后返回：

```text
ok
```

上位机必须：

```text
1. 等待 ok 后再发送下一条正式轨迹。
2. 收到 error:<code> 后停止继续发送轨迹。
3. 按行区分状态帧、系统 OK 和 G-code ok，状态帧不推进发送队列。
```

## 3. 支持的 G-code

```text
G0 X Y F      快速定位。
G1 X Y F      正式轨迹点。
G20/G21       英寸/毫米，推荐 G21。
G90/G91       绝对/相对坐标，推荐 G90。
G4 P          暂停，占位支持。
M0/M2/M30     停止/程序结束，占位支持。
$X            清报警。
$H            启动回零。
$G            查询模态。
?             立即状态查询。
!             暂停。
~             恢复。
```

暂不支持：

```text
G2/G3 圆弧直接下发到 MCU。
Z 轴。
主轴。
冷却。
探针。
刀补。
坐标系 G54-G59。
GRBL EEPROM 设置流。
```

圆弧、贝塞尔、多向量曲线、点云轨迹都应由上位机离散成连续 `G1`。从 `v0.24.1` 开始，上位机轨迹规划按钮支持 `G1 直线`、`G2 顺圆`、`G3 逆圆` 三种模式；其中 `G2/G3` 只表示上位机规划模式，串口仍发送带 `F`、`ID`、`LIM=1` 的连续 `G1` 点。

## 4. 字符含义

```text
G      G-code 指令。
0/1    G0 快速定位，G1 直线插补点。
X      末端 X 坐标，单位 mm。
Y      末端 Y 坐标，单位 mm。
F      进给速度，单位 mm/min。
;      注释开始，MCU 不参与运动解析，但 ACK 会完整回显。
ID     上位机轨迹点编号，用于日志匹配。
LIM    上位机限位检查标记，建议 LIM=1。
ok     当前行被接受。
seq    ACK 序号。
cs     当前行 ASCII 累加和低 8 位。
line   MCU 实际收到的完整行。
< >    状态帧开始和结束。
|      状态字段分隔符。
```

## 5. 速度规划要求

`G1` 必须带 `F`：

```text
G1 X120.000 Y80.000 F900
G1 X120.050 Y80.010 F1200
G1 X120.100 Y80.020 F1500
```

UI `运行速度(mm/s)` 是上位机规划器的目标末端速度。`F` 是上位机已经规划好的真实发送速度，单位为 `mm/min`，不是给 MCU 做二次规划的原始期望值。

上位机需要处理：

```text
起步加速。
停止减速。
圆弧速度限制。
角点降速。
小线段速度限制。
电机转速上限。
负载、惯量和机械共振。
57 步进电机不丢步的保守速度和加速度。
```

下位机会把 `F` 换算成当前段双轴 pps，并尽量连续执行相邻段。高质量轨迹仍依赖上位机给出的真实速度规划。

从 `v0.24.2` 开始，上位机按真实几何段和累计弧长做速度规划：

```text
直线是真实直线段。
G2/G3 是真实圆弧段。
小车轨迹由直线段和 R12 轮拱圆弧段组成。
只有真实尖角参与角点降速。
圆弧内部不再按短折线拐角反复限速。
```

从 `v0.24.3` 开始，主 UI 不再显示速度图像，只显示轨迹预览、已发送轨迹、下位机反馈轨迹和五连杆机构仿真。速度/加速度监控由 `SCARA_UI/V_monitor.py` 独立窗口负责，数据只来自状态帧 `MPos:x,y` 转换后的真实反馈坐标；G-code 的 `ok` 只表示已接收/入队，不当作实际位置反馈。

## 6. 限位职责

从 `v0.23.1` 开始，正式轨迹限位由上位机负责：

```text
APP_HOST_OWNS_LIMIT_CHECKS = 1
HOSTCAP ... grbl_stream=1 scara_plan=1 gcode_arc=1 jog=1 binary_traj=0 dda=1
```

原因：

```text
限位判断需要遍历整条轨迹。
五连杆机构需要对大量轨迹点做逆解和正解回代。
C8T6 算力有限，不适合在 MCU 上做整条路径的复杂限位扫描。
上位机更适合做全局轨迹、限位和速度规划。
```

上位机发送前必须遍历所有轨迹点，检查：

```text
XY 是否在工作空间内。
五连杆逆解是否存在。
正解回代误差是否可接受。
关节角是否在允许范围内。
电机脉冲是否在允许范围内。
轨迹段是否穿越禁区。
是否已回零。
HOME1/HOME2 或其他限位输入是否异常。
速度、加速度、拐角速度是否保守。
```

当前上位机轨迹按钮的路径预检查至少会拦截并显示：

```text
Y 下限超限。
左右基座距离小于/大于允许范围。
M1/M2 角度低于下限或超过上限，并显示超出角度。
左右主动臂交叉，并显示交叉量。
主动臂低于基座线，并显示超出量。
圆弧半径小于弦长一半。
目标、速度、半径等输入不是有效数字。
```

下位机仍保留最低限度检查：

```text
G-code 语法。
五连杆几何逆解是否存在。
电机是否使能。
STOP/ESTOP。
回零输入。
状态回传。
```

也就是说：下位机不再把软件脉冲范围和关节角范围作为正式轨迹限位依据。如果轨迹点错误，应由上位机在发送前拦截。

## 7. 状态回传

状态帧：

```text
<Idle|MPos:x,y|JPos:p1,p2|FS:feed_mm_min,spindle|Bf:planner_free,rx_free|Q:planner_used|E:n|Seg:count,free,low,underrun|H:h1,h2|HS:home_state|A1:en,run,cur_pps,tgt_pps|A2:en,run,cur_pps,tgt_pps|Lz:armed,ready,marking,power>
```

字段含义：

```text
Idle/Run 当前是否还有运动或缓冲轨迹。
MPos     MCU 根据软件脉冲正解估算的 XY。
JPos     双轴软件脉冲计数。
FS       当前 G-code 进给和主轴/功率字。
Bf       规划缓冲剩余、RX 行队列剩余。
Q        规划缓冲已用段数。
E        步进底层错误 bit。
Seg      timed segment 缓冲计数、剩余、低水位和欠载次数。
H        HOME1/HOME2 输入，1 表示触发。
HS       回零状态。
A1/A2    轴状态：使能、运行、当前 pps、目标 pps。
Lz       激光状态：armed、relay_ready、marking、power_permille。
```

上位机 UI 建议 5-10 Hz 发送：

```text
?
```

或：

```text
HEARTBEAT 42
```

## 8. 回零

上位机发送：

```text
$H
```

MCU 自动流程：

```text
Idle -> Axis1Search -> Axis1Return -> Axis2Search -> Axis2Return -> Done
```

`$H` 返回 `ok` 只表示回零流程已启动，不表示回零完成。

如果第一次 `$H` 后限位开关没有触发，状态会停在：

```text
HS:Axis1Search
```

此时第二次 `$H` 会返回：

```text
error:5
```

这是正常现象，表示回零正在进行或电机仍忙。正确流程：

```text
1. 发送 $H。
2. 观察 ? 或自动状态帧。
3. 等待 HS:Done。
4. 如果卡住，发送 HOME_SENSOR 检查输入。
5. 若要中断，发送 STOP 或 ESTOP，再 CLEAR_ERROR。
```

`v0.23.1` 已允许在 `HS:Done` 或 `HS:Error` 状态下再次 `$H`。

## 9. 错误码

两类错误：

```text
error:<code>   G-code 即时错误。
E:<bits>       步进底层错误位。
```

`error:<code>`：

```text
error:2    BAD_NUMBER，数字字段解析失败。
error:3    BAD_DOLLAR_CMD，不支持的 $ 命令。
error:4    BAD_FEED，F 速度字段非法。
error:5    HOME_START_FAIL，$H 回零启动失败。
error:8    LINE_PENDING_BUSY，上位机发送太快，没有等 ok。
error:15   MOTION_REJECTED，运动目标被拒绝，常见原因是几何逆解失败、电机未使能或当前有错误位。
error:20   UNSUPPORTED_WORD，不支持的 G/M/字段。
error:25   DUPLICATE_AXIS，同一行重复 X 或 Y。
```

`E:<bits>`：

```text
E:0    无底层错误。
E:1    软限位错误。v0.23.1 正式轨迹默认不再由下位机做软限位。
E:2    急停。
E:4    通信看门狗超时。
E:8    未使能时尝试运动。
```

组合示例：

```text
E:5 = 1 + 4 = 软限位 + 通信看门狗超时。
```

注意：

```text
error:5  是 $H 回零启动失败。
E:5      是底层错误位组合。
```

## 10. 高频收发测试

```powershell
cd C:\Users\22602\Desktop\SCARA\SCARA_F103
powershell -NoProfile -ExecutionPolicy Bypass -File tools\host_planned_stream_stress.ps1 -Port COM13 -Count 3000 -FeedMin 500 -FeedMax 1800 -EnableMotion
```

默认逐条显示：

```text
TX ...
RX ...
MATCH ...
```

只看进度：

```powershell
-QuietLines
```

### GRBL/G-code 反馈误差压测与分析

烧录当前 `0.29.0-grbl-scara` 固件后，先用正式 G-code 链路确认状态帧、反馈坐标和误差统计：

```powershell
cd C:\Users\22602\Desktop\SCARA\SCARA_F103
powershell -NoProfile -ExecutionPolicy Bypass -File tools\feedback_error_stress.ps1 -Port COM13 -Count 1000 -FeedMmMin 900 -MaxErrorMm 1.0 -CsvPath .\logs\feedback_gcode.csv -EnableMotion
```

当前正式链路由 UI 懒生成 G-code，MCU 负责 parser、look-ahead planner、SCARA segment preparation 和 TIM2/TIM1/TIM4 STEP 输出。旧 BinaryTraj/host-timed 点轨迹脚本只作为历史参考，不再作为正式验收入口。

UI 正式运行 `G1/G2/G3` 时，预览轨迹和实际 G-code 发送共用同一套限位、速度和回零门控：

```text
G1 直线：UI 做工作空间和关节限位检查后发送真实 G1。
G2/G3 圆弧：UI 保留预览，正式链路发送真实 G-code/或按当前规划器生成的 G1 流。
点动和轨迹发送前必须已经收到 HS:Done 或 STAT hs=Done，UI 才解除 is_homed 门控。
状态帧 MPos/JPos/Q/E/Seg/H/HS/A1/A2/Lz 是反馈和 drain 判断依据，ACK 只表示接收/入队。
```

模拟 UI 控键矩阵，覆盖方向点动、电机点动、轨迹按钮、急停和状态 drain：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\ui_control_matrix_check.ps1 -Port COM13 -FeedMmMin 900
```

模拟 UI 轨迹按钮，覆盖 G1、G2、G3 和两条小车轮廓：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\ui_trajectory_stress.ps1 -Port COM13 -Count 3000 -FeedMmMin 900
```

离线分析反馈 CSV，直接输出轴向峰值、轴向 RMS、范数峰值、范数 RMS 和最差样本位置：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\analyze_feedback_error_csv.ps1 -CsvPath .\logs\feedback_gcode.csv -MaxErrorMm 1.0 -MaxRmsMm 0.5 -WorstCount 10
```

一键执行项目自检、串口链路检查、GRBL 状态/ACK 检查、UI 控键矩阵、UI 轨迹压力测试和 CSV 误差分析：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\final_validation.ps1 -Port COM13 -Count 300 -FeedMmMin 900 -MaxErrorMm 1.0 -MaxRmsMm 0.3 -OutDir .\logs\final_validation_latest
```

验收重点：

```text
HOSTCAP 包含 grbl_stream=1 scara_plan=1 gcode_arc=1 jog=1 binary_traj=0 dda=1 hz=10000
状态帧包含 MPos/JPos/Bf/Q/E/Seg/H/HS/A1/A2/Lz
状态帧包含 HS:Done，回零完成后 UI 才解除运动门控
压测结束为 Idle/Q:0/E:0
FEEDBACK_ERROR 不超过设定阈值
CSV 分析的 max_x/max_y/rms_x/rms_y 用于定位单轴抖动和参数优化方向
```

## 11. PyQt 上位机参考 UI

路径：

```text
C:\Users\22602\Desktop\SCARA\SCARA_F103\tools\robot_upper_sim
```

运行：

```powershell
conda activate robot
cd C:\Users\22602\Desktop\SCARA\SCARA_F103\tools\robot_upper_sim
python upper_sim.py
```

功能：

```text
生成 3000 点直线 + 圆弧轨迹。
上位机侧做五连杆正逆解。
上位机侧做速度规划。
实时显示串口 TX/RX。
显示末端轨迹、五连杆姿态和状态字段；速度/加速度曲线由 `SCARA_UI/V_monitor.py` 监控窗口显示。
支持手动发送调试命令。
```

## 12. 开环限制

当前没有编码器。系统能确认：

```text
指令完整收到并回显。
指令通过基本解析后入队。
下位机按软件脉冲计划输出脉冲。
状态、限位、错误可以回传。
```

系统不能证明：

```text
机械端绝不丢步。
负载变化不会造成位置误差。
驱动器限流、共振、卡滞不会导致失步。
```

若后续要证明机械端真实到位，需要增加编码器、驱动器报警反馈或外部位置传感器。
