# SCARA_F103 控制与串口调试说明

本文档用于课程设计现场调试：构建、烧录、串口协议、错误码、回零、上位机测试 UI。

## 1. 工程路径

```text
C:\Users\22602\Desktop\SCARA\SCARA_F103
```

## 2. 构建

```powershell
cd C:\Users\22602\Desktop\SCARA\SCARA_F103
cmake --build --preset Debug
```

自检：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\verify_project.ps1
```

当前验证重点：

```text
固件版本：0.29.0-grbl-scara
串口波特率：115200 8N1
通信看门狗：默认关闭
正式运动入口：GRBL-style G-code stream
运动职责边界：UI 生成真实 G-code；MCU 负责 look-ahead、SCARA segment preparation 和 STEP 输出
```

当前项目计划以 `PROMPT.md` 中“2026-06-12 续接项目计划：GRBL-SCARA 四层流式运动内核”为准。旧 BinaryTraj、host-timed 点轨迹和实验文本链路不再作为正式调试入口。

## 3. 烧录

使用 VS Code 任务或现有 CMSIS-DAP/OpenOCD 配置烧录 `build\Debug\SCARA_F103.hex`。

烧录后串口参数：

```text
115200 baud
8 data bits
No parity
1 stop bit
Newline: \n 或 \r\n
```

## 4. 基础串口命令

```text
VERSION          查询固件版本。
HOSTCAP          查询上下位机职责边界。
PING             链路测试，返回 OK PONG。
STATUS           长状态查询。
?                立即请求一帧 <...> 状态。
ERRORS           展开底层错误 bit。
HOME_SENSOR      查看 HOME1/HOME2 输入。
ENABLE 1         使能双轴输出。
ENABLE 0         关闭双轴输出。
STOP             受控停止；主 UI“停止”会清除上位机待发送队列。
ESTOP            急停；主 UI“急停”会保留上位机待发送队列。
CLEAR_ERROR      清除错误位。
ZERO             软件清零。
WATCHDOG OFF     关闭通信看门狗。
WATCHDOG ON 3000 开启 3000 ms 通信看门狗。
```

## 激光 PWM 与继电器

激光加工接口使用：

```text
PA7 / TIM3_CH2   1 kHz PWM，经确认合适的电平转换后连接激光单根 PWM 控制线。
PA2              继电器控制，高电平吸合，上电默认低电平。
STM32 GND        与激光黑线/电源负极共地，作为单根 PWM 控制线的电压参考。
```

根据开发板引脚说明，`PA0` 与板载 `K1/WKUP` 复用，因此不用于激光；`PA2`、`PA7` 不与板载 LED 或按键冲突。继电器使用常开触点：

```text
12V+ -> 保险/实体急停 -> 继电器 COM -> NO -> 激光红线
激光黑线 -> 12V-
```

当前实物只有正负极电源线和一根 PWM 控制线，因此该控制线必须以激光黑线/电源负极为参考。连接 `PA7` 前，必须先测量控制线对激光黑线的空载电压。若接近 12V，禁止直接连接 STM32，必须增加电平转换或光耦。

激光正负极直接接电源且没有继电器/MOSFET 断电级时，固件无法保证复位、烧录、线缆脱落或 MCU 失电期间不出光。必须增加实体急停控制的继电器或常断功率 MOSFET。对常见“高电平有效”输入，还应在电平转换器的激光侧增加合适的下拉，使控制线悬空时保持关闭。

上电安全状态必须是 `0%/关闭`，不是 `1.0%` 最低加工功率。当前固件默认：

```text
APP_LASER_COMMISSIONED = 1   已接入继电器断电通道，允许 LASER ARM。
APP_LASER_PWM_ACTIVE_HIGH = 1   已验证行为：PA7 低电平关闭，高电平增加功率。
上电后立即钳位 PA7 为低电平；空闲、抬笔、停止和故障时完全停止 TIM3，并将 PA7 保持为普通推挽 GPIO 低电平，不再依赖 0% PWM。
启动后 1000 ms 内拒绝 LASER ARM。
```

STM32 复位释放到执行 `main()` 之前，PA7 仍会短暂处于悬空输入状态。对于已验证的高电平有效行为，应在确认控制线电压兼容后于激光输入侧增加合适的下拉，使悬空状态保持关闭；若控制线存在 12V 内部上拉，必须使用电平转换器，禁止直接接入 PA7。只有继电器或常断 MOSFET 才能覆盖 MCU 未运行、烧录和掉电状态。

解除 commissioning 锁前必须断开激光 12V 电源并完成：

```text
1. 测量 PA7 对 STM32 GND：上电、复位和空闲时均为安全电平。
2. 确认 STM32 GND 与激光黑线/电源负极可靠共地。
3. 使用示波器或逻辑分析仪验证 ARM 前无脉冲，L1 时只有设定的低占空比。
4. 确认控制线电压不会超过 STM32 允许范围。
5. 确认输入极性、继电器高电平吸合和常开触点断电路径后，才能接入激光主电源。
6. 最后通过继电器/MOSFET 断电级重新连接激光电源。
```

若确认模块为低电平有效，必须先设计“悬空时保持高电平关闭”的硬件，再将 `APP_LASER_PWM_ACTIVE_HIGH` 改为 `0`。禁止在激光通电状态下试改极性。软件控制不能替代实体急停、护目镜、防护罩和独立断电装置。

```text
LASER POWER 10   设置 1.0% 功率，允许 1..50（0.1%..5.0%）。
LASER ARM        只授权激光加工，不立即吸合继电器，PWM 保持 0%。
LASER DISARM     立即 PWM 归零并释放继电器。
LASER STATUS     查询 armed/ready/marking/power_permille/safety。
G1 ... L0        抬笔段，不出光。
G1 ... L1        落笔段，按设置功率出光。
G1 ... L2        继电器预吸合等待段，PWM 关闭。
```

UI 的“激光开启/关闭”按钮默认关闭。开启后，非 `silent` 轨迹段和手动点动可出光，`silent` 连接段不出光；从抬笔切换到落笔前会插入约 100 ms 的 `LASER_PREP` 等待段，使继电器先吸合。任务完成、停止、急停、回零、断连或错误后自动关闭。预览操作永不 ARM 激光。

`LASER STATUS` 的 `safety` 位定义：bit0=PWM 输出路径已通过 commissioning、bit1=启动锁定已结束、bit2=已 commissioning、bit3=高电平有效。当前配置的 bit3 为 1；空闲时 TIM3 停止属于正常状态。

推荐手动调试顺序：

```text
VERSION
HOSTCAP
WATCHDOG OFF
CLEAR_ERROR
ENABLE 1
G21
G90
G0 X-35.000 Y145.000 F600 ;ID=SEED LIM=1
G1 X-34.900 Y145.000 F800 ;ID=0001 LIM=1
?
```

使用 `SCARA_UI` 点动时，上位机会在第一条点动 G-code 前自动发送：

```text
CLEAR_ERROR
ENABLE 1
```

这样可以避免刚烧录或刚急停后电机未使能，导致下位机在启动运动块时返回 `error:15`。`OK ENABLE 1`、`OK CLEAR_ERROR`、`OK ZERO` 只表示系统命令执行完成，不属于点动 G-code 的小写 `ok` ACK，上位机不会用这些系统 OK 推进点动队列。

### 坐标系与软件零点

当前项目有两个 X 坐标表达：

```text
UI 坐标：左电机为 X=0，右电机为 X=150，UI 零点为 X=75, Y=220。
MCU 坐标：双电机中点为 X=0，左电机约 X=-75，右电机约 X=75。
```

`SCARA_UI` 会自动转换：

```text
发送：X_mcu = X_ui - 75
接收：X_ui  = X_mcu + 75
```

直接用串口工具手动发 G-code 时必须使用 MCU 坐标。例如 UI 中心零点附近应发送 `X0 Y220`，不是 `X75 Y220`。烧录 v0.24.1 后默认软件零点对应固件偏置：

```text
APP_MOTOR1_ZERO_MRAD = 2251
APP_MOTOR2_ZERO_MRAD = 890
APP_PARAM_FLASH_VERSION = 6
```

状态帧的 `MPos:x,y` 会有少量脉冲量化误差。v0.27.5 起，UI、固件和测试默认使用 `6400 PPR`；上位机按半脉冲穿越时刻规划并以固件 timed-DDA 精确回放校验压缩，可将轨迹波纹压到单微步物理分辨率附近，但无法消除机械间隙、杆件弹性或真实失步。`APP_PARAM_FLASH_VERSION=6` 会在首次运行时恢复全部当前源码默认参数。注意：步进电机是开环系统，烧录或 `ZERO` 只改变软件坐标解释，不会自动把实体杆件移动到新对称零点；真实点动前必须确认机械姿态已经与 UI `X=75,Y=220` 对应。

## 5. G-code 通信协议

上位机逐行发送 G-code。下位机成功接收、解析并入队后，返回：

```text
ok
```

示例：

```text
TX: G1 X-34.900 Y145.000 F800 ;ID=0001 LIM=1
RX: ok
```

上位机必须检查：

```text
ok            本行 G-code 已被接收/入队。
error:<code>  本行 G-code 未被正常接受，应停止继续发送轨迹。
<...>/STAT    状态帧不是 ACK，不能推进发送队列。
```

上位机必须等到小写 `ok` 后才能发送下一条正式轨迹指令。

## 6. 支持的 G-code 子集

```text
G0 X Y F      快速定位。
G1 X Y F      直线插补点。
G20           英寸单位，不推荐使用。
G21           毫米单位，推荐。
G90           绝对坐标。
G91           相对坐标。
G4 P          暂停，占位支持。
M0/M2/M30     停止/程序结束，占位支持。
$X            清除报警。
$H            启动回零。
$G            查询 G-code 模态。
?             立即状态查询。
!             暂停。
~             恢复。
```

正式轨迹推荐格式：

```text
G1 X120.050 Y80.010 F1200 ;ID=0123 LIM=1
```

字符含义：

```text
G      G-code 指令前缀。
0/1    G0 快速定位，G1 直线插补点。
X      末端 X 坐标，单位 mm。
Y      末端 Y 坐标，单位 mm。
F      进给速度，单位 mm/min，由上位机规划。
;      注释开始，MCU 不参与运动解析，但会完整回显。
ID     上位机轨迹点编号，便于日志匹配。
LIM    上位机已完成限位检查标记，建议 `LIM=1`。
\n     一行命令结束。
```

## 7. 状态回传

自动状态帧约 5 Hz 推送，也可以发送 `?` 立即查询：

```text
<Idle|MPos:x,y|JPos:p1,p2|FS:feed_mm_min,spindle|Bf:planner_free,rx_free|Q:planner_used|E:n|Seg:count,free,low,underrun|H:h1,h2|HS:home_state|A1:en,run,cur_pps,tgt_pps|A2:en,run,cur_pps,tgt_pps|Lz:armed,ready,marking,power>
```

字段含义：

```text
Idle/Run 当前是否空闲或有运动/队列。
MPos     MCU 根据软件脉冲正解估算的末端 XY。
JPos     双电机软件脉冲计数。
FS       当前 G-code 进给和主轴/功率字。
Bf       规划缓冲剩余、RX 行队列剩余。
Q        规划缓冲已用段数。
E        步进底层错误 bit。
Seg      timed segment 缓冲计数、剩余、低水位和欠载次数。
H        HOME1/HOME2 输入，1 表示触发。
HS       回零状态机阶段。
A1/A2    轴状态：使能、运行、当前 pps、目标 pps。
Lz       激光状态：armed、relay_ready、marking、power_permille。
```

状态帧不是某条 G-code 的 ACK，可能穿插在 `ok` 前后。上位机需要按行区分：

```text
ok ...       指令应答。
error:<n>    指令错误。
<...>        状态推送。
```

## 8. 错误码速查

固件有两类错误：

```text
error:<code>   G-code 流协议错误，表示当前这一行没有被正常接受。
E:<bits>       状态帧里的步进底层错误位，可以叠加。
```

`error:<code>`：

```text
error:2    数字字段解析失败，例如 Xabc、G 后面没有数字。
error:3    不支持的 $ 命令，当前只支持 $X、$H、$G。
error:4    F 速度字段非法，例如 F0、F-100 或 F 后面不是数字。
error:5    $H 回零启动失败，通常是正在运动、回零未空闲或错误未清除。
error:8    上位机发送太快，已有一条 pending 行，必须等 ok 后再发下一条。
error:15   运动目标被拒绝，常见原因是几何逆解失败、电机未使能、急停或当前有错误位。
error:20   不支持的 G/M/字段。
error:25   同一行重复出现 X 或 Y。
```

`E:<bits>`：

```text
E:0    无底层错误。
E:1    软限位错误。v0.23.1 正式轨迹默认不再由下位机做软限位。
E:2    急停错误。
E:4    通信看门狗超时。
E:8    未使能时尝试运动。
```

组合值：

```text
E:3    1 + 2，软限位 + 急停。
E:5    1 + 4，软限位 + 通信看门狗超时。
E:12   4 + 8，通信看门狗超时 + 未使能运动。
```

特别注意：

```text
error:4  是 G-code 的 F 字段错误。
E:4      是通信看门狗超时。
error:5  是 $H 回零启动失败。
E:5      是底层错误位组合。
```

## 9. 限位职责

从 `v0.23.1` 开始，正式轨迹限位由上位机负责：

```text
APP_HOST_OWNS_LIMIT_CHECKS = 1
HOSTCAP ... grbl_stream=1 scara_plan=1 gcode_arc=1 jog=1 binary_traj=0 dda=1
```

上位机发送轨迹前必须遍历所有点，完成：

```text
XY 工作空间检查。
五连杆逆解是否存在。
正解回代误差检查。
关节角范围检查。
电机脉冲范围检查。
轨迹段是否穿越禁区。
速度、加速度、拐角速度是否保守。
限位开关状态是否允许继续运动。
```

下位机只保留：

```text
G-code 语法检查。
五连杆几何逆解是否存在。
电机是否使能。
STOP/ESTOP。
回零输入和状态回传。
```

## 10. `$H` 第二次 `error:5`

`$H` 返回 `ok` 只表示“回零流程已经启动”，不表示已经完成。

如果第一次 `$H` 后限位开关没有触发，状态可能停留在：

```text
HS:Axis1Search
HS:Axis1Return
HS:Axis2Search
HS:Axis2Return
```

此时第二次发送 `$H` 会返回：

```text
error:5
```

这是正常现象，表示回零流程或电机运动仍未结束。正确流程：

```text
1. 发送 $H。
2. 持续发送 ? 或观察自动状态。
3. 等待 HS:Done。
4. 如果卡住，发送 HOME_SENSOR 检查限位输入。
5. 需要中断时发送 STOP 或 ESTOP，再 CLEAR_ERROR。
```

`v0.23.1` 已允许在 `HS:Done` 或 `HS:Error` 状态下重新 `$H`；如果仍在搜索/回退过程中，第二次 `$H` 仍会返回 `error:5`。

## 11. 高频轨迹测试

```powershell
cd C:\Users\22602\Desktop\SCARA\SCARA_F103
powershell -NoProfile -ExecutionPolicy Bypass -File tools\host_planned_stream_stress.ps1 -Port COM13 -Count 3000 -FeedMin 500 -FeedMax 1800 -EnableMotion
```

UI 控键矩阵检查：
```powershell
cd C:\Users\22602\Desktop\SCARA\SCARA_F103
powershell -NoProfile -ExecutionPolicy Bypass -File tools\ui_control_matrix_check.ps1 -Port AUTO
```

UI 轨迹规划压力检查：
```powershell
cd C:\Users\22602\Desktop\SCARA\SCARA_F103
powershell -NoProfile -ExecutionPolicy Bypass -File tools\ui_trajectory_stress.ps1 -Port AUTO -Count 3000 -FeedMmMin 900
```

该脚本按上位机轨迹按钮逻辑生成：
```text
G1 直线：当前点 -> 目标点
G2 顺圆：当前点 -> 目标点，按半径选择顺时针圆弧
G3 逆圆：当前点 -> 目标点，按半径选择逆时针圆弧
小车轨迹1：按 SOURCE\小车轨迹1.png 尺寸生成固定轮廓
小车轨迹2：按 SOURCE\小车轨迹2.png 尺寸生成固定轮廓
```

发送前会遍历整条路径。若超限，会显示具体点、具体轴或结构和超出量，例如 M1/M2 角度超限、左右基座距离超限、主动臂交叉或主动臂低于基座线。

脚本会显示每段 `PATH SAFE`、每 100 点进度、drain 状态和最终 `PATH PASS`。正式 ACK 以小写 `ok` 为准，状态帧只用于 drain 判断，不会逐条刷屏。

上位机离线规划检查：

```powershell
cd C:\Users\22602\Desktop\SCARA
$env:PYTHONDONTWRITEBYTECODE='1'
C:\Users\22602\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe SCARA_UI\tests\trajectory_planner_check.py
```

该检查使用真实上位机规划器输出的点和 `F mm/min`，验证 G1/G2/G3 的相邻速度、圆弧中段速度、小车 1/2 图纸尺寸和五连杆限位。

## 12. PyQt 上位机仿真 UI

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

缺少依赖：

```powershell
pip install PyQt5 pyserial
```

UI 功能：

```text
生成直线 + 圆弧 3000 点轨迹。
上位机侧做五连杆正逆解和速度规划。
实时显示 TX/RX。
主 UI 绘制末端轨迹、五连杆姿态和状态字段。
速度/加速度曲线由 `SCARA_UI\V_monitor.py` 监控窗口显示，运行 `SCARA_UI\main.py` 时会自动启动。
支持手动发送串口命令。
```
