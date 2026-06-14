这是一份面向 **当前 `SCARA_F103` CubeMX 配置** 和 **并联 SCARA 机械臂结构** 的下位机代码编写计划。

使用开环57步进电机,配合 杰美康DM556数字步进驱动器和STM32C8T6单片机,相关手册在SORCE文件夹下

请根据这个目标计划,逐步完善项目

中间所需要的结构等各种参数,可以先拟定,后续我会在全局接口统一修改

每次版本迭代,写入一个Version.md文件中

最后需要生成一份操作指南Control.md,教学如何使用VScode+openocd配合cmake进行编译和调试,来确保下位机的正确性

---

# 〇、2026-06-12 续接项目计划：GRBL-SCARA 四层流式运动内核

本节根据 2026-06-11 至 2026-06-12 历史会话续写，是当前项目的优先计划。下面旧章节保留为早期分阶段背景；若旧章节与本节冲突，以本节为准。

## 0.1 历史会话结论

当前问题的根因不再按单点“解算错误”处理，而是按整条运动链路重构处理：旧链路在点动、反向点动和长轨迹发送时容易出现加减速不连续、同步等待、队列枯竭或上位机一次性展开大量轨迹点的问题。

当前决策锁定为：

- 正式运动入口统一为真实 G-code 流，`BinaryTraj`、host-timed 点轨迹、UI 二进制发送模式和实验文本链路不再作为正式路径。
- 以 GRBL v1.1 的 parser、look-ahead planner、step segment preparation、realtime command 和 character-counting streaming 思路为参考，移植为适配五连杆 SCARA 的 STM32F103 轻量实现。
- 不做整仓库回退，也不直接覆盖当前工程；保留已经验证的回零、限位、激光安全、400/6400 PPR 设置、UI 坐标转换和现场调试工具。
- 点动统一使用 `$J=G91 ...`，单次点动必须 exact-stop，反向点动不得与前一次运动混合规划。

## 0.2 当前实现基线

当前 `v0.29.0` 已完成第一阶段骨架：

- 上位机：`GcodeJob` 懒生成最多 64 条待发送命令，串口未确认窗口限制为 224 bytes，串口读写移入 `QThread`。
- MCU RX：保留 256-byte DMA 环形接收，`?`、`!`、`~`、Ctrl-X 和 Jog Cancel 绕过普通 G-code 解析。
- Planner：32 个笛卡尔规划块，支持 G0/G1/G2/G3、I/J 与 R 圆弧、junction deviation、reverse/forward look-ahead。
- Segment：16 个 timed segment，每段目标 5 ms，主循环准备 segment，TIM2 只消费 step-event，TIM1/TIM4 用 one-pulse 输出固定脉宽 STEP。
- SCARA：segment preparation 阶段做五连杆逆解，按关节 PPS 限制自适应缩短 segment。
- Laser：M3/M4/M5 和 S0..1000 接入规划屏障，M4 可按实时速度缩放功率，停止、暂停、故障、空闲均应关断 PWM。

已通过的离线验证：

```text
powershell -NoProfile -ExecutionPolicy Bypass -File SCARA_F103\tools\verify_project.ps1
C:\Users\22602\.conda\envs\Robot\python.exe SCARA_UI\tests\sender_strategy_check.py
C:\Users\22602\.conda\envs\Robot\python.exe SCARA_UI\tests\sender_benchmark_check.py
C:\Users\22602\.conda\envs\Robot\python.exe SCARA_UI\tests\trajectory_planner_check.py
```

验证结果：固件构建通过，Flash 当前约 55536 bytes；100000-command sender benchmark 的 `peak_pending=64`。

## 0.3 下一阶段必须完成

### A. 文档与接口对齐

- `Control.md`、`Work.md` 和测试流程必须统一到 v0.29.0：正式运动链路是 GRBL-style G-code stream，而不是旧的逐条 ACK G1、BinaryTraj 或 host-timed segment。
- 状态帧以 `<Idle|MPos|JPos|FS|Bf|Seg...>` 为现场调试基准；旧 `Q:`、`M:`、`P:` 字段只作为兼容或历史说明。
- 保留 GPLv3 来源声明：移植或改写自 GRBL 的 planner/stepper 思路必须在 `GRBL_LICENSE.md` 和源文件注释中说明。

### B. 固件硬化

- 审核 `motion_planner.c` 中关节加速度限制的单位：当前 segment 跳速约束使用旧 `APP_ACCEL_MAX` 脉冲参数，需要明确它是 `pps/s` 还是旧位置规划残留，并给 6400 PPR、DM556、当前机构的默认安全值。
- 补齐 first-segment、exact-stop、dwell、laser barrier、Jog Cancel、hold/resume/reset 的边界测试，防止 `s_last_segment_*` 状态在停止、清空、换任务后继承旧速度。
- 确认 TIM2 variable step-event 在零步、单轴步、双轴同步步、高 PPS 和方向翻转时不产生阻塞延迟或方向建立时间不足。
- 对 G2/G3 圆弧在小半径、跨象限、R 正负、I/J 非法半径误差场景做 parser/planner 单元测试。

### C. 上位机清理

- UI 正式发送路径只能生成真实几何 G-code；预览点可以保留为显示数据，但发送时不得把整条轨迹转换成 `list(points)`。
- 删除或隔离残留的 binary helper、binary stress 按钮、旧实验文本入口和旧模式切换文案；若函数暂时保留给回放/分析，必须从 UI 主流程断开。
- 串口线程不得在 UI 主线程同步等待 ACK、状态查询或调用 `processEvents()` 来维持发送。

### D. 实机验收

- 烧录后先在电机和激光功率断开状态下验证 `VERSION`、`?`、`!`、`~`、Ctrl-X、Jog Cancel 和混合 G0/G1/G2/G3 burst。
- 用示波器确认 TIM1/TIM4 STEP 脉宽、方向建立时间、加减速 step-event 间隔连续性和空闲/暂停/故障时无激光 PWM。
- 低速、默认 `20 mm/s / 100 mm/s^2`、目标高速三档分别执行至少 30 次左右 30 mm exact-stop jog 往返。
- 验收目标：无 RX overflow、planner starvation、segment underrun；软件脉冲闭合；物理回程误差不超过 0.3 mm；反向点动不再形成 1 mm 级垂直误差或扁平槽轨迹。

## 0.4 后续版本建议

- `v0.29.1`：文档同步、旧 UI 链路清理、segment 加速度单位校准、离线测试补齐。
- `v0.29.2`：实机串口 burst、realtime、示波器检查和安全链路验收。
- `v0.30.0`：完成 30 次往返与长任务流式加工验收后，冻结为 GRBL-SCARA baseline。

---

# 一、当前硬件与工程基础确认

根据当前项目配置，下位机基础资源如下：

| 功能 | 当前配置 |
|---|---|
| MCU | STM32F103C8T6 |
| 主频 | HSE 8 MHz → PLL 72 MHz |
| M1 脉冲 | PA8 / TIM1_CH1 / PWM |
| M1 方向 | PB12 / GPIO Open-Drain |
| M1 使能 | PB13 / GPIO Open-Drain |
| M2 脉冲 | PB6 / TIM4_CH1 / PWM |
| M2 方向 | PB7 / GPIO Open-Drain |
| M2 使能 | PB8 / GPIO Open-Drain |
| 上位机通信 | USART1 / PA9 TX / PA10 RX |
| USART1 RX | DMA1_Channel5 Circular |
| USART1 TX | DMA1_Channel4 Normal |
| 轨迹调度 | TIM2 / 1 kHz 中断 |
| 调试烧录 | PA13 / PA14 SWD |
| LED | 当前未使用 PC13，避免复用冲突 |

这个配置适合正式开发双步进电机开环控制。

你当前 PUL/DIR/ENA 都是 **Open-Drain / 开漏输出**，因此推荐接法是：

| DM556 / 2DM556 | STM32 |
|---|---|
| PUL+ | +5V |
| PUL- | PA8 / PB6 |
| DIR+ | +5V |
| DIR- | PB12 / PB7 |
| ENA+ | +5V 或不接 |
| ENA- | PB13 / PB8 |

如果你实际是共阴直连，即 PUL- / DIR- 接 GND，STM32 接 PUL+ / DIR+，则需要把 PWM 和 GPIO 改为 Push-Pull，否则高电平可能不可靠。

---

# 二、机械结构理解与控制对象定义

你现在的机械臂是并联 SCARA 构型：

> 两个电机分别驱动两个主动臂，主动臂通过轴承、连杆、从动臂带动末端执行器运动。

这类结构通常类似五杆并联机构，具有：

1. 两个主动关节角：
   - `theta1`
   - `theta2`

2. 两个主动臂长度：
   - `L1`
   - `L2`

3. 两个从动臂长度：
   - `L3`
   - `L4`

4. 两个电机基座间距：
   - `B`

5. 末端坐标：
   - `x`
   - `y`

下位机最底层控制对象不是末端 `x/y`，而是两个电机的目标脉冲：

```text
motor1_target_pulse
motor2_target_pulse
```

后续逐步实现：

```text
脉冲控制
↓
角度控制
↓
关节空间轨迹控制
↓
笛卡尔空间轨迹控制
↓
示教与轨迹复现
```

---

# 三、总体软件架构

建议不要直接在 `Core/Src/main.c` 里堆功能，而是在工程中新增 `UserApp/` 目录，分模块开发。

推荐结构：

```text
SCARA_F103/
  UserApp/
    app_config.h
    board_pins.h
    stepper_driver.h
    stepper_driver.c
    motion_planner.h
    motion_planner.c
    trajectory.h
    trajectory.c
    serial_dma.h
    serial_dma.c
    protocol.h
    protocol.c
    scara_kinematics.h
    scara_kinematics.c
    teach.h
    teach.c
    app_main.h
    app_main.c
```

各模块职责如下：

| 模块 | 职责 |
|---|---|
| `app_config` | 全局参数、限速、机械臂尺寸、细分、电机方向 |
| `board_pins` | 当前 CubeMX 引脚与定时器映射 |
| `stepper_driver` | 控制 TIM1/TIM4 输出脉冲、方向、使能、当前位置估算 |
| `motion_planner` | 单轴速度、加速度、相对/绝对位置规划 |
| `trajectory` | 双轴同步、轨迹插补、队列执行 |
| `serial_dma` | USART1 DMA 环形接收、发送队列 |
| `protocol` | 上位机命令解析、状态回传 |
| `scara_kinematics` | 并联 SCARA 正逆运动学 |
| `teach` | 示教点记录、轨迹复现 |
| `app_main` | 初始化各模块，主循环调度 |

---

# 四、开发阶段规划

建议分 7 个阶段写代码，每个阶段都可单独测试，不要一开始就写完整 SCARA 轨迹。

---

## 阶段 1：底层步进脉冲输出

目标：

> 让 M1、M2 能稳定正反转、停止、急停。

### 需要实现

在 `stepper_driver.c/h` 中实现：

1. 初始化步进驱动模块。
2. 启动 TIM1_CH1、TIM4_CH1 PWM。
3. 设置 PWM 频率。
4. 设置方向 GPIO。
5. 设置使能 GPIO。
6. 停止单轴输出。
7. 急停双轴输出。
8. 维护当前速度 `current_pps`。
9. 维护目标速度 `target_pps`。
10. 维护软件位置 `position_pulse`。

### 推荐 API

| 函数 | 作用 |
|---|---|
| `Stepper_Init()` | 初始化状态 |
| `Stepper_Enable(axis, enable)` | 使能/释放驱动器 |
| `Stepper_SetDir(axis, dir)` | 设置方向 |
| `Stepper_SetPps(axis, pps)` | 设置某轴脉冲频率 |
| `Stepper_Stop(axis)` | 停止某轴 |
| `Stepper_EStopAll()` | 急停所有轴 |
| `Stepper_UpdatePosition(dt)` | 根据脉冲/速度更新软件位置 |
| `Stepper_GetState()` | 获取状态用于串口回传 |

### 注意 TIM1

TIM1 是高级定时器，启动 M1 PWM 时必须确保主输出使能。优先使用 HAL 的 `HAL_TIM_PWM_Start()`，不要一开始就手写寄存器。

### PWM 更新策略

后续不能频繁 Stop/Start。

推荐：

- 速度为 0 时关闭通道或将 CCR 设为 0；
- 速度非 0 时根据 pps 更新 ARR 和 CCR；
- CCR 始终为 ARR 的一半，保持 50% 占空比。

### 限制参数

初期建议：

| 参数 | 建议值 |
|---|---:|
| 最小有效 pps | 16 pps |
| 初期最大 pps | 10000 pps |
| 后期最大 pps | 50000 pps |
| 初期加速度 | 1000~5000 pps/s |
| DIR 建立时间 | 1 ms 保守处理 |

---

## 阶段 2：USART1 DMA 通信协议

目标：

> 上位机可以稳定发送命令，下位机可以回传状态。优先支持“上位机发送脉冲命令控制运动”。

你现在 USART1 已经配置了 RX DMA Circular 和 TX DMA Normal，但代码中还没有启动 DMA 接收。

### 需要实现

在 `serial_dma.c/h` 中实现：

1. 启动 USART1 RX DMA Circular。
2. 使用 DMA 写指针解析新数据。
3. 按 `\n` 或 `\r\n` 组包。
4. 提供行命令缓冲区。
5. 支持非阻塞发送。
6. 避免在中断中解析复杂命令。
7. 避免在中断中发送长字符串。

### 通信建议

考虑你使用塔克无线 DAP，建议默认采用 **请求-应答模式**，不要默认高频主动刷状态。

默认：

```text
上位机 -> STATUS
下位机 -> STAT ...
```

可选：

```text
STREAM ON
STREAM OFF
```

`STREAM ON` 才周期发送状态。

### 初期协议命令

| 命令 | 作用 |
|---|---|
| `PING` | 返回 `OK PONG` |
| `STATUS` | 回传一次状态 |
| `ENABLE 1` | 使能驱动 |
| `ENABLE 0` | 释放驱动 |
| `SPEED m1_pps m2_pps` | 双轴连续速度 |
| `STOP` | 减速停止 |
| `ESTOP` | 立即停止 |
| `ZERO` | 软件位置清零 |
| `MOVE_REL m1_pulse m2_pulse v1 v2` | 双轴相对脉冲位移 |
| `MOVE_ABS m1_pulse m2_pulse v1 v2` | 双轴绝对软件位置 |
| `ACCEL a1 a2` | 设置两轴加速度 |
| `STREAM ON/OFF` | 打开/关闭状态流 |

### 状态返回

建议统一：

```text
STAT tick=... mode=... err=... m1_pos=... m2_pos=... m1_target=... m2_target=... m1_cur=... m2_cur=... m1_run=... m2_run=...
```

字段用整数，少用浮点，便于解析。

---

## 阶段 3：单轴与双轴位置控制

目标：

> 上位机发送目标脉冲，下位机完成加减速运动。

你说通信部分当前希望：

> 上位机发送对应的电机脉冲，控制运动。

因此这一阶段先不做 SCARA 逆解，只做电机脉冲层。

### 要实现的运动模式

1. 连续速度模式：
   ```text
   SPEED m1_pps m2_pps
   ```

2. 相对位移模式：
   ```text
   MOVE_REL m1_delta_pulse m2_delta_pulse v1_pps v2_pps
   ```

3. 绝对位置模式：
   ```text
   MOVE_ABS m1_target_pulse m2_target_pulse v1_pps v2_pps
   ```

4. 回软件零点：
   ```text
   MOVE_ABS 0 0 v1 v2
   ```

5. 软件清零：
   ```text
   ZERO
   ```

### 关键逻辑

绝对位置运动必须是：

```text
delta = target_position - current_position
```

不能每次从 0 开始。

相对位移运动必须是：

```text
target_position = current_position + delta
```

### 双轴同步

`MOVE_REL` 和 `MOVE_ABS` 需要考虑双轴同时到达。

简单策略：

1. 计算两轴位移绝对值：
   ```text
   d1 = abs(delta1)
   d2 = abs(delta2)
   ```

2. 给定最大速度 `v1_max`、`v2_max`。

3. 计算预计时间：
   ```text
   t1 = d1 / v1
   t2 = d2 / v2
   T = max(t1, t2)
   ```

4. 调整较短轴速度：
   ```text
   v_short = d_short / T
   ```

这样双轴大致同步到达。

---

## 阶段 4：1 kHz 轨迹调度器

目标：

> 使用 TIM2 1 kHz 中断统一更新运动状态，而不是主循环里用 delay 控制。

### TIM2 中断中应该做什么

可以做：

- 更新每轴当前速度；
- 按加速度逼近目标速度；
- 判断是否需要减速；
- 更新 PWM ARR/CCR；
- 更新软件位置估算；
- 设置状态标志。

不要做：

- 串口 printf；
- 复杂字符串解析；
- 大量浮点运算；
- SCARA 逆解；
- 阻塞等待。

### 运动状态机

建议每轴维护：

| 字段 | 说明 |
|---|---|
| `enabled` | 是否使能 |
| `mode` | IDLE / SPEED / MOVE / ESTOP |
| `dir` | 当前方向 |
| `target_pps` | 目标速度 |
| `current_pps` | 当前速度 |
| `accel_pps_s` | 加速度 |
| `position_pulse` | 软件当前位置 |
| `target_position_pulse` | 目标位置 |
| `remaining_pulse` | 剩余脉冲 |
| `running` | PWM 是否输出 |
| `error` | 错误标志 |

### 减速判断

位移模式下，需要提前减速：

```text
stop_distance = current_speed^2 / (2 * accel)
```

当剩余位移小于停止距离时开始减速。

---

## 阶段 5：SCARA 并联机构运动学

目标：

> 将末端坐标 `x/y` 转换为两个电机角度 `theta1/theta2`，再转换为脉冲。

这个阶段在电机脉冲控制稳定后再做。

### 坐标关系

对于并联 SCARA / 五杆机构，通常需要定义：

| 参数 | 含义 |
|---|---|
| `base_distance` | 两个电机轴之间距离 |
| `active_arm_len_1` | 左主动臂长度 |
| `active_arm_len_2` | 右主动臂长度 |
| `passive_arm_len_1` | 左从动臂长度 |
| `passive_arm_len_2` | 右从动臂长度 |
| `motor1_zero_offset` | 电机 1 零点角度偏置 |
| `motor2_zero_offset` | 电机 2 零点角度偏置 |
| `motor1_dir_sign` | 电机 1 方向符号 |
| `motor2_dir_sign` | 电机 2 方向符号 |
| `microstep1` | 电机 1 每圈脉冲 |
| `microstep2` | 电机 2 每圈脉冲 |

### 推荐先实现关节空间

先实现：

```text
JOINT theta1_rad theta2_rad vmax accel
```

也就是直接控制两个主动关节角。

之后再实现：

```text
XY x_mm y_mm vmax accel
```

不要一开始直接做复杂笛卡尔轨迹。

### 逆运动学输出

逆解输出：

```text
theta1_rad
theta2_rad
```

再换算为：

```text
pulse1 = theta1_rad / (2π) * microstep1 * reducer_ratio1
pulse2 = theta2_rad / (2π) * microstep2 * reducer_ratio2
```

如果没有减速器：

```text
reducer_ratio = 1
```

### 必须加入工作空间检查

并联 SCARA 不是所有 `x/y` 都可达，必须检查：

1. 点到左电机中心距离是否在范围内；
2. 点到右电机中心距离是否在范围内；
3. 两圆交点是否存在；
4. 解是否满足机械装配分支；
5. 角度是否超过机械限位；
6. 是否接近奇异位形。

如果不可达，返回错误，不运动。

---

## 阶段 6：轨迹插补

目标：

> 上位机给目标点或轨迹段，下位机本地插补，生成连续的关节目标。

### 推荐先实现三类轨迹

#### 1. 关节空间点到点

命令：

```text
JOGJ theta1 theta2 vmax accel
```

或使用脉冲：

```text
MOVE_ABS pulse1 pulse2 v1 v2
```

#### 2. 笛卡尔直线

命令：

```text
LINE x1 y1 x2 y2 duration
```

或：

```text
MOVL x y vmax accel
```

下位机每 1 ms 插补：

```text
x(t), y(t)
↓
inverse_kinematics
↓
theta1(t), theta2(t)
↓
pulse1(t), pulse2(t)
```

#### 3. 轨迹点队列

命令：

```text
TRAJ_BEGIN count
TRAJ_POINT x y t
TRAJ_POINT x y t
...
TRAJ_END
```

下位机保存轨迹点，TIM2 中断按时间执行。

---

## 阶段 7：示教功能

目标：

> 支持记录当前位置、保存示教点、按顺序复现。

由于当前系统没有编码器，所谓示教有两种方式。

### 方式 A：软件示教

上位机控制机械臂移动到某个位置，然后发送：

```text
TEACH_ADD
```

下位机记录当前软件位置：

```text
pulse1
pulse2
```

或者记录逆解坐标：

```text
x
y
```

这种方式依赖软件位置估算。

### 方式 B：手动拖动示教

如果没有编码器，不能可靠知道手动拖动后的真实位置。  
因此不建议宣称支持“断电拖动示教”。

如果后续加编码器或关节传感器，可以支持真实示教。

### 示教命令建议

| 命令 | 作用 |
|---|---|
| `TEACH_CLEAR` | 清空示教点 |
| `TEACH_ADD` | 记录当前点 |
| `TEACH_LIST` | 返回点列表 |
| `TEACH_RUN` | 运行示教轨迹 |
| `TEACH_STOP` | 停止示教 |
| `TEACH_SAVE` | 保存到 Flash，后续实现 |
| `TEACH_LOAD` | 从 Flash 读取，后续实现 |

初期不要急着写 Flash 存储，先用 RAM 点表。

---

# 五、协议分层建议

为了后续扩展，不要把协议命令写得混乱。建议按层次分。

---

## 1. 基础通信命令

| 命令 | 返回 |
|---|---|
| `PING` | `OK PONG` |
| `HELP` | 命令列表 |
| `STATUS` | 当前状态 |
| `STREAM ON` | 开启周期状态 |
| `STREAM OFF` | 关闭周期状态 |

---

## 2. 电机底层命令

| 命令 | 说明 |
|---|---|
| `ENABLE 1/0` | 使能/释放 |
| `ZERO` | 当前软件位置清零 |
| `SPEED pps1 pps2` | 连续速度 |
| `MOVE_REL dp1 dp2 v1 v2` | 相对脉冲 |
| `MOVE_ABS p1 p2 v1 v2` | 绝对脉冲 |
| `ACCEL a1 a2` | 设置加速度 |
| `STOP` | 减速停止 |
| `ESTOP` | 立即停脉冲 |

---

## 3. 关节空间命令

| 命令 | 说明 |
|---|---|
| `JOINT theta1 theta2 vmax accel` | 运动到关节角 |
| `JREL dtheta1 dtheta2 vmax accel` | 关节相对运动 |
| `JSTATUS` | 返回关节角 |

单位建议：

```text
theta: mrad 或 urad 整数
速度: mrad/s
加速度: mrad/s^2
```

下位机尽量少用浮点文本。

---

## 4. 笛卡尔空间命令

| 命令 | 说明 |
|---|---|
| `MOVL x y vmax accel` | 末端直线运动 |
| `GOTOXY x y vmax accel` | 末端点到点 |
| `WHERE` | 返回估算末端坐标 |
| `HOME` | 走软件零点或回安全位 |

单位建议：

```text
x/y: 0.001 mm 即 um 整数
速度: mm/s 或 um/s
```

---

# 六、安全限制设计

SCARA 机械臂必须加限制，不能只听上位机命令。

## 1. 电机层限制

| 限制 | 建议 |
|---|---|
| 最大 pps | 初期 10000，验证后 50000 |
| 最大加速度 | 初期 5000 pps/s |
| 最小 pps | 16 pps 以下停脉冲 |
| 最大软件位置 | 根据机械极限设置 |
| 最小软件位置 | 根据机械极限设置 |
| DIR 建立时间 | 1 ms |
| 急停 | 立即关闭 PWM |

---

## 2. 关节层限制

| 限制 | 示例 |
|---|---|
| theta1_min | 根据机械臂实测 |
| theta1_max | 根据机械臂实测 |
| theta2_min | 根据机械臂实测 |
| theta2_max | 根据机械臂实测 |
| joint_speed_max | 根据电机能力 |
| joint_accel_max | 根据负载能力 |

---

## 3. 笛卡尔层限制

| 限制 | 说明 |
|---|---|
| 工作空间边界 | 不允许目标点超出可达区域 |
| 奇异区域 | 靠近奇异点减速或禁止 |
| 最小连杆夹角 | 避免机械干涉 |
| 最大连杆夹角 | 避免翻肘或越界 |
| 软限位 | 不运动并返回错误 |

---

# 七、状态机设计

建议系统有一个全局状态：

| 状态 | 含义 |
|---|---|
| `BOOT` | 上电初始化 |
| `IDLE` | 空闲 |
| `DISABLED` | 驱动未使能 |
| `SPEED` | 连续速度 |
| `MOVE_PULSE` | 脉冲位置运动 |
| `MOVE_JOINT` | 关节运动 |
| `MOVE_XY` | 笛卡尔运动 |
| `TRAJECTORY` | 轨迹队列执行 |
| `TEACH` | 示教模式 |
| `STOPPING` | 减速停止 |
| `ESTOP` | 急停 |
| `ERROR` | 错误锁定 |

错误处理建议：

- 普通错误返回 `ERR ...`，不一定锁死；
- 运动越界必须拒绝执行；
- 急停后进入 `ESTOP`，必须 `CLEAR_ERROR` 或 `RESET` 后恢复；
- 通信丢失时可以自动减速停止。

---

# 八、主循环与中断分工

## main loop 负责

- 串口命令解析；
- 执行协议命令；
- 更新状态上报；
- 处理轨迹队列非实时部分；
- 错误处理；
- 示教点管理。

## TIM2 1kHz 中断负责

- 单轴速度斜坡更新；
- 位移减速判断；
- 双轴同步状态更新；
- PWM 频率更新；
- 软件位置估算；
- 设置运动完成标志。

## USART/DMA 中断负责

- DMA 半传输/完成处理；
- IDLE 中断处理；
- 标记有新串口数据；
- 不解析复杂命令。

---

# 九、推荐开发顺序

不要直接写完整 SCARA。建议严格按以下顺序：

## 第 1 步：编译框架

新增 `UserApp/`，加入 CMake，空函数能编译。

验收：

```text
工程可编译
可烧录
main loop 正常运行
```

---

## 第 2 步：串口 PING/STATUS

实现 USART DMA 接收和简单协议。

验收：

```text
PING -> OK PONG
STATUS -> STAT tick=...
```

---

## 第 3 步：单轴 PWM 测试

实现：

```text
SPEED 500 0
SPEED -500 0
SPEED 0 500
SPEED 0 -500
STOP
ESTOP
```

验收：

- M1 正反转正常；
- M2 正反转正常；
- STOP 减速；
- ESTOP 立即停。

---

## 第 4 步：位置运动

实现：

```text
MOVE_REL 1600 0 500 0
MOVE_REL -1600 0 500 0
MOVE_ABS 0 0 500 500
ZERO
```

验收：

- 每次运动都从当前位置开始；
- 不会莫名回零；
- 当前位置状态正确。

---

## 第 5 步：双轴同步

实现双轴同时到达。

验收：

```text
MOVE_REL 1600 3200 800 800
```

两轴同时开始，同时结束。

---

## 第 6 步：关节角控制

实现：

```text
JOINT theta1 theta2 vmax accel
```

先不做末端 XY。

验收：

- 上位机输入两个关节角；
- 下位机换算成脉冲；
- 双轴同步运动到目标角。

---

## 第 7 步：SCARA 逆运动学

实现：

```text
GOTOXY x y vmax accel
```

验收：

- 可达点可以运动；
- 不可达点返回错误；
- 软限位有效。

---

## 第 8 步：直线插补

实现：

```text
MOVL x y vmax accel
```

下位机 1 kHz 插补或主循环预生成队列。

验收：

- 末端轨迹近似直线；
- 速度连续；
- 不抖动；
- 越界提前报错。

---

## 第 9 步：轨迹队列

实现：

```text
TRAJ_BEGIN
TRAJ_POINT
TRAJ_END
TRAJ_RUN
TRAJ_STOP
```

验收：

- 多点连续执行；
- 上位机不用实时发送每个脉冲；
- 通信短暂停顿不影响已缓存轨迹。

---

## 第 10 步：示教

实现 RAM 示教点：

```text
TEACH_CLEAR
TEACH_ADD
TEACH_LIST
TEACH_RUN
```

验收：

- 可记录当前位置；
- 可复现示教点序列；
- STOP/ESTOP 可中断。

---

# 十、需要 AI 写代码时的分阶段提示词

后续你可以不要一次性让 AI 写全部代码，而是逐步给它任务。

## 第一次任务：工程框架

目标：

> 在 `SCARA_F103` 中新增 `UserApp`，建立 `app_main`、`stepper_driver`、`serial_dma`、`protocol` 基础框架，加入 CMake，保持可编译。

---

## 第二次任务：步进 PWM

目标：

> 基于 PA8/TIM1_CH1 和 PB6/TIM4_CH1，实现两路 DM556 步进 PWM 输出，支持设置 pps、方向、停止、急停。

---

## 第三次任务：USART DMA 协议

目标：

> 基于 USART1 DMA Circular，实现非阻塞行协议，支持 PING/STATUS/SPEED/STOP/ESTOP。

---

## 第四次任务：位置运动

目标：

> 实现 MOVE_REL、MOVE_ABS、ZERO、ACCEL，使用 TIM2 1 kHz 更新速度斜坡和软件位置。

---

## 第五次任务：SCARA 运动学

目标：

> 根据并联 SCARA 五杆机构参数，实现正逆运动学、工作空间检查、关节角到脉冲换算。

---

## 第六次任务：轨迹插补

目标：

> 实现关节空间和笛卡尔空间点到点运动、直线插补、轨迹队列。

---

# 十一、当前阶段最重要的设计取舍

## 1. 上位机不要高频发送每个脉冲

你提到：

> 通信部分，上位机发送对应的电机脉冲，控制运动，最后需要实现轨迹控制，插补，示教等功能。

建议理解为：

- 初期：上位机发送目标脉冲数；
- 中期：上位机发送目标角度或目标点；
- 后期：上位机发送轨迹段或示教点；
- 不要让上位机实时发送每个脉冲。

原因：

- 串口不适合逐脉冲控制；
- 无线 DAP 更不适合高频脉冲流；
- Python Qt 也不适合硬实时；
- 脉冲必须由 STM32 定时器本地生成。

---

## 2. 下位机必须负责实时运动

正确架构应该是：

```text
上位机：发送目标、轨迹、参数、示教命令
STM32：负责插补、限速、加速度、脉冲输出、安全停止
```

---

## 3. 当前没有编码器，所有位置都是开环估算

必须明确：

- `ZERO` 是软件清零；
- `HOME` 如果没有限位开关，也只是回软件零点；
- 手动移动机械臂后，下位机不知道真实位置；
- 示教如果没有编码器，只能记录软件运动到达的位置。

---

# 十二、最终建议

你现在可以正式开始写代码。建议先实现最小闭环：

```text
PING
STATUS
SPEED
STOP
ESTOP
```

然后再加：

```text
MOVE_REL
MOVE_ABS
ZERO
ACCEL
```

最后再进入：

```text
JOINT
GOTOXY
MOVL
TRAJ
TEACH
```
