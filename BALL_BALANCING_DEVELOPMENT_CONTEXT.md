# 小球平衡系统开发上下文

> **当前生效参数（2026-08-02）**：外环 P=`0.40`、内环 P=`1.10`、I/D 均为 `0`；视觉速度控制限幅 `±8 cm/s`、外环目标速度限幅 `±5 cm/s`、内环正常倾角限幅 `±4°`、倾角变化率 `1.0°/20ms`、最终机械保护 `+10°/-12°`、电机绝对位置命令速度 `60 RPM`。本文后续旧测试记录如与此处冲突，以本段为准。
>
> **外环限幅与 P 的配套关系（重要）**：外环目标速度限幅必须满足 `限幅 ≈ 外环P × 位置半量程`。当前 `0.40 × 12.5 = 5.0`。若限幅显著小于此值，误差超过 `限幅/P` 后外环即饱和，串级结构在大误差区退化为定速调速器，位置信息丢失，会出现"球远离目标时平台提前满量程反打、球被摁住无法到位"的现象。改动外环 P 时必须同步复核该限幅。

本文档记录当前 STM32F4 / MaixCAM / Emm_V5 一轴小球平衡系统的控制架构、关键参数、已知问题和后续调试顺序，供后续开发直接查阅。

## 1. 控制目标

通过 MaixCAM 获取小球在摆杆上的位置与速度，控制步进电机的摆杆倾角，使小球稳定在任意视觉坐标目标位置。

例如：

```c
ball_target_set(5.0f);
```

表示目标位置为视觉坐标的 `+5 cm`。

## 2. 控制架构

当前采用位置—速度串级 PID：

```text
目标位置 ball_target_y
        ↓
外环：位置 PID（pidY）
        ↓
目标小球速度 pidY.out
        ↓
内环：速度 PID（pidY_Speed）
        ↓
平台目标倾角 pidY_Speed.out
        ↓
motor_set_angle()
        ↓
Emm_V5 绝对位置命令
```

设计原则：

- 不读取电机编码器、电角度或真实位置反馈。
- 电机视作基于上电回零点的开环绝对角度执行器。
- PID 最终计算的是相对于上电零点的绝对摆杆倾角。
- 使用 Emm_V5 绝对位置命令，保持 `raF=1`。

## 3. 核心参数与坐标定义

### 电机角度换算

位于 `Core/Src/pid.c`：

```c
#define PULSES_PER_DEG  8.89f
```

换算关系：

```text
1° ≈ 8.89 脉冲
1 脉冲 ≈ 0.1125°
```

当前串口位置命令速度：

```c
#define MOTOR_POSITION_SPEED_RPM  20u
```

暂时不要继续修改该速度；应优先限制控制器输出并完成 PID 调参。

### 最终机械软件限幅

位于 `Core/Inc/pid.h`：

```c
#define ANGLE_LIMIT_MAX   10.0f
#define ANGLE_LIMIT_MIN  -12.0f
```

这是最终保护限幅，不应作为正常闭环控制的工作范围。

### 动态目标位置

```c
extern volatile float ball_target_y;
void ball_target_set(float target_y);
float ball_target_get(void);
```

位置控制在目标相对坐标系下运行：

```c
relative_y = Y - ball_target_y;
pidY.target = 0.0f;
pidY.now = relative_y;
```

**重要：** `pidY.target` 必须保持 `0.0f`。需要改变目标位置时，必须调用：

```c
ball_target_set(target_cm);
```

不要通过把 `pidY.target` 改成 `5.0f` 等方式设置绝对目标。

## 4. 当前文件职责

### `Core/Inc/pid.h`

- PID 数据结构与通用函数声明。
- 最终机械角度限幅。
- 静摩擦起动参数。
- 动态目标位置接口。
- 串级 PID 实例声明。

当前视觉速度低通系数：

```c
#define BALL_VELOCITY_FILTER_ALPHA  0.25f
```

静摩擦起动配置：

```c
#define STUCK_POSITION_THRESHOLD     1.0f
#define STUCK_POSITION_DELTA         0.03f
#define STUCK_TICKS_REQUIRED         10u
#define BREAKAWAY_POS_ANGLE          2.0f
#define BREAKAWAY_NEG_ANGLE         -3.0f
#define BREAKAWAY_RELEASE_DISTANCE   0.25f
```

### `Core/Src/pid.c`

负责：

- 通用 PID 计算；
- 电机绝对角度命令；
- 动态目标坐标转换；
- 位置—速度串级 PID；
- 视觉速度低通；
- 静摩擦起动补偿；
- 负方向角度补偿。

当前串级核心逻辑：

```c
/* 原始视觉速度先限幅到 ±8 cm/s，再低通滤波。 */
control_velocity = clamp(ball_velocity, -8.0f, 8.0f);
filtered_velocity += BALL_VELOCITY_FILTER_ALPHA *
                     (control_velocity - filtered_velocity);

pid_cal(&pidY);
pidY.out = clamp(pidY.out, -2.0f, 2.0f);

pidY_Speed.target = pidY.out;
pidY_Speed.now = filtered_velocity;
pid_cal(&pidY_Speed);
pidY_Speed.out = clamp(pidY_Speed.out, -3.0f, 3.0f);

pidY_velocity_damping = pidY_Speed.out;
float cmd_angle = pidY_velocity_damping;
```

`pidY_velocity_damping` 是遗留变量名；目前实际表示内速度环的倾角输出，不再是旧版直接速度阻尼。

负方向补偿：

```c
if(cmd_angle < 0.0f) {
    cmd_angle *= 1.2f;
}
```

因此负方向静摩擦起动角：

```text
-3.0° × 1.2 = -3.6°
```

### `Core/Src/main.c`

当前 PID 初始值：

```c
pid_init(&pidY, POSITION_PID, 0.40f, 0.0f, 0.0f, 0.0f);
pid_init(&pidY_Speed, POSITION_PID, 1.1f, 0.0f, 0.0f, 0.0f);
ball_target_set(5.0f);
```

控制实际周期约为 `20 ms / 50 Hz`。变量名 `g_flag_10ms` 是旧命名；`SysTick_Handler()` 实际每 20 次 SysTick 才触发控制。

### `Core/Src/stm32f4xx_it.c`

- USART2 DMA + 空闲中断解析 MaixCAM 数据。
- 数据包格式：`[0xAA][float error][float velocity][checksum]`。
- 接收后的位置和速度均经过符号换向。
- 当前视觉速度允许范围是 `±50 cm/s`，包含 `±20~30 cm/s` 的异常尖峰或真实快速运动。

## 5. VOFA 通道说明

VOFA 的所有数值均乘以 `100` 发送：

```text
ch0：最后一次发送给电机的软件目标角度，° ×100
ch1：小球视觉位置 Y，cm ×100
ch2：MaixCAM 原始小球速度，cm/s ×100
ch3：外位置环输出，即小球目标速度，cm/s ×100
ch4：内速度环输出，即平台倾角，° ×100
```

`ch0` 是软件命令值，**不是** 电机真实角度反馈。

示例：

```text
292,41,0,275,302
```

含义：

```text
ch0 = +2.92°
ch1 = +0.41 cm
ch2 = 0 cm/s
ch3 = +2.75 cm/s
ch4 = +3.02°
```

对于目标 `+5 cm`、外环 P=`0.60`、内环 P=`1.10`，上述数据在数学上是正确的。

## 6. 当前已完成内容

1. 支持通过 `ball_target_set()` 设置任意小球目标位置。
2. 已从直接位置角度 PID 改为位置—速度串级 PID。
3. 外环输出小球目标速度，内环使用视觉速度输出平台倾角。
4. 已删除旧版“位置角度 + 直接速度阻尼角”的重复速度反馈路径。
5. 电机继续使用上电零点下的绝对位置命令，不读取电机反馈。
6. 静摩擦检测及起动逻辑已支持动态目标切换。
7. 正负方向不对称通过负角度 `×1.2` 进行补偿。
8. 当前最终机械限幅为 `+8° / -10°`。

## 7. 当前主要问题

控制方向总体正确，但大误差时会发生严重过冲，平台反复撞到最终机械限幅。

典型数据：

```text
-989,730,3058,-138,-3515
```

表示：

```text
ch0 = -9.89°，已接近 -10° 限幅
ch1 = +7.30 cm
ch2 = +30.58 cm/s
ch3 = -1.38 cm/s
ch4 = -35.15°，内环未经限幅的输出过大
```

另一个方向：

```text
798,-823,0,794,1007
```

表示：

```text
ch0 = +7.98°，已接近 +8° 限幅
ch1 = -8.23 cm
ch2 = 0 cm/s
ch3 = +7.94 cm/s
ch4 = +10.07°
```

根本原因：

1. 视觉速度进入内环前没有限幅；
2. 外环目标速度没有限幅；
3. 内环倾角输出没有正常工作范围限幅；
4. 仅依靠最终 `+8°/-10°` 限幅截断，导致平台近似全力正打与全力反打，形成大范围振荡。

## 8. 已实施：三层控制限幅

已增加串级控制的正常工作范围。继续测试时，先保持当前 PID 参数，不要直接提高增益。

当前正常工作范围：

```text
视觉速度参与内环的限幅：±8.0 cm/s
外环目标速度 ch3 限幅：±2.0 cm/s
内环正常倾角输出 ch4 限幅：±3.0°
```

建议控制路径：

```text
视觉原始速度
   ↓
速度限幅 ±8 cm/s
   ↓
低通滤波
   ↓
内环速度反馈 now

外环 pidY.out
   ↓
目标速度限幅 ±2 cm/s
   ↓
内环 pidY_Speed.target

内环 pidY_Speed.out
   ↓
倾角限幅 ±3°
   ↓
静摩擦起动补偿
   ↓
负方向 ×1.2
   ↓
最终机械限幅 +10° / -12°
   ↓
motor_set_angle()
```

正常闭环工作角度应保持在约 `±3°` 内；`+8°/-10°` 仅作为最后一级保护。

## 9. 限幅加入后的调参基线

建议先使用：

```c
pid_init(&pidY, POSITION_PID, 0.40f, 0.0f, 0.0f, 0.0f);
pid_init(&pidY_Speed, POSITION_PID, 1.10f, 0.0f, 0.0f, 0.0f);
```

即：

```text
外环 P = 0.40
内环 P = 1.10
所有 I = 0
所有 D = 0
```

调参步骤：

1. 先测试 `ball_target_set(2.0f)`，再测试 `-2.0f`，最后才测试 `+5.0f`。
2. 先调内速度环：观察 `ch3` 与 `ch2` 的速度跟随关系。
3. `ch2` 长期跟不上 `ch3` 时，内环 P 每次增加最多 `0.1`。
4. `ch2` 明显来回超过 `ch3`、平台频繁换向时，内环 P 每次降低最多 `0.1`。
5. 内环稳定后再调外环 P：太慢则从 `0.40` 向上小步增加；越过目标太多则向下小步减小。
6. 只有系统 P 控制稳定、且持续存在固定稳态误差时，才考虑少量外环积分。

当前阶段禁止贸然加入积分或微分。

## 10. 验证标准

加入限幅后，VOFA 应满足：

```text
ch3 不超过 ±200（±2.00 cm/s）
ch4 不超过 ±300（±3.00°；静摩擦补偿时可略有例外）
ch0 不应长期或反复出现 +798 / -989
ch2 与 ch3 的趋势应逐渐一致
小球接近目标时，ch3 应逐步减小并反向制动
```

## 11. 构建验证

修改后执行：

```bash
git diff --check -- Core/Inc/pid.h Core/Src/pid.c Core/Src/main.c
cmake --build build/Debug --parallel 2
```

此前 CMake 构建已成功。`.specstory` 目录中存在与本控制器修改无关的历史空白问题，不应为通过全仓 `git diff --check` 而修改这些历史文件。

## 12. 后续开发约束

后续修改必须保持：

1. 保留位置—速度串级 PID 的总体结构；
2. 保留动态目标 `ball_target_set()`；
3. 保留静摩擦起动逻辑；
4. 保留负方向 `×1.2` 补偿；
5. 保留最终 `+8°/-10°` 机械保护限幅；
6. 先实施三层控制限幅，再继续 PID 调参；
7. 不读取电机角度或编码器反馈；
8. 不改回速度模式，继续使用当前绝对位置命令方式；
9. 未验证限幅效果前，不提高 PID 增益。
