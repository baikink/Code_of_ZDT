/**
 * ============================================================================
 * 文件名称：UART_Tuning.c
 * 功能描述：UART在线调参模块实现 - 支持手机App滑杆/摇杆和串口文本命令
 *
 * 通信协议详解：
 *   ┌──────────────────────────────────────────────────────────────┐
 *   │  【滑杆 (Slider)】用于PID参数实时调节                       │
 *   │  用途：拖动手机App上的滑杆，实时修改PID参数                │
 *   │  数据格式：[slider,参数名,浮点值]                           │
 *   │                                                              │
 *   │  支持的参数名：                                              │
 *   │    直立环：ukp(比例) uki(积分) ukd(微分)                    │
 *   │    速度环：vkp(比例) vki(积分) vkd(微分)                   │
 *   │    转向环：tkp(比例) tki(积分) tkd(微分)                   │
 *   │    其他：  mb(机械中值) ser(舵机角度0~180度)               │
 *   │                                                              │
 *   │  示例：                                                      │
 *   │    [slider,ukp,-165.000]  → 设置直立环Kp=-165              │
 *   │    [slider,mb,-1.500]     → 设置机械中值=-1.5°             │
 *   │    [slider,ser,90.000]    → 设置舵机角度=90°                │
 *   ├──────────────────────────────────────────────────────────────┤
 *   │  【摇杆 (Joystick)】用于小车运动控制                        │
 *   │  用途：通过手机App的双摇杆控制小车运动                       │
 *   │  数据格式：[joystick,摇杆ID,LV,RH,预留]                    │
 *   │        LV = 左摇杆垂直方向 (-128~+127)                       │
 *   │        RH = 右摇杆水平方向 (-128~+127)                       │
 *   │                                                              │
 *   │  控制方式：                                                  │
 *   │    左摇杆上下 → 控制前进/后退速度                            │
 *   │      LV > 0 → 前进                                          │
 *   │      LV < 0 → 后退                                          │
 *   │      LV = 0 → 停止                                          │
 *   │    右摇杆左右 → 控制转向角度                                 │
 *   │      RH > 0 → 左转（目标航向角减小）                        │
 *   │      RH < 0 → 右转（目标航向角增大）                        │
 *   │      RH = 0 → 不转向                                        │
 *   │                                                              │
 *   │  示例：                                                      │
 *   │    [joystick,1,50,0,0]   → 中速前进                         │
 *   │    [joystick,1,-80,0,0]  → 快速后退                         │
 *   │    [joystick,1,0,-30,0]  → 原地右转                         │
 *   │    [joystick,1,60,-20,0] → 前进同时右转                     │
 *   ├──────────────────────────────────────────────────────────────┤
 *   │  【文本命令】用于串口助手调试                                 │
 *   │  格式：命令内容# 或 [命令内容]                               │
 *   │                                                              │
 *   │  命令列表：                                                  │
 *   │    HELP/?          → 显示可用命令列表                        │
 *   │    GET_PID         → 获取所有PID参数当前值                   │
 *   │    RESET           → 重置所有PID参数为默认值                 │
 *   │    ukp-165#        → 设置直立环Kp=-165（文本格式）         │
 *   │    spd200#         → 设置目标速度=200                        │
 *   │    stop#           → 紧急停止                                │
 *   │    ang90#          → 设置目标航向角=90°                      │
 *   └──────────────────────────────────────────────────────────────┘
 *
 * 工作流程：
 *   UART中断接收 → 存入命令队列 → 主循环调用Task()解析执行
 * ============================================================================
 */

#include "UART_Tuning/UART_Tuning.h"
#include "Circle_Mode/Circle_Mode.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ============================================================================
 * 模块内部变量
 * ============================================================================ */

/** UART接收缓冲区 - 暂存接收到的字符 */
static char rx_buffer[UART_TUNING_RX_BUFFER_SIZE];

/** 接收缓冲区写入位置 */
static uint8_t rx_index = 0;

/** 数据包模式标志（1=正在接收[]括号内的数据包）*/
static uint8_t packet_mode = 0;

/** 命令队列深度 - 缓冲多条待处理的UART命令，防止快速发送时丢包 */
#define CMD_QUEUE_DEPTH  32

/** 命令队列缓冲区 - 环形队列结构 */
static char cmd_queue[CMD_QUEUE_DEPTH][UART_TUNING_RX_BUFFER_SIZE];

/** 队列中当前待处理的命令数量 */
static volatile uint8_t cmd_queue_count = 0;

/** 有新命令到达标志 */
static volatile bool command_ready = false;

/** 队列头指针 - 指向下一个要取出的命令 */
static uint8_t cmd_queue_head = 0;

/** 队列尾指针 - 指向下一个要存入的位置 */
static uint8_t cmd_queue_tail = 0;

/** 浮点数转字符串的临时缓冲区 */
static char decimal_str_buffer[16];

/** 摇杆最大速度系数（脉冲/秒）*/
static int16_t joystick_max_speed = 200;

/* ============================================================================
 * 辅助函数：整数转固定小数位的字符串
 *
 * 功能：将放大1000倍的整数转换为"整数.3位小数"格式的字符串
 * 示例：输入 -165000 → 输出 "-165.000"
 *       输入 12345   → 输出 "12.345"
 *
 * 为什么用×1000的定点数？
 *   因为嵌入式系统浮点传输易出错，App端发送的浮点数实际是
 *   以毫单位（×1000）发送的整数，这里需要还原显示
 * ============================================================================
 */
static char* Int_To_Decimal_String(int32_t value_1000x)
{
    int index = 0;
    int integer_part = 0;
    int decimal_part = 0;

    /* 处理负号 */
    if (value_1000x < 0)
    {
        decimal_str_buffer[index++] = '-';
        value_1000x = -value_1000x;
    }

    /* 分离整数部分和小数部分 */
    integer_part = value_1000x / 1000;
    decimal_part = value_1000x % 1000;

    /* 转换整数部分 */
    if (integer_part == 0)
    {
        decimal_str_buffer[index++] = '0';
    }
    else
    {
        char digits[6];
        int num_digits = 0;
        int temp = integer_part;

        /* 从低位到高位提取各位数字 */
        while (temp > 0 && num_digits < 6)
        {
            digits[num_digits++] = temp % 10;
            temp /= 10;
        }

        /* 反向输出（高位在前）*/
        for (int i = num_digits - 1; i >= 0; i--)
        {
            decimal_str_buffer[index++] = '0' + digits[i];
        }
    }

    /* 添加小数点和3位小数 */
    decimal_str_buffer[index++] = '.';
    decimal_str_buffer[index++] = '0' + (decimal_part / 100);         /* 百分位 */
    decimal_str_buffer[index++] = '0' + ((decimal_part / 10) % 10);  /* 十分位 */
    decimal_str_buffer[index++] = '0' + (decimal_part % 10);          /* 个位 */
    decimal_str_buffer[index] = '\0';

    return decimal_str_buffer;
}

/**
 * 发送带前缀和后缀的格式化浮点数
 * @param prefix 前缀标签（如"ukp="）
 * @param value_1000x 放大1000倍的实际值
 * @param suffix 后缀（通常为空""）
 */
static void Send_Formatted_Float(const char *prefix, int32_t value_1000x, const char *suffix)
{
    UART_Tuning_SendString(prefix);
    UART_Tuning_SendString(Int_To_Decimal_String(value_1000x));
    UART_Tuning_SendLine(suffix);
}

/* ============================================================================
 * UART中断接收处理函数
 *
 * 功能：在UART中断中逐字符接收数据，识别完整命令并存入队列
 *
 * 支持三种命令格式：
 *   1. [data]  - 方括号包裹的数据包（来自手机App的滑杆/摇杆数据）
 *   2. cmd#    - 井号结尾的文本命令（来自串口助手）
 *   3. 其他    - 普通字符串（按换行符分割）
 *
 * 注意：此函数在中断上下文中运行，必须快速返回！
 *       实际的命令解析和执行放在主循环的Task()中进行
 * ============================================================================
 */
void UART1_IRQHandler(void)
{
    /* 检测并清除接收错误（溢出/帧错误/奇偶校验错误） */
    if (DL_UART_Main_getRawInterruptStatus(UART_PID_INST,
            DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
            DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
            DL_UART_MAIN_INTERRUPT_PARITY_ERROR))
    {
        DL_UART_Main_clearInterruptStatus(UART_PID_INST,
            DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
            DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
            DL_UART_MAIN_INTERRUPT_PARITY_ERROR);
        /* 丢弃当前不完整的命令，重置接收状态 */
        rx_index = 0;
        packet_mode = 0;
    }

    /* 循环读取FIFO中的所有已接收字符 */
    while (!DL_UART_isRXFIFOEmpty(UART_PID_INST))
    {
        char received_char = DL_UART_receiveData(UART_PID_INST);

        /* ---- 检测数据包起始标记 '[' ---- */
        if (received_char == '[')
        {
            packet_mode = 1;     /* 进入数据包接收模式 */
            rx_index = 0;        /* 重置缓冲区 */
            continue;
        }

        /* ---- 数据包接收模式 ---- */
        if (packet_mode == 1)
        {
            /* 检测数据包结束标记 ']' */
            if (received_char == ']')
            {
                rx_buffer[rx_index] = '\0';  /* 字符串结束符 */

                /* 将完整的命令存入队列 */
                if (cmd_queue_count < CMD_QUEUE_DEPTH)
                {
                    strcpy(cmd_queue[cmd_queue_tail], rx_buffer);
                    cmd_queue_tail = (cmd_queue_tail + 1) % CMD_QUEUE_DEPTH;
                    cmd_queue_count++;
                }

                command_ready = true;
                rx_index = 0;
                packet_mode = 0;   /* 退出数据包模式 */
            }
            /* 正常字符：存入缓冲区 */
            else if (rx_index < UART_TUNING_RX_BUFFER_SIZE - 1)
            {
                rx_buffer[rx_index++] = received_char;
            }
            /* 缓冲区溢出：丢弃本次数据包 */
            else
            {
                rx_index = 0;
                packet_mode = 0;
            }
            continue;
        }

        /* ---- 检测文本命令结束标记 '#' ---- */
        if (received_char == '#')
        {
            if (rx_index > 0)
                rx_buffer[rx_index] = '\0';
            else
                rx_buffer[0] = '\0';

            /* 将文本命令存入队列 */
            if (cmd_queue_count < CMD_QUEUE_DEPTH)
            {
                strcpy(cmd_queue[cmd_queue_tail], rx_buffer);
                cmd_queue_tail = (cmd_queue_tail + 1) % CMD_QUEUE_DEPTH;
                cmd_queue_count++;
            }

            command_ready = true;
            rx_index = 0;
            continue;
        }

        /* ---- 普通字符：存入缓冲区等待后续处理 ---- */
        if (rx_index < UART_TUNING_RX_BUFFER_SIZE - 1)
        {
            rx_buffer[rx_index++] = received_char;
        }
        else
        {
            rx_index = 0;  /* 缓冲区溢出，重置 */
        }
    }
}

/* ============================================================================
 * 字符串转浮点数辅助函数
 * ============================================================================
 */
static float str_to_float(const char *s)
{
    float result = 0.0f;
    bool is_negative = false;

    /* 跳过前导空格 */
    while (*s == ' ' || *s == '\t') s++;

    /* 处理符号 */
    if (*s == '+' || *s == '-') { if (*s == '-') is_negative = true; s++; }

    /* 整数部分 */
    while (*s >= '0' && *s <= '9') { result = result * 10.0f + (float)(*s - '0'); s++; }

    /* 小数部分 */
    if (*s == '.')
    {
        s++;
        float frac = 0.1f;
        while (*s >= '0' && *s <= '9') { result += (float)(*s - '0') * frac; frac *= 0.1f; s++; }
    }

    return is_negative ? -result : result;
}

/* ============================================================================
 * 字符串转定点数（×1000）辅助函数
 *
 * 功能：将字符串如"-165.000"转换为整数-165000
 * 用于App发送的浮点数参数解析
 * ============================================================================
 */
static int32_t String_To_Int1000x(const char *str)
{
    int result = 0;
    bool is_negative = false;

    /* 跳过前导空格 */
    while (*str == ' ' || *str == '\t')
        str++;

    /* 处理符号 */
    if (*str == '+' || *str == '-')
    {
        if (*str == '-')
            is_negative = true;
        str++;
    }

    /* 整数部分 */
    while (*str >= '0' && *str <= '9')
    {
        result = result * 10 + (*str - '0');
        str++;
    }

    /* 小数部分：转换为×1000的整数 */
    if (*str == '.')
    {
        str++;
        int decimal_count = 0;
        int decimal_digits = 0;

        /* 最多读取3位小数 */
        while (decimal_count < 3 && *str >= '0' && *str <= '9')
        {
            decimal_digits = decimal_digits * 10 + (*str - '0');
            str++;
            decimal_count++;
        }

        /* 补齐3位（如"5"变成"500"，".05"变成"50"）*/
        while (decimal_count < 3)
        {
            decimal_digits = decimal_digits * 10;
            decimal_count++;
        }

        result = result * 1000 + decimal_digits;
    }
    else
    {
        /* 无小数点：直接×1000 */
        result = result * 1000;
    }

    if (is_negative)
        result = -result;

    return (int32_t)result;
}

/* ============================================================================
 * 命令处理函数：HELP - 显示帮助信息
 *
 * 输出所有可用的文本命令列表
 * ============================================================================
 */
static void Process_Help_Command(void)
{
    /* PID参数设置命令（简写形式）*/
    UART_Tuning_SendLine("ukp uki ukd vkp vki vkd tkp tki tkd mb ser sm");
    /* 系统命令 */
    UART_Tuning_SendLine("GET_PID RESET HELP");
}

/* ============================================================================
 * 命令处理函数：GET_PID - 获取所有PID参数当前值
 *
 * 以"参数名=值"的格式输出，供App或上位机显示
 * 数值采用×1000的定点数格式传输
 * ============================================================================
 */
static void Process_Get_PID_Command(void)
{
    /* 直立环参数 (Upright: Kp, Ki, Kd) */
    Send_Formatted_Float("ukp=", (int32_t)(balance_params.upright.Kp * 1000.0f + 0.5f), "");
    Send_Formatted_Float("uki=", (int32_t)(balance_params.upright.Ki * 1000.0f + 0.5f), "");
    Send_Formatted_Float("ukd=", (int32_t)(balance_params.upright.Kd * 1000.0f + 0.5f), "");

    /* 速度环参数 (Velocity: Kp, Ki, Kd) */
    Send_Formatted_Float("vkp=", (int32_t)(balance_params.velocity.Kp * 1000.0f + 0.5f), "");
    Send_Formatted_Float("vki=", (int32_t)(balance_params.velocity.Ki * 1000.0f + 0.5f), "");
    Send_Formatted_Float("vkd=", (int32_t)(balance_params.velocity.Kd * 1000.0f + 0.5f), "");

    /* 转向环参数 (Turn: Kp, Ki, Kd) */
    Send_Formatted_Float("tkp=", (int32_t)(balance_params.turn.Kp * 1000.0f + 0.5f), "");
    Send_Formatted_Float("tki=", (int32_t)(balance_params.turn.Ki * 1000.0f + 0.5f), "");
    Send_Formatted_Float("tkd=", (int32_t)(balance_params.turn.Kd * 1000.0f + 0.5f), "");

    /* 机械中值 (Mechanical Balance) */
    Send_Formatted_Float("mb=", (int32_t)(balance_state.mechanical_balance * 1000.0f + 0.5f), "");

    UART_Tuning_SendLine("OK");
}

/* ============================================================================
 * 命令处理函数：设置PID参数（文本命令格式）
 *
 * 支持两种格式：
 *   1. 简写格式：ukp-165#  （参数名+数值，无分隔符）
 *   2. 完整格式：spd200# stop# ang90#
 *
 * 参数说明：
 *   u/v/t 开头 → 分别对应直立环/速度环/转向环
 *   kp/ki/kd  → 对应该环的比例/积分/微分系数
 *   mb        → 机械中值
 *   spd       → 目标速度（脉冲/秒）
 *   stop      → 紧急停止（速度清零+锁定航向角）
 *   ang       → 目标航向角度（度）
 * ============================================================================
 */
static void Process_Set_Param_Command(const char *command)
{
    #define PREFIX_LEN_UPRIGHT  3  /* "ukp" "uki" "ukd" 的长度 */
    #define PREFIX_LEN_VELOCITY 3  /* "vkp" "vki" "vkd" 的长度 */
    #define PREFIX_LEN_TURN     3  /* "tkp" "tki" "tkd" 的长度 */
    #define PREFIX_LEN_MB       2  /* "mb" 的长度 */

    int32_t new_value_1000x = 0;
    bool param_found = false;

    /* 根据首字符判断参数类型 */
    switch (command[0])
    {
        /* ---- 直立环参数 (Upright Loop) ---- */
        case 'u':
            if (strncmp(command, "ukp", PREFIX_LEN_UPRIGHT) == 0)
            {
                new_value_1000x = String_To_Int1000x(command + PREFIX_LEN_UPRIGHT);
                balance_params.upright.Kp = (float)new_value_1000x / 1000.0f;
                Send_Formatted_Float("ok ukp ", new_value_1000x, "");
                param_found = true;
            }
            else if (strncmp(command, "uki", PREFIX_LEN_UPRIGHT) == 0)
            {
                new_value_1000x = String_To_Int1000x(command + PREFIX_LEN_UPRIGHT);
                balance_params.upright.Ki = (float)new_value_1000x / 1000.0f;
                Send_Formatted_Float("ok uki ", new_value_1000x, "");
                param_found = true;
            }
            else if (strncmp(command, "ukd", PREFIX_LEN_UPRIGHT) == 0)
            {
                new_value_1000x = String_To_Int1000x(command + PREFIX_LEN_UPRIGHT);
                balance_params.upright.Kd = (float)new_value_1000x / 1000.0f;
                Send_Formatted_Float("ok ukd ", new_value_1000x, "");
                param_found = true;
            }
            break;

        /* ---- 速度环参数 (Velocity Loop) ---- */
        case 'v':
            if (strncmp(command, "vkp", PREFIX_LEN_VELOCITY) == 0)
            {
                new_value_1000x = String_To_Int1000x(command + PREFIX_LEN_VELOCITY);
                balance_params.velocity.Kp = (float)new_value_1000x / 1000.0f;
                /* 自动计算Ki = Kp/200（经验公式）*/
                balance_params.velocity.Ki = balance_params.velocity.Kp / 200.0f;
                Send_Formatted_Float("ok vkp ", new_value_1000x, "");
                param_found = true;
            }
            else if (strncmp(command, "vki", PREFIX_LEN_VELOCITY) == 0)
            {
                new_value_1000x = String_To_Int1000x(command + PREFIX_LEN_VELOCITY);
                balance_params.velocity.Ki = (float)new_value_1000x / 1000.0f;
                Send_Formatted_Float("ok vki ", new_value_1000x, "");
                param_found = true;
            }
            else if (strncmp(command, "vkd", PREFIX_LEN_VELOCITY) == 0)
            {
                new_value_1000x = String_To_Int1000x(command + PREFIX_LEN_VELOCITY);
                balance_params.velocity.Kd = (float)new_value_1000x / 1000.0f;
                Send_Formatted_Float("ok vkd ", new_value_1000x, "");
                param_found = true;
            }
            break;

        /* ---- 转向环参数 (Turn Loop) ---- */
        case 't':
            if (strncmp(command, "tkp", PREFIX_LEN_TURN) == 0)
            {
                new_value_1000x = String_To_Int1000x(command + PREFIX_LEN_TURN);
                balance_params.turn.Kp = (float)new_value_1000x / 1000.0f;
                Send_Formatted_Float("ok tkp ", new_value_1000x, "");
                param_found = true;
            }
            else if (strncmp(command, "tki", PREFIX_LEN_TURN) == 0)
            {
                new_value_1000x = String_To_Int1000x(command + PREFIX_LEN_TURN);
                balance_params.turn.Ki = (float)new_value_1000x / 1000.0f;
                Send_Formatted_Float("ok tki ", new_value_1000x, "");
                param_found = true;
            }
            else if (strncmp(command, "tkd", PREFIX_LEN_TURN) == 0)
            {
                new_value_1000x = String_To_Int1000x(command + PREFIX_LEN_TURN);
                balance_params.turn.Kd = (float)new_value_1000x / 1000.0f;
                Send_Formatted_Float("ok tkd ", new_value_1000x, "");
                param_found = true;
            }
            break;

        /* ---- 机械中值 (Mechanical Balance) ---- */
        case 'm':
            if (strncmp(command, "mb", PREFIX_LEN_MB) == 0)
            {
                new_value_1000x = String_To_Int1000x(command + PREFIX_LEN_MB);
                balance_state.mechanical_balance = (float)new_value_1000x / 1000.0f;
                Send_Formatted_Float("ok mb ", new_value_1000x, "");
                param_found = true;
            }
            break;

        case 's':
            /* 设置摇杆最大速度：sm200# → 最大速度系数=200 */
            if (strncmp(command, "sm", 2) == 0)
            {
                int16_t max_spd = (int16_t)atoi(command + 2);
                if (max_spd > 0 && max_spd <= 500)
                {
                    joystick_max_speed = max_spd;
                    UART_Tuning_SendString("ok sm=");
                    UART_Tuning_SendLine(Int_To_Decimal_String((int32_t)max_spd * 1000));
                }
                else
                {
                    UART_Tuning_SendLine("err sm(1~500)");
                }
                param_found = true;
            }
            /* 设置目标速度：spd200# → 目标速度=200脉冲/秒 */
            else if (strncmp(command, "spd", 3) == 0)
            {
                int16_t speed = (int16_t)atoi(command + 3);
                Balance_Car_Set_Target_Velocity(speed, speed);
                UART_Tuning_SendLine("ok");
                param_found = true;
            }
            /* 紧急停止：stop# → 速度清零+锁定当前航向角 */
            else if (strcmp(command, "stop") == 0 || strcmp(command, "STOP") == 0)
            {
                Balance_Car_Set_Target_Velocity(0, 0);
                Balance_Car_Set_Target_Angle_Z(IMU_Angle_Z());
                UART_Tuning_SendLine("ok");
                param_found = true;
            }
            break;

        /* ---- 圆圈模式参数 (Circle Mode) ---- */
        case 'R':
        case 'r':
            /* 设置绕圈半径：R80# → 半径80cm */
            {
                int16_t radius = (int16_t)atoi(command + 1);
                if (radius > 0)
                {
                    Circle_Mode_Set_Radius(radius);
                    UART_Tuning_SendString("ok R=");
                    UART_Tuning_SendLine(Int_To_Decimal_String((int32_t)radius * 1000));
                }
                else
                {
                    UART_Tuning_SendLine("err R>0");
                }
                param_found = true;
            }
            break;

        case 'D':
        case 'd':
            /* 设置绕圈方向：D1# → 顺时针，D0# → 逆时针 */
            {
                uint8_t dir = (uint8_t)atoi(command + 1);
                Circle_Mode_Set_Direction(dir);
                UART_Tuning_SendString("ok D=");
                UART_Tuning_SendLine(dir ? "CW" : "CCW");
                param_found = true;
            }
            break;

        /* ---- 航向角控制命令 ---- */
        case 'a':
            /* 设置目标航向角：ang90# → 目标角度=90° */
            if (strncmp(command, "ang", 3) == 0)
            {
                float target_angle = str_to_float(command + 3);
                if (target_angle < -180.0f || target_angle > 180.0f) { UART_Tuning_SendLine("err"); break; }
                Balance_Car_Set_Target_Angle_Z(target_angle);
                UART_Tuning_SendLine("ok");
                param_found = true;
            }
            break;

        default:
            break;
    }

    /* 未识别的命令 */
    if (!param_found)
        UART_Tuning_SendLine("err");

    #undef PREFIX_LEN_UPRIGHT
    #undef PREFIX_LEN_VELOCITY
    #undef PREFIX_LEN_TURN
    #undef PREFIX_LEN_MB
}

/* ============================================================================
 * 命令处理函数：App数据包解析（滑杆 & 摇杆）
 *
 * 这是手机App通信的核心处理函数！
 *
 * [滑杆 Slider]用于PID参数实时调节
 *    当用户在App上拖动滑杆时，App会发送此类数据包
 *    应用场景：在线调试PID参数，无需重新编译烧录
 *
 * [摇杆 Joystick]用于小车运动控制
 *    当用户推动App上的虚拟摇杆时，App会发送此类数据包
 *    应用场景：遥控小车前进、后退、转向
 * ============================================================================
 */
static void Process_App_Packet(const char *packet)
{
    /* 复制数据包到本地缓冲区（避免修改原数据）*/
    char local_buf[UART_TUNING_RX_BUFFER_SIZE];
    strncpy(local_buf, packet, sizeof(local_buf) - 1);
    local_buf[sizeof(local_buf) - 1] = '\0';

    /* 解析第一个字段：数据类型标签（"slider" 或 "joystick"）*/
    char *Tag = strtok(local_buf, ",");
    if (Tag == NULL) return;

    /*
     * ════════════════════════════════════════════════════════════
     *  [滑杆 (Slider)] 数据包处理
     * ════════════════════════════════════════════════════════════
     *
     * 数据格式：slider,参数名,值
     *
     * 参数名与对应关系：
     *   ┌──────┬─────────────────────────────────┬──────────────┐
     *   │ 名称 │           含义                  │  推荐范围    │
     *   ├──────┼─────────────────────────────────┼──────────────┤
     *   │ ukp  │ 直立环比例系数 Kp               │ -200 ~ -800  │
     *   │ uki  │ 直立环积分系数 Ki（通常为0）    │  0           │
     *   │ ukd  │ 直立环微分系数 Kd               │ -5   ~ -20   │
     *   │ vkp  │ 速度环比例系数 Kp               │ -2   ~ -15   │
     *   │ vki  │ 速度环积分系数 Ki               │ -0.02~ -0.2  │
     *   │ vkd  │ 速度环微分系数 Kd（通常为0）    │  0           │
     *   │ tkp  │ 转向环比例系数 Kp               │  3   ~ 10    │
     *   │ tki  │ 转向环积分系数 Ki（通常为0）    │  0           │
     *   │ tkd  │ 转向环微分系数 Kd（通常为0）    │  0           │
     *   │ mb   │ 机械中值（°）                   │ -3   ~ +3    │
     *   │   ser  │ 舵机角度（°）                  │  0   ~ 180   │
     *   └──────┴─────────────────────────────────┴──────────────┘
     *
     * 使用示例：
     *   App界面显示一个滑杆，标签为"ukp"，范围[-800, -200]
     *   用户拖动到-165的位置，App发送：[slider,ukp,-165.000]
     *   小车接收到后立即更新 upright.Kp = -165.0
     */
    if (strcmp(Tag, "slider") == 0)
    {
        /* 解析参数名和值 */
        char *Name = strtok(NULL, ",");
        char *Value = strtok(NULL, ",");
        if (Name == NULL || Value == NULL) return;

        /* 字符串转浮点数 */
        float val = str_to_float(Value);

        /* 根据参数名更新对应的PID参数 */
        if      (strcmp(Name, "ukp") == 0) balance_params.upright.Kp = val;
        else if (strcmp(Name, "uki") == 0) balance_params.upright.Ki = val;
        else if (strcmp(Name, "ukd") == 0) balance_params.upright.Kd = val;
        else if (strcmp(Name, "vkp") == 0) { balance_params.velocity.Kp = val; balance_params.velocity.Ki = val / 200.0f; }
        else if (strcmp(Name, "vki") == 0) balance_params.velocity.Ki = val;
        else if (strcmp(Name, "vkd") == 0) balance_params.velocity.Kd = val;
        else if (strcmp(Name, "tkp") == 0) balance_params.turn.Kp = val;
        else if (strcmp(Name, "tki") == 0) balance_params.turn.Ki = val;
        else if (strcmp(Name, "tkd") == 0) balance_params.turn.Kd = val;
        else if (strcmp(Name, "mb") == 0) balance_state.mechanical_balance = val;
        else return;  /* 未知的参数名，忽略 */

        UART_Tuning_SendLine("ok");  /* 响应确认 */
    }

    /*
     * ════════════════════════════════════════════════════════════
     *  [摇杆 (Joystick)] 数据包处理
     * ════════════════════════════════════════════════════════════
     *
     * 数据格式：joystick,摇杆ID,LV,RH,预留字节
     *
     * 摇杆布局（类似游戏手柄）：
     *   ┌─────────────────────────────┐
     *   │                             │
     *   │     [左摇杆]     [右摇杆]   │
     *   │      ↑↓           ←→       │
     *   │     (LV)          (RH)      │
     *   │                             │
     *   └─────────────────────────────┘
     *
     * 左摇杆 LV (Left Vertical，垂直方向):
     *   范围：-128 ~ +127
     *   +127（最上）→ 全速前进（约+508脉冲/秒）
     *   0（中间）  → 停止
     *   -128（最下）→ 全速后退（约-512脉冲/秒）
     *   计算公式：speed = LV / 25.0 * 100.0
     *
     * 右摇杆 RH (Right Horizontal，水平方向):
     *   范围：-128 ~ +127
     *   +127（最右）→ 向左转（目标角度减小12.7°）
     *   0（中间）  → 不转向
     *   -128（最左）→ 向右转（目标角度增大12.8°）
     *   计算公式：angle_offset = -RH / 10.0
     *
     * 控制示例：
     *   [joystick,1,50,0,0]  → 中速前进，不转向
     *   [joystick,1,-80,0,0] → 快速后退
     *   [joystick,1,0,-30,0] → 原地向右转
     *   [joystick,1,60,-20,0]→ 前进的同时向右转弯
     *   [joystick,1,0,40,0]  → 原地向左转
     */
    else if (strcmp(Tag, "joystick") == 0)
    {
        /* 解析各字段 */
        strtok(NULL, ",");           /* 跳过摇杆ID */
        char *LV_str = strtok(NULL, ",");  /* 左摇杆垂直值 */
        char *RH_str = strtok(NULL, ",");  /* 右摇杆水平值 */
        strtok(NULL, ",");           /* 跳过预留字节 */
        if (LV_str == NULL || RH_str == NULL) return;

        /* 转换为有符号整数 */
        int8_t LV = (int8_t)atoi(LV_str);
        int8_t RH = (int8_t)atoi(RH_str);

        /*
         * 左摇杆控制速度
         * LV值映射到目标速度（脉冲/秒）
         * 例：LV=50 → 速度=300脉冲/秒（中速前进）
         *     LV=-80 → 速度=-480脉冲/秒（快速后退）
         */
        Balance_Car_Set_Target_Velocity(
            (int16_t)(LV / 25.0f * joystick_max_speed),   /* 左轮目标速度 */
            (int16_t)(LV / 25.0f * joystick_max_speed)    /* 右轮目标速度 */
        );

        /*
         * 右摇杆控制转向
         * RH值映射到航向角偏移量（度）
         * 采用增量式控制：在当前角度基础上偏移
         * 例：RH=-30 → 当前角度+3.0°（向右转）
         *     RH=40  → 当前角度-4.0°（向左转）
         */
        if (RH != 0)
        {
            float current_angle = IMU_Angle_Z();
            Balance_Car_Set_Target_Angle_Z(current_angle - RH / 10.0f);
        }

        UART_Tuning_SendLine("ok");  /* 响应确认 */
    }
}

/* ============================================================================
 * 命令处理函数：SAVE - 保存参数到Flash（暂未实现）
 * ============================================================================
 */
static void Process_Save_Command(void)
{
    UART_Tuning_SendLine("err");  /* TODO: 实现Flash保存功能 */
}

/* ============================================================================
 * 命令处理函数：RESET - 重置所有PID参数为默认值
 * ============================================================================
 */
static void Process_Reset_Command(void)
{
    Balance_Car_PID_Init();  /* 重新初始化（恢复默认值）*/
    UART_Tuning_SendLine("ok");
}

/* ============================================================================
 * 初始化函数
 *
 * 功能：清空接收缓冲区和命令队列，配置并使能UART中断
 * 调用时机：系统启动时调用一次
 * ============================================================================
 */
void UART_Tuning_Init(void)
{
    /* 清空接收缓冲区 */
    memset(rx_buffer, 0, sizeof(rx_buffer));

    /* 清空命令队列 */
    memset(cmd_queue, 0, sizeof(cmd_queue));

    /* 重置所有状态变量 */
    rx_index = 0;
    command_ready = false;
    cmd_queue_count = 0;
    cmd_queue_head = 0;
    cmd_queue_tail = 0;
    packet_mode = 0;

    /* 配置UART中断：优先级3（低于PID定时器的优先级2，不打断控制循环）
     * RX FIFO + 溢出错误检测确保即使PID ISR期间有字符丢失，
     * 也会丢弃整条损坏命令而不会解析出错误值 */
    NVIC_SetPriority(UART_PID_INST_INT_IRQN, 3);
    NVIC_ClearPendingIRQ(UART_PID_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_PID_INST_INT_IRQN);
}

/* ============================================================================
 * 任务处理函数（主循环调用）
 *
 * 功能：从命令队列头部取出一条命令并分发执行
 * 调用时机：在主循环中周期性调用（建议每10ms调用一次）
 *
 * 设计特点：
 *   - 非阻塞设计：无命令时立即返回
 *   - 队列机制：即使处理较慢也不会丢失中断接收到的命令
 *   - 先进先出：保证命令执行顺序
 * ============================================================================
 */
void UART_Tuning_Task(void)
{
    char local_cmd[UART_TUNING_RX_BUFFER_SIZE];

    /* 处理队列中所有待处理的命令（不只处理一条） */
    while (cmd_queue_count > 0)
    {
        /* 关中断取出命令，防止与IRQ竞态 */
        __disable_irq();
        strncpy(local_cmd, cmd_queue[cmd_queue_head], sizeof(local_cmd) - 1);
        local_cmd[sizeof(local_cmd) - 1] = '\0';
        cmd_queue_head = (cmd_queue_head + 1) % CMD_QUEUE_DEPTH;
        cmd_queue_count--;
        __enable_irq();

        /* 根据命令类型分发处理 */

        /* 帮助命令 */
        if (strcmp(local_cmd, "HELP") == 0 || strcmp(local_cmd, "help") == 0 || strcmp(local_cmd, "?") == 0)
            Process_Help_Command();

        /* 获取PID参数 */
        else if (strcmp(local_cmd, "GET_PID") == 0)
            Process_Get_PID_Command();

        /* 保存参数（暂未实现）*/
        else if (strcmp(local_cmd, "SAVE") == 0)
            Process_Save_Command();

        /* 重置参数 */
        else if (strcmp(local_cmd, "RESET") == 0)
            Process_Reset_Command();

        /* 圆圈模式开启 */
        else if (strcmp(local_cmd, "C_ON") == 0)
        {
            Circle_Mode_Start();
            UART_Tuning_SendLine("ok C_ON");
        }
        /* 圆圈模式关闭 */
        else if (strcmp(local_cmd, "C_OFF") == 0)
        {
            Circle_Mode_Stop();
            UART_Tuning_SendLine("ok C_OFF");
        }

        /* 包含逗号 → App数据包（滑杆或摇杆）*/
        else if (strstr(local_cmd, ",") != NULL)
            Process_App_Packet(local_cmd);

        /* 其他 → 文本命令（如 ukp-165#）*/
        else
            Process_Set_Param_Command(local_cmd);

    } /* end while (cmd_queue_count > 0) */

    command_ready = false;
}

/* ============================================================================
 * 发送字符串（不带换行符）
 * ============================================================================
 */
void UART_Tuning_SendString(const char *str)
{
    if (str == NULL) return;

    /* 逐字符发送（阻塞方式）*/
    for (uint8_t i = 0; str[i] != '\0'; i++)
        DL_UART_transmitDataBlocking(UART_PID_INST, str[i]);
}

/* ============================================================================
 * 发送字符串（带\r\n换行符）
 * ============================================================================
 */
void UART_Tuning_SendLine(const char *str)
{
    UART_Tuning_SendString(str);
    DL_UART_transmitDataBlocking(UART_PID_INST, '\r');  /* 回车 */
    DL_UART_transmitDataBlocking(UART_PID_INST, '\n');  /* 换行 */
}
