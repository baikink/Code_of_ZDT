/**
  ******************************************************************************
  * @file    test_calibration.c
  * @brief   系统参数标定测试代码
  * @note    将此代码临时添加到main.c的主循环中进行测试
  ******************************************************************************
  */

/*
 * 测试1：电机脉冲与倾角关系
 *
 * 在main.c的初始化后添加：
 */

void Test_Pulses_To_Angle(void)
{
    printf("\r\n========== 测试1：电机脉冲与倾角关系 ==========\r\n");
    printf("准备：将滑槽调至水平，钢球放中心\r\n");
    printf("按任意键开始测试...\r\n");

    // 等待按键或延时
    HAL_Delay(3000);

    // 测试不同脉冲数
    int32_t test_pulses[] = {50, 100, 200, 300, 500};

    for(int i = 0; i < 5; i++)
    {
        printf("\r\n--- 测试 %ld 脉冲 ---\r\n", test_pulses[i]);

        // 正向移动
        Emm_V5_QPos_Control(1, test_pulses[i]);
        HAL_Delay(2000);  // 等待到位

        printf("请测量并记录当前倾角（度）：______\r\n");
        printf("钢球移动距离（cm）：______\r\n");
        HAL_Delay(3000);  // 给时间记录

        // 回到原点
        Emm_V5_QPos_Control(1, -test_pulses[i]);
        HAL_Delay(2000);

        printf("已回到原点，按任意键继续下一组...\r\n");
        HAL_Delay(3000);
    }

    printf("\r\n========== 测试1完成 ==========\r\n\r\n");
}


/*
 * 测试2：钢球动态响应
 *
 * 观察钢球从静止到开始移动需要的最小倾角
 */

void Test_Ball_Response(void)
{
    printf("\r\n========== 测试2：钢球响应测试 ==========\r\n");
    printf("钢球放在中心位置\r\n");
    HAL_Delay(3000);

    // 从小脉冲开始，逐渐增大
    int32_t pulses = 10;

    while(pulses <= 200)
    {
        printf("\r\n测试 %ld 脉冲\r\n", pulses);

        Emm_V5_QPos_Control(1, pulses);
        HAL_Delay(2000);

        printf("钢球是否开始移动？(Y/N)：______\r\n");
        printf("如果移动，记录移动距离（cm）：______\r\n");
        HAL_Delay(3000);

        // 回到原点
        Emm_V5_QPos_Control(1, -pulses);
        HAL_Delay(2000);

        pulses += 10;  // 每次增加10脉冲
        HAL_Delay(2000);
    }

    printf("\r\n========== 测试2完成 ==========\r\n\r\n");
}


/*
 * 测试3：机械限位测试
 *
 * 找到连杆机构的最大安全行程
 */

void Test_Mechanical_Limits(void)
{
    printf("\r\n========== 测试3：机械限位测试 ==========\r\n");
    printf("警告：请密切观察，发现机构到达限位立即停止！\r\n");
    HAL_Delay(3000);

    printf("\r\n正向测试...\r\n");
    printf("将缓慢移动，到达限位或感觉阻力增大时记录脉冲数\r\n");
    HAL_Delay(2000);

    // 小步进移动，手动观察
    int32_t total_pulses = 0;

    for(int i = 0; i < 50; i++)  // 最多移动50次
    {
        Emm_V5_QPos_Control(1, 50);  // 每次50脉冲
        total_pulses += 50;

        printf("已移动: %ld 脉冲，按USER按钮停止\r\n", total_pulses);
        HAL_Delay(500);

        // TODO: 添加按键检测，按下则break
    }

    printf("\r\n正向限位脉冲数：%ld\r\n", total_pulses);
    printf("记录此值：______\r\n\r\n");

    // 回到中心
    printf("回到中心...\r\n");
    Emm_V5_QPos_Control(1, -total_pulses);
    HAL_Delay(3000);

    // 反向测试（类似）
    printf("\r\n反向测试...\r\n");
    // ... 类似代码

    printf("\r\n========== 测试3完成 ==========\r\n\r\n");
}
