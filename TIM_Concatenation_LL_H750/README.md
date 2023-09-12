# TIM_Concatenation_LL_H750

该示例演示两个定时器级联使用。这里使用TIM2作为主定时器，TIM4作为从定时器。主定时器溢出后会给从定时器一个时钟信号驱动从定时器计数。

程序中通过计数值定期翻转 PA4 引脚输出，可以在该引脚外接示波器或者逻辑分析仪等验证现象。

除默认生成的代码，只在 main.c 文件添加几行手动编写的代码。

对于哪个TIM可以被哪个TIM触发，ITRx是多少可以参考芯片参考手册的 TIMx internal trigger connection (TIMx 内部触发连接) 表格。