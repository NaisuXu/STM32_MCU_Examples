# TIM_Sync_LL_H750

该示例演示通过一个TIM使能时同步触发使能另一个TIM。

本例中使用TIM1作为主机，使用TIM1的使能信号作为触发信号，使用TIM3作为从机。该例程实现的效果为，当TIM1、TIM3都初始化后，TIM3使能时并不会立即工作，只有当TIM1使能后TIM3才开始工作。

为了方便观察，两个TIM的CH1都设置为PWM输出方式：TIM1_CH1 -> PE9 、 TIM3_CH1 -> PA6 。

除默认生成的代码，只在 main.c 文件添加几行手动编写的代码。

对于哪个TIM可以被哪个TIM触发，ITRx是多少可以参考芯片参考手册的 TIMx internal trigger connection (TIMx 内部触发连接) 表格。
