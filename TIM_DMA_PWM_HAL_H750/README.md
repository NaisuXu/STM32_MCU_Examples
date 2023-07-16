# TIM_DMA_PWM_HAL_H750

该示例演示通过 TIM_CH 请求来触发DMA，然后通过DMA从内存中搬运数据修改 TIM_CCR 寄存器的值，从而实现修改输出的PWM的占空比。该演示参考ST官方例程 TIM_DMA 。除默认生成的代码，只在 main.c 文件添加几行手动编写的代码。PWM输出引脚为 PB0 。
