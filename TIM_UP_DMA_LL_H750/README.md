# TIM_UP_DMA_LL_H750

该示例演示通过 TIM_UP 请求来触发DMA搬运数据。本例中通过DMA从内存中搬运数据修改 GPIOx->BSRR 寄存器的值，从而实现修改GPIO口输出电平。使用该方式下可以实现同步控制一组GPIO口的输出，可以当作方波或是PWM输出等功能。除默认生成的代码，只在 main.c 文件添加几行手动编写的代码。本例中输出引脚为 PB0、PB1、PB2、PB3 。
