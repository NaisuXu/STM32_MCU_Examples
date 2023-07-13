# UART_RxTxIT_LL_H750

该示例演示UART收发数据，当RX接收到数据后通过TX进行发送。除默认生成的代码，只在 main.c 和 stm32h7xx_it.c 文件添加几行手动编写的代码。

串口参数为115200 8N1。

PA9   ------> USART1_TX
PA10   ------> USART1_RX

