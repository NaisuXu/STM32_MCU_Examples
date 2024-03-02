# ETH_DP83848_DHCP_NonOS_Poll_F407

本示例用于演示STM32F407以太网，本示例实现DHCP动态获取IP和可以被Ping通，更多内容也可以参考下面文章：

 [《STM32单片机示例：ETH_DP83848_DHCP_NonOS_Poll_F407》https://blog.csdn.net/Naisu_kun/article/details/136358210](https://blog.csdn.net/Naisu_kun/article/details/136358210)



本例程PHY使用 DP83848 ，RMII接口，启用UART6，用来打印输出信息，串口参数为 `115200 8N1` 。除默认生成的代码，额外只在 `main.c` 和 `lwip.c` 文件添加几行手动编写的代码。



