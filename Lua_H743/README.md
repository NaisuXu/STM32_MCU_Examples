# Lua_H743

本示例用于演示移植Lua到单片机，更多内容也可以参考下面文章：

 [《单片机移植Lua（STM32H743移植Lua-5.4.6）》https://blog.csdn.net/Naisu_kun/article/details/136133467](https://blog.csdn.net/Naisu_kun/article/details/136133467)



本例程使用 NUCLEO-H743ZI2 开发板，启用默认使能的串口，用来打印输出信息，串口参数为 `115200 8N1` 。除默认生成的代码，以及移植的Lua源码，额外只在 `main.c` 文件添加几行手动编写的代码。




