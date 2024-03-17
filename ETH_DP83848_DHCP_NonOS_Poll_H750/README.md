# ETH_DP83848_DHCP_NonOS_Poll_H750

本示例用于演示STM32H750以太网，本示例实现DHCP动态获取IP和可以被Ping通，基础内容可以参考下面文章：

 [《STM32单片机示例：ETH_LAN8742_DHCP_NonOS_Poll_H743》https://blog.csdn.net/Naisu_kun/article/details/136437235](https://blog.csdn.net/Naisu_kun/article/details/136437235)

和上文相比区别是PHY换成了DP83848（MII接口），最主要的只是把 `ethernetif.c` 文件中LAN8742相关字段替换成DP83848即可。



