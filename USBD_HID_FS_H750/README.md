# USBD_HID_FS_H750

该示例演示USB作为HID设备使用，该示例实现和双向数据透传功能，一包可以发送64字节，设备在接收到来自主机的消息后进行回传。

引脚使用为 `PA12 - DP` `PA11 - DM` ，`VID = 1155 (0x0483)` `PID = 22352 (0x5750)` 。

除默认生成的代码，需要修改下面几处：

**usbd_custom_hid_if.c**

```c
/** Usb HID report descriptor. */
__ALIGN_BEGIN static uint8_t CUSTOM_HID_ReportDesc_FS[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END =
{
0x06, 0x00, 0xFF,  // Usage Page (Vendor Defined 0xFF00)
0x09, 0x00,        // Usage (0x00)
0xA1, 0x01,        // Collection (Application)
0x09, 0x01,        //   Usage (0x01)
0x15, 0x00,        //   Logical Minimum (0)
0x26, 0xFF, 0x00,  //   Logical Maximum (255)
0x75, 0x08,        //   Report Size (8)
0x95, 0x40,        //   Report Count (64)
0x81, 0x00,        //   Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
0x09, 0x02,        //   Usage (0x02)
0x15, 0x00,        //   Logical Minimum (0)
0x26, 0xFF, 0x00,  //   Logical Maximum (255)
0x75, 0x08,        //   Report Size (8)
0x95, 0x40,        //   Report Count (64)
0x91, 0x00,        //   Output (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
0xC0,              // End Collection
// 34 bytes
};
```


```c
uint32_t size = 0;
uint8_t buff[64];

// 收到来自主机的数据时会触发该事件
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state)
{
  UNUSED(event_idx);
  UNUSED(state);

  size = USBD_LL_GetRxDataSize(&hUsbDeviceFS, CUSTOM_HID_EPOUT_ADDR); // 获取收到的数据长度

  USBD_CUSTOM_HID_HandleTypeDef *hhid = (USBD_CUSTOM_HID_HandleTypeDef *)(hUsbDeviceFS.pClassData);
  for(int i=0; i<size; i++)
  {
	  buff[i]=hhid->Report_buf[i]; // 读取接收到的数据
  }
  
  USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, buff, size);
  
  // 开启下一次接收
  if (USBD_CUSTOM_HID_ReceivePacket(&hUsbDeviceFS) != (uint8_t)USBD_OK)
  {
	  return -1;
  }

  return (USBD_OK);
}
```



**usbd_conf.h**

```c
/*---------- 接收缓冲区大小 -----------*/
/*---------- 对于全速设备收和发一个包最大都为64字节 -----------*/
#define USBD_CUSTOMHID_OUTREPORT_BUF_SIZE     64U
/*---------- 报告描述符长度 -----------*/
#define USBD_CUSTOM_HID_REPORT_DESC_SIZE     34U
/*---------- 端电查询时间间隔 -----------*/
/*---------- 对于全速设备该值为1表示最快可以每1ms通讯一次 -----------*/
#define CUSTOM_HID_FS_BINTERVAL     0x5U
/*---------- -----------*/
```



**usbd_customhid.h**

```c
// 接收一个包最大为64字节
#define CUSTOM_HID_EPIN_SIZE        0x40U
// 发送一个包最大为64字节
#define CUSTOM_HID_EPOUT_SIZE        0x40U
```



