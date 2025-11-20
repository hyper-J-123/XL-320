# XL320 操作函数
## 概括
### 1.功能：
* 设置控制模式
* 发送指令
* 设置转速
* 设置目标位置（当控制模式设置为2：关节模式时起作用）
* 设置LED灯
* 读取当前位置
### 2.使用方法
* 驱动函数主要写在core/src/main.c里面
* 用stm32cubeIDE打开
### 3.串口通信协议core
* 0xA8 func data_length id1 position1 id2 position2 id3 position3 id4 position4 id5 position5 id6 position6 id7 position7 id8 position8 id9 position9 sum
* data_length 2
* position 2
###4.python程序
* serial_read读取XL320的id和position
* serial_write写入id和position
