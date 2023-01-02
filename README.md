# 介绍
一个matter开发板的控制程序 
GPIO2 3可以控制开关
GPIO6 7可以控制开关和占空比
基于合宙的esp32c3板子测试的，有无串口均可

焊有排针，烧写好程序的版本可以在下面的链接获得
https://item.taobao.com/item.htm?id=696155970253

matter QQ交流群：723997544

# 前提
已经安装esp-idf esp-matter
如果没有安装也可直接少些release的bin文件


# 如何使用
1. 打开multi_controller
2. 执行
    ``` shell
    idf.py erase-flash flash monitor -p /dev/serial/by-id/usb-1a86_USB_Single_Serial_553C016917-if00
    ```
    其中/dev/serial/by-id/usb-1a86_USB_Single_Serial_553C016917-if00是你的串口设备
3. 执行后扫码连接设备即可控制


![二维码](QR.png)

# 支持的板子
支持esp32c3  其他的应该也能用不过我没试

# esp-idf version
4.4.2-dirty

# esp-matter version
commit 85abe2cdd457f6f1d198af0ff1ee339ccd3c9bfb


