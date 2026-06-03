# Servo Control Code for Spin LED

## Hardware
- 电机
- esp32
- SimpleFOC板
- 12V大电池（给SimpleFOC供电）
- 5V小电池（给led供电）

## Software
- VSC + PlatformIO（插件）

## 调试流程
1. 参考**电机连接文件**连接esp32，电机，编码器，SimpleFOC，电池，将esp32连接到电脑上
2. 运行代码，使用platformIO串口监视器（Serial monitor）查看代码运行情况并控制电机旋转速度

在serial monitor常用电机控制命令：
v：旋转速度，如v20，以20rad/s旋转
T：旋转角度，如T20，旋转到20rad角度为止