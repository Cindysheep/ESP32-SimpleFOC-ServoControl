#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include <iostream>
#include <sstream>
#include "BluetoothSerial.h" //蓝牙库

// M创动工坊测试代码，支持蓝牙串口输入，支持PC串口输入，硬件与客服见淘宝店：mcdgf.taobao.com
// 蓝牙控制：请使用安卓手机安装蓝牙控制串口APK，PC串口：发送Txx，xx表示数值。
//  SDA 21
//  SCL 22
//  magnetic sensor instance - I2C

struct PIDParams
{
    float P;
    float I;
    float D;
    float LPF_Tf;
};

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BluetoothSerial SerialBT; // 实例化蓝牙

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 12);

float target_torque = 0.03;  // 扭矩值
char model = 'p';         // 模式控制，默认位置模式
float change_value = 0.5; // 变化值

PIDParams pos_pid = {20, 25, 0, 0};      // 位置环参数
PIDParams vel_pid = {0.1f, 1, 0, 0.01f}; // 速度环参数

// 添加必要的全局变量
float target_position = 0;
float target_velocity = 10;        // 默认速度10rad/s
const float max_velocity = 110;    // 最大速度限制
const float default_velocity = 10; // 默认速度
String serialBuffer = "";          // 串口缓冲区

Commander command = Commander(Serial); // 使用串口发送命令
void doTarget(char *cmd) { command.scalar(&target_angle, cmd); }

void setup()
{
    // initialise magnetic sensor hardware
    Wire.setClock(400000);
    sensor.init();

    // link the motor to the sensor
    motor.linkSensor(&sensor);
    // power supply voltage [V]
    driver.voltage_power_supply = 12;
    driver.init();
    // link the motor and the driver
    motor.linkDriver(&driver);
    // choose FOC modulation (optional)
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    // set motion control loop to be used
    motor.controller = MotionControlType::angle;

    // 初始化PID参数
    motor.P_angle.P = pos_pid.P;
    motor.P_angle.I = pos_pid.I;
    motor.PID_velocity.P = vel_pid.P;
    motor.PID_velocity.I = vel_pid.I;
    motor.LPF_velocity.Tf = vel_pid.LPF_Tf;

    // 限制最大電壓和最大速度
    motor.voltage_limit = 6;
    motor.velocity_limit = 50;

    motor.PID_velocity.output_ramp = 1200; // 调整这个值可以影响电机的加速和减速性能。较高的值会使电机加速和减速更快，但可能导致振动或电流峰值。
    motor.LPF_velocity.Tf = 0.01f;         // 这可以滤除电机的噪声和高频振动，从而使速度控制更加稳定。

    Serial.begin(115200);
    // comment out if not needed
    motor.useMonitoring(Serial);
    // initialize motor
    motor.init();
    // align sensor and start FOC
    motor.initFOC();
    // add target command T
    command.add('T', doTarget, "target angle"); // 通过串口T命令发送位置，比如T6.28,表示电机转6.28弧度,即1圈
    SerialBT.begin("ESP32 Motor Control M");    // 蓝牙设备的名称
    Serial.println(F("电机准备就绪。"));
    _delay(1000);
}

// 蓝牙串口调试函数
void serial_debug()
{
    while (SerialBT.available() > 0)
    {
        char c = SerialBT.read();

        if (c == '\n' || c == '\r')
        {
            processCommand(serialBuffer);
            serialBuffer = "";
        }
        else
        {
            serialBuffer += c;
        }
    }
}

void processCommand(String cmd)
{
    cmd.trim();
    if (cmd.length() == 0)
        return;

    char modeChar = cmd[0];
    String param = cmd.substring(1);

    switch (modeChar)
    {
    /*--- 模式控制 ---*/
    case 'p': // 位置模式
        model = 'p';
        if (param.length() > 0)
        {
            // 相对位置模式：p100表示从当前位置转动100弧度
            target_position = sensor.getAngle() + param.toFloat();
        }
        else
        {
            // 立即停止在当前位置
            target_position = sensor.getAngle();
        }
        break;

    case 'v': // 速度模式
        model = 'v';
        if (param.length() > 0)
        {
            target_velocity = constrain(param.toFloat(), -max_velocity, max_velocity);
        }
        else
        {
            target_velocity = default_velocity;
        }
        break;

    case 't': // 扭矩模式
        model = 't';
        break;

    /*--- 运动控制 ---*/
    case 's': // 位置归零
        target_position = 0;
        break;

    case 'f': // 正转（带可选速度参数）
        model = 'v';
        if (param.length() > 0)
        {
            target_velocity = constrain(param.toFloat(), 0, max_velocity);
        }
        else
        {
            target_velocity = default_velocity;
        }
        break;

    case 'b': // 反转（带可选速度参数）
        model = 'v';
        if (param.length() > 0)
        {
            target_velocity = constrain(-param.toFloat(), -max_velocity, 0);
        }
        else
        {
            target_velocity = -default_velocity;
        }
        break;

    default:
        SerialBT.println("Unknown command");
        return;
    }

    // 发送状态反馈
    SerialBT.print("Mode: ");
    SerialBT.print(model);
    if (model == 'p')
    {
        SerialBT.print(" | Target Position: ");
        SerialBT.println(target_position, 2);
    }
    else if (model == 'v')
    {
        SerialBT.print(" | Target Velocity: ");
        SerialBT.println(target_velocity, 2);
    }
}

void loop()
{
    motor.loopFOC(); // 给上劲
    serial_debug();  //
    // 模式切换控制
    switch (model)
    {
    case 'p':
        motor.controller = MotionControlType::angle;
        motor.move(target_position);
        break;
    case 'v':
        motor.controller = MotionControlType::velocity;
        motor.move(target_velocity);
        break;
    case 't':
        motor.controller = MotionControlType::torque;
        motor.move(target_torque); // 需要根据实际情况调整扭矩控制，這部分暫時未寫
        break;
    }

    command.run(); // 监控串口输入的命令
}
