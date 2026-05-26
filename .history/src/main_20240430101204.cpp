#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include <iostream>
#include <sstream>
#include "BluetoothSerial.h" //蓝牙库

// SDA 21
// SCL 22
// magnetic sensor instance - I2C
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BluetoothSerial SerialBT; // 实例化蓝牙

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 12);

float target_angle = 0;   // 位置、速度、扭矩值
char model = 'a';         // 模式控制，默认位置模式
float change_value = 0.5; // 变化值

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
    // velocity PI controller parameters
    motor.PID_velocity.P = 0.1;
    motor.PID_velocity.I = 0.02;
    //  motor.PID_velocity.D = 0.01;
    // maximal voltage to be set to the motor
    motor.voltage_limit = 6;
    // velocity low pass filtering time constant
    // the lower the less filtered
    motor.LPF_velocity.Tf = 0.01f;
    // angle P controller
    motor.P_angle.P = 20; // 位置PID的P值
    // maximal velocity of the position control
    motor.velocity_limit = 10;
    motor.PID_velocity.output_ramp = 1200; // 调整这个值可以影响电机的加速和减速性能。较高的值会使电机加速和减速更快，但可能导致振动或电流峰值。
    motor.LPF_velocity.Tf = 0.01f;         // 这可以滤除电机的噪声和高频振动，从而使速度控制更加稳定。

    Serial.begin(115200);
    // comment out if not needed
    motor.useMonitoring(Serial);
    // initialize motor
    motor.init();
    // align sensor and start FOC
    motor.initFOC();

    SerialBT.begin("ESP32 Motor Control"); // 蓝牙设备的名称
    Serial.println(F("电机准备就绪。"));
    Serial.println(F("通过串口发送命令, 角度Axx, 速度Vxx, 扭矩Txx:"));
    _delay(1000);
}

// 蓝牙串口调试函数
int commaPosition;
void serial_debug() // 蓝牙控制字段
{
    if (SerialBT.available() > 0)
    {
        char DATA = SerialBT.read();
        delay(5);
        switch (DATA)
        {
        /*---模式控制-----*/
        case 'a': // 位置模式
            model = 'a';
            target_angle = sensor.getAngle();
            break;
        case 'v':
            model = 'v';
            break;
        case 't':
            model = 't';
            break;

        /*----速度控制-----*/
        case '+':
            target_angle = target_angle + change_value;
            break;
        case '-':
            target_angle = target_angle - change_value;
            break;
        /*-----位置控制-----*/
        case 's':
            target_angle = 0;
            break;
        case 'f': // 前进
            model = 'v';
            target_angle = 20;
            break;
        case 'b': // 后退
            model = 'v';
            target_angle = -20;
            break;
        }

        SerialBT.print("model:");
        SerialBT.print(model);
        SerialBT.print("   target:");
        SerialBT.println(target_angle);
        SerialBT.println("--------------------");
    }
}

void receive_number()
{
    String received_chars = ""; // 初始化接收到的字符变量
    while (SerialBT.available())
    {
        char inChar = (char)SerialBT.read(); // 从蓝牙串口读取数据
        received_chars += inChar;
        if (inChar == '\n')
        {
            // 执行用户命令
            String command = received_chars;
            // 检查命令是否仅由数字字符组成
            bool isNumeric = true;
            for (size_t i = 0; i < command.length(); i++)
            {
                if (!isdigit(command.charAt(i)) && command.charAt(i) != '.' && command.charAt(i) != '-')
                {
                    isNumeric = false;
                    break;
                }
            }
            if (isNumeric)
            {
                motor.controller = MotionControlType::angle;
                std::string str = command.c_str(); // 将 String 转换为 std::string
                std::istringstream ss(str);        // 使用 std::istringstream 解析命令
                ss >> target_angle;
                // 用于调试的打印目标角度
                SerialBT.print("目标角度：");
                SerialBT.println(target_angle);
                motor.move(target_angle);
            }
            else
            {
                SerialBT.println("无效命令：不是数字值");
            }
            received_chars = ""; // 为下一个命令重置 received_chars
        }
    }
}

void loop()
{
    motor.loopFOC(); // 给上劲
    serial_debug();  //
    // 读取不同的命令，实现位置模式、速度模式、扭矩模式的切换
    if (model == 'a')
    {
        motor.controller = MotionControlType::angle;
        motor.move(target_angle);
    }
    else if (model == 'v')
    {
        motor.controller = MotionControlType::velocity;
        motor.move(target_angle);
    }
    else if (model == 't')
    {
        motor.controller = MotionControlType::torque;
        motor.move(target_angle);
    }
    receive_number();
}
