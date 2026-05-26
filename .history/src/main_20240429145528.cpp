#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include <iostream>
#include <sstream>

// SDA 21
// SCL 22
// magnetic sensor instance - I2C
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 12);

// angle set point variable
float target_angle = 0;
float target_angle0 = 0;//电机0的值



// instantiate the commander
// Commander command = Commander(Serial);
// void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }

void setup() {
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
  motor.controller = MotionControlType::torque;  
  // velocity PI controller parameters
  motor.PID_velocity.P = 10;
  motor.PID_velocity.I = 0.005;  
//  motor.PID_velocity.D = 0.01;
  // maximal voltage to be set to the motor
  motor.voltage_limit = 6;  
  // velocity low pass filtering time constant
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01f;  
  // angle P controller
  motor.P_angle.P = 30;  //位置PID的P值
  // maximal velocity of the position control
  motor.velocity_limit = 12;
  motor.PID_velocity.output_ramp = 1200;//调整这个值可以影响电机的加速和减速性能。较高的值会使电机加速和减速更快，但可能导致振动或电流峰值。
  motor.LPF_velocity.Tf = 0.01f;//这可以滤除电机的噪声和高频振动，从而使速度控制更加稳定。  

  // use monitoring with serial
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);
  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();


  Serial.println(F("电机准备就绪。"));
  Serial.println(F("通过串口发送命令, 角度Axx, 速度Vxx, 扭矩Txx:"));
  _delay(1000);
}
//==============串口接收==============
int commaPosition;
char comma;//用于读取逗号
String serialReceiveUserCommand() {  
  // a string to hold incoming data
  static String received_chars;  
  String command = "";
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the string buffer:
    received_chars += inChar;
    // end of user input
    if (inChar == '\n') {      
      // execute the user command
      command = received_chars;
      std::string str = command.c_str();//将String类转换为string类数据
      std::istringstream ss(str);  //定义字符串流    
      commaPosition = command.indexOf('\n');//检测字符串中的换行符，返回换行符的位置
      if(commaPosition != -1)//如果有换行存在就向下执行
      {
          ss>>target_angle0>>comma>>target_angle1;//字符串流输出到电机0和电机1
          target_angle = command.substring(0,commaPosition).toDouble();  //电机角度，双浮点数据类型
          bHassend = false ;//每发送一次位置，发一次标记
          Serial.print(target_angle0);
          Serial.print(comma);
          Serial.println(target_angle1);
      }
      // reset the command buffer 
      received_chars = "";
    }
  }
  return command;
}

static String received_chars;  
void loop() {
    motor.loopFOC();//给上劲
    motor.move(target_angle0);//电机0实时控制到目标角度
    serialReceiveUserCommand();//通过串口获取位置信息

读取不同的命令，实现位置模式、速度模式、扭矩模式的切换
    String command = "";
    while (Serial.available()) 
    {
        char inChar = (char)Serial.read();
        received_chars += inChar;
        if (inChar == '\n') 
        {      
            command = received_chars;
            commaPosition = command.indexOf('\n');//检测字符串中的换行符
            if(commaPosition != -1)//如果有换行存在就向下执行
            {
                char ch = command[0];
                if (ch == 'A' )
                {
                    motor.controller = MotionControlType::angle;
                    target_angle = command.substring(1,commaPosition).toDouble();  //电机角度                    
                    Serial.println(target_angle);
                } 
                else if(ch == 'V')
                {
                    motor.controller = MotionControlType::velocity;
                    target_angle = command.substring(1,commaPosition).toDouble();  //电机速度
                    if(abs(target_angle)>300)//限制最大弧度转速不超过每秒300弧度
                        target_angle=300;                    
                    Serial.println(target_angle);
                }
                else if(ch == 'T')
                {
                    motor.controller = MotionControlType::torque;
                    motor.velocity_limit= 20;
                    target_angle = command.substring(1,commaPosition).toDouble();  //电机扭矩
                    if(abs(target_angle)>12)//限制最大
                        target_angle=12;                    
                    Serial.println(target_angle);
                }
            received_chars = "";
            }
        }
    }
}

