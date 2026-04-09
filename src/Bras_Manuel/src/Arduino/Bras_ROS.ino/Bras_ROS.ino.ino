#include <ros.h>
#include "BrasMSGS.h"
#include <sensor_msgs/JointState.h>
#include <SoftwareSerial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16MultiArray.h>

#include <std_msgs/Float64MultiArray.h>
SoftwareSerial mySerial(2, 3); // Rx, Tx

#include "Dynamixel.h"
#define SERIAL_SERVO mySerial
#define PIN_RTS 4

//ros::NodeHandle  nh;


const byte servoID_0 = 0x00;//Axe1
const byte servoID_1 = 0x01;//Axe2
const byte servoID_2 = 0x02;//Axe2
const byte servoID_3 = 0x03;//Axe3
const byte servoID_4 = 0x04;//Axe3
const byte servoID_5 = 0x05;//Axe4
const byte servoID_6 = 0x06;//Axe5
const byte servoID_7 = 0x07;//Axe6
const byte servoID_8 = 0x08;//Gripper
const byte servoID_all = 0xFE;

std_msgs::Int16MultiArray retour_angle;  

void JointState( const std_msgs::Int16MultiArray joint_states);
//ros::Subscriber<std_msgs::Int16MultiArray> sub("retour_angles", JointState);
//ros::Publisher chatter("chatter_ard", &retour_angle);


void JointState( const std_msgs::Int16MultiArray joint_states){ 
     
    for(int i=0; i<8; i++)
    {
      retour_angle.data[i] = joint_states.data[i];
    }
    
    SimplePos(servoID_0, joint_states.data[0], 100);//Axe1
    DoublePose(servoID_1,servoID_2, joint_states.data[1], joint_states.data[1],100);//Axe2
    DoublePose(servoID_3,servoID_4, joint_states.data[2], joint_states.data[2],100);
    SimplePos(servoID_5, joint_states.data[3], 100);
    SimplePos(servoID_6, joint_states.data[4], 100);
    WriteServo(servoID_7, joint_states.data[5],XL430_P);
//    WriteServo(servoID_8, state.pos[8],XL430_P);

//    chatter.publish(&retour_angle);

}


void Script_Bidon()
{
    SimplePos(servoID_0, 1000, 30);
    DoublePose(servoID_1,servoID_2, 1200, 1200,30);//Axe2
    delay(4000);
    DoublePose(servoID_3,servoID_4, 1024, 1024,20);
    delay(4000);
    SimplePos(servoID_5, 400, 30);
    SimplePos(servoID_6, 1024, 30);
    WriteServo(servoID_7, 1400,XL430_P);

    delay(4000);
    
    SimplePos(servoID_0, 3000, 30);
    DoublePose(servoID_1,servoID_2, 2200, 2200,30);//Axe2
    delay(4000);
    DoublePose(servoID_3,servoID_4, 2200, 2200,30);
    delay(4000);
    SimplePos(servoID_5, 3400, 30);
    SimplePos(servoID_6, 3000, 30);
    WriteServo(servoID_7, 2000,XL430_P);

    delay(4000);
    
    SimplePos(servoID_0, 2000, 30);
    DoublePose(servoID_1,servoID_2, 1500, 1500,30);//Axe2
    delay(4000);
    DoublePose(servoID_3,servoID_4, 2000, 2000,30);
    delay(4000);
    SimplePos(servoID_5, 2000, 30);
    SimplePos(servoID_6, 2000, 30);
    WriteServo(servoID_7, 2000,XL430_P);

    delay(4000);

}





void setup() { 
//  nh.getHardware()->setBaud(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  delay(50);
//  nh.initNode(); 
 //Serial.begin(9600);

//  nh.advertise(chatter);
//  nh.subscribe(sub);

  BeginServo();
  delay(1000);

  Reboot(servoID_1);
  Reboot(servoID_2);

  ServoInit(servoID_0,0,4095);//Axe1
  ServoInit(servoID_1,1024,3072);//Axe2
  ServoInit(servoID_2,1024,3072);//Axe2
  ServoInit(servoID_3,1024,3072);//Axe3
  ServoInit(servoID_4,1024,3072);//Axe3
  ServoInit(servoID_5,0,4095);//Axe4
  ServoInit(servoID_6,1024,3072);//Axe5 

  WriteServo(servoID_7, 1024,XL430_MIN);
  WriteServo(servoID_7, 3072,XL430_MAX);
  WriteServo(servoID_7, 1,XL430_T);
  WriteServo(servoID_7, 2048,XL430_P);

  WriteServo(servoID_8, 0,XL430_MIN);
  WriteServo(servoID_8, 4095,XL430_MAX);
  WriteServo(servoID_8, 1,XL430_T);
  WriteServo(servoID_8, 900,XL430_P);

}

void loop() {  

Script_Bidon();
//  nh.spinOnce();
  delay(500);
}
