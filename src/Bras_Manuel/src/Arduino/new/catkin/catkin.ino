#include <ros.h>
#include "BrasMSGS.h"
#include "ax12.h"
#include "Dynamixel.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16MultiArray.h>

#include <std_msgs/Float64MultiArray.h>
#define SERIAL_SERVO Serial3
#define SERIAL_CAM Serial2

#define PIN_RTS 4
#define TAILLE_ARRAY 8

ros::NodeHandle  nh;

char dim0_label[] = "joint";

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

ros::Subscriber<std_msgs::Int16MultiArray> sub("retour_angles", JointState);
ros::Publisher chatter("chatter_ard", &retour_angle);



void JointState( const std_msgs::Int16MultiArray joint_states){ 
     
    for(int i=0; i<8; i++)
    {
      retour_angle.data[i] = joint_states.data[i];
    }
    
    SimplePos(servoID_0, retour_angle.data[0], 100);//Axe1
    DoublePose(servoID_1,servoID_2, retour_angle.data[1], retour_angle.data[1],100);//Axe2
    DoublePose(servoID_3,servoID_4, retour_angle.data[2], retour_angle.data[2],100);
    SimplePos(servoID_5, retour_angle.data[3], 100);
    SimplePos(servoID_6, retour_angle.data[4], 100);
    motor_position(retour_angle.data[5]);
//    WriteServo(servoID_8, state.pos[8],XL430_P);

    

}



void Script_Bidon()
{
    SimplePos(servoID_0, 1000, 30);
    DoublePose(servoID_1,servoID_2, 1200, 1200,50);//Axe2
    delay(4000);
    DoublePose(servoID_3,servoID_4, 1200, 1200,50);
    delay(4000);
    SimplePos(servoID_5, 4000, 30);
    SimplePos(servoID_6, 1024, 30);
    motor_position(300);

    delay(4000);
    
    SimplePos(servoID_0, 3000, 30);
    DoublePose(servoID_1,servoID_2, 2200, 2200,50);//Axe2
    delay(4000);
    DoublePose(servoID_3,servoID_4, 2200, 2200,50);
    delay(4000);
    SimplePos(servoID_5, 3400, 30);
    SimplePos(servoID_6, 3000, 30);
    motor_position(512);

    delay(4000);
    
    SimplePos(servoID_0, 2000, 30);
    DoublePose(servoID_1,servoID_2, 1700, 1700,50);//Axe2
    delay(4000);
    DoublePose(servoID_3,servoID_4, 2000, 2000,50);
    delay(4000);
    SimplePos(servoID_5, 2000, 30);
    SimplePos(servoID_6, 2000, 30);
    motor_position(1023-300);

    delay(4000);

}


AX12 motor={AX12()};
long baud;

void setup() { 

  Serial.begin (115200);                             // inicializa el SoftSerial a 115,2 Kb/s
  baud = 2000000L / (35);
  AX12::init (baud);
  int detect;                                    // array para detectar automáticamente las ID de 2 motores
  AX12::autoDetect (&detect, 1);                     // detección de IDs
  motor.id = detect;                        // asigna las ID detectadas a los motores definidos previamente
  Serial.print("Moteur detecte : ");
  Serial.println(detect);

  retour_angle.layout.dim           = (std_msgs::MultiArrayDimension *)
                     malloc(sizeof(std_msgs::MultiArrayDimension) * TAILLE_ARRAY);
  retour_angle.layout.dim_length    = 0;
  retour_angle.data_length          = TAILLE_ARRAY;
  retour_angle.layout.dim[0].label  = dim0_label;
  retour_angle.layout.dim[0].size   = TAILLE_ARRAY;
  retour_angle.layout.dim[0].stride = 1 * TAILLE_ARRAY;
  retour_angle.layout.data_offset   = 0;
  retour_angle.data                 = (int *)malloc(sizeof(int) * TAILLE_ARRAY);

  
  nh.getHardware()->setBaud(9600);
  nh.advertise  (chatter);
  nh.initNode   (); 
  nh.subscribe  (sub);

  BeginServo();
  delay(1000);

  Reboot(servoID_0);
  Reboot(servoID_1);
  Reboot(servoID_2);
  Reboot(servoID_3);
  Reboot(servoID_4);
  Reboot(servoID_5);
  Reboot(servoID_6);

  ServoInit(servoID_0,0,4095);    //Axe1
  ServoInit(servoID_1,1024,3072); //Axe2
  ServoInit(servoID_2,1024,3072); //Axe2
  ServoInit(servoID_3,1024,3072); //Axe3
  ServoInit(servoID_4,1024,3072); //Axe3
  ServoInit(servoID_5,0,4095);    //Axe4
  ServoInit(servoID_6,1024,3072); //Axe5 
  motor_init ();

//  WriteServo(servoID_8, 0,XL430_MIN); //End Effector
//  WriteServo(servoID_8, 4095,XL430_MAX);
//  WriteServo(servoID_8, 1,XL430_T);
//  WriteServo(servoID_8, 900,XL430_P);

}

void loop() {
 chatter.publish(&retour_angle); 
  nh.spinOnce();
  delay(1);
  //Script_Bidon();

}

void motor_init () {
  
    motor.writeInfo (24, 1);               // habilita el torque
    motor.setEndlessTurnMode(false);                   // lo pone en modo de rotación continua
   // setear la inercia y todo eso
    motor.writeInfo (26, 0);
    motor.writeInfo (27, 0);
    motor.writeInfo (28, 95);
    motor.writeInfo (29, 95);
    motor.writeInfo (48, 1);
    motor.writeInfo (14, 1023);
    motor.writeInfo (11, 85);
    motor.writeInfo (12, 60);
    motor.writeInfo (13, 190);
    motor.writeInfo (5, 150);  
}


void motor_position(int pos)
{
  if (pos < 0) motor.writeInfo (30, 0);
  else if (pos > 1023) motor.writeInfo (30, 1023);
  else motor.writeInfo (30, pos);
}
