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

#define PIN_RTS 4
#define TAILLE_ARRAY 8
#define PINCE_FERME 512
#define PINCE_OUVERTE 850

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
std_msgs::Int16MultiArray Position; 
int Button = 0;
void JointState( const std_msgs::Int16MultiArray joint_states);

ros::Subscriber<std_msgs::Int16MultiArray> sub("retour_angles", JointState);
ros::Publisher chatter("chatter_ard", &Position);



void JointState( const std_msgs::Int16MultiArray joint_states){ 
     
    for(int i=0; i<8; i++)
    {
      retour_angle.data[i] = joint_states.data[i];
    }
    
    SimplePos(servoID_0, retour_angle.data[0], 20);//Axe1
    DoublePose(servoID_1,servoID_2, retour_angle.data[1], retour_angle.data[1],20);//Axe2
    DoublePose(servoID_3,servoID_4, retour_angle.data[2], retour_angle.data[2],20);
    SimplePos(servoID_5, retour_angle.data[3], 20);
    SimplePos(servoID_6, retour_angle.data[4], 20);
    motor_position(0,retour_angle.data[5]);// Rotation caméra
    motor_position(1,retour_angle.data[6]);// Pince

    nh.spinOnce();
    chatter.publish(&Position); 

}



void Script_Bidon()
{
    SimplePos(servoID_0, 1000, 20);
    DoublePose(servoID_1,servoID_2, 2048, 2048,50);//Axe2
    delay(4000);
    DoublePose(servoID_3,servoID_4, 1200, 1200,50);
    delay(4000);
    SimplePos(servoID_5, 4000, 30);
    SimplePos(servoID_6, 1024, 30);
    motor_position(servoID_7 - 7, 400);
    motor_position(servoID_8 - 7, 500);

    delay(4000);
    
    SimplePos(servoID_0, 3000, 30);
    DoublePose(servoID_1,servoID_2, 2200, 2200,50);//Axe2
    delay(4000);
    DoublePose(servoID_3,servoID_4, 2200, 2200,50);
    delay(4000);
    SimplePos(servoID_5, 3400, 30);
    SimplePos(servoID_6, 3000, 30);
    motor_position(servoID_7 - 7, 500);
    motor_position(servoID_8 - 7, 600);

    delay(4000);
    
    SimplePos(servoID_0, 2000, 30);
    DoublePose(servoID_1,servoID_2, 2500, 2500,50);//Axe2
    delay(4000);
    DoublePose(servoID_3,servoID_4, 2000, 2000,50);
    delay(4000);
    SimplePos(servoID_5, 2000, 30);
    SimplePos(servoID_6, 2000, 30);
    motor_position(servoID_7 - 7, 600);
    motor_position(servoID_8 - 7, 700);

    delay(4000);

}


AX12 motor [2]={AX12()};
long baud;

void setup() { 

  Serial.begin (115200);                             // inicializa el SoftSerial a 115,2 Kb/s
  baud = 1000000;
  AX12::init (baud);
  int detect[2];                                    // array para detectar automáticamente las ID de 2 motores
  AX12::autoDetect (detect, 2);  
  for (byte i = 0; i < 2; i++) {                  // detección de IDs
    motor[i].id = detect[i];  
  } 
  for (int i = 0; i < 2; i++) {                   // asigna las ID detectadas a los motores definidos previamente
    Serial.print("Moteur detecte : ");
    Serial.println(detect[i]);
  }

// ----- Setup Int16MultiArray ------
  
  Position.layout.dim           = (std_msgs::MultiArrayDimension *)
                     malloc(sizeof(std_msgs::MultiArrayDimension) * TAILLE_ARRAY);
  Position.layout.dim_length    = 0;
  Position.data_length          = TAILLE_ARRAY;
  Position.layout.dim[0].label  = dim0_label;
  Position.layout.dim[0].size   = TAILLE_ARRAY;
  Position.layout.dim[0].stride = 1 * TAILLE_ARRAY;
  Position.layout.data_offset   = 0;
  Position.data                 = (int *)malloc(sizeof(int) * TAILLE_ARRAY);

  retour_angle.layout.dim           = (std_msgs::MultiArrayDimension *)
                     malloc(sizeof(std_msgs::MultiArrayDimension) * TAILLE_ARRAY);
  retour_angle.layout.dim_length    = 0;
  retour_angle.data_length          = TAILLE_ARRAY;
  retour_angle.layout.dim[0].label  = dim0_label;
  retour_angle.layout.dim[0].size   = TAILLE_ARRAY;
  retour_angle.layout.dim[0].stride = 1 * TAILLE_ARRAY;
  retour_angle.layout.data_offset   = 0;
  retour_angle.data                 = (int *)malloc(sizeof(int) * TAILLE_ARRAY);


// ----- Setup Ros -----
  nh.getHardware()->setBaud(1000000);  
  nh.initNode   (); 

  nh.advertise  (chatter);
  nh.subscribe  (sub);

// ----- Setup Moteurs ----- 
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
  ServoInit(servoID_1,1500,2500); //Axe2
  ServoInit(servoID_2,1500,2500); //Axe2
  ServoInit(servoID_3,0,4095); //Axe3
  ServoInit(servoID_4,0,4095); //Axe3
  ServoInit(servoID_5,0,4095);    //Axe4
  ServoInit(servoID_6,0,4095); //Axe5 
  motor_init ();

}

void loop() {

  chatter.publish(&Position); 
  // ---- Récupération position de départ ----
  for (byte i = 0; i <= 6; i++) 
  {
    Position.data[i] = ReadPos(i);
    if (i<=7)
    {
      Position.data[i] = motor[i-7].readData(PRESENT_POSITION, AX_READ_DATA);
    }
  }
  delay(500);


  Button = retour_angle.data[7];
  Global_Reboot(Button);

  delay(500);
  nh.spinOnce();
  chatter.publish(&Position); 
}

//  Script_Bidon();
/*
    motor_position(servoID_7 - 7, 400);
    motor_position(servoID_8 - 7, 500);
    delay(1000);
    motor_position(servoID_7 - 7, 500);
    motor_position(servoID_8 - 7, 600);
    delay(1000);
    motor_position(servoID_7 - 7, 600);
    motor_position(servoID_8 - 7, 700);
    delay(1000);
    */
  
/*  SimplePos(servoID_1, 1500, 500);
  delay(1000);
  ReadPos(1);
  delay(1000);
  SimplePos(servoID_1, 3000, 500);
  delay(1000);
  ReadPos(1);
  delay(1000);
*/


void motor_init () {
  for(int i = 0; i < 2; i++){
    int error = motor[i].readData(3, 1); // Lire l'erreur du moteur
    if (error & 0x20) { // Vérifier si l'erreur de surcharge est détectée (bit 5)
      motor[i].clearOverloadError(); // Effacer l'erreur de surcharge
      Serial.println("Overload error cleared.");
    }
    
    motor[i].writeInfo (24, 1);               // habilita el torque
    motor[i].setEndlessTurnMode(false);                   // lo pone en modo de rotación continua
   // setear la inercia y todo eso
    motor[i].writeInfo (26, 0);
    motor[i].writeInfo (27, 0);
    motor[i].writeInfo (28, 95);
    motor[i].writeInfo (29, 95);
    motor[i].writeInfo (48, 1);
    motor[i].writeInfo (14, 1023);
    motor[i].writeInfo (11, 85);
    motor[i].writeInfo (12, 60);
    motor[i].writeInfo (13, 190);
    motor[i].writeInfo (5, 150);  

    motor_position(i, 500); //set motor position to closed
  }
}


void motor_position(int i, int pos)
{
  if (pos < 0) motor[i].writeInfo (30, 0);
  else if (pos > 1023) motor[i].writeInfo (30, 1023);
  else motor[i].writeInfo (30, pos);
}


void Global_Reboot(int Button)
{
  if (Button == 1)
  {
    for(byte i = 0; i < 7; i++)
      Reboot(i);
    for(int i = 0; i < 2; i++)
    {
      int error = motor[i].readData(3, 1); // Lire l'erreur du moteur
      if (error & 0x20) 
      { // Vérifier si l'erreur de surcharge est détectée (bit 5)
        motor[i].clearOverloadError(); // Effacer l'erreur de surcharge
        Serial.println("Overload error cleared.");
      }
    }
  }
}
