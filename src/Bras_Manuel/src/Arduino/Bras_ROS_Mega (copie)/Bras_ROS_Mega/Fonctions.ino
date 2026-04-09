#define _PI 3.14159265359

//------------------ Fonctions pour le robot ----------------------//
void ServoInit(const byte );




//--------Fonction pour init les servo au angles limites-----------//
void ServoInit(const byte Srv1,int LIMinf,int LIMsup)
{ 
  WriteServo(Srv1, LIMinf,CW_ANGLE_LIMIT);
  WriteServo(Srv1, LIMsup,CCW_ANGLE_LIMIT);
  WriteServo(Srv1, 1,TORQUE_ENABLE);
}
