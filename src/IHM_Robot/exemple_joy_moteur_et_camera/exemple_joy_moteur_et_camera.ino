#define nullptr NULL


#include <ArduinoHardware.h>
#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <SabertoothSimplified.h>
#include <Servo.h>

// Declaration des Sabertooth
SabertoothSimplified ST1(Serial1); // Communique avec le premier Sabertooth sur un Arduino Mega
SabertoothSimplified ST2(Serial2); // Communique avec le deuxiÃ¨me Sabertooth sur un Arduino Mega

// Definition de la broche de la LED
#define LED 13

// Declaration de l'objet NodeHandle ROS
ros::NodeHandle nh;

// Fonction de rappel pour le topic /joy_out
void joyCallback(const sensor_msgs::Joy& msg);

Servo monServo;  // Crée un objet servo pour contrôler le servomoteur
int angle = 120;
// Abonnement au topic /joy_out
ros::Subscriber<sensor_msgs::Joy> sub("/joy_ihm", joyCallback);
 

void setup() {
  pinMode(LED, OUTPUT); // Configurer la broche de la LED comme sortie
  monServo.attach(9);  // Attache le servomoteur à la broche 9
  Serial1.begin(9600); // Initialiser le port serie pour le premier Sabertooth
  Serial2.begin(9600); // Initialiser le port serie pour le deuxieme Sabertooth
  nh.initNode();        // Initialiser la communication ROS
  nh.subscribe(sub);    // S'abonner au topic /joy_out
}

void loop() {
  nh.spinOnce(); // Traiter les messages ROS
  delay(2);     // Petite pause
}

// Fonction de rappel pour le topic /joy_out
void joyCallback(const sensor_msgs::Joy& msg) {
  // Extraction des valeurs des axes et des boutons

  char floatStr[10];
  char log_msg[50];
  
  float xAxis = msg.axes[0];
  float yAxis = msg.axes[1];
  float xAxis2 = msg.axes[2];
  float yAxis2 = msg.axes[3];
  int buttonA = msg.buttons[0];

  // Affichage des valeurs pour le debogage
  /*Serial.print("Axis 1: "); Serial.println(xAxis);
  Serial.print("Axis 2: "); Serial.println(yAxis);
  Serial.print("Axis 3: "); Serial.println(xAxis2);
  Serial.print("Axis 4: "); Serial.println(yAxis2);
  Serial.print("Button A: "); Serial.println(buttonA);*/
  
  int button_recule = msg.buttons[5];
  int button_avance = msg.buttons[7];
  
  //sprintf(log_msg, "Axis 1: %s", floatStr);

    //Serial.print("Button_recule: "); Serial.println(button_recule);
    //Serial.print("Button_avance: "); Serial.println(button_avance);

  if (button_recule == 1) {  // button_recule
        angle = max(angle - 10, 0);  // Diminue l'angle, sans aller en dessous de 0
        monServo.write(angle);
        //Serial.print("Button_recule: "); Serial.println(button_recule);
    } else if (button_avance == 1) {  // button_avance
        angle = min(angle + 10, 180);  // Augmente l'angle, sans dépasser 180
        monServo.write(angle);
         //Serial.print("Button_avance: "); Serial.println(button_avance);
    }
  
  dtostrf(xAxis, 6, 2, floatStr); // Convertit xAxis en chaÃ®ne de caractÃ¨res
  sprintf(log_msg, "Axis 1: %s", floatStr);
  nh.loginfo(log_msg);


  dtostrf(yAxis, 6, 2, floatStr); // Convertit yAxis en chaÃ®ne de caractÃ¨res
  sprintf(log_msg, "Axis 2: %s", floatStr);
  nh.loginfo(log_msg);


  dtostrf(xAxis2, 6, 2, floatStr); // Convertit xAxis2 en chaÃ®ne de caractÃ¨res
  sprintf(log_msg, "Axis 3: %s", floatStr);
  nh.loginfo(log_msg);

  dtostrf(yAxis2, 6, 2, floatStr); // Convertit yAxis2 en chaÃ®ne de caractÃ¨res
  sprintf(log_msg, "Axis 4: %s", floatStr);
  nh.loginfo(log_msg);


  sprintf(log_msg, "Button A: %d", buttonA);
  nh.loginfo(log_msg);

  // Allumer la LED en fonction de l'état du bouton A
  if (buttonA == 1) {
    digitalWrite(LED, HIGH);
  } 
  else {
    digitalWrite(LED, LOW);
  }

  // Utiliser les valeurs des axes du premier joystick pour contrôler le premier Sabertooth
  float motor1Speed = map(yAxis, -1.0, 1.0, -127, 127);
  float motor2Speed = map(xAxis, -1.0, 1.0, -127, 127);
  
  // Calculer la vitesse et la direction des moteurs du premier Sabertooth
  int m1 = motor1Speed + motor2Speed;
  int m2 = motor1Speed - motor2Speed;
  
  // Limiter les valeurs entre -127 et 127
  m1 = constrain(m1, -127, 127);
  m2 = constrain(m2, -127, 127);
  
  // Envoyer les commandes au premier Sabertooth
  ST1.motor(1, m1);
  ST1.motor(2, m2);
  
  // Utiliser les valeurs des axes du deuxième joystick pour contrôler le deuxième Sabertooth
  float motor3Speed = map(xAxis2, -1.0, 1.0, -127, 127);
  float motor4Speed = map(yAxis2, -1.0, 1.0, -127, 127);
  
  // Calculer la vitesse et la direction des moteurs du deuxième Sabertooth
  int m3 = motor3Speed + motor4Speed;
  int m4 = motor3Speed - motor4Speed;
  
  // Limiter les valeurs entre -127 et 127
  m3 = constrain(m3, -127, 127);
  m4 = constrain(m4, -127, 127);
  
  // Envoyer les commandes au deuxième Sabertooth
  ST2.motor(1, m3);
  ST2.motor(2, m4);

}
