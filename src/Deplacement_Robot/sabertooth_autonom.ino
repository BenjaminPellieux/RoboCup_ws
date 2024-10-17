#define nullptr NULL
#include <ros.h>
#include <geometry_msgs/Point.h>// On inclut un nouveau type de message pour communiquer de la Ras vers l'arduino 
#include <SabertoothSimplified.h>

SabertoothSimplified ST1(Serial1); // Utilise Serial1 pour communiquer avec le premier Sabertooth sur un Arduino Mega
SabertoothSimplified ST2(Serial2); // Utilise Serial2 pour communiquer avec le deuxième Sabertooth sur un Arduino Mega

// Définir la broche de la LED
#define LED 13

// Définir un objet ROS NodeHandle
ros::NodeHandle nh;

// Déclarer la fonction de rappel pour le topic /joy
void msgCallback(const geometry_msgs::Point& msg);

// Définir un abonnement au topic /joy
geometry_msgs::Point point_msg;
ros::Subscriber<geometry_msgs::Point> sub("/direction", &msgCallback);

void setup() {

  // Configurer la broche de la LED comme sortie
  pinMode(LED, OUTPUT);

  // Initialise le port série pour la communication avec le premier Sabertooth
  Serial1.begin(9600);

  // Initialise le port série pour la communication avec le deuxième Sabertooth
  Serial2.begin(9600); // Port série différent du premier Sabertooth

  // Initialiser la communication ROS
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce(); // Traiter les messages ROS

  // Votre code principal ici
}

// Définir la fonction de rappel pour le topic /joy
void msgCallback(const geometry_msgs::Point& msg) {
  // Obtenez les valeurs des axes et des boutons depuis le message ROS
  float x = msg.axes[0];
  float y = msg.axes[1];
  float z = msg.axes[2]; // Axes du deuxième joystick

  Serial.println("\n Ordre moteur 1");
  Serial.println(x);
  Serial.println("\n Ordre moteur 2: ");
  Serial.println(y);
 
  // Utiliser les valeurs des axes du premier joystick pour contrôler le premier Sabertooth
  float motor1Speed = map(x, -1.0, 1.0, -127, 127);
  float motor2Speed = map(y, -1.0, 1.0, -127, 127);
  
  // Calculer la vitesse et la direction des moteurs du premier Sabertooth
  int m1 = motor1Speed;
  int m2 = motor2Speed;
  
  // Limiter les valeurs entre -127 et 127
  m1 = constrain(m1, -127, 127);
  m2 = constrain(m2, -127, 127);
  
  // Envoyer les commandes au premier Sabertooth
  ST1.motor(1, m1);
  ST1.motor(2, m2);
}

