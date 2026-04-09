#define nullptr NULL
#include <ArduinoHardware.h>
#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <Servo.h>


// Definition de la broche de la LED
#define LED 13

// Declaration de l'objet NodeHandle ROS
ros::NodeHandle nh;

// Fonction de rappel pour le topic /joy_out
void joyCallback(const sensor_msgs::Joy& msg);

// Abonnement au topic /joy_out

Servo monServo;  // Crée un objet servo pour contrôler le servomoteur
int angle = 50; 
  ros::Subscriber<sensor_msgs::Joy> sub("/joy", joyCallback);

void setup() {
  //Serial.begin(9600);
  monServo.attach(9);  // Attache le servomoteur à la broche 9
  pinMode(LED, OUTPUT); // Configurer la broche de la LED comme sortie
  nh.initNode();        // Initialiser la communication ROS
  nh.subscribe(sub);    // S'abonner au topic /joy_out
}

void loop() {
  nh.spinOnce(); // Traiter les messages ROS
  delay(5);     // Petite pause

 
}

// Fonction de rappel pour le topic /joy_out
void joyCallback(const sensor_msgs::Joy& msg) {
  // Extraction des valeurs des axes et des boutons
  
  int button_recule = msg.buttons[5];
  int button_avance = msg.buttons[7];
  
    /*Serial.print("Button_recule: "); Serial.println(button_recule);
    Serial.print("Button_avance: "); Serial.println(button_avance);*/

  if (button_recule == 1) {  // button_recule
        angle = max(angle - 40, 0);  // Diminue l'angle, sans aller en dessous de 0
        monServo.write(angle);
    } else if (button_avance == 1) {  // button_avance
        angle = min(angle + 40, 180);  // Augmente l'angle, sans dépasser 180
        monServo.write(angle);
    }
  // Affichage des valeurs pour le debogage
  /*Serial.print("Axis 1: "); Serial.println(xAxis);
  Serial.print("Axis 2: "); Serial.println(yAxis);
  Serial.print("Axis 3: "); Serial.println(xAxis2);
  Serial.print("Axis 4: "); Serial.println(yAxis2);
  Serial.print("Button A: "); Serial.println(buttonA);*/

}


