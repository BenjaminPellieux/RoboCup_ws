//-------------Début fichier fonction dynamixel-----------------------------------//
void WriteServo(byte servoID, int Value ,char Param[2]);

void BeginServo()
{
  pinMode(PIN_RTS,OUTPUT);
  digitalWrite(PIN_RTS, LOW); // Assurez-vous que le MAX485 commence en mode réception
  SERIAL_SERVO.begin(57600);
}

void WriteServo(byte servoID, int Value ,char* Param){  
  int Adr,len;
  sscanf(Param,"%d,%d",&Adr,&len);
  byte Length, notchecksum;
  
  if(len == 4){
    char Value4 = Value >> 24; 
    char Value3 = Value >> 16;
    char Value2 = Value >> 8;  //same as /256 and truncating
    char Value1 = Value % 256;
    byte notchecksum = ~lowByte(servoID + 0x07 + 0x03 + Adr + Value4 + Value3 + Value2 + Value1);
    byte Length = 0x07;
    digitalWrite(PIN_RTS,HIGH); // put MAX485 into transmitting mode
    delay(10);
    SERIAL_SERVO.write(0xFF); // Start bytes
    SERIAL_SERVO.write(0xFF);
    SERIAL_SERVO.write(servoID); // Servo ID
    SERIAL_SERVO.write(Length); // Length of message (2 + 3 parameters)
    SERIAL_SERVO.write(0x03); // Write
    SERIAL_SERVO.write(Adr); // Adresse   
    SERIAL_SERVO.write(Value1);  
    SERIAL_SERVO.write(Value2);
    SERIAL_SERVO.write(Value3);  
    SERIAL_SERVO.write(Value4);   
    SERIAL_SERVO.write(notchecksum); // notCheckSum
    SERIAL_SERVO.flush(); // for good measure
    delay(SERIAL_DELAY);
    digitalWrite(PIN_RTS,LOW); // put MAX485 back into receiving mode
    delay(SERIAL_DELAY);
  }
  else if (len == 2)
  {
    char Value2 = Value >> 8;  //same as /256 and truncating
    char Value1 = Value % 256;
    byte notchecksum = ~lowByte(servoID + 0x05 + 0x03 + Adr + Value2 + Value1);
    byte Length = 0x05;
    
    digitalWrite(PIN_RTS,HIGH); // put MAX485 into transmitting mode
    delay(10);
    SERIAL_SERVO.write(0xFF); // Start bytes
    SERIAL_SERVO.write(0xFF);
    SERIAL_SERVO.write(servoID); // Servo ID
    SERIAL_SERVO.write(Length); // Length of message (2 + 3 parameters)
    SERIAL_SERVO.write(0x03); // Write
    SERIAL_SERVO.write(Adr); // Adresse   
    SERIAL_SERVO.write(Value1);  
    SERIAL_SERVO.write(Value2);  
    SERIAL_SERVO.write(notchecksum); // notCheckSum
    SERIAL_SERVO.flush(); // for good measure
    delay(SERIAL_DELAY);
    digitalWrite(PIN_RTS,LOW); // put MAX485 back into receiving mode
    delay(SERIAL_DELAY);
  }
  else 
  {
    byte notchecksum = ~lowByte(servoID + 0x04 + 0x03 + Adr + Value);    
    byte Length = 0x04;
    
    digitalWrite(PIN_RTS,HIGH); // put MAX485 into transmitting mode
    delay(10);
    SERIAL_SERVO.write(0xFF); // Start bytes
    SERIAL_SERVO.write(0xFF);
    SERIAL_SERVO.write(servoID); // Servo ID
    SERIAL_SERVO.write(Length); // Length of message (2 + 3 parameters)
    SERIAL_SERVO.write(0x03); // Write
    SERIAL_SERVO.write(Adr); // Adresse   
    SERIAL_SERVO.write((byte)Value);   
    SERIAL_SERVO.write(notchecksum); // notCheckSum
    SERIAL_SERVO.flush(); // for good measure
    delay(SERIAL_DELAY);
    digitalWrite(PIN_RTS,LOW); // put MAX485 back into receiving mode
    delay(SERIAL_DELAY);
    
  } 
}

void DoublePose(byte servoID1,byte servoID2, int Pos1, int Pos2, int Speed)//Pilotage angle+ vitesse pour 2 servo en simultanne
{ 

    char Value2 = Pos1 >> 8;  //same as /256 and truncating
    char Value1 = Pos1 % 256;

    char Value4 = map(Pos2, 1024, 3072, 3072, 1024) >> 8 ;  //same as /256 and truncating
    char Value3 = map(Pos2, 1024, 3072, 3072, 1024) % 256;
    
    char Speed2 = Speed >> 8;  //same as /256 and truncating
    char Speed1 = Speed % 256;
    
    byte Length = 0x0E;
    byte notchecksum = ~lowByte(0xFE + Length + 0x83 + 0x1E + 0x04 + servoID1 + Value2 + Value1 + Speed1 + Speed2 + servoID2 + Value3 + Value4 + Speed1 + Speed2); // avec vitesse
    digitalWrite(PIN_RTS,HIGH); // put MAX485 into transmitting mode
    delay(10);
    SERIAL_SERVO.write(0xFF); // Start bytes
    SERIAL_SERVO.write(0xFF);
    SERIAL_SERVO.write(0xFE);//Tout le monde
    SERIAL_SERVO.write(Length); // 
    SERIAL_SERVO.write(0x83); // Ecriture multiple    
    SERIAL_SERVO.write(0x1E); // Position 
    SERIAL_SERVO.write(0x04);  //Tg ca marche
    SERIAL_SERVO.write(servoID1); // Servo ID1
    SERIAL_SERVO.write(Value1);  
    SERIAL_SERVO.write(Value2);
    SERIAL_SERVO.write(Speed1);//pour vitesse
    SERIAL_SERVO.write(Speed2);//pour vitesse
    SERIAL_SERVO.write(servoID2); // Servo ID1      
    SERIAL_SERVO.write(Value3);  
    SERIAL_SERVO.write(Value4); 
    SERIAL_SERVO.write(Speed1);//pour vitesse
    SERIAL_SERVO.write(Speed2);//pour vitesse    
    SERIAL_SERVO.write(notchecksum); // notCheckSum
    SERIAL_SERVO.flush(); // for good measure
    delay(SERIAL_DELAY);
    digitalWrite(PIN_RTS,LOW); // put MAX485 back into receiving mode
    delay(SERIAL_DELAY);
}

int ReadPos(byte servoID)
{
  byte Length = 0x04;
  byte Instruction = 0x02;
  byte StartAddress = 0x24;
  byte DataLength = 0x02;
  byte checksum = ~(servoID + Length + Instruction + StartAddress + DataLength);

  // Envoyer la commande de lecture
  digitalWrite(PIN_RTS, HIGH); // Mettre MAX485 en mode émission
  delay(10);

  SERIAL_SERVO.write(0xFF); // Octets de début
  SERIAL_SERVO.write(0xFF);
  SERIAL_SERVO.write(servoID); // ID du servomoteur
  SERIAL_SERVO.write(Length);
  SERIAL_SERVO.write(Instruction); // Instruction de lecture
  SERIAL_SERVO.write(StartAddress); // Adresse de départ pour la position actuelle
  SERIAL_SERVO.write(DataLength); // Longueur des données à lire
  SERIAL_SERVO.write(checksum); // Checksum

  SERIAL_SERVO.flush(); // Pour s'assurer que les données sont envoyées
  delay(1); // Un petit délai pour assurer la transmission

  digitalWrite(PIN_RTS, LOW); // Mettre MAX485 en mode réception
  delay(10); // Attendre un peu pour la réponse

  // Lire la réponse du servomoteur
  if (SERIAL_SERVO.available() > 0) {
    byte incomingByte[8]; // Taille minimale pour lire la réponse
    for (int i = 0; i < 8; i++) {
      if (SERIAL_SERVO.available() > 0) {
        incomingByte[i] = SERIAL_SERVO.read();
//        Serial.print("Byte ");
//        Serial.print(i);
//        Serial.print(": 0x");
//        Serial.println(incomingByte[i], HEX);
      } else {
        Serial.println("Erreur : nombre de bytes insuffisants reçus.");
        return;
      }
    }

    // Vérifier les octets de début
    if (incomingByte[0] == 0xFF && incomingByte[1] == 0xFF) {
      byte receivedID = incomingByte[2];
      byte receivedLength = incomingByte[3];
      byte error = incomingByte[4];
      byte posLowByte = incomingByte[5];
      byte posHighByte = incomingByte[6];
      byte receivedChecksum = incomingByte[7];

      // Calculer le checksum de la réponse
      byte calculatedChecksum = ~(receivedID + receivedLength + error + posLowByte + posHighByte);

      if (receivedChecksum == calculatedChecksum) {
        int position = (posHighByte << 8) | posLowByte;
        Serial.print("Position: ");
        Serial.println(position);
        return position;
      } else {
        Serial.println("Checksum incorrect!");
      }
    } else {
      Serial.println("Erreur : réponse invalide.");
    }
  } else {
    Serial.println("Erreur : aucune réponse reçue.");
  }
}

void SimplePos(byte servoID, int Pos, int Speed)//Pilotage angle+ vitesse
{
    char Value2 = Pos >> 8;  //same as /256 and truncating
    char Value1 = Pos % 256;
    
    char Speed2 = Speed >> 8;  //same as /256 and truncating
    char Speed1 = Speed % 256;

    byte Length = 0x07;
    byte notchecksum = ~lowByte(servoID + Length + 0x03 + 0x1E + Value2 + Value1 + Speed1 + Speed2);
    
    digitalWrite(PIN_RTS,HIGH); // put MAX485 into transmitting mode
    delay(10);
    SERIAL_SERVO.write(0xFF); // Start bytes
    SERIAL_SERVO.write(0xFF);
    SERIAL_SERVO.write(servoID); // Servo ID
    SERIAL_SERVO.write(Length); // Length of message (2 + 3 parameters)
    SERIAL_SERVO.write(0x03); // Write
    SERIAL_SERVO.write(0x1E); // Starting address for Goal Position
    SERIAL_SERVO.write(Value1);  
    SERIAL_SERVO.write(Value2);
    SERIAL_SERVO.write(Speed1);//pour vitesse
    SERIAL_SERVO.write(Speed2);//pour vitesse  
    SERIAL_SERVO.write(notchecksum); // notCheckSum
    SERIAL_SERVO.flush(); // for good measure
    delay(SERIAL_DELAY);
    digitalWrite(PIN_RTS,LOW); // put MAX485 back into receiving mode
    delay(SERIAL_DELAY);
}

void Reboot(byte servoID)
{
    byte Length = 0x02;
    byte notchecksum = ~lowByte(servoID + Length + 0x08); // avec vitesse
    digitalWrite(PIN_RTS,HIGH); // put MAX485 into transmitting mode
    delay(10);
    SERIAL_SERVO.write(0xFF); // Start bytes
    SERIAL_SERVO.write(0xFF);
    SERIAL_SERVO.write(servoID); // Servo ID
    SERIAL_SERVO.write(Length); // Length of message (2 + 3 parameters)
    SERIAL_SERVO.write(0x08); // Reboot
    SERIAL_SERVO.write(notchecksum); // notCheckSum 
    SERIAL_SERVO.flush(); // for good measure
    delay(SERIAL_DELAY);
    digitalWrite(PIN_RTS,LOW); // put MAX485 back into receiving mode
    delay(SERIAL_DELAY);
}
