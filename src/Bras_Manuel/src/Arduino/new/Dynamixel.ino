//-------------Début fichier fonction dynamixel-----------------------------------//
void WriteServo(byte servoID, int Value ,char Param[2]);
void WriteCam(byte servoID, int Value ,char Param[2]);

void BeginServo()
{
  pinMode(PIN_RTS,OUTPUT);
  SERIAL_SERVO.begin(57600);
  SERIAL_CAM.begin(57600);
//  Serial.begin(9600);
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

void WriteCam(byte servoID, int Value ,char* Param){  
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
    SERIAL_CAM.write(0xFF); // Start bytes
    SERIAL_CAM.write(0xFF);
    SERIAL_CAM.write(servoID); // Servo ID
    SERIAL_CAM.write(Length); // Length of message (2 + 3 parameters)
    SERIAL_CAM.write(0x03); // Write
    SERIAL_CAM.write(Adr); // Adresse   
    SERIAL_CAM.write(Value1);  
    SERIAL_CAM.write(Value2);
    SERIAL_CAM.write(Value3);  
    SERIAL_CAM.write(Value4);   
    SERIAL_CAM.write(notchecksum); // notCheckSum
    SERIAL_CAM.flush(); // for good measure
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
    SERIAL_CAM.write(0xFF); // Start bytes
    SERIAL_CAM.write(0xFF);
    SERIAL_CAM.write(servoID); // Servo ID
    SERIAL_CAM.write(Length); // Length of message (2 + 3 parameters)
    SERIAL_CAM.write(0x03); // Write
    SERIAL_CAM.write(Adr); // Adresse   
    SERIAL_CAM.write(Value1);  
    SERIAL_CAM.write(Value2);  
    SERIAL_CAM.write(notchecksum); // notCheckSum
    SERIAL_CAM.flush(); // for good measure
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
    SERIAL_CAM.write(0xFF); // Start bytes
    SERIAL_CAM.write(0xFF);
    SERIAL_CAM.write(servoID); // Servo ID
    SERIAL_CAM.write(Length); // Length of message (2 + 3 parameters)
    SERIAL_CAM.write(0x03); // Write
    SERIAL_CAM.write(Adr); // Adresse   
    SERIAL_CAM.write((byte)Value);   
    SERIAL_CAM.write(notchecksum); // notCheckSum
    SERIAL_CAM.flush(); // for good measure
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
    SERIAL_SERVO.write(0x1E); // Adresse   
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
