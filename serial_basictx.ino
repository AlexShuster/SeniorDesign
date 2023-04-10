#define HWSERIAL Serial1

void setup (void)
{
  Serial.begin(9600);
  HWSERIAL.begin(9600);


}  // end of setup


void loop (void)
{
  float vel = 75.63;
  float spin = 2008.4;

  HWSERIAL.print(vel);
  //HWSERIAL.print(",");
  HWSERIAL.print(spin);
  //HWSERIAL.print(";");

  delay(500);

//  char* vel_buff = (char*)&vel;
//  char* spin_buff = (char*)&spin;
//
//  for (int i = 0; i < sizeof(float); i++) {
//    HWSERIAL.write(vel_buff[i]);    
//    Serial.println((int)(vel_buff[i]));
//  }
//
//  delay(500);
//
//  for(int i = 0; i < sizeof(float); i++) {
//    HWSERIAL.write(spin_buff[i]);
//    Serial.println((int)(spin_buff[i]));
//  }
      

  
  //if (HWSERIAL.available() > 0) {
    //HWSERIAL.print(vel);
    //HWSERIAL.print(spin);
    //incomingByte = HWSERIAL.read();
    //Serial.print("UART received: ");
    //Serial.println(incomingByte, DEC);
    //HWSERIAL.print("UART received:");
    //HWSERIAL.println(incomingByte, DEC);
  //}
  //delay(500);



  
//  int incomingByte;
//
//  if (Serial.available() > 0) {
//    incomingByte = Serial.read();
//    Serial.print("USB received: ");
//    Serial.println(incomingByte, DEC);
//    HWSERIAL.print("USB received:");
//    HWSERIAL.println(incomingByte, DEC);
//  }
//  if (HWSERIAL.available() > 0) {
//    incomingByte = HWSERIAL.read();
//    Serial.print("UART received: ");
//    Serial.println(incomingByte, DEC);
//    HWSERIAL.print("UART received:");
//    HWSERIAL.println(incomingByte, DEC);
//  }
  




  

//  //int incomingByte;
//
//  //if (Serial.available() > 0) {
//    //incomingByte = Serial.read();
//    //Serial.print("USB received: ");
//    //Serial.println(incomingByte, DEC);
//    //HWSERIAL.print("USB received:");
//    //HWSERIAL.println(incomingByte, DEC);
//  //}
//  float vel = 85.7934;
//  float spin = 1513.543;
//  char* vel_buff = (char*)&vel;
//  char* spin_buff = (char*)&spin;
//
//  
//  if (HWSERIAL.available() > 0) {
//    //incomingByte = HWSERIAL.read();
//
//      // send test string
//    for (int i = 0; i < sizeof(float); i++) {
//      HWSERIAL.write(vel_buff[i]);
//    //SPI.transfer (vel_buff[i]);
//
//      Serial.print("UART received: ");
//      Serial.println(vel_buff[i]);
//
//      //HWSERIAL.write(vel_buff[i]);
//    ///SPI.transfer (vel_buff[i]);
//      
//    }
//
//    
//    //HWSERIAL.print("UART received:");
//    //HWSERIAL.println(incomingByte, DEC);
//  }
//
//  //char c;
//
//
//
//  
//
//  
//
//  // send test string
//  for (int i = 0; i < sizeof(float); i++)
//    ///SPI.transfer (vel_buff[i]);
//
//  for(int i = 0; i < sizeof(float); i++)
//    //SPI.transfer(spin_buff[i]);

 

  //delay (1000);  // 1 seconds delay
}  // end of loop
