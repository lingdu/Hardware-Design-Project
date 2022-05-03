#include <Wire.h>

int redbuttonpin = 2;
int whitebuttonpin = 3;
int blackbuttonpin = 4;
byte value = 0x00;
//#A4 = SDA
//#A5 = SCL
unsigned char presscount = 0x00;
void setup()
{
  Wire.begin();
  Wire.setClock( 400000L);
  pinMode(whitebuttonpin, INPUT);
  pinMode(redbuttonpin, INPUT);
  pinMode(blackbuttonpin, INPUT);
  Serial.begin(9600);
  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println("\nI2C Scanner");
}
 


void loop(){
  byte error, address=0x20;
  int nDevices=0;
  if(digitalRead(redbuttonpin)== HIGH){
    presscount++;
    Serial.print("Presses: ");
    Serial.println(presscount);
    Wire.beginTransmission(address);
    Wire.write(0x01);
    //Wire.write(presscount);
    error = Wire.endTransmission();
 
    if (error == 0){
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4){
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
    else{
      Serial.println("Device not found!");
      }
    delay(200);  
  }
  else if(digitalRead(whitebuttonpin)== HIGH){
    presscount++;
    Serial.print("Presses: ");
    Serial.println(presscount);
    Wire.beginTransmission(address);
    Wire.write(0x02); //of onder:
    //Wire.write(presscount);
    error = Wire.endTransmission();
 
    if (error == 0){
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4){
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
    else{
      Serial.println("Device not found!");
      }
    delay(200);  
  }
  else if(digitalRead(blackbuttonpin)== HIGH){
    presscount++;
    Serial.print("Presses: ");
    Serial.println(presscount);
    Wire.beginTransmission(address);
    Wire.write(0x03); //REGISTER
    error = Wire.endTransmission();//false

    while (Wire.requestFrom(address, 1) <0);
    value = Wire.read();
    Serial.println(value,HEX);
    value = Wire.read();
    Serial.println(value,HEX);
    if (error == 0){
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4){
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
    else{
      Serial.println("Device not found!");
      }
    delay(300);  
  }
}

//werkt!
/*
void loop()
{
  byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ ){
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0){
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4){
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
    delay(1);
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
 
  delay(5000);           // wait 5 seconds for next scan
}
*/
