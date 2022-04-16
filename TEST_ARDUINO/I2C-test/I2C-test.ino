#include <Wire.h>

int buttonpin = 2;
//#A4 = SDA
//#A5 = SCL
unsigned char presscount = 0x00;
void setup()
{
  Wire.begin();
  pinMode(buttonpin, INPUT);
  Serial.begin(9600);
  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println("\nI2C Scanner");
}
 


void loop(){
  byte error, address=0x20;
  int nDevices=0;
  if(digitalRead(buttonpin)== HIGH){
    presscount++;
    Serial.println(presscount);
    Serial.println("Button Pressed");
    Wire.beginTransmission(address);
    Wire.write(presscount);
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
