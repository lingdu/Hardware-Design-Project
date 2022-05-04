#include <Wire.h>

// Pin config hardware
int redbuttonpin = 2;
int whitebuttonpin = 3;
int blackbuttonpin = 4;
//#A4 = SDA
//#A5 = SCL


// COMMANDS
byte CS_HIGH = 0x01;
byte CS_LOW = 0x02;
byte NEWDEV_OK = 0x03;

byte SLAVE_ADDRESS = 0x20;
int I2C_read_slave = 0; //0= disabled, 1 = enabled


byte error;
int nDevices = 0;
byte value = 0x00;
int user_command = 255;

void setup(){
  Wire.begin();
  Wire.setClock(400000L); // Uncommented: 400 kHz fast mode I2C | Commented: 100 kHz normal mode I2C
  pinMode(whitebuttonpin, INPUT);
  pinMode(redbuttonpin, INPUT);
  pinMode(blackbuttonpin, INPUT);
  Serial.begin(9600);
  while (!Serial); // Wait for serial monitor
  Serial.println("\nI2C Scanner");
  Serial.println("Possible command integer inputs: CS_HIGH (1), CS_LOW (2), NEWDEV_OK (3)");
}

void I2C_WriteSlave(byte address, byte command){
    Wire.beginTransmission(address);
    Wire.write(command);
    error = Wire.endTransmission();
 
    if (error == 0){
      Serial.print("I2C device 0x");
      if (SLAVE_ADDRESS<16)
        Serial.print("0");
      Serial.print(SLAVE_ADDRESS,HEX);
      Serial.println("  acknowledged!");
      nDevices++;
    }
    else if (error == 4){
      Serial.print("Unknown error at address 0x");
      if (SLAVE_ADDRESS<16)
        Serial.print("0");
      Serial.println(SLAVE_ADDRESS,HEX);
    }
    else{
      Serial.println("Device not found!");
    }
}
 
void loop(){
  if(Serial.available() > 0){
      user_command = Serial.parseInt();
      if(user_command == 1 || user_command == 2 || user_command == 3){
          //
      }
      else if(user_command == 0){
        Serial.println("Possible command integer inputs: CS_HIGH (1), CS_LOW (2), NEWDEV_OK (3)");
        user_command = 255;  
      }
      else{
        Serial.println("Command not known!");
        Serial.println("Possible command integer inputs: CS_HIGH (1), CS_LOW (2), NEWDEV_OK (3)");
        user_command = 255;
      }
  }
  
  if(digitalRead(redbuttonpin)== HIGH || user_command == 1){
    Serial.println("CS_HIGH");
    I2C_WriteSlave(SLAVE_ADDRESS, CS_HIGH);
    delay(200); // Delay for button
    user_command = 255;
  }
  else if(digitalRead(whitebuttonpin)== HIGH || user_command == 2){
    Serial.println("CS_LOW");
    I2C_WriteSlave(SLAVE_ADDRESS, CS_LOW);
    delay(200); // Delay for button
    user_command = 255;
  }
  else if(digitalRead(blackbuttonpin)== HIGH || user_command == 3){
    Serial.println("NEWDEV_OK");
    I2C_WriteSlave(SLAVE_ADDRESS, NEWDEV_OK);
    delay(200); // Delay for button
    user_command = 255;
  }

  else if(I2C_read_slave){
    Wire.beginTransmission(SLAVE_ADDRESS);
    Wire.write(0x05); //REGISTER
    error = Wire.endTransmission();//false

    while (Wire.requestFrom(SLAVE_ADDRESS, 1) <0);
    value = Wire.read();
    Serial.println(value,HEX);
    
    if (error == 0){
      Serial.print("I2C device found at address 0x");
      if (SLAVE_ADDRESS<16)
        Serial.print("0");
      Serial.print(SLAVE_ADDRESS,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4){
      Serial.print("Unknown error at address 0x");
      if (SLAVE_ADDRESS<16)
        Serial.print("0");
      Serial.println(SLAVE_ADDRESS,HEX);
    }
    else{
      Serial.println("Device not found!");
      }
    delay(300);
  }
}

/*
// I2C Scanner: scans the I2C bus for all possible addresses
void loop(){
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
