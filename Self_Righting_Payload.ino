/************************************************************************************************************
Team 1799 Self Righting Payload (2013)
 - Program for Edge Reseache's: Autonomous Self Righting Payload
 - Included Sensors:
  - ITG-3200 (Gyro)
  - ADXL-345 (Accelerometer)
  - MPX4250AP (Pressure Sensor)
*************************************************************************************************************/

#define atenservopin 3
#define deployservopin 5

#include <Wire.h>
#include <Servo.h>

byte _buff[6];

Servo atenservo;
Servo deployservo;

//gyro registers
char GYRO_XOUT_H = 0x1D;
char GYRO_XOUT_L = 0x1E;
char GYRO_YOUT_H = 0x1F;
char GYRO_YOUT_L = 0x20;
char GYRO_ZOUT_H = 0x21;
char GYRO_ZOUT_L = 0x22;
char DLPF_FS = 0x16;
char GYRO_POWER_CTL = 0x2D; // Power Control Register
char GYR0_DATA_FORMAT = 0x31;

//gyro register settings
char DLPF_CFG_0 = (1<<0);
char DLPF_CFG_1 = (1<<1);
char DLPF_CFG_2 = (1<<2);
char DLPF_FS_SEL_0 = (1<<3);
char DLPF_FS_SEL_1 = (1<<4);

//accel. registers
char ACCEL_DATAX_L = 0x32;
char ACCEL_DATAX_H = 0x33;
char ACCEL_DATAY_L = 0x34;
char ACCEL_DATAY_H = 0x35;
char ACCEL_DATAZ_L = 0x36;
char ACCEL_DATAZ_H = 0x37;
char ACCEL_POWER_CTL = 0x2D; // Power Control Register
char ACCEL_DATA_FORMAT = 0x31;

char itgAddress = 0x69; // Gyro Address
char adxlAddress = 0x53; // Accel. Address

int count = 0;// to be fixed later!!!

//Preasure Sensor
int pressurePin = A1;
int pressureValue = 0;


void setup()
{
  Serial.begin(9600);
  
  Wire.begin();
  
  atenservo.attach(atenservopin);
  deployservo.attach(deployservopin);
  
  char id=0; 
  id = itgRead(itgAddress, 0x00);  
  Serial.print("ID: ");
  Serial.println(id, HEX);
  
  itgWrite(itgAddress, DLPF_FS, (DLPF_FS_SEL_0|DLPF_FS_SEL_1|DLPF_CFG_0));
  
   //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  adxlWrite(ACCEL_DATA_FORMAT, 0x01);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  adxlWrite(ACCEL_POWER_CTL, 0x08);  
}


void loop()
{
  // Gyro axis rates.
  int itg_xRate, itg_yRate, itg_zRate;
  
  itg_xRate = itg_readX();
  itg_yRate = itg_readY();
  itg_zRate = itg_readZ();
  
  // Accelerometer axis rates.
  // each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
  int adxl_xRate = (((int)_buff[1]) << 8) | _buff[0];   
  int adxl_yRate = (((int)_buff[3]) << 8) | _buff[2];
  int adxl_zRate = (((int)_buff[5]) << 8) | _buff[4];
  
  double adxl_mag = 0;
  
      uint8_t howManyBytesToRead = 6;
  adxlRead( ACCEL_DATAX_L, howManyBytesToRead, _buff); //read the acceleration data from the ADXL345
  
  //Gyro serial prints.
  Serial.print(itg_xRate);
  Serial.print('\t');
  Serial.print(itg_yRate);
  Serial.print('\t');
  Serial.println(itg_zRate);
  
  //Accel. serial prints.
  Serial.print("x: ");
  Serial.print(adxl_xRate);
  Serial.print(" y: ");
  Serial.print(adxl_yRate);
  Serial.print(" z: ");
  Serial.println(adxl_zRate);  

    // finding the magnitude of a 3D dimmensional vector
    adxl_mag = sqrt((adxl_xRate * adxl_xRate) + (adxl_yRate * adxl_yRate) + (adxl_zRate * adxl_zRate)); 
    
    if ((itg_xRate < 30) && (itg_xRate > -30)&&(itg_yRate < 30) && (itg_yRate > -30) && (itg_zRate < 30) && (itg_zRate > -30) && (adxl_mag > .9)&&(adxl_mag < 1.1)){
          if (count >= 10){
          atenservo.write(atenservo.read() + 180); 
          deployservo.write(deployservo.read() + 180);{
          }   
    }else count++;
    }    
    else count=0;
  delay(500); // only read every 0,5 seconds
}

//This function will write a value to a register on the itg-3200.
//Parameters:
//  char address: The I2C address of the sensor. For the ITG-3200 breakout the address is 0x69.

//  char registerAddress: The address of the register on the sensor that should be written to.
//  char data: The value to be written to the specified register.
void itgWrite(char address, char registerAddress, char data)
{
  //Initiate a communication sequence with the desired i2c device
  Wire.beginTransmission(address);
  //Tell the I2C address which register we are writing to
  Wire.write(registerAddress);
  //Send the value to write to the specified register
  Wire.write(data);
  //End the communication sequence
  Wire.endTransmission();
}

//This function will read the data from a specified register on the ITG-3200 and return the value.
//Parameters:
//  char address: The I2C address of the sensor. For the ITG-3200 breakout the address is 0x69.
//  char registerAddress: The address of the register on the sensor that should be read
//Return:
//  unsigned char: The value currently residing in the specified register
unsigned char itgRead(char address, char registerAddress)
{
  //This variable will hold the contents read from the i2c device.
  unsigned char data=0;
  
  //Send the register address to be read.
  Wire.beginTransmission(address);
  //Send the Register Address
  Wire.write(registerAddress);
  //End the communication sequence.
  Wire.endTransmission();

  //Ask the I2C device for data
  Wire.beginTransmission(address);
  Wire.requestFrom(address, 1);
  
  //Wait for a response from the I2C device
  if(Wire.available()){
    //Save the data sent from the I2C device
    data = Wire.read();
  }
  
  //End the communication sequence.
  Wire.endTransmission();
  
  //Return the data read during the operation
  return data;
}

//This function is used to read the X-Axis rate of the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees per second. 
//Usage: int xRate = readX();
int itg_readX(void)
{
  int data=0;
  data = itgRead(itgAddress, GYRO_XOUT_H)<<8;
  data |= itgRead(itgAddress, GYRO_XOUT_L);  
  
  return data;
}

//This function is used to read the Y-Axis rate of the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees per second. 
//Usage: int yRate = readY();
int itg_readY(void)
{
  int data=0;
  data = itgRead(itgAddress, GYRO_YOUT_H)<<8;
  data |= itgRead(itgAddress, GYRO_YOUT_L);  
  
  return data;
}

//This function is used to read the Z-Axis rate of the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees per second. 
//Usage: int zRate = readZ();
int itg_readZ(void)
{
  int data=0;
  data = itgRead(itgAddress, GYRO_ZOUT_H)<<8;
  data |= itgRead(itgAddress, GYRO_ZOUT_L);  
  
  return data;
}

void adxlWrite(byte address, byte val)
{
  Wire.beginTransmission(adxlAddress); // start transmission to device 
  Wire.write(address);             // send register address
  Wire.write(val);                 // send value to write
  Wire.endTransmission();         // end transmission
}

// Reads num bytes starting from address register on device in to _buff array
void adxlRead(byte address, int num, byte _buff[]) {
  Wire.beginTransmission(adxlAddress); // start transmission to device 
  Wire.write(address);             // sends address to read from
  Wire.endTransmission();         // end transmission

  Wire.beginTransmission(adxlAddress); // start transmission to device
  Wire.requestFrom(adxlAddress, num);    // request 6 bytes from device

  int i = 0;
  while(Wire.available())         // device may send less than requested (abnormal)
  { 
    _buff[i] = Wire.read();    // receive a byte
    i++;
  }
  Wire.endTransmission();         // end transmission
}

int altitude(void)  //reads data from the pressure sensor and converts it into altitude
{
 
  // read the value from the sensor: MPX4250AP
  pressureValue = analogRead(pressurePin);
 return 3.28084*((1-(pow(((pressureValue)/0.004)/101.325,1/5.25588)))/0.00002557);
                 
}
