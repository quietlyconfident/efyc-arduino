#include <SparkFunLSM9DS1.h>
#include <SparkFunMPL3115A2.h>
#include "SparkFun_Si7021_Breakout_Library.h"
#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>

// ------------------------ Si Sensor (Temperature and Humidity Sensor) ------------------
// Declare the variable for the temperature and the humidity.
float temperature_sisensor = 0;
float humidity = 0;

//Instantiation of the Weather object
Weather si_sensor;

//--------------------- LCD Screen ----------------------------
//Connect the LCD pin to the digital port 2.
#define txPin 2
SoftwareSerial LCD = SoftwareSerial(0, txPin);
const int LCDdelay=10;

//--------------------------- (MPL3115A2 Altitude and Temperature Sensor) -----------
//Instantiation of the pressure object
MPL3115A2 pressure;
float mp_pressure = 0;
float mp_altitude = 0;
float mp_temperature = 0;

//----------------------------- (LSM9DS1 Accelometer, GyroMeter and Magnetometer Sensor -----------)
//Instantiation of the object for LSM9DS1 class
LSM9DS1 agm;
// Both SDO_XM and SDO_G are pulled high
#define LSM9DS1_M 0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW
#define DECLINATION -8.58 //For the calculation of the heading


//--------------------- All of the setup file goes here ------------------------------
void setup() {
  
  // Set a baudrate for the serial and LCD screen
  Serial.begin(9600);
  LCD.begin(9600);

  //Pinmode confirmation for the txpin (LCD)
  pinMode(txPin, OUTPUT);

  //Initialize the I2C sensors and ping them: Uncomment the following line for the ping
  si_sensor.begin();
  pressure.begin();

  //Configure the altitude and pressure sensor settings
   pressure.setModeAltimeter(); // Measure altitude above sea level in meters
   pressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
  
   pressure.setOversampleRate(7); // Set Oversample to the recommended 128
   pressure.enableEventFlags(); // Enable all three pressure and temp event flags 

   // Configure the setting for the agm sensor. Set the I2C communication protocol
   // setting and addresses.
   agm.settings.device.commInterface = IMU_MODE_I2C;
   agm.settings.device.mAddress = LSM9DS1_M;
   agm.settings.device.agAddress = LSM9DS1_AG;

   // Uncomment if necessary. Makes sure that the agm sensor works
   /*   if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1)
      ;
  }
  */
}

// ----------------------- All of the Loop file goes here -----------------------------
void loop() {
  
  //-------------- Function call of Si-Sensor -------------------------------
  getSiInfo();

  //--------------- Function Call for Mp-Sensor ------------------------------
  getMpInfo();

   //--------------- Function Call for AGM-Sensor ------------------------------
  getAgmInfo();
  

  //-------------- Function call for the LCD and Serial Port result print --------------------
  printInfo();
  //Responsible for the calculation of the pitch and roll
  getPitchRollInfo(agm.ax, agm.ay, agm.az, -agm.my, -agm.mx, agm.mz);
}

// ----------------------------- Function of Si-Sensor --------------------------------
void getSiInfo(){
  // Measure Relative Humidity from Si7021
  humidity = si_sensor.getRH();

  // Measure Temperature from the Si7021
  temperature_sisensor = si_sensor.getTempF();
}

// ---------------------------------- Function of Mp-Sensor ------------------------------

// Is responsible for calculating the altitude, pressure, temperature
  void getMpInfo(){
  mp_altitude = pressure.readAltitude();
  mp_pressure = pressure.readPressure();
  mp_temperature = pressure.readTempF();
  }

// ------------------------------------- Function of agm Sensor ----------------------------
  // Update the sensor values whenever new data is available
  
 void getAgmInfo (){
  if ( agm.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    agm.readGyro();
  }
  if ( agm.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    agm.readAccel();
  }
  if ( agm.magAvailable() )
  {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    agm.readMag();
  }
 }

 //This function is responsible for the calculation of the pitch, roll and heading
 //from the calculation of the acelerometer and magnetometers data 
 
 void getPitchRollInfo(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  
  float heading;
  if (my == 0)
    heading = (mx < 0) ? PI : 0;
  else
    heading = atan2(mx, my);
    
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;
  
  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;

  // ---------------------------------------------------------//
  Serial.println();
  Serial.print("Heading");
  Serial.print(heading, 2);
  clearLCD();
  lcdPosition(0,0);
  LCD.print("Heading");
  lcdPosition(1,0);
  LCD.print(heading, 2);
  delay(3000);
  //------------------------------------------------------------//
  Serial.println();
  Serial.print("Pitch");
  Serial.print(pitch, 2);
  clearLCD();
  lcdPosition(0,0);
  LCD.print("Pitch");
  lcdPosition(1,0);
  LCD.print(pitch, 2);
  delay(3000);
  //--------------------------------------------------------------//
  Serial.println();
  Serial.print("Roll");
  Serial.print(roll, 2);
  clearLCD();
  lcdPosition(0,0);
  LCD.print("Roll");
  lcdPosition(1,0);
  LCD.print(roll, 2);
  delay(3000);
}

// ------------------------------ Function for LCD Screen --------------------------------

/* Uncomment if necessary */

// Maintains the position of the LCD screen
void lcdPosition(int row, int col) {
  LCD.write(0xFE);   //command flag
  LCD.write((col + row*64 + 128));    //position 
  delay(LCDdelay);
}

//Clears the LCD as the command is called.
void clearLCD(){
  LCD.write(0xFE);   //command flag
  LCD.write(0x01);   //clear command.
  delay(LCDdelay);
}

//a general function to call the command flag for issuing all other commands
void serCommand(){      
  LCD.write(0xFE);
}

// This function are for turning the backlight On and Off
/*
void backlightOn() {  //turns on the backlight
  LCD.write(0x7C);   //command flag for backlight stuff
  LCD.write(157);    //light level.
  delay(LCDdelay);
}
void backlightOff(){  //turns off the backlight
  LCD.write(0x7C);   //command flag for backlight stuff
  LCD.write(128);     //light level for off.
   delay(LCDdelay);
}
*/


// ------------------------------- Function to print on a LCD and Serial Screen -------------------
void printInfo(){
  // You cannot concaneate the string in C because the string are really Char.
  //---------------------------------------------//
  Serial.print("Temperature: ");
  Serial.print(temperature_sisensor);
  clearLCD();
  lcdPosition(0,0);
  LCD.print("Temperature (F)");
  lcdPosition(1,0);
  LCD.print(temperature_sisensor);
  delay(1000);
  // ---------------------------------------------//
  Serial.println();
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print("%");
  clearLCD();
  lcdPosition(0,0);
  LCD.print("Humidity (%)");
  lcdPosition(1,0);
  LCD.print(humidity);
  delay(1000);
  //------------------------------------------------//
  Serial.println();
  Serial.print("Pressure (Pa)");
  Serial.print(mp_pressure);
  Serial.print("");
  clearLCD();
  lcdPosition(0,0);
  LCD.print("Pressure (Pa)");
  lcdPosition(1,0);
  LCD.print(mp_pressure);
  delay(1000);
  //--------------------------------------------------//
  Serial.println();
  Serial.print("Altitude (m)");
  Serial.print(mp_altitude);
  Serial.print("");
  clearLCD();
  lcdPosition(0,0);
  LCD.print("Altitude (m)");
  lcdPosition(1,0);
  LCD.print(mp_altitude);
  delay(1000);
  //---------------------------------------------------//
  Serial.println();
  Serial.print("Temperature2 (F)");
  Serial.print(mp_temperature);
  Serial.print("");
  clearLCD();
  lcdPosition(0,0);
  LCD.print("Temperature2 (F)");
  lcdPosition(1,0);
  LCD.print(mp_temperature);
  delay(1000);
  //---------------------------------------------------//
  Serial.println();
  Serial.print("(Gyrometer (d/s)");
  Serial.print(agm.calcGyro(agm.gx), 2);
  Serial.print(", ");
  Serial.print(agm.calcGyro(agm.gy), 2);
  Serial.print(", ");
  Serial.print(agm.calcGyro(agm.gz), 2);
  Serial.print("");
  clearLCD();
  lcdPosition(0,0);
  LCD.print("Gyrometer (d/s)");
  lcdPosition(1,0);
  LCD.print(agm.calcGyro(agm.gx), 2);
  LCD.print(", ");
  LCD.print(agm.calcGyro(agm.gy), 2);
  LCD.print(", ");
  LCD.print(agm.calcGyro(agm.gz), 2);
  delay(1000);
  //------------------------------------------------------//
  Serial.println();
  Serial.print("(Accelometer");
  Serial.print(agm.calcAccel(agm.ax), 2);
  Serial.print(", ");
  Serial.print(agm.calcAccel(agm.ay), 2);
  Serial.print(", ");
  Serial.print(agm.calcAccel(agm.az), 2);
  Serial.print("");
  clearLCD();
  lcdPosition(0,0);
  LCD.print("(Accelometer");
  lcdPosition(1,0);
  LCD.print(agm.calcAccel(agm.ax), 2);
  LCD.print(", ");
  LCD.print(agm.calcAccel(agm.ay), 2);
  LCD.print(", ");
  LCD.print(agm.calcAccel(agm.az), 2);
  delay(1000);
  // ---------------------------------------------------------//
  Serial.println();
  Serial.print("(Magnetometer");
  Serial.print(agm.calcMag(agm.mx), 2);
  Serial.print(", ");
  Serial.print(agm.calcMag(agm.my), 2);
  Serial.print(", ");
  Serial.print(agm.calcAccel(agm.mz), 2);
  Serial.print("");
  clearLCD();
  lcdPosition(0,0);
  LCD.print("Magnetometer");
  lcdPosition(1,0);
  LCD.print(agm.calcMag(agm.mx), 2);
  LCD.print(", ");
  LCD.print(agm.calcMag(agm.my), 2);
  LCD.print(", ");
  LCD.print(agm.calcMag(agm.mz), 2);
  delay(1000);
  
}




