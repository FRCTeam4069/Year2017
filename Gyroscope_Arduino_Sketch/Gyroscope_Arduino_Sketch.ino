#include <Wire.h>
#include "NAxisMotion.h"        //Contains the bridge code between the API and the Arduino Environment
//#include <SoftwareSerial.h>

NAxisMotion mySensor; 

unsigned long serStreamTime=0;
unsigned long lastStreamTime = 0;     //To store the last streamed time stamp
const int streamPeriod = 20;          //To stream at 50Hz without using additional timers (time period(ms) =1000/frequency(Hz))
float heading;
char sheading[32];

void setup()
{
  Wire.begin(0x51);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(pushEvent);
  I2C.begin();
  mySensor.initSensor();          //The I2C Address can be changed here inside this function in the library default is 0x28
  mySensor.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
  mySensor.setUpdateMode(MANUAL);  //The default is AUTO. Changing to MANUAL requires calling the relevant update functions prior to calling the read functions

//NOTE: Serial interferes with roborio's MXP serial port, they are the same uart lines! LIDAR uses MXP serial, so be careful here.
//  Serial.begin(115200);
//  Serial.print("Hello");
}
void loop()
{
  if ((millis() - lastStreamTime) >= streamPeriod)
  {
    lastStreamTime = millis();    
    mySensor.updateEuler();        //Update the Euler data into the structure of the object
    mySensor.updateCalibStatus();  //Update the Calibration Status
    heading = mySensor.readEulerHeading();

//   Serial.print(" H: ");
//   Serial.print(heading); //mySensor.readEulerHeading()); //Heading data
//   Serial.println("deg ");

    dtostrf(heading,6,2,sheading);
//#ifdef DEBUG_ON
//    sprintf(sheading,"%f",heading);
//    Serial.print("SHEAD:");
//    Serial.println(sheading);
//#endif
  }
  //delay(10);
}
void receiveEvent(int howMany)
{
//  Serial.print("GotString!");
  String rxString = "";
  while ( Wire.available() > 0 )
  {
    char n=(char)Wire.read();
    if(((int)n)>0) //((int)(' ')))
    rxString += n; 
  }  
 // Wire.write("abcdefghijk");  
//  Serial.print(rxString);
}

void pushEvent(int howmany)
{
  char str[16];
  sprintf(str,"%s      ",sheading); //val); //heading);
  Wire.write(str); //"0123456789A"); //sheading); //"0123456789A"); 
  //Serial.print("Sent:");
  //Serial.println(str);
}

