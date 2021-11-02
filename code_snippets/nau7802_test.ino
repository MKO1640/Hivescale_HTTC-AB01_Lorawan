/*
  Use the Qwiic Scale to read load cells and scales
  By: Nathan Seidle @ SparkFun Electronics
  Date: March 3rd, 2019
  License: This code is public domain but you buy me a beer if you use this 
  and we meet someday (Beerware license).

  The Qwiic Scale is an I2C device that converts analog signals to a 24-bit
  digital signal. This makes it possible to create your own digital scale
  either by hacking an off-the-shelf bathroom scale or by creating your
  own scale using a load cell.

  This example merely outputs the raw data from a load cell. For example, the
  output may be 25776 and change to 43122 when a cup of tea is set on the scale.
  These values are unitless - they are not grams or ounces. Instead, it is a
  linear relationship that must be calculated. Remeber y = mx + b?
  If 25776 is the 'zero' or tare state, and 43122 when I put 15.2oz of tea on the
  scale, then what is a reading of 57683 in oz?

  (43122 - 25776) = 17346/15.2 = 1141.2 per oz
  (57683 - 25776) = 31907/1141.2 = 27.96oz is on the scale
  
  SparkFun labored with love to create this code. Feel like supporting open
  source? Buy a board from SparkFun!
  https://www.sparkfun.com/products/15242

  Hardware Connections:
  Plug a Qwiic cable into the Qwiic Scale and a RedBoard Qwiic
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 9600 baud to see the output
*/

#include <Wire.h>

#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_NAU7802
#include <RunningMedian.h>
#define Vext_on  digitalWrite(Vext, LOW);
#define Vext_off digitalWrite(Vext, HIGH);
RunningMedian samples = RunningMedian(20);
NAU7802 myScale; //Create instance of the NAU7802 class

void setup()
{
  boardInitMcu();
  Serial.begin(115200);
  Serial.println("Qwiic Scale Example");
  pinMode(Vext, OUTPUT);
  Wire.begin();
  Wire.setClock(400000);
  if (myScale.begin() == false)
  {
    Serial.println("Scale not detected. Please check wiring. Freezing...");
    
  }
  Serial.println("Scale detected!");
  //myScale.clearBit(4,0x1b);
  delay(600);
  if (myScale.getBit(4,0x1b) == true)
  {
    Serial.println("PGA-Bypass is enabled.");
  }
  else 
  {
      Serial.println("PGA-Bypass is disabled.");
  }
  myScale.setLDO(NAU7802_LDO_3V0);
  myScale.setGain(128);
  myScale.setChannel(0);
  delay(600);
}



void loop()
{
Vext_on
delay(300);
if (myScale.begin() == false)
  {
    Serial.println("Scale not detected. Please check wiring. Freezing...");
    
  }
  Serial.print("Scale detected! ");
  myScale.setLDO(NAU7802_LDO_3V0);
  myScale.setGain(128);
  myScale.setChannel(0);
  myScale.setCalibrationFactor(8800);
  myScale.setZeroOffset(44.406);
  myScale.calibrateAFE();
  myScale.waitForCalibrateAFE();
  int i=30;
  delay(30);
  while(myScale.available() == false|i>2000){
    delay(1);
    i++;
  }

  Serial.print("Available after ");
  Serial.print(i);
  Serial.println(" ms");
    
  if(myScale.available() == true)
  {
    int x = (myScale.getReading()-8800)/44.406;
    for (int i=0;i<40;i++){
      samples.add(x);
    }

    Serial.print("Reading Median: ");
    Serial.print(samples.getMedian());
    //Serial.println((myScale.getAverage(10)-8800)/44.406,0);
    Serial.print("  Average: ");
    Serial.println((myScale.getAverage(20)-8800)/44.406);
  }
  else{Serial.println("not avilable");}
  myScale.powerDown();
Vext_off
delay(3000); 
}
