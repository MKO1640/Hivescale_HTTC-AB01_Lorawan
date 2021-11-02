/**
 * This is an example of joining, sending and receiving data via LoRaWAN using a more minimal interface.
 * 
 * 
 * Autor Michael Kurzweil 
 */
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_BME280.h>
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"
#include <EEPROM.h>
#include <cy_em_eeprom.h>

#define Debug
//Set these OTAA parameters to match your app/node in TTN

uint8_t devEui[] = { 0x07, 0x00, 0xB3, 0x0D, 0x50, 0x7E, 0x0D, 0x00 };
uint8_t appEui[] = { 0x07, 0x00, 0xB3, 0x0D, 0x50, 0x7E, 0x0D, 0x00 };
uint8_t appKey[] = { 0x00, 0xF0, 0x90, 0x01, 0x20, 0x08, 0x0B, 0x00, 0x40, 0x00, 0x40, 0x70, 0x03, 0x40, 0x00, 0x0B};
//uint8_t appKey[] = { 0x0F, 0x90, 0x12, 0x08, 0xB0, 0x40, 0x04, 0x70, 0x34, 0x00, 0xB0, 0x84, 0x00, 0x70, 0x03, 0x08 };

/* ABP para*/
uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda,0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef,0x67 };
uint32_t devAddr =  ( uint32_t )0x007e6ae1;
                           
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = LORAWAN_CLASS;

/*the data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 60 *1000; // sec*ms

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 2;
/*!
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowledgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 4;

/* This Settings are for the NAU7802
TODO Config Tool and save Values (No Eprom we need to use the first bytes of Flash */

NAU7802 myScale;
#define scale_offset     8800
#define scale_cal_factor 44.406
#define AVG_SIZE         30
int16_t weight;

uint16_t baseline;
uint16_t temperature ;
uint16_t humidity,pressure;

Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

#define Vext_on  digitalWrite(Vext, LOW);
#define Vext_off digitalWrite(Vext, HIGH);

static void prepareTxFrame( uint8_t port )
{
	Vext_on
  delay(100);
  Wire.begin();
   // Wait for starting Sensors
  Serial.println("read sensors:");
  ///**************** Read BME280 **************************
  
  Serial.println("read BME280:");
  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    while (1) delay(10);
  }
  
  //bme_temp->printSensorDetails();
  //bme_pressure->printSensorDetails();
  //bme_humidity->printSensorDetails();
  sensors_event_t temp_event, pressure_event, humidity_event;
  bme_temp->getEvent(&temp_event);
  bme_pressure->getEvent(&pressure_event);
  bme_humidity->getEvent(&humidity_event);
  
  humidity= humidity_event.relative_humidity*100;
  temperature= (temp_event.temperature+50.00)*100;
  pressure = (pressure_event.pressure-200)*10;

  bme.MODE_SLEEP;

  ///**************** Read NAU7802 ***************************
  
  Wire.setClock(400000);
  if (myScale.begin() == false)
    {Serial.println("Scale not detected. Please check wiring.");}
  else
    {Serial.println("Scale detected!");}
  if (myScale.getZeroOffset()!=scale_offset){
    Serial.println("Scale config and calibrate...");
    myScale.setLDO(NAU7802_LDO_3V0);
    myScale.setGain(128);
    myScale.setChannel(0);
    myScale.setCalibrationFactor(scale_cal_factor);
    myScale.setZeroOffset(scale_offset);
    //myScale.calibrateAFE();
    //myScale.waitForCalibrateAFE();
  }
  else {
    Serial.println("Scale was ready..");
  }
  myScale.calibrateAFE();
  myScale.waitForCalibrateAFE();
  int i=600;
  delay(600);
  
  //wait if availabe timeout ca.2000ms
  while(myScale.available() == false|i>2000){
    delay(1);
    i++;
  }
#ifdef Debug 
  Serial.print("Available after ");
  Serial.print(i);
  Serial.println(" ms");
#endif 
  if (myScale.available() == true)
   {
    weight = ((myScale.getAverage(AVG_SIZE)-scale_offset)/scale_cal_factor);
    Serial.print("read NAU Raw =");
    Serial.println(weight);
    }
  else
    {Serial.println("NAU7802 not ready");
    weight = 0;}
  
  myScale.powerDown();
  Wire.end();
  Vext_off
// *******************Read 1Wire********************************
// 

// ***************** Read Voltage *****************************
  uint16_t batteryVoltage = getBatteryVoltage();
#ifdef Debug
  Serial.print(F("Temperature = "));
  Serial.print(temperature);
  Serial.println(" *C");

  Serial.print(F("Humidity = "));
  Serial.print(humidity);
  Serial.println(" %");

  Serial.print(F("Pressure = "));
  Serial.print(pressure);
  Serial.println(" hPa");

  Serial.print(F("batteryVoltage = "));
  Serial.print(batteryVoltage);
  Serial.println(" V");

  Serial.print(F("weight = "));
  Serial.print(weight,3);
  Serial.println(" KG");
#endif  
//  ***************** Make Payload *****************************
  appData[0] = (uint8_t)(batteryVoltage >> 8);
  appData[1] = (uint8_t)batteryVoltage;

  appData[2] = (uint8_t)(temperature >> 8);
  appData[3] = (uint8_t)temperature;
  
  appData[4] = (uint8_t)(humidity >> 8);
  appData[5] = (uint8_t)humidity;

  appData[8] = (uint8_t)(pressure >> 8);
  appData[9] = (uint8_t)pressure;
  
  appData[6] = (uint8_t) ((weight/2) >> 8);
  appData[7] = (uint8_t) (weight/2) ;
   /*
 unsigned char *puc;

  puc = (unsigned char *)(weight);
  appData[8] = puc[0];
  appData[9] = puc[1];
  appData[10] = puc[2];
  appData[11] = puc[3];
 */
#ifdef Debug
  Serial.print("T-BME280=");
  Serial.print((temperature-5000.00)/100);
  Serial.print("C, RH=");
  Serial.print(humidity/100);
  Serial.print("%, Pressure=");
  Serial.print(pressure);
  Serial.print(" hPA, BatteryVoltage:");
  Serial.print(batteryVoltage/1000.00,2);
  Serial.print(" V , NAU7802= :");
  Serial.print(weight);
  Serial.println(" Raw");
  Serial.print("Payload : ");
  for (int i=0;i<=11;i++){
    Serial.print(" ");
    Serial.print(appData[i]);
    }
  Serial.println(" Raw");
#endif
}
void setup() {
	//#ifdef Debug
      Serial.begin(115200);   // TODO for Power saving switch the serial output off. Switch on by User Button at startup
  //#endif

  pinMode(Vext, OUTPUT); //Set Vext Pin to Output. This pin can use as Power Suply for Sensors up to 350mA  

#if(AT_SUPPORT)
	enableAt();
#endif
	deviceState = DEVICE_STATE_INIT;
	LoRaWAN.ifskipjoin();
}

void loop()
{
	switch( deviceState )
	{
		case DEVICE_STATE_INIT:
		{
#if(LORAWAN_DEVEUI_AUTO)
			LoRaWAN.generateDeveuiByChipID();
#endif
#if(AT_SUPPORT)
			getDevParam();
#endif
			printDevParam();
			LoRaWAN.init(loraWanClass,loraWanRegion);
			deviceState = DEVICE_STATE_JOIN;
			break;
		}
		case DEVICE_STATE_JOIN:
		{
			LoRaWAN.join();
			break;
		}
		case DEVICE_STATE_SEND:
		{
      prepareTxFrame( appPort );
			LoRaWAN.send();
			deviceState = DEVICE_STATE_CYCLE;
			break;
		}
		case DEVICE_STATE_CYCLE:
		{
			// Schedule next packet transmission
			txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
			LoRaWAN.cycle(txDutyCycleTime);
			deviceState = DEVICE_STATE_SLEEP;
			break;
		}
		case DEVICE_STATE_SLEEP:
		{
			LoRaWAN.sleep();
			break;
		}
		default:
		{
			deviceState = DEVICE_STATE_INIT;
			break;
		}
	}
}
