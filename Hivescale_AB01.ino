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
#include "Config.cpp"

#ifdef NAU7802_en
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"
NAU7802 myScale;
#endif

#ifdef HX711_en
#include "HX711.h"
// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = GPIO2;
const int LOADCELL_SCK_PIN = GPIO1;

HX711 scale;
#endif

uint16_t baseline;
uint16_t temperature ;
uint16_t humidity,pressure;
int16_t weight;

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

#ifdef NAU7802_en
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
#endif 
#ifdef HX711_en

#endif
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
