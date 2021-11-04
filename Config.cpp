#ifndef CONFIG_H_
#define CONFIG_H_

#include "Arduino.h"
#include "LoRaWan_APP.h"

#define HX711_en
//#define NAU7802_en
#define DS18B20_en
#define BME280_en

#define Debug
//Set these OTAA parameters to match your app/node in TTN

uint8_t devEui[] = { 0x07, 0x00, 0xB3, 0x0D, 0x50, 0x7E, 0x0D, 0x00 };
uint8_t appEui[] = { 0x07, 0x00, 0xB3, 0x0D, 0x50, 0x7E, 0x0D, 0x00 };
uint8_t appKey[] = { 0x00, 0xF0, 0x90, 0x01, 0x20, 0x08, 0x0B, 0x00, 0x40, 0x00, 0x40, 0x70, 0x03, 0x40, 0x00, 0x0B};

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

/* This Settings are for the NAU7802 or HX711
TODO Config Tool and save Values (No Eprom we need to use the first bytes of Flash */
#define scale_offset     8800
#define scale_cal_factor 44.406
#define AVG_SIZE         30

#endif /* #ifndef CONFIG_H_ */