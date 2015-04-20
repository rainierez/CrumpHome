// My Garage Mote combined with a motion sensor, Temp/Humidity, and Lux sensors
// Get the RFM69 and SPIFlash library at: https://github.com/LowPowerLab/
// Make sure you adjust the settings in the configuration section below !!!

// **********************************************************************************
// Copyright Felix Rusu, LowPowerLab.com
// Library and code by Felix Rusu - felix@lowpowerlab.com
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//                                                        
// You should have received a copy of the GNU General    
// Public License along with this program.
// If not, see <http://www.gnu.org/licenses/>.
//                                                        
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************

#include <Wire.h>
#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>
#include <LowPower.h> //get library from: https://github.com/lowpowerlab/lowpower
                      //writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/
#include "DHT.h"         //https://github.com/adafruit/DHT-sensor-library
#include <BH1750FVI.h>   //https://github.com/Genotronex/BH1750FVI_Master

//*********************************************************************************************
// *********** IMPORTANT SETTINGS - YOU MUST CHANGE/ONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#define NODEID          72
#define NETWORKID       214
#define GATEWAYID       1
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "chrisWcrumpacker" //exactly the same 16 characters/bytes on all nodes!
//#define IS_RFM69HW    //uncomment only for RFM69HW! Remove/comment if you have RFM69W!
//*********************************************************************************************

#define ACK_TIME      30 // max # of ms to wait for an ack
#define LEDPIN        9
#define BATT_MONITOR  A6  // Sense VBAT_COND signal (when powered externally should read ~3.25v/3.3v (1000-1023), when external power is cutoff it should start reading around 2.85v/3.3v * 1023 ~= 880 (ratio given by 10k+4.7K divider from VBAT_COND = 1.47 multiplier)
#define BATT_CYCLES   30  //read and report battery voltage every this many wakeup cycles (ex 30cycles * 8sec sleep = 240sec/4min)
#define MOTIONPIN      1 //hardware interrupt 1 (D3)
#define DHTPIN          A2
  
// Uncomment whatever type you're using!
#define DHTTYPE        DHT11   // DHT 11 
//#define DHTTYPE        DHT22   // DHT 22  (AM2302)
//#define DHTTYPE        DHT21   // DHT 21 (AM2301)

//#define SERIAL_EN             //comment this out when deploying to an installed SM to save a few KB of sketch size
#define SERIAL_BAUD    115200
#ifdef SERIAL_EN
#define DEBUG(input)   {Serial.print(input); delay(1);}
#define DEBUGln(input) {Serial.println(input); delay(1);}
#else
#define DEBUG(input);
#define DEBUGln(input);
#endif

DHT dht(DHTPIN, DHTTYPE);
BH1750FVI LightSensor;
RFM69 radio;
char* buff;
volatile boolean motionDetected=false;
float batteryVolts = 5;
char* BATstr="BAT:5.00v";
char tempChar[5];
char humChar[5];
char sendBuf[32];
byte sendLen;
float tempf = 0, hum = 0;
int lux = 0;

void setup() {
  Serial.begin(SERIAL_BAUD);
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
#endif
  radio.encrypt(ENCRYPTKEY);
  
  dht.begin();
  
  LightSensor.begin();
  LightSensor.SetAddress(Device_Address_H);
  LightSensor.SetMode(Continuous_H_resolution_Mode);
  
  pinMode(MOTIONPIN, INPUT);
  attachInterrupt(MOTIONPIN, motionIRQ, RISING);
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);
  
  buff = "Startup,1;";
  Serial.println(buff);
  radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
  delay(1000);
}

void motionIRQ()
{
  motionDetected=true;
}

byte batteryReportCycles=0;
void loop() {
  stsUpdate();
  if (motionDetected)
  {
    digitalWrite(LEDPIN, HIGH);
    sprintf(sendBuf, "motion,1;");
    sendLen = strlen(sendBuf);

    if (radio.sendWithRetry(GATEWAYID, sendBuf, sendLen))
    {
      DEBUG("MOTION ACK:OK! RSSI:");
      DEBUG(radio.RSSI);
      batteryReportCycles = 0;
    }
    else DEBUG("MOTION ACK:NOK...");

    radio.sleep();
    digitalWrite(LEDPIN, LOW);
  }
  else if (batteryReportCycles == BATT_CYCLES)
  {
    reportDHT();
    reportLUX();
    delay(250);
    sprintf(sendBuf, "voltage,%s;", BATstr);
    sendLen = strlen(sendBuf);
    radio.send(GATEWAYID, sendBuf, sendLen);
    radio.sleep();
    batteryReportCycles=0;
  }
  motionDetected=false; //do NOT move this after the SLEEP line below or motion will never be detected
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  batteryReportCycles++;
}

/*==================================
// Report DHT values
===================================*/
void reportDHT() {
  delay(250);
  tempf = dht.readTemperature(true);
  dtostrf(tempf, 3,1, tempChar);
  sprintf(sendBuf, "temp,%s;", tempChar);
  radio.sendWithRetry(GATEWAYID, sendBuf, strlen(sendBuf));
  
  delay(250);
  hum = dht.readHumidity();
  dtostrf(hum, 3,1, humChar);
  sprintf(sendBuf, "hum,%s;", humChar);
  radio.sendWithRetry(GATEWAYID, sendBuf, strlen(sendBuf));
}

/*==================================
// Report Lux value
===================================*/
void reportLUX() {
  delay(250);
  uint16_t lux = LightSensor.GetLightIntensity();
  sprintf(sendBuf, "lux,%d;", lux);
  radio.sendWithRetry(GATEWAYID, sendBuf, strlen(sendBuf));
}

byte cycleCount=BATT_CYCLES;
void stsUpdate()
{
  if (cycleCount++ == BATT_CYCLES) //only read battery every BATT_CYCLES sleep cycles
  {
    unsigned int readings=0;
    for (byte i=0; i<10; i++) //take 10 samples, and average
      readings+=analogRead(BATT_MONITOR);
    batteryVolts = (readings / 10.0) * 0.00322 * 1.42;
    dtostrf(batteryVolts, 3,2, BATstr); //update the BATStr which gets sent every BATT_CYCLES or along with the MOTION message
    sprintf(sendBuf, "voltage,%s;", BATstr);
    sendLen = strlen(sendBuf);
    radio.send(GATEWAYID, sendBuf, sendLen);
    radio.sleep();
    cycleCount = 0;
    reportDHT();
    reportLUX();
  }
}
