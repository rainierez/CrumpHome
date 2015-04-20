// **********************************************************************************************************
// Copyright (2015) Felix Rusu of http://lowpowerlab.com
// *************************************************************************************************************
// License
// *************************************************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 2 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE.  See the GNU General Public        
// License for more details.                              
//                                                        
// You should have received a copy of the GNU General    
// Public License along with this program; if not, write 
// to the Free Software Foundation, Inc.,                
// 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
//                                                        
// Licence can be viewed at                               
// http://www.fsf.org/licenses/gpl.txt                    
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// *************************************************************************************************************
// v2.0 Combining MightyBoost Control
// v1.3 Update serial prints
// v1.0 Custom for my environment
// **********************************************************************************************************

#include <RFM69.h>
#include <RFM69registers.h>
#include <SPI.h>
//#include <stdio.h>

//*****************************************************************************************************************************
// ADJUST THE SETTINGS BELOW DEPENDING ON YOUR HARDWARE/SITUATION!
//*****************************************************************************************************************************
#define GARAGE1NODEID         47      //GarageMote 1
#define SW1NODEID             79      //SwitchMote 1
#define SECNODEID             61      //SecurityMote
#define MM1NODEID             88      //MotionMote 1
#define SM1NODEID             76      //SensorMote 1

#define NODEID                1
#define NETWORKID             214

#define FREQUENCY             RF69_915MHZ
#define ENCRYPTKEY            "sampleEncryptKey" //has to be same 16 characters/bytes on all nodes, not more not less!
#define IS_RFM69HW                  //uncomment only for RFM69HW! Leave out if you have RFM69W!

#define SERIAL_BAUD           9600
#define ACK_TIME              40    // # of ms to wait for an ack

#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
#else
  #define LED           9 // Moteinos have LEDs on D9
#endif

#define MB_LED                5     // LED pin, should be analog for fading effect (PWM)
#define BUTTON                3     // Power button pin
#define SIG_SHUTOFF           6     // Signal to Pi to ask for a shutdown
#define SIG_BOOTOK            A0    // Signal from Pi that it's OK to cutoff power
#define OUTPUT_5V             4     // HIGH on this pin will switch the "5V*" output ON
#define BATTERYSENSE          A7    // Sense VBAT_COND signal (when powered externally should read ~3.25v/3.3v (1000-1023), when external power is cutoff it should start reading around 2.85v/3.3v * 1023 ~= 880 (ratio given by 10k+4.7K divider from VBAT_COND = 1.47 multiplier)
                                    // hence the actual input voltage = analogRead(A7) * 0.00322 (3.3v/1024) * 1.47 (10k+4.7k voltage divider ratio)
                                    // when plugged in this should be 4.80v, nothing to worry about
                                    // when on battery power this should decrease from 4.15v (fully charged Lipoly) to 3.3v (discharged Lipoly)
                                    // trigger a shutdown to the target device once voltage is around 3.4v to allow 30sec safe shutdown
#define LOWBATTERYTHRESHOLD    3.6  // a shutdown will be triggered to the target device when battery voltage drops below this (Volts)

#define RESETHOLDTIME           500   // Button must be hold this many mseconds before a reset is issued (should be much less than SHUTDOWNHOLDTIME)
#define SHUTDOWNHOLDTIME        2000  // Button must be hold this many mseconds before a shutdown sequence is started (should be much less than ForcedShutoffDelay)
#define ShutoffTriggerDelay     6000  // will start checking the SIG_BOOTOK line after this long
#define RecycleTime             50000 // window of time in which SIG_BOOTOK is expected to go HIGH
                                      // should be at least 3000 more than Min
                                      // if nothing happens after this window, if button is 
                                      // still pressed, force cutoff power, otherwise switch back to normal ON state
#define RESETPULSETIME          500   // When reset is issued, the SHUTOFF signal is held HIGH this many ms
#define ForcedShutoffDelay   7500 // when SIG_BOOTOK==0 (PI in unknown state): if button is held
                                  // for this long, force shutdown (this should be less than RecycleTime)
#define ShutdownFinalDelay   4500 // after shutdown signal is received, delay for this long
                                  // to allow all PI LEDs to stop activity (pulse LED faster)

#define PRINTPERIOD              10000

//*****************************************************************************************************************************
int chk_volt_time = 30000;
int lastValidReading = 1;
unsigned long lastValidReadingTime = 0;
unsigned long NOW=0;
int PowerState = 0;
long lastPeriod = -1;
float battAvg = 0;                    // Keeping a running average of samples taken in the timer interrupt
byte battCount = 0;                   // Count of samples taken;
byte reportCount = 6;                // The count at which to report
float systemVoltage = 5;

RFM69 radio;
bool promiscuousMode = false; //set to 'true' to sniff all packets on the same network
byte readSerialLine(char* input, char endOfLineChar=10, byte maxLength=64, uint16_t timeout=50);
char buff[50];

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(10);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(SIG_BOOTOK, INPUT);
  pinMode(SIG_SHUTOFF, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(MB_LED, OUTPUT);
  pinMode(OUTPUT_5V, OUTPUT);
  pinMode(A7, INPUT);
  digitalWrite(SIG_SHUTOFF, LOW);//added after sudden shutdown quirks, DO NOT REMOVE!
  digitalWrite(OUTPUT_5V, LOW);//added after sudden shutdown quirks, DO NOT REMOVE!
  
  Serial.print("1,Startup,1;");
  delay(600);
  check_systemVoltage(true);
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW
  radio.setHighPower(); //must include only for RFM69HW!
#endif
  radio.encrypt(ENCRYPTKEY);
}

byte ackCount=0;
byte inputLen=0;
char input[64];
void loop() {
  int reading = digitalRead(BUTTON);
  
  if (millis() - lastPeriod > chk_volt_time) {
    lastPeriod=millis();
    check_systemVoltage(true);
  }
  
  digitalWrite(SIG_SHUTOFF, LOW);//added after sudden shutdown quirks, DO NOT REMOVE!
  
  boolean batteryLow = systemVoltage < LOWBATTERYTHRESHOLD;
  
  NOW = millis();
  
  if (batteryLow || reading != lastValidReading && NOW - lastValidReadingTime > 200) {
    lastValidReading = reading;
    lastValidReadingTime = NOW;
    
    if (batteryLow || reading == 0)
    {
      //make sure the button is held down for at least 'SHUTDOWNHOLDTIME' before taking action (this is to avoid accidental button presses and consequently Pi shutdowns)
      NOW = millis();
      while (!batteryLow && (PowerState == 1 && millis()-NOW < SHUTDOWNHOLDTIME)) { delay(10); if (digitalRead(BUTTON) != 0) return; }
          
      //RESETHOLDTIME is satisfied, now check if button still held until SHUTDOWNHOLDTIME is satisfied
      analogWrite(LED, 128); //dim the LED to show something's going on
      while (!batteryLow && (PowerState == 1 && millis()-NOW < SHUTDOWNHOLDTIME))
      {
        if (digitalRead(BUTTON) != 0)
        {
          if (BOOTOK())       //SIG_BOOTOK is HIGH so Pi is running the shutdowncheck.sh script, ready to intercept the RESET PULSE
          {
            digitalWrite(SIG_SHUTOFF, HIGH);
            delay(RESETPULSETIME);
            digitalWrite(SIG_SHUTOFF, LOW);

            NOW = millis();
            boolean recycleDetected=false;
            while (millis()-NOW < RecycleTime) //blink LED while waiting for BOOTOK to go high
            {
              //blink 3 times and pause
              digitalWrite(LED, LOW);
              delay(100);
              digitalWrite(LED, HIGH);
              delay(100);
              digitalWrite(LED, LOW);
              delay(100);
              digitalWrite(LED, HIGH);
              delay(100);
              digitalWrite(LED, LOW);
              delay(100);
              digitalWrite(LED, HIGH);
              delay(500);

              if (!BOOTOK()) recycleDetected = true;
              else if (BOOTOK() && recycleDetected)
                return;
            }
            return; //reboot pulse sent but it appears a reboot failed; exit all checks
          }
          else return; //ignore everything else (button was held for RESETHOLDTIME, but SIG_BOOTOK was LOW)
        }
      }
          
      //SIG_BOOTOK must be HIGH when Pi is ON. During boot, this will take a while to happen (till it executes the "shutdowncheck" script
      //so I dont want to cutoff power before it had a chance to fully boot up
      if (batteryLow || (PowerState == 1 && BOOTOK()))
      {
        Serial.print("1,signaling,request to shutdown;");
        // signal Pi to shutdown
        digitalWrite(SIG_SHUTOFF, HIGH);

        //NOW wait for the Pi to signal back
        NOW = millis();
        float in, out;
        boolean forceShutdown = true;
        
        while (millis()-NOW < RecycleTime)
        {
          if (in > 6.283) in = 0;
          in += .00628;
          
          out = sin(in) * 127.5 + 127.5;
          analogWrite(MB_LED,out);
          delayMicroseconds(1500);
          
          //account for force-shutdown action (if button held for ForcedShutoffDelay, then force shutdown regardless)
          if (millis()-NOW <= (ForcedShutoffDelay-SHUTDOWNHOLDTIME) && digitalRead(BUTTON) != 0)
            forceShutdown = false;
          if (millis()-NOW >= (ForcedShutoffDelay-SHUTDOWNHOLDTIME) && forceShutdown)
          {
            Serial.print("1,signaling,forced shutdown;");
            delay(250);
            Serial.print("1,5v output,off;");
            PowerState = 0;
            digitalWrite(MB_LED, PowerState); //turn off LED to indicate power is being cutoff
            digitalWrite(OUTPUT_5V, PowerState);
            break;
          }
          
          if (millis() - NOW > ShutoffTriggerDelay)
          {
            // Pi signaling OK to turn off
            if (BOOTOK())
            {
              PowerState = 0;
              digitalWrite(MB_LED, PowerState); //turn off LED to indicate power is being cutoff
              NOW = millis();
              while (millis()-NOW < ShutdownFinalDelay)
              {
                if (in > 6.283) in = 0;
                in += .00628;
                
                out = sin(in) * 127.5 + 127.5;
                analogWrite(MB_LED,out);
                delayMicroseconds(300);
              }
              Serial.print("1,5v outout,off;");
              digitalWrite(OUTPUT_5V, PowerState);
              break;
            }
          }
        }
        
        // last chance: if power still on but button still pressed, force cutoff power
        if (PowerState == 1 && digitalRead(BUTTON) == 0)
        {
          Serial.print("1,signaling,forced shutdown;");
          delay(250);
          Serial.print("1,5v output,off;");
          PowerState = 0;
          digitalWrite(OUTPUT_5V, PowerState);
        }
        
        digitalWrite(SIG_SHUTOFF, LOW);
      }
      else if (PowerState == 1 && analogRead(SIG_BOOTOK)<800)
      {
        NOW = millis();
        unsigned long NOW2 = millis();
        int analogstep = 255 / ((ForcedShutoffDelay-SHUTDOWNHOLDTIME)/100); //every 500ms decrease LED intensity
        while (digitalRead(BUTTON) == 0)
        {
          if (millis()-NOW2 > 100)
          {
            analogWrite(MB_LED, 255 - ((millis()-NOW)/100)*analogstep);
            NOW2 = millis();
          }
          if (millis()-NOW > ForcedShutoffDelay-SHUTDOWNHOLDTIME)
          {
            Serial.print("1,5v output,off;");
            //TODO: add blinking here to signal final shutdown delay
            PowerState = 0;
            digitalWrite(OUTPUT_5V, PowerState);
            break;
          }
        }
      }
      else if (PowerState == 0)
      {
        Serial.print("1,5v output,on;");
        PowerState = 1;
        digitalWrite(OUTPUT_5V, PowerState); //digitalWrite(LED, PowerState);
      }
    }
    if (PowerState && BOOTOK()) {
      digitalWrite(MB_LED, LOW);
    } else {
      digitalWrite(MB_LED, PowerState);
    }
  }
  
  //process any serial input
  inputLen = readSerialLine(input);
      
  if (inputLen > 0) { handle_serial_input(input, inputLen); }

  if (radio.receiveDone()) {
    digitalWrite(LED,HIGH);
    Serial.print(radio.SENDERID, DEC);Serial.print(",");
    for (byte i = 0; i < radio.DATALEN; i++) {
      Serial.print((char)radio.DATA[i]);
    }
    if (radio.ACKRequested()){
      byte theNodeID = radio.SENDERID;
      radio.sendACK();
    }
    digitalWrite(LED,LOW);
  }
}

boolean BOOTOK() {
  return analogRead(SIG_BOOTOK) > 800;
}

void check_systemVoltage(boolean report_it) {
  systemVoltage = analogRead(BATTERYSENSE) * 0.00322 * 1.47;
  Serial.print("1,voltage,"); Serial.print(systemVoltage); Serial.print(";"); 
}

// reads a line feed (\n) terminated line from the serial stream
// returns # of bytes read, up to 255
// timeout in ms, will timeout and return after so long
byte readSerialLine(char* input, char endOfLineChar, byte maxLength, uint16_t timeout)
{
  byte inputLen = 0;
  Serial.setTimeout(timeout);
  inputLen = Serial.readBytesUntil(endOfLineChar, input, maxLength);
  input[inputLen]=0;//null-terminate it
  Serial.setTimeout(0);
  return inputLen;
}

void handle_serial_input(char* input, int inputLen) {
  boolean send_request = false;
  byte toNodeID;
  char cmd_buff[10];  
  byte retries = 2;
  
  // Checking Gateway status by way of reporting the voltage powering the gateway
  if (input[0]=='G' && input[1]=='W' && input[3] == 'S' && input[4] == 'T' && input[5] == 'S') {
    check_systemVoltage(true);
  }
  
  /* Serial line formatted as follows, "GR<n><x>" where <n> is the garage number, <x> is the desired state. 
  examples: (No quotes)
   "GD1_OPN" would open the Garage 1 door.
   "GD1_CLS" would close the Garage 1 door.
   "GD1_STS would request the current status of the Garage 1 door */
  else if (input[0]=='G' && input[1]=='D' && inputLen >= 7)
  {
    if(input[2] == 1) {toNodeID = GARAGE1NODEID;}
    
    if ((input[4] == 'O' && input[5]=='P' && input[6]=='N') ||
    (input[4] == 'C' && input[5]=='L' && input[6]=='S') ||
    (input[4] == 'S' && input[5]=='T' && input[6]=='S')) {
      send_request = true;
      sprintf(cmd_buff, "%c%c%c", input[4], input[5], input[6]);
    }
  }
  
  /* Serial line formatted as follows, "SW<n>_BTN<x>:<y>" where <n> is the switch number, <x> is the button number, <y> is the desired state. . 
  examples: (No Quotes)
   "SW1_BTN1:0" would set the middle button on switch 1 to off 
   "SW1_BTN0:1" would set the top button on switch 1 to on
   "SW1_SSR:1" would set the SSR and it's associated button to on
   "SW1_STS3" would request the current status of the bottom button on switch 1*/
  else if (input[0]=='S' && input[1]=='W') {
    if (input[2] == 1) { toNodeID = SW1NODEID; }
    
    if (input[4] == 'B' && input[5]=='T' && input[6]=='N' && inputLen >= 10) {
      sprintf(cmd_buff, "BTN%d:%d", input[7]-'0', input[9]-'0'); 
      send_request = true;
    } else if (input[4] == 'S' && input[5] == 'S' && input[6] == 'R' && inputLen >= 9) {
      sprintf(cmd_buff, "SSR:%i", input[8]-'0');
      send_request = true;
    } else if (input[4] == 'S' && input[5] == 'T' && input[6]== 'S' && inputLen >= 8) {
      if (input[7] == '0' || input[7] == '1' || input[7] == '2' || input[7] == '3') {
        sprintf(cmd_buff, "STS%i", input[7]-'0');
        send_request = true;
      }
    }
  }
  
  /* Serial line formatted as follows, 
    "SEC_STS" Requests the security system status and should return each zone value and the armed and alarm statuses
    "SEC_ARM<0, 2, or 3>" arms the system; where 0 is Disarm, 2 is soft arm, and 3 is full arm*/
  else if (input[0] == 'S' && input[1] == 'E' && input[2] == 'C') {
    toNodeID = SECNODEID;
    
    if (input[4] == 'S' && input[5] == 'T' && input[6] == 'S' && inputLen >= 7) {
      sprintf(cmd_buff, "STS");
      send_request = true;
    }
    if (input[4] == 'A' && input[5] == 'R' && input[6] == 'M' && inputLen >= 8) {
      if (input[7] == '0' || input[7] == '2' || input[7] == '3') {
        sprintf(cmd_buff, "ARM%d", input[7]-'0');
        send_request = true;
        retries = 4;
      }
    }
  }
  
  /* Serial line formatted as follows, 
    "SM1_LUX" Requests the lumen value from SensorMote 1
    "SM1_TPF" Requests the temp in *f from SensorMote 1 
    "SM1_HUM" Requests the humidity from SensorMote 1
    "SM1_ALL" Requests the data points from SensorMote 1*/
  else if (input[0] == 'S' && input[1] == 'M') {
    if (input[2] == 1) { toNodeID = SM1NODEID; }
    
    if (inputLen >= 7) {
      send_request = true;
      sprintf(cmd_buff, "%c%c%c", input[4], input[5], input[6]);
    }
  }
  
  if (send_request) {    
    delay(250);
    int cmd_len = strlen(cmd_buff);
    if(!radio.sendWithRetry(toNodeID, cmd_buff, cmd_len, retries)){
      Serial.print(toNodeID);
      Serial.print(",Send failed,0;");
    }
  }
}
