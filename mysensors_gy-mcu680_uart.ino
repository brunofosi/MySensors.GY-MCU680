/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 *  DESCRIPTION
 *
 *  BME680 GY-MCU680 UART Air Quality Sensor
 *  
 *  This sensor measures Temperature (xx.xx cº), Humidity (xx.xx %), Pressure, Gas (ohm), IAQ (0-500)
 *
 */


//------------------------------------------------------------------------------

// if you uncomment this, you can get test and debug updates about the sensor' wireless connection by using the serial monitor tool.
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24                            // A 2.4Ghz transmitter and receiver, often used with MySensors.
#define MY_RF24_PA_LEVEL RF24_PA_MIN           // This sets a low-power mode for the radio. Useful if you use the verison with the bigger antenna, but don't want to power that from a separate power source. It can also fix problems with fake Chinese versions of the radio.
// #define MY_RADIO_RFM69                         // 433Mhz transmitter and reveiver.


// Libraries
#include <MySensors.h>
#include <SoftwareSerial.h>


// Feel free to change this:
unsigned long co2MeasurementInterval = 20000;     // Time to wait between reads (in milliseconds).
SoftwareSerial mySerial(4, 5);                  // RX, TX . You can choose other pins if you prefer.
uint16_t temp1=0;
int16_t temp2=0;
unsigned char Re_buf[30],counter=0;
unsigned char sign=0;
//int led = 13;

// Mysensors settings
#define CHILD_ID_BME680temp 1                          // The sensor' ID on this node.
#define CHILD_ID_BME680hum 2
#define CHILD_ID_BME680baro 3
#define CHILD_ID_BME680gas 4
#define CHILD_ID_BME680iaq 5
#define CHILD_ID_BME680iaqa 6

MyMessage msgBmeTemp(CHILD_ID_BME680temp, V_TEMP);
MyMessage msgBmeHum(CHILD_ID_BME680hum, V_HUM);
MyMessage msgBmePress(CHILD_ID_BME680baro, V_PRESSURE);
MyMessage msgBmeIAQ(CHILD_ID_BME680iaq, V_VAR1);
MyMessage msgBmeGas(CHILD_ID_BME680gas, V_VAR2);
MyMessage msgBmeIAQA(CHILD_ID_BME680iaqa, V_VAR3);



void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("BOSCH BME680 Air Quality Sensor", "1.0");

  // Register attached sensor(s) to gateway
    present(CHILD_ID_BME680temp, S_TEMP);
    present(CHILD_ID_BME680hum, S_HUM);
    present(CHILD_ID_BME680baro, S_BARO);
    present(CHILD_ID_BME680iaq, S_CUSTOM);
    present(CHILD_ID_BME680iaqa, S_CUSTOM);
    present(CHILD_ID_BME680gas, S_CUSTOM);          
}


void setup() 
{
  delay(1000);
  Serial.begin(115200);  
  delay(1000);
  mySerial.begin(9600);
  delay(1000);
  mySerial.listen();  
  delay(3000); 
  mySerial.write(0XA5); 
  mySerial.write(0X55); 
  mySerial.write(0X3F);  
  mySerial.write(0X39); 
  delay(200); 
  mySerial.write(0XA5); 
  mySerial.write(0X56);  
  mySerial.write(0X02);   
  mySerial.write(0XFD);
  delay(500); 

}


void loop() 
{
  float Temperature ,Humidity;
  unsigned char i=0,sum=0;
  uint32_t Gas;
  uint32_t Pressure;
  uint16_t IAQ;
  int16_t  Altitude;
  uint8_t IAQ_accuracy;
  

  // You should not change these variables:
  static unsigned long previousCo2Millis = 0; // Used to remember the time of the last temperature measurement.
  unsigned long currentMillis = millis(); // The time since the sensor started, counted in milliseconds. This script tries to avoid using the Sleep function, so that it could at the same time be a MySensors repeater.


  if (currentMillis - previousCo2Millis >= co2MeasurementInterval) { // this only gets triggered when enough time has passed.
 
  while (mySerial.available()) {   
    Re_buf[counter]=(unsigned char)mySerial.read();
    
    if(counter==0&&Re_buf[0]!=0x5A) return;         
    if(counter==1&&Re_buf[1]!=0x5A)
  {
    counter=0;
     return;
   };           
    counter++;       
    if(counter==20)               
    {    
       counter=0;                  
       sign=1;
    }      
  }
  if(sign)
  {  
     sign=0;
     
     if(Re_buf[0]==0x5A&&Re_buf[1]==0x5A )        
     {    
       
            for(i=0;i<19;i++)
               sum+=Re_buf[i]; 
             if(sum==Re_buf[i] ) 
             {
                     temp2=(Re_buf[4]<<8|Re_buf[5]);   
                     Temperature=(float)temp2/100;
                     temp1=(Re_buf[6]<<8|Re_buf[7]);
                     Humidity=(float)temp1/100; 
                     Pressure=((uint32_t)Re_buf[8]<<16)|((uint16_t)Re_buf[9]<<8)|Re_buf[10];
                     IAQ_accuracy= (Re_buf[11]&0xf0)>>4;
                     IAQ=((Re_buf[11]&0x0F)<<8)|Re_buf[12];
                     Gas=((uint32_t)Re_buf[13]<<24)|((uint32_t)Re_buf[14]<<16)|((uint16_t)Re_buf[15]<<8)|Re_buf[16];
                     Altitude=(Re_buf[17]<<8)|Re_buf[18]; 
                     Serial.print("Temperature:");
                     Serial.print(Temperature); 
                     Serial.print(" ,Humidity:"); 
                     Serial.print(Humidity); 
                     Serial.print(" ,Pressure:"); 
                     Serial.print(Pressure);     
                     Serial.print("  ,IAQ:");
                     Serial.print(IAQ); 
                     Serial.print(" ,Gas:"); 
                     Serial.print(Gas ); 
                     Serial.print("  ,Altitude:"); 
                     Serial.print(Altitude);                       
                     Serial.print("  ,IAQ_accuracy:"); 
                     Serial.println(IAQ_accuracy);
                     delay(1000); 
                    // return Humidity;  

            }   
    

   }
  }
    Serial.println("BME680 - Sending data request to sensor.");
    previousCo2Millis = currentMillis;


    float temp = Temperature; 
    float hum = Humidity;
    uint32_t pressu = Pressure;
    uint16_t iaq = IAQ;
    uint32_t gas = Gas;
    uint8_t iaqaccur = IAQ_accuracy;
    
    Serial.println("Temperature = " + String(temp));
    send(msgBmeTemp.set(Temperature, 2));
    Serial.println("Humidity = " + String(hum));
    send(msgBmeHum.set(Humidity, 2));
    Serial.println("Pressure = " + String(pressu)); 
    send(msgBmePress.set(Pressure, 0));
    Serial.println("IAQ = " + String(iaq));
    send(msgBmeIAQ.set(IAQ, 0));
    Serial.println("Gas = " + String(gas));
    send(msgBmeGas.set(Gas, 0));
    Serial.println("IAQ_accuracy = " + String(iaqaccur)); 
    send(msgBmeIAQA.set(IAQ_accuracy, 0));
    Serial.print("BME680 - zzzzZZZZzzzzZZZZzzzz\n");
    
  }
}
