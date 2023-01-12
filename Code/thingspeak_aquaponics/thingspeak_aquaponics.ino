#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DFRobot_ESP_EC.h>
#include "DFRobot_ESP_PH.h"
#include "DHT.h"
#include <WiFiManager.h>
#include "ThingSpeak.h"

#define TRIGGER_PIN 0

#define ONE_WIRE_BUS 25               

#define DHTPIN 32   
#define DHTTYPE DHT21  

#define TdsSensorPin A0
#define VREF 3.3             
#define SCOUNT  30           

#define PH_PIN 33         
#define ESPADC 4095.0     
#define ESPVOLTAGE 3300  

int analogBuffer[SCOUNT];    
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float voltage, averageVoltage ,phValue , tdsValue, ecValue, temperature = 25,temperature1;

DFRobot_ESP_PH ph;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DFRobot_ESP_EC ec;
DHT dht(DHTPIN, DHTTYPE);
WiFiClient  client;

//const char* ssid = "Duyhandsome";   
//const char* pass = "Khongbiet00";   

unsigned long myChannelNumber = 1988671;
const char * myWriteAPIKey = "8RHJ7F2DUDBG4MF0";

// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 30000;

int timeout = 120;

void setup() {
  Serial.begin(115200);  
  WiFiManager wm;
  EEPROM.begin(32);
  ec.begin();
  ph.begin();
  sensors.begin();
  dht.begin();
  pinMode(TdsSensorPin,INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  ThingSpeak.begin(client);  
  
  WiFiManager wn;
  //wm.resetSettings();
  bool res;
  //res = wm.autoConnect(); // auto generated AP name from chipid
  //res = wm.autoConnect("Aquaponics"); // anonymous ap
  res = wm.autoConnect("Aquaponics","123456789"); 

  if(!res) {
     Serial.println("Failed to connect...");
     digitalWrite(LED_BUILTIN, HIGH);
    // ESP.restart();
    } 
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);

     //if you get here you have connected to the WiFi    
   //  Serial.println("CONNECTED!!! :)");
   //  digitalWrite(LED_BUILTIN, LOW);
    
 
 // Serial.println("Connecting to ");
  //Serial.println(ssid);
  
 // WiFi.begin(ssid, pass);

  /*while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);*/
}

void loop() {
  
  voltage = analogRead(A0); // A0 is the gpio 36
  sensors.requestTemperatures();
  temperature1 = sensors.getTempCByIndex(0); 
  ecValue = ec.readEC(voltage, temperature); // convert voltage to EC with temperature compensation
  float t = dht.readTemperature();

  if ( digitalRead(TRIGGER_PIN) == LOW) {
    WiFiManager wm;    
    wm.resetSettings();
  
    // set configportal timeout
    wm.setConfigPortalTimeout(timeout);
 
    if (!wm.startConfigPortal("Aquaponics")) {
      Serial.println("failed to connect and hit timeout");
      delay(3000);
      //reset and try again, or maybe put it to deep sleep
      ESP.restart();
      delay(5000);
    }
  }
  if (isnan(t)) {
    Serial.println("Failed to read from DHT");
    delay(500);
  } else {
    Serial.print("  Nhiet do phong: "); 
    Serial.print(t);
    Serial.println("ºC");
    delay(500);
  }
  Serial.print("  Nhiet do nuoc :");
  Serial.print(temperature1, 2);
  Serial.println("ºC");
  delay(500);

  //tds sensor
   static unsigned long analogSampleTimepoint = millis();
   if(millis()-analogSampleTimepoint > 40U)     //every 40 milliseconds,read the analog value from the ADC
   {
     analogSampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT) 
         analogBufferIndex = 0;
   }   
   static unsigned long printTimepoint = millis();
   if(millis()-printTimepoint > 80U)
   {
      printTimepoint = millis();
      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
      analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 4095.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      float compensationCoefficient=1.0+0.02*(temperature-25.0);                     //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationVolatge=averageVoltage/compensationCoefficient;              //temperature compensation
      tdsValue=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value
      //Serial.print("voltage:");
      //Serial.print(averageVoltage,2);
      //Serial.print("V   ");
      Serial.print("  TDS:");
      Serial.print(tdsValue,0);
      Serial.println("ppm");
      delay(500);
   }
 
  Serial.print("  EC :");
  Serial.println(ecValue, 2);
  
  ec.calibration(voltage, temperature); // calibration process by Serail CMD

 //ph sensor
 static unsigned long timepoint = millis();
 if (millis() - timepoint > 1000U) //time interval: 1s
 {
  timepoint = millis();
  //voltage = rawPinValue / esp32ADC * esp32Vin
  voltage = analogRead(PH_PIN) / ESPADC * ESPVOLTAGE; // read the voltage
  //Serial.print("voltage:");
  //Serial.println(voltage, 4);

  phValue = ph.readPH(voltage, temperature); // convert voltage to pH with temperature compensation
  Serial.print("  pH :");
  Serial.println(phValue, 4);
  Serial.println("");
 }
    ThingSpeak.setField(1,temperature1);
    ThingSpeak.setField(2,ecValue);
    ThingSpeak.setField(3,tdsValue);
    ThingSpeak.setField(4,phValue);
    ThingSpeak.setField(5,t);
    // Write to ThingSpeak. There are up to 8 fields in a channel, allowing you to store up to 8 different
    // pieces of information in a channel.  Here, we write to field 1.
    int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);

    lastTime = millis();
  }

  int getMedianNum(int bArray[], int iFilterLen) 
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      bTab[i] = bArray[i];
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++) 
      {
      for (i = 0; i < iFilterLen - j - 1; i++) 
          {
        if (bTab[i] > bTab[i + 1]) 
            {
        bTemp = bTab[i];
            bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
         }
      }
      }
      if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
      else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}
