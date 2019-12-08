// TinyLoRa DHT22 - ABP TTN Packet Sender (Multi-Channel)
// Tutorial Link: https://learn.adafruit.com/the-things-network-for-feather/using-a-feather-32u4
//
// Adafruit invests time and resources providing this open source code.
// Please support Adafruit and open source hardware by purchasing
// products from Adafruit!
//
// Copyright 2015, 2016 Ideetron B.V.
//
// Modified by Brent Rubell for Adafruit Industries, 2018
/************************** Configuration ***********************************/
#include <TinyLoRa.h>
#include <SPI.h>
#define VBATPIN A7
// Visit your thethingsnetwork.org device console
// to create an account, or if you need your session keys.

// Network Session Key (MSB)
uint8_t NwkSkey[16] = { 0x48, 0xE8, 0x59, 0xF2, 0xBE, 0x0E, 0x22, 0xD5, 0x14, 0x9B, 0xA9, 0x3D, 0xD6, 0xB0, 0x43, 0x5E };

// Application Session Key (MSB)
uint8_t AppSkey[16] = { 0xCE, 0xDD, 0x91, 0xA8, 0x71, 0x47, 0x21, 0xB8, 0x03, 0xCF, 0x6B, 0xD5, 0xAF, 0x4A, 0x57, 0x9D };

// Device Address (MSB)
uint8_t DevAddr[4] = { 0x26,0x01,0x13,0x4D };

/************************** Example Begins Here ***********************************/
// Data Packet to Send to TTN
unsigned char loraData[10] = "HELLO GUY";
// How many times data transfer should occur, in seconds
const unsigned int sendInterval = 30;

// Pinout for Adafruit Feather 32u4 LoRa
//TinyLoRa lora = TinyLoRa(7, 8, 4);

// Pinout for Adafruit Feather M0 LoRa
TinyLoRa lora = TinyLoRa(3, 8, 4);

// pin the DHT22 is connected to

union cvt{
  float number;
  byte bytelist[4];
  }floatAsBytes;
void setup()
{
  delay(2000);
  Serial.begin(9600);
  //while (! Serial);
 
  // Initialize pin LED_BUILTIN as an output
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize LoRa
  Serial.print("Starting LoRa...");
  // define multi-channel sending
  lora.setChannel(MULTI);
  // set datarate
  lora.setDatarate(SF7BW125);
  if(!lora.begin())
  {
    Serial.println("Failed");
    Serial.println("Check your radio");
    while(true);
  }
  Serial.println("OK");
}

void loop()
{
  unsigned char payload;
  uint8_t mydata[9]; // SEND COORDINATES
  readValues(&payload);
 
  mydata[0] = payload; //Put Voltage on payload[0]
  
  floatAsBytes.number = 50.4560; //longitude
  
  Serial.print("Bytes of longitude : ");
  Serial.print(floatAsBytes.bytelist[0]);


  mydata[1]=floatAsBytes.bytelist[0];
  mydata[2]=floatAsBytes.bytelist[1];
  mydata[3]=floatAsBytes.bytelist[2];
  mydata[4]=floatAsBytes.bytelist[3];

  floatAsBytes.number = 15.4560; //lattitude
  
  mydata[5]=floatAsBytes.bytelist[0];
  mydata[6]=floatAsBytes.bytelist[1];
  mydata[7]=floatAsBytes.bytelist[2];
  mydata[8]=floatAsBytes.bytelist[3];
  


  /*
  
  for (int i =0 ; i++ ; i<=3){
    mydata[i+1] = floatAsBytes.bytelist[i];
    Serial.print(floatAsBytes.bytelist[i]);
  } 
  Serial.println("");
  
  floatAsBytes.number = 15.4560; //lattitude
  for (int i = 0 ; i++ ; i<=3){
    mydata[i+5] = (unsigned char)floatAsBytes.bytelist[i];
  }*/

  Serial.println("Sending LoRa Data...");
  lora.sendData(mydata, sizeof(mydata), lora.frameCounter);
  Serial.print("Frame Counter: ");Serial.println(lora.frameCounter);
  lora.frameCounter++;

  // blink LED to indicate packet sent
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.println("delaying...");
  delay(sendInterval * 1000);
}
void readValues(unsigned char *value) {
    // Payload format Bytes: [(Bat-Voltage - 2) * 100]
    double measuredvbat = analogRead(VBATPIN);
    measuredvbat /= 4;    // we divided by 2, so multiply back
    Serial.print("VBat: " ); Serial.println(measuredvbat);
    value[0] = (unsigned char)measuredvbat; //we're unsigned
}
