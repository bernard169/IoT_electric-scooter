#include <Adafruit_GPS.h>
#include <TinyLoRa.h>
#include <SPI.h>
#define VBATPIN A7


uint32_t timer = millis();

//----------- LORA ---------------------\\

// Network Session Key (MSB)
uint8_t NwkSkey[16] = { 0x48, 0xE8, 0x59, 0xF2, 0xBE, 0x0E, 0x22, 0xD5, 0x14, 0x9B, 0xA9, 0x3D, 0xD6, 0xB0, 0x43, 0x5E };

// Application Session Key (MSB)
uint8_t AppSkey[16] = { 0xCE, 0xDD, 0x91, 0xA8, 0x71, 0x47, 0x21, 0xB8, 0x03, 0xCF, 0x6B, 0xD5, 0xAF, 0x4A, 0x57, 0x9D };

// Device Address (MSB)
uint8_t DevAddr[4] = { 0x26,0x01,0x13,0x4D };

/************************** Example Begins Here ***********************************/
// Data Packet to Send to TTN

// How many times data transfer should occur, in seconds
const unsigned int sendInterval = 30;

// Pinout for Adafruit Feather M0 LoRa
TinyLoRa lora = TinyLoRa(3, 8, 4);

union cvt{
  float number;
  byte bytelist[4];
}floatAsBytes;

// ------------ GPS ---------------\\

#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);


  
void setup()
{
  delay(2000);
  Serial.begin(115200);
  GPS.begin(9600);
  //while (! Serial);

    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  
 
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
  
  //------------- PREPARING BATTERY VOLTAGE -----------\\
  
  //unsigned char payload;
  uint8_t mydata[9]; 
  readBattery(mydata);  //&payload); we don't put the & because mydata is already a pointer
 
  // mydata[0] = payload; //Put Voltage on payload[0]      The value is already in mydata[0]




  //------------ PREPARING GPS COORDINATES ------------\\
  
  if(!(int)GPS.fix){       // Enough Satellites for GPS position
    
    floatAsBytes.number = GPS.longitude; 
    if(GPS.lon == 'O')floatAsBytes.number*=-1;   // longitude is OUEST so negative 
    
    Serial.print("Longitude : ");
    Serial.println(floatAsBytes.number);
  
    mydata[1]=floatAsBytes.bytelist[0];
    mydata[2]=floatAsBytes.bytelist[1];
    mydata[3]=floatAsBytes.bytelist[2];
    mydata[4]=floatAsBytes.bytelist[3];
  
    floatAsBytes.number = GPS.latitude; 
    if(GPS.lon == 'S')floatAsBytes.number*=-1;   // latitude is SUD so negative
  
    Serial.print("Latitude : ");
    Serial.println(floatAsBytes.number);
    
    mydata[5]=floatAsBytes.bytelist[0];
    mydata[6]=floatAsBytes.bytelist[1];
    mydata[7]=floatAsBytes.bytelist[2];
    mydata[8]=floatAsBytes.bytelist[3];
    
  }else{    // No satellites

    floatAsBytes.number =0;
    mydata[1]=floatAsBytes.bytelist[0];
    mydata[2]=floatAsBytes.bytelist[1];
    mydata[3]=floatAsBytes.bytelist[2];
    mydata[4]=floatAsBytes.bytelist[3];
    floatAsBytes.number =0;
    mydata[5]=floatAsBytes.bytelist[0];
    mydata[6]=floatAsBytes.bytelist[1];
    mydata[7]=floatAsBytes.bytelist[2];
    mydata[8]=floatAsBytes.bytelist[3];
  }

  

//--------------- SENDING ------------------\\

  Serial.println("Sending LoRa Data...");
  lora.sendData(mydata, sizeof(mydata), lora.frameCounter);
  Serial.print("Frame Counter: ");Serial.println(lora.frameCounter);
  lora.frameCounter++;

  // blink LED to indicate packet sent
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);


// ------------- DELYING ---------------\\
  Serial.println("delaying...");
  while(millis()-timer<(sendInterval*1000)){
    char c = GPS.read();
    if (GPS.newNMEAreceived()) {
      Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
      if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
        return; // we can fail to parse a sentence in which case we should just wait for another
    }
  }
  timer=millis();
  //delay(sendInterval * 1000);

  
}



void readBattery(unsigned char *value) {
    // Payload format Bytes: [(Bat-Voltage - 2) * 100]
    double measuredvbat = analogRead(VBATPIN);
    measuredvbat /= 4;    // we divided by 2, so multiply back
    Serial.print("VBat: " ); Serial.println(measuredvbat);
    value[0] = (unsigned char)measuredvbat; //we're unsigned
}
