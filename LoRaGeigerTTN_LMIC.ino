//LoraGeigerTTN Geiger Counter ported to LMIC for HopeRF modem and Octupus controller by Fab-Lab.eu  
#include <lmic.h>
#include <hal/hal.h>
#include <Adafruit_BME680.h>
#include <Wire.h>

#define MAXCNT 100 
#define CalFactor 1 

volatile int counter = 0; 
unsigned long oldTime = 0; 

// LoraWAN Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
// https://github.com/matthijskooijman/arduino-lmic
// -------- LoRa PinMapping FeatherWing Octopus
const lmic_pinmap lmic_pins = {  
  .nss = 2,                            // Connected to pin D
  .rxtx = LMIC_UNUSED_PIN,             // For placeholder only, Do not connected on RFM92/RFM95
  .rst = LMIC_UNUSED_PIN,              // Needed on RFM92/RFM95? (probably not) D0/GPIO16 
  .dio = {
    15, 15, LMIC_UNUSED_PIN         }
};

static const u1_t PROGMEM DEVEUI[8]={
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //  <—— your DEVEUI
void os_getDevEui (u1_t* buf) { 
  memcpy_P(buf, DEVEUI, 8);
}

static const u1_t PROGMEM APPEUI[8]={
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};  // <—— your APPEUI
void os_getArtEui (u1_t* buf) { 
  memcpy_P(buf, APPEUI, 8);
}

static const u1_t PROGMEM APPKEY[16]={
  0x09,0xB6,0x6D,0xCE,0xDA,0x48,0xE9,0x0B,0x4F,0x65,0x0C,0xDC,0xDC,0x5D,0x50,0xC8};
void os_getDevKey (u1_t* buf) {  
  memcpy_P(buf, APPKEY, 16);
};

volatile int LoRaWAN_Tx_Ready      = 0; // Merker für ACK 

int LoRaWAN_Rx_Payload = 0 ;
// -------- LoRa Event 
void onEvent (ev_t ev) { 
  Serial.print(os_getTime());
  Serial.print(": ");
  switch(ev) {
  case EV_SCAN_TIMEOUT:
    Serial.println(F("EV_SCAN_TIMEOUT"));
    break;
  case EV_BEACON_FOUND:
    Serial.println(F("EV_BEACON_FOUND"));
    break;
  case EV_BEACON_MISSED:
    Serial.println(F("EV_BEACON_MISSED"));
    break;
  case EV_BEACON_TRACKED:
    Serial.println(F("EV_BEACON_TRACKED"));
    break;
  case EV_JOINING:
    Serial.println(F("EV_JOINING"));
    break;
  case EV_JOINED:
    Serial.println(F("EV_JOINED"));
    // Disable link check validation (automatically enabled
    // during join, but not supported by TTN at this time).
    LMIC_setLinkCheckMode(0);
    break;
  case EV_RFU1:
    Serial.println(F("EV_RFU1"));
    break;
  case EV_JOIN_FAILED:
    Serial.println(F("EV_JOIN_FAILED"));
    break;
  case EV_REJOIN_FAILED:
    Serial.println(F("EV_REJOIN_FAILED"));
    break;
    break;
  case EV_TXCOMPLETE:
    Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    if (LMIC.txrxFlags & TXRX_ACK)
      Serial.println(F("Received ack"));
    if (LMIC.dataLen) {
      Serial.println(F("Received "));
      Serial.println(LMIC.dataLen);
      Serial.println(F(" bytes of payload"));
      LoRaWAN_Rx_Payload = 0; 
      for (int i = 0;i<LMIC.dataLen;i++) { 
        Serial.println(LMIC.frame[i+ LMIC.dataBeg],HEX);
        LoRaWAN_Rx_Payload = 256*LoRaWAN_Rx_Payload+LMIC.frame[i+ LMIC.dataBeg];
      }
    }
    LoRaWAN_Tx_Ready = 1;
    // Schedule next transmission
    //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
    break;
  case EV_LOST_TSYNC:
    Serial.println(F("EV_LOST_TSYNC"));
    break;
  case EV_RESET:
    Serial.println(F("EV_RESET"));
    break;
  case EV_RXCOMPLETE:
    // data received in ping slot
    Serial.println(F("EV_RXCOMPLETE"));
    break;
  case EV_LINK_DEAD:
    Serial.println(F("EV_LINK_DEAD"));
    break;
  case EV_LINK_ALIVE:
    Serial.println(F("EV_LINK_ALIVE"));
    break;
  default:
    Serial.println(F("Unknown event"));
    break;
  }
}

// BME680 Lib written by Limor Fried & Kevin Townsend for Adafruit Industries, http://www.adafruit.com/products/3660
Adafruit_BME680 boschBME680; // Objekt Bosch Umweltsensor

void setup(){ // Einmalige Initialisierung
  Serial.begin(115200);
  // -- Initialisiere LoraWAN 
  os_init();             // LMIC LoraWAN
  LMIC_reset();          // Reset the MAC state 
  LMIC.txpow = 27;       // Maximum TX power 
  LMIC.datarate=DR_SF12; // Long Range
  LMIC.rps = updr2rps(LMIC.datarate);

  Wire.begin(); // ---- Initialisiere den I2C-Bus 
  if (Wire.status() != I2C_OK) Serial.println("Something wrong with I2C");

  if (!boschBME680.begin(118)) { 
    Serial.println("Failed to communicate BME680");
    while (1) {
      delay(1);
    };
  }

  // Set up Bosch BME 680
  boschBME680.setTemperatureOversampling(BME680_OS_8X);
  boschBME680.setHumidityOversampling(BME680_OS_2X);
  boschBME680.setPressureOversampling(BME680_OS_4X);
  boschBME680.setIIRFilterSize(BME680_FILTER_SIZE_3);
  boschBME680.setGasHeater(320, 150); // 320*C for 150 ms

  
  attachInterrupt(digitalPinToInterrupt(0), count, FALLING); 
  Serial.println("");
  Serial.println("Start waiting for geiger pulses ...");

}

void loop() { // Kontinuierliche Wiederholung 

  unsigned long time; 
  unsigned long dt; 
  float rate; 
  int err;

  if (counter == MAXCNT) { 
    detachInterrupt(digitalPinToInterrupt(0)); 
    time = millis(); 
    dt = time-oldTime; 
    rate = (float)MAXCNT*60.0*1000.0/(float)dt/CalFactor; 
    Serial.println(round(rate));

 
  { //Block------------------------------ sende Daten an TTN  
    int port = 10;
    static uint8_t mydata[4];
    int wert=round(rate*10);
    mydata[0] = wert >> 8; 
    mydata[1] = wert & 0xFF;
    wert=round(boschBME680.readTemperature()*10);
    mydata[2] = wert >> 8; 
    mydata[3] = wert & 0xFF;
    // Check if there is not a current TX/RX job running
    //if (LMIC.opmode & OP_TXRXPEND) {
    if (LMIC.opmode & (1 << 7)) { 
      Serial.println(F("OP_TXRXPEND, not sending"));
    } 
    else {
      // Prepare upstream data transmission at the next possible time.
      LoRaWAN_Tx_Ready = 0;                                 // Merker für ACK
      LMIC_setTxData2(port, mydata, sizeof(mydata), 0);     // Sende         
      Serial.println(F("Packet queued"));
      while(LoRaWAN_Tx_Ready==0) {
        yield();
        os_runloop_once();
      };  // Warte bis gesendet
    }
  } // Blockende
  
    
    Serial.println("waiting 60 seconds");
    delay(60000);
    
    oldTime = millis(); 
    counter = 0;     
    attachInterrupt(digitalPinToInterrupt(0), count, FALLING);   
  }


}

void count() 
{ 
  counter++; 
}
