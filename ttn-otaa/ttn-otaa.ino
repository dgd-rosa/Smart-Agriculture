/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "DHT.h"
#include "data.h"

#define DHTPIN A1 // pin for DTH11
#define DHTTYPE DHT11 // DHT 11
#define SoilPIN A0 //pin for SoilMoisture Sensor
#define MOTORPIN 6
DHT dht(DHTPIN, DHTTYPE);

boolean pumpState = false; //Pump state (ON or OFF)
int humidity = 0;
int temperature = 0;
int soil_moisture = 0;
int humidity_threshold = 60;
int temperature_threshold = 24;
int moisture_threshold = 300;

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]= {0x46, 0x01, 0x04, 0xD0, 0x7E, 0xD5, 0xB3, 0x70};
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]= {0x56, 0x34, 0x12, 0x90, 0x78, 0x56, 0x34, 0x12};
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0xC9, 0x70, 0xBB, 0xA3, 0xC5, 0x04, 0x79, 0x93, 0x54, 0x68, 0x29, 0x88, 0x7A, 0x83, 0xB9, 0x88 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

uint8_t mydata[5];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 10;


// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");

    byte downlinkData[2] = "";
    
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
              for (int i = 0; i < LMIC.dataLen; i++){
                if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
                  Serial.print(F("0"));
                }
                Serial.println(LMIC.frame[LMIC.dataBeg + i], HEX);

                downlinkData[i] = LMIC.frame[LMIC.dataBeg + i];
              }
              if(downlinkData[0] == "p"){
                if(downlinkData[1]==1){
                  pumpState = true;
                  Serial.println("Downlink: Turning On the Pump");
                } else if(downlinkData[1]==0){
                  pumpState = false;
                  Serial.println("Downlink: Turning On the Pump");
                }
              } else if (downlinkData[0] == "h"){
                Serial.print("Downlink: Changing Humidity Threshold: ");
                Serial.println(downlinkData[1]);
                humidity_threshold = downlinkData[1];
              } else if (downlinkData[0] == "t") {
                Serial.print("Downlink: Changing Temperature Threshold: ");
                Serial.println(downlinkData[1]);
                temperature_threshold = downlinkData[1];
              } else if (downlinkData[0] == "m") {
                Serial.print("Downlink: Changing Moisture Threshold: ");
                Serial.println(downlinkData[1]);
                moisture_threshold = downlinkData[1];
              }
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
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

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        //measurements of the environment
        measureSensor();
        uplinkMessageFormat();
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void uplinkMessageFormat(){
    soil_moisture = (unsigned int)soil_moisture;
    unsigned int mask = 0xff;
    mydata[0] = temperature;   
    mydata[1] = humidity;
    mydata[2] = soil_moisture & mask;
    mydata[3] = soil_moisture>>8;
    mydata[3] += pumpState<<7;
    Serial.println("Messages per byte: ");
    Serial.print(mydata[0]);Serial.print('\t');Serial.println(mydata[1]);
    Serial.print(mydata[2]);Serial.print('\t');Serial.println(mydata[3]);
}

void measureSensor(){
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();
  soil_moisture = analogRead(SoilPIN);
  // testa se retorno é valido, caso contrário algo está errado.
  if (isnan(temperature) || isnan(humidity)) 
  {
    Serial.println("Failed to read from DHT");
  } 
  else
  { 
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.print(" %\t");
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" ºC");
    Serial.print("%\tHumidity: ");
    Serial.print(humidity);
  }
}

void setup() {
    Serial.begin(115200);
    dht.begin();
    
    Serial.println(F("Starting"));

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {
    
    if(temperature > temperature_threshold || humidity < humidity_threshold || soil_moisture < moisture_threshold ){
      if(pumpState = false){
        pumpState = true;
        digitalWrite(MOTORPIN, HIGH);
      }
    } else{
      if (pumpState == true){
        pumpState = false;
        digitalWrite(MOTORPIN, LOW);
      }
    }
    os_runloop_once();
}
