/*
 * Mitsubishi PAR-WT50R-E Home Assistant Sensor
 * 
 * This code was written to work for:
 * - Adafruit Feather ESP32-S3
 * - Adafruit RFM69HCW Featherwing (868MHz)
 * 
 * This sketch uses the packet mode of the radio.
 *
 * TODO:
 * - More details on changes made on the controller
 * - Find out what mode 5 is
 * - Do everything with bitshifting and bitmasks for endianness compatibility 
 * - Use interrupts & multicore (if available)
 */ 

#include <SPI.h>
#include <WiFi.h>
#include <Arduino_JSON.h>

// Adafruit RFM69HCW Breakout / FeatherWing
#define RFM69HCW_G0_IRQ   5   // E - PayloadReady/CrcOK
#define RFM69HCW_CS       6   // D - SPI Chip Select
#define RFM69HCW_RST      9   // C - Reset
#define RFM69HCW_G2_DIO2  11  // A - Data

#define SSID "myaccesspoint"
#define PASS "trashpassword"

// Home Assistant Details
#define HOST "192.168.1.xxx"
#define PORT 8123
#define AUTH "abcdefghijklmnopqrstuvwxyz.ABCDEFGHIJKLMNOPQRSTUVWXYZ.0123456789.abcdefghijklmnopqrstuvwxyz.ABCDEFGHIJKLMNOPQRSTUVWXYZ.0123456789.abcdefghijklmnopqrstuvwxyz.ABCDEFGHIJKLMNOPQRSTUVWXYZ"

#define HA_0_TYPE SENSOR
#define HA_0_ID "heating"
#define HA_0_UID "0"
#define HA_0_DEV "mode"
#define HA_0_UNIT ""
#define HA_0_NAME "Heating Mode"

#define HA_1_TYPE SENSOR
#define HA_1_ID "heating"
#define HA_1_UID "1"
#define HA_1_DEV "thermostat"
#define HA_1_UNIT "\u00b0C"
#define HA_1_NAME "Heating Thermostat"

// Base ID for the heating
#define BASEID 0xFF

// 867.8MHz, B/W: 12.5kHz, Deviation: 4.8KHz, 9600kbps
#define BASEFREQ      867.8032
#define FREQDEV       4800
#define BITRATE       9600
#define BANDWIDTH     DCCFREQ_4 << 5 | RXBWMANT_20 << 3 | RXBWEXP_5
#define AFC_BANDWIDTH DCCFREQ_4 << 5 | RXBWMANT_20 << 3 | RXBWEXP_5

// RFM69HCW clock speed - do not change
#define FXOSC 32000000

// Bandwidth Macro Enumeration
enum { DCCFREQ_16, DCCFREQ_8, DCCFREQ_4, DCCFREQ_2, DCCFREQ_1, DCCFREQ_0_5, DCCFREQ_0_25, DCCFREQ_0_125 };
enum { RXBWMANT_16, RXBWMANT_20, RXBWMANT_24 };
enum { RXBWEXP_0, RXBWEXP_1, RXBWEXP_2,  RXBWEXP_3,  RXBWEXP_4, RXBWEXP_5,  RXBWEXP_6,  RXBWEXP_7 };

const char* ssid = SSID;
const char* pass = PASS;

const char* host = HOST;
const uint16_t port = PORT;
const char* auth = AUTH;

enum { BINARY, SENSOR };

const char modes[7][11] = {
  "Off",
  "Hot Water",
  "Heating",
  "Mode 3",
  "Mode 4",
  "Mode 5", // Unknown - came on for 3 hours one once, small power draw
  "Legionella"
};

struct payload { 
  union {
    uint8_t rfm_output[32];
    struct {
      uint8_t len;      // Size of message, always 0x1C
      uint8_t addr;     // 0x8B: node, 0x0B broadcast
      // Custom header
      uint8_t netmsb;   // Unique Network?
      uint8_t netlsb;   // Unique Network?  
      uint8_t dst;      // Dest: 0=RX, 1-9=RC
      uint8_t src;      // Src: 0=RX, 1-9=RC
      uint16_t unk;     // Unknown - Status? Pairing?
      // Message
      uint8_t msg[20];  // Encapsulated message
      uint8_t crc;      // Internal 8-bit CRC of msg
      uint8_t pad[3];   // Padding to go from 29 to 32 bytes
    }__attribute__((packed));
  };
};

struct payload msg;

uint8_t spi_cmd(uint8_t addr, uint8_t value) {
  uint8_t status;

  digitalWrite(RFM69HCW_CS, LOW);
  SPI.transfer(addr);
  status = SPI.transfer(value);
  digitalWrite(RFM69HCW_CS, HIGH);
  
  return status;
}

// Set the wnr bit to write (Section 5.2.1) 
uint8_t spi_write(uint8_t addr, uint8_t value) {
  return spi_cmd(addr | 0x80, value);
}

// Pass nothing when reading
uint8_t spi_read(uint8_t addr) {
  return spi_cmd(addr, 0x00);
}

void rfminit() {
    // Setup pins other than SPI, VDD and ground
  pinMode(RFM69HCW_CS, OUTPUT);
  pinMode(RFM69HCW_RST, OUTPUT);
  pinMode(RFM69HCW_G2_DIO2, INPUT);

  // Section 5.2.1 - SPI Interface
  digitalWrite(RFM69HCW_CS, HIGH);
  
  // Section 7.2.2 - Manual Reset
  digitalWrite(RFM69HCW_RST, HIGH);
  delayMicroseconds(100);
  digitalWrite(RFM69HCW_RST, LOW);
  // Spec says 5ms, but takes longer
  delay(10);

  // Fire up the SPI
  SPI.begin();
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  
  // Read the version register, if it's not 0x24, something is wrong
  if(spi_read(0x10) == 0x24)
    Serial.println("RFM69HCW: Initialized");
  
  // RegOpMode: Standby Mode & wait
  spi_write(0x01, 0x04);
  while(!(spi_read(0x27)) & 0x80);
  
  // Packet, FSK. No Shaping 
  spi_write(0x02, 0x00);

  // Packet: Set DIO0 as PayloadReady
  spi_write(0x25, 0x40);
  
  // Packet: Syncword detection on 2 bytes
  spi_write(0x2E, 0x88);
  
  // Packet: Syncword define = 0x2DD4
  spi_write(0x2F, 0x2D);
  spi_write(0x30, 0xD4);
  
  // Packet: Variable length, check yet ignore CRC, Address filtering
  spi_write(0x37, 0x9C);

  // Packet: Max length = 64 bytes + 2 CRC
  spi_write(0x38, 0x42);
  
  // Packet: Node Address byte = 0x8B
  spi_write(0x39, 0x8B);
  
  // Packet: Broadcast Address byte = 0x0B
  spi_write(0x3A, 0x0B);
   
  // RegBitrate*: Bitrate needed for Bit Synchronizer
  uint16_t bitrate = FXOSC / BITRATE;
  spi_write(0x03, (bitrate >> 8) & 0xFF);
  spi_write(0x04, bitrate & 0xFF);
 
  /*
   * Essential FSK Settings:
   * - 0x05-6 RegFdev*:      Set Frequency Deviation      
   * - 0x0B   RegAfcCtrl:    Improved AFC routine 
   * - 0x1E   RegAfcFei:     AFC on on Rx start
   * - 0x29   RegRssiThresh: RSSI Trigger @ -72dB
   * - 0x6F   RegTestDagc:   Fade improvement
   */
  uint16_t freqdev = FREQDEV / (FXOSC / 524288);
  spi_write(0x05, (freqdev >> 8) & 0x3F);
  spi_write(0x06, freqdev & 0xFF);

  // RegFrf*: Frequency across 3 bytes  
  uint32_t freqrf = (BASEFREQ * 1000000) / (FXOSC / 524288);
  spi_write(0x07, (freqrf >> 16) & 0xFF);
  spi_write(0x08, (freqrf >> 8) & 0xFF);
  spi_write(0x09, freqrf & 0xFF);

  // RegRxBw: Tuned bandwidth (narrow)
  spi_write(0x19, BANDWIDTH);

  // RSSI Threshold
  spi_write(0x29, 75 * 2);
 
  // AFC bandwidth (wider)
  spi_write(0x1A, AFC_BANDWIDTH);
  // AFC On - At Start
  spi_write(0x1E, 0x04);
  // Use AFC Improved
  spi_write(0x0B, 0x20);
  // DAGC: FM ModIdx < 2 = 0x20, FM ModIdx > 2 = 0x20
  spi_write(0x6F, 0x20);
  
  // RegOpMode: Receiving mode, auto sequencer disabled
  spi_write(0x01, 0x10);
  while(!(spi_read(0x27)) & 0x80);

  // Close down SPI - redundant if FIFO not used, and needed again for the TFT
  SPI.endTransaction();

  Serial.println("RFM69HCW: Configured");
}

void sendPayload(bool type, String endpoint, String uid, String dev, String unit, String name, String state) {
  // We can't use HTTPClient as it might clash with other WiFi libraries
  WiFiClient client;

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
    }
  }
  /* 
   *  Home Assistant HTTP Sensor JSON format
   * https://www.home-assistant.io/integrations/http
   * https://developers.home-assistant.io/docs/api/rest/#post-apistatesentity_id
   */
  String url;
  if (type == BINARY) {
    url = "/api/states/binary_sensor." + endpoint + "_" + uid + "_" + dev;
  } else {
    url = "/api/states/sensor." + endpoint + "_" + uid + "_" + dev;
  }
  
  // Build out the JSON payload
  JSONVar payload;
  payload["state"] = state;
  payload["attributes"]["device_class"] = dev;
  payload["attributes"]["friendly_name"] = name;
  if (unit.length() > 0) {
    payload["attributes"]["unit_of_measurement"] = unit;
  }
  String data = JSON.stringify(payload);

  // HTTP POST
  String httpreq = F("POST ");
  httpreq += url;
  httpreq += F(" HTTP/1.1\r\nHost: ");
  httpreq += host;
  httpreq += ":";
  httpreq += String(port);
  httpreq += F("\r\nAuthorization: Bearer ");
  httpreq += String(auth);
  httpreq += F("\r\nAccept-Encoding: identity;q=1,chunked;q=0.1,*;q=0");
  httpreq += F("\r\nConnection: close");
  httpreq += F("\r\nContent-Type: application/json");
  httpreq += F("\r\nContent-Length: ");
  httpreq += String(data.length());
  httpreq += F("\r\n\r\n");
  httpreq += data;
  
  // Try to connect
  if (!client.connect(host, port)) {
    Serial.print(F("connection failed to "));
    Serial.println(String(host));
    delay(5000);
    return;
  }
  
  // Try to send the data
  if (client.connected()) {
    client.println(httpreq);
    Serial.println(F("POST Success"));
  } else {
    Serial.println(F("POST Failed"));
  }

  // Clean up and close out the connection
  client.flush();
  client.stop();
}

void payloadProcess() {
  String state;
  
  if ((msg.netlsb == (BASEID - msg.src))) {
    switch (msg.msg[0]) {
        
      // RC Status (the ambient temperature)
      case 0x43:
        sendPayload(HA_1_TYPE, HA_1_ID, String(BASEID, HEX), HA_1_DEV, HA_1_UNIT, HA_1_NAME, String((msg.msg[5] & 0x7F) * 0.5));
        break;

      // RX Status (the state of the boiler)
      case 0x63:
        // Check to see if it's a known mode
        if (msg.msg[6] < 7) {
          state = modes[msg.msg[6]];
        } else {
          state = "Mode" + String(msg.msg[6]);
        }
        sendPayload(HA_0_TYPE, HA_0_ID, String(BASEID, HEX), HA_0_DEV, HA_0_UNIT, HA_0_NAME, String(state));
        break;

      default:
        break;
    }
  }
}

bool payloadCollect() {
  // If there's data in the FIFO
  if (spi_read(0x28) & 0x04) {
    int x=0;

    // Scrape FIFO 
    while(spi_read(0x28) & 0x40)
      msg.rfm_output[x++] = spi_read(0x00);

    return 1;
  }
  return 0;
}

void setup() {
  // Core 0 setup - RFM receiver & Serial
  Serial.begin(115200);
  delay(5000);

  // Configure the RFM69HCW receiver via SPI
  rfminit();

  // Wireless
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  Serial.println(F("Connected"));

  // Attach an interrupt to flag when there's a payload 
  // attachInterrupt(PIN_DIO0, payloadReady, RISING);

  // Do this entirely via SPI, rather than interrupts (TODO)
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
}

void loop() {
  if (payloadCollect()) {
    payloadProcess();
  }
}
