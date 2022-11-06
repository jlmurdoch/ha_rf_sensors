/*
 * Lidl Auriol Weather Station Home Assistant Sensor
 * This code was written to work specifically with:
 * - Adafruit Feather ESP32-S3 TFT
 * - Adafruit RFM69HCW Featherwing 433MHz
 * - Adafruit DPS310 Pressure Sensor (via I2C)
 */
#include <SPI.h>
#include <WiFi.h>
#include <Arduino_JSON.h>

// If we want to use the TFT to show the readings
#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_TFT)
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h> 
#endif

#include <Adafruit_DPS310.h>

// Adafruit RFM69HCW Breakout / FeatherWing
#define RFM69HCW_G0_IRQ   5   // E - PayloadReady/CrcOK
#define RFM69HCW_CS       6   // D - SPI Chip Select
#define RFM69HCW_RST      9   // C - Reset
#define RFM69HCW_G2_DIO2  11  // A - Data

// Wireless
#define SSID "myaccesspoint"
#define PASS "trashpassword"

// Home Assistant Details
#define HOST "192.168.1.xxx"
#define PORT 8123
#define AUTH "abcdefghijklmnopqrstuvwxyz.ABCDEFGHIJKLMNOPQRSTUVWXYZ.0123456789.abcdefghijklmnopqrstuvwxyz.ABCDEFGHIJKLMNOPQRSTUVWXYZ.0123456789.abcdefghijklmnopqrstuvwxyz.ABCDEFGHIJKLMNOPQRSTUVWXYZ"

#define HA_0_TYPE SENSOR
#define HA_0_ID "climate"
#define HA_0_UID "0"
#define HA_0_DEV "temperature"
#define HA_0_UNIT "\u00b0C"
#define HA_0_NAME "Temperature"
#define HA_0_STATS "measurement"

#define HA_1_TYPE SENSOR
#define HA_1_ID "climate"
#define HA_1_UID "1"
#define HA_1_DEV "humidity"
#define HA_1_UNIT "%"
#define HA_1_NAME "Humidity"
#define HA_1_STATS "measurement"

#define HA_2_TYPE BINARY
#define HA_2_ID "climate"
#define HA_2_UID "2"
#define HA_2_DEV "battery"
#define HA_2_UNIT ""
#define HA_2_NAME "Battery Power"
#define HA_2_STATS ""

#define HA_3_TYPE SENSOR
#define HA_3_ID "climate"
#define HA_3_UID "3"
#define HA_3_DEV "pressure"
#define HA_3_UNIT "hPa"
#define HA_3_NAME "Pressure"
#define HA_3_STATS "measurement"

// Thermometer - Varies +/-83kHz (B/W: 166MHz)
#define BASEFREQ      433.73
#define BITRATE       2000
#define BANDWIDTH     DCCFREQ_4 << 5 | RXBWMANT_24 << 3 | RXBWEXP_0
#define AFC_BANDWIDTH DCCFREQ_4 << 5 | RXBWMANT_24 << 3 | RXBWEXP_0

#define PULSE_LEAD  0x1 // HIGH of 500ms (488-544)
#define PULSE_SHORT 0x3 // LOW of 1000ms (980-1012)
#define PULSE_LONG  0x7 // LOW of 2000ms (1956-1996)
#define BIT_COUNT   36 

// RFM69HCW clock speed - do not change
#define FXOSC 32000000

// TFT setup
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// DPS Sensor
Adafruit_DPS310 dps;

// RFM Bandwidth Macro Enumeration
enum { DCCFREQ_16, DCCFREQ_8, DCCFREQ_4, DCCFREQ_2, DCCFREQ_1, DCCFREQ_0_5, DCCFREQ_0_25, DCCFREQ_0_125 };
enum { RXBWMANT_16, RXBWMANT_20, RXBWMANT_24 };
enum { RXBWEXP_0, RXBWEXP_1, RXBWEXP_2,  RXBWEXP_3,  RXBWEXP_4, RXBWEXP_5,  RXBWEXP_6,  RXBWEXP_7 };

// Home Assistant sensor type
enum { BINARY, SENSOR };

// Initialise the variables used
uint8_t thisValue, lastValue, thisGap, pulses = 0;
uint32_t thisTime, lastTime = 0;
uint64_t thisdata, lastdata = 0;

// WiFi connection details
const char* ssid = SSID;
const char* pass = PASS;

// Home Assistant connection details
const char* host = HOST;
const uint16_t port = PORT;
const char* auth = AUTH;

// Keep a stateful copy of the readings, so we can refresh the TFT
struct tft_values {
  uint8_t active;
  uint8_t id;
  float temperature;
  uint8_t humidity;
  float pressure;
};
struct tft_values readings[4];

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
  
  // Continuous, OOK
  spi_write(0x02, 0x48);
   
  // RegBitrate*: Bitrate needed for Bit Synchronizer
  uint16_t bitrate = FXOSC / BITRATE;
  spi_write(0x03, (bitrate >> 8) & 0xFF);
  spi_write(0x04, bitrate & 0xFF);

  // RegFrf*: Frequency across 3 bytes  
  uint32_t freqrf = (BASEFREQ * 1000000) / (FXOSC / 524288);
  spi_write(0x07, (freqrf >> 16) & 0xFF);
  spi_write(0x08, (freqrf >> 8) & 0xFF);
  spi_write(0x09, freqrf & 0xFF);

  // 50 Ohm LNA
  spi_write(0x18, 0x88);

  // RegRxBw: Tuned bandwidth (narrow)
  spi_write(0x19, BANDWIDTH);
 
  // AFC bandwidth (wider)
  spi_write(0x1A, AFC_BANDWIDTH);

  // Use OOK Peak
  spi_write(0x1B, 0x40);
  
  // Size between pulses in -dB. Valid range for the sensor: 1 to 99dB
  spi_write(0x1D, 1);

  // Fast tail off with the DAGC
  spi_write(0x6F, 0x30);

  // RegOpMode: Receiving mode, auto sequencer disabled
  spi_write(0x01, 0x10);
  while(!(spi_read(0x27)) & 0x80);

  // Close down SPI - redundant if FIFO not used, and needed again for the TFT
  SPI.endTransaction();

  Serial.println("RFM69HCW: Configured");
}

void sendPayload(bool type, String endpoint, String uid, String dev, String unit, String name, String stats, String state) {
  // We can't use HTTPClient as it might clash with other WiFi libraries
  WiFiClient client;

  // Check to see if the Wifi is still alive
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
    }
  }

  /* 
   *  Home Assistant HTTP Sensor 
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
  if (stats.length() > 0) {
    payload["attributes"]["state_class"] = stats;
  }
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
    // Serial.println(F("POST Success"));
  } else {
    Serial.println(F("POST Failed"));
  }
  
  // Clean up and close out the connection
  client.flush();
  client.stop();
}

#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_TFT)
void tftDisplay() {
  // Clear screen
  tft.fillScreen(0x0000);
  tft.setCursor(0, 10);

  // Print the temperature 
  tft.setTextColor(0xF800); // Red
  tft.print((float)readings[0].temperature, 2);
  // Appears to be codepage 437 to print a degree symbol
  tft.print("\xF7""C ");

  // Print the humidity      
  tft.setTextColor(0x001F); // Blue
  tft.print(readings[0].pressure, 2);
  tft.println(" hPa");

  for (uint8_t i=1; i<4; i++) {
    if (readings[i].active) {
      // Print the channel
      tft.setCursor(0, 10 + (i*30));
      tft.setTextColor(0xFFFF);
      tft.print(i);

      // Print the sensor ID
      tft.setTextColor(0xFFE0); //Yellow
      tft.print("[");
      tft.print(readings[i].id);
      tft.print("] ");

      // Print the temperature 
      tft.setTextColor(0xF800); // Red
      tft.print((float)readings[i].temperature, 1);
      // Appears to be codepage 437 to print a degree symbol
      tft.print("\xF7""C");

      // Break
      tft.setTextColor(0x07E0); // Green
      tft.print(" / ");

      // Print the humidity      
      tft.setTextColor(0x001F); // Blue
      tft.print(readings[i].humidity);
      tft.println("%");
    }
  }
}

void tftsetup() {
  // Initialise the ST7789 - 240x135
  // 7=TFT_CS, 21=TFT_I2C_POWER, 39=TFT_DC, 40=TFT_RST, 45=TFT_BACKLITE
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);

  // Turn on the TFT / I2C power
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(10);

  // Initialize TFT
  tft.init(135, 240); 
  tft.setRotation(3);
  tft.setTextWrap(false);

  // Clear screen, print chunky holding text
  tft.fillScreen(0x0000);
  tft.setCursor(30, 55);
  tft.setTextColor(0xFFFF);
  tft.setTextSize(3);
  tft.println("Waiting...");

  // Use the smaller font size going forward
  tft.setTextSize(2);
}
#endif

void processData() {
  // ensure two consecutive packets exactly the same
  if (thisdata == lastdata) {
    // Avoid using bitpacking due to endianness
    uint8_t humidity = thisdata & 0x7F;
    float temperature = (((int16_t)((thisdata >> 8) & 0xFFF0)) >> 4) * 0.1;
    uint8_t channel = ((thisdata >> 24) & 0x3) + 1;
    uint8_t manual = (thisdata >> 26) & 0x1;
    uint8_t battery = (thisdata >> 27) & 0x1; 
    uint8_t id = (thisdata >> 28) & 0xFF;

    // Push the data
    sendPayload(HA_0_TYPE, HA_0_ID, String(id), HA_0_DEV, HA_0_UNIT, HA_0_NAME, HA_0_STATS, String(temperature, 1));
    sendPayload(HA_1_TYPE, HA_1_ID, String(id), HA_1_DEV, HA_1_UNIT, HA_1_NAME, HA_1_STATS, String(humidity));
    sendPayload(HA_2_TYPE, HA_2_ID, String(id), HA_2_DEV, HA_2_UNIT, HA_2_NAME, HA_2_STATS, battery ? "off" : "on");

    if (dps.temperatureAvailable() && dps.pressureAvailable()) {
      sensors_event_t temp_event, pressure_event;
      dps.getEvents(&temp_event, &pressure_event);
      sendPayload(HA_0_TYPE, HA_0_ID, String("0"), HA_0_DEV, HA_0_UNIT, HA_0_NAME, HA_0_STATS, String(temp_event.temperature, 2));
      sendPayload(HA_3_TYPE, HA_3_ID, String("0"), HA_3_DEV, HA_3_UNIT, HA_3_NAME, HA_3_STATS, String(pressure_event.pressure, 2));
      readings[0].active = 1;
      readings[0].id = id;
      readings[0].temperature = temp_event.temperature;
      readings[0].pressure = pressure_event.pressure;      
    }  

            
#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_TFT)
    // This is for recording / updating the data in a stateful way
    readings[channel].active = 1;
    readings[channel].id = id;
    readings[channel].temperature = temperature;
    readings[channel].humidity = humidity;
    // Now print the data
    tftDisplay();
#endif
    
    // stop duplicates
    delay(500);
    lastdata = 0;
  //  
  } else if (((thisdata >> 7) & 0x1F) == 0x1E) {
    // First observation of something that looks good... wait for a second
    lastdata = thisdata;
  }
}

void setup() {
  // Serial console: e.g. cu -s 115200 -l /dev/ttyACM0
  Serial.begin(115200);
  delay(5000);

  // Set up the RFM69HCW via SPI
  rfminit();

  // Set up the WiFi
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_TFT)
  // Set up the TFT Screen if we have one
  tftsetup();
#endif
  dps.begin_I2C();
  dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
  dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
}

void loop() {
  // Check the G2 / DIO2 pin for a change
  thisValue = digitalRead(RFM69HCW_G2_DIO2);

  // If there is a data change, process it
  if (thisValue != lastValue) {
    // Record the time this has changed
    thisTime = micros();
    // Round-down the ms value to 256ms chunks and recenter
    thisGap = (thisTime - lastTime - 128) >> 8;
    
    // Check the size of the pulse we have received
    switch (thisGap) {
      // Valid Data to be processed
      case PULSE_SHORT: // 1000 ms = 0
      case PULSE_LONG: // 2000 ms = 1
        // Make room for a new bit
        thisdata <<= 1;
        // Set the bit to the what the gap size represents
        thisdata |= (!(~thisGap & 0x04));
        // Increment the counter
        pulses++;
        // Wait for next bit
        break;
    
      // Valid pre-bit pulse that can be safely skipped
      case PULSE_LEAD:
        if (~thisValue) {
          break;
        }

      // Reset if we don't see a valid pulse
      default:
        thisdata = 0;
        pulses = 0;
    }
    // If we have enough data, work with it and reset
    if (pulses == BIT_COUNT) {
      processData();
      thisdata = 0;
      pulses = 0; 
    }
    // Record the major data for the next pass
    lastValue = thisValue;
    lastTime = thisTime;      
  }
}
