/*
 * ZivyObraz.eu - Orchestrate your ePaper displays
 *
 * You need to change some initial things like ePaper type etc. - see below.
 *
 * Default password for Wi-Fi AP is: zivyobraz
 *
 * Code based on both:
 * https://github.com/ZinggJM/GxEPD2/blob/master/examples/GxEPD2_WiFi_Example/GxEPD2_WiFi_Example.ino
 * and
 * https://github.com/LaskaKit/ESPink-42/blob/main/SW/Weatherstation_info/Weatherstation_info.ino
 *
 * Compile with board type: ESP32 dev module
 *
 * Libraries:
 * Analog read with calibration data: https://github.com/madhephaestus/ESP32AnalogRead/
 * EPD library: https://github.com/ZinggJM/GxEPD2
 * EPD library for 4G "Grayscale": https://github.com/ZinggJM/GxEPD2_4G
 * WiFi manager by tzapu https://github.com/tzapu/WiFiManager
 * QRCode generator: https://github.com/ricmoo/QRCode
 * SHT4x (temperature, humidity): https://github.com/adafruit/Adafruit_SHT4X
 * BME280 (temperature, humidity, pressure): https://github.com/adafruit/Adafruit_BME280_Library
 * SCD41 (CO2, temperature, humidity): https://github.com/sparkfun/SparkFun_SCD4x_Arduino_Library
 *
 * original code made by Jean-Marc Zingg and LaskaKit
 * modified by @MultiTricker (https://michalsevcik.eu)
 */

/////////////////////////////////
// Uncomment for correct board
/////////////////////////////////

//#define ESPink_V2
//#define ESPink_V3
//#define ESP32S3Adapter
//#define ES3ink
//#define MakerBadge_revB // also works with A and C
//#define MakerBadge_revD
//#define REMAP_SPI
//#define TTGO_T5_v23 // tested only with 2.13" variant

//////////////////////////////////////////////////////////////
// Uncomment if one of the sensors will be connected
// Supported sensors: SHT40/41/45, SCD40/41, BME280 
//////////////////////////////////////////////////////////////

//#define SENSOR

//////////////////////////////////////////////////////////////
// Uncomment correct color capability of your ePaper display
//////////////////////////////////////////////////////////////

//#define TYPE_BW // black and white
//#define TYPE_3C // 3 colors - black, white and red/yellow
//#define TYPE_4C // 4 colors - black, white, red and yellow
//#define TYPE_GRAYSCALE // grayscale - 4 colors
//#define TYPE_7C // 7 colors

///////////////////////////////////////////////
// That's all!
// Code of ZivyObraz follows
///////////////////////////////////////////////

////////////
// Board
////////////

// https://www.laskakit.cz/laskakit-espink-esp32-e-paper-pcb-antenna/?variantId=12419
// + LaskaKit ESPInk-42 all-in-one board

#ifdef ESPink_V2
  #define PIN_SS 5
  #define PIN_DC 17
  #define PIN_RST 16
  #define PIN_BUSY 4
  #define PIN_CS2 35
  #define ePaperPowerPin 2

#elif defined ESPink_V3
  #define PIN_SS 10
  #define PIN_DC 48
  #define PIN_RST 45
  #define PIN_BUSY 36
  #define PIN_CS2 35
  #define ePaperPowerPin 47
  #define PIN_SPI_MOSI 11
  #define PIN_SPI_CLK 12
  #define PIN_SDA 42
  #define PIN_SCL 2
  #define PIN_ALERT 9

#elif defined ESP32S3Adapter
  // With ESP32-S3 DEVKIT from laskakit.cz
  #define PIN_SS 10
  #define PIN_DC 41
  #define PIN_RST 40
  #define PIN_BUSY 13
  #define ePaperPowerPin 47
  #define enableBattery 9
  #define PIN_SPI_CLK 12
  #define PIN_SPI_MOSI 11

  #include <esp_adc_cal.h>
  #include <soc/adc_channel.h>

#elif defined ES3ink
  // for version P1.1
  #define PIN_SS 10
  #define PIN_DC 7
  #define PIN_RST 5
  #define PIN_BUSY 6
  #define PIN_CS2 35
  #define ePaperPowerPin 3
  #define enableBattery 40
  #define RGBledPin 48
  #define RGBledPowerPin 14

  #include <esp_adc_cal.h>
  #include <soc/adc_channel.h>

#elif defined MakerBadge_revB
  #define PIN_SS 41
  #define PIN_DC 40
  #define PIN_RST 39
  #define PIN_BUSY 42
  #define ePaperPowerPin 16

#elif defined MakerBadge_revD
  #define PIN_SS 41
  #define PIN_DC 40
  #define PIN_RST 39
  #define PIN_BUSY 42
  #define ePaperPowerPin 16
  #define enableBattery 14

#elif defined TTGO_T5_v23
  #define PIN_SS 5
  #define PIN_DC 17
  #define PIN_RST 16
  #define PIN_BUSY 4
  #define ePaperPowerPin 2

#elif defined EPDIY_V7
  #define PIN_SS 5
  #define PIN_DC 17
  #define PIN_RST 16
  #define PIN_BUSY 4
  #define ePaperPowerPin 2

#else
  #error "Board not defined!"
#endif

#include <FastEPD.h>
FASTEPD display;

////////////////////////////
// Library etc. includes
////////////////////////////

// M5Stack CoreInk
#ifdef M5StackCoreInk
  #include <M5CoreInk.h>
#endif

// WiFi
#include <WiFi.h>
#include <WiFiManager.h>

// ADC reading
#include <ESP32AnalogRead.h>

#include <QRCodeGenerator.h>
QRCode qrcode;

// TWI/I2C library
#include <Wire.h>

#ifdef ES3ink
  #include <Adafruit_NeoPixel.h>
  Adafruit_NeoPixel pixel(1, RGBledPin, NEO_GRB + NEO_KHZ800);
#endif

// Supported sensors
#ifdef SENSOR
  // SHT40/41/45
  #include "Adafruit_SHT4x.h"
Adafruit_SHT4x sht4 = Adafruit_SHT4x();

  // SCD40/41
  #include "SparkFun_SCD4x_Arduino_Library.h"
SCD4x SCD4(SCD4x_SENSOR_SCD41);

  // BME280
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BME280.h>
Adafruit_BME280 bme;
#endif

/* ---- ADC reading - indoor Battery voltage ---- */
#ifdef ES3ink
  #define vBatPin ADC1_GPIO2_CHANNEL
  #define dividerRatio 2.018

#elif defined M5StackCoreInk
  #define vBatPin 35

#elif defined MakerBadge_revB
  #define vBatPin 6
  #define BATT_V_CAL_SCALE 1.00

#elif defined MakerBadge_revD
  #define vBatPin 6
  #define BATT_V_CAL_SCALE 1.05

#elif defined TTGO_T5_v23
  #define vBatPin 35

#elif defined ESP32S3Adapter
  ESP32AnalogRead adc;
  #define vBatPin ADC1_GPIO9_CHANNEL  
  #define dividerRatio 2.018

#elif defined ESPink_V3
  #include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
  SFE_MAX1704X lipo(MAX1704X_MAX17048);

#else
  ESP32AnalogRead adc;
  #define vBatPin 34
  #define dividerRatio 1.769
#endif

/* ---- Server Zivy obraz ----------------------- */
const char *host = "cdn.zivyobraz.eu";
const char *firmware = "2.3";
const String wifiPassword = "zivyobraz";
const String urlWiki = "https://wiki.zivyobraz.eu ";

/* ---------- Deepsleep time in minutes --------- */
uint64_t defaultDeepSleepTime = 2; // if there is a problem with loading images,
                                   // this time will be used as fallback to try again soon
uint64_t deepSleepTime = defaultDeepSleepTime; // actual sleep time in minutes, value is changed
                                               // by what server suggest in response headers
/* ---------------------------------------------- */

/*-------------- ePaper resolution -------------- */
// Get display width from selected display class
#define DISPLAY_RESOLUTION_X 1600
#define DISPLAY_RESOLUTION_Y 1200

// #define DISPLAY_RESOLUTION_X display.epd2.WIDTH
// #define DISPLAY_RESOLUTION_Y display.epd2.HEIGHT
/* ---------------------------------------------- */

/* variables */
String ssid; // Wi-Fi ssid
int8_t rssi; // Wi-Fi signal strength
float d_volt; // indoor battery voltage
RTC_DATA_ATTR uint64_t timestamp = 0;
RTC_DATA_ATTR uint8_t notConnectedToAPCount = 0;
uint64_t timestampNow = 1; // initialize value for timestamp from server

void setEPaperPowerOn(bool on)
{
  // use HIGH/LOW notation for better readability
#if (defined ES3ink) || (defined MakerBadge_revD)
  digitalWrite(ePaperPowerPin, on ? LOW : HIGH);
#elif !defined M5StackCoreInk
  digitalWrite(ePaperPowerPin, on ? HIGH : LOW);
#endif
}

const String getWifiSSID()
{
  String wifiSSID = WiFi.SSID();
  Serial.println("Wifi SSID: " + wifiSSID);

  // Replace special characters
  wifiSSID.replace("%", "%25");
  wifiSSID.replace(" ", "%20");
  wifiSSID.replace("#", "%23");
  wifiSSID.replace("$", "%24");
  wifiSSID.replace("&", "%26");
  wifiSSID.replace("'", "%27");
  wifiSSID.replace("(", "%28");
  wifiSSID.replace(")", "%29");
  wifiSSID.replace("*", "%2A");
  wifiSSID.replace("+", "%2B");
  wifiSSID.replace(",", "%2C");
  wifiSSID.replace("/", "%2F");
  wifiSSID.replace(":", "%3A");
  wifiSSID.replace(";", "%3B");
  wifiSSID.replace("=", "%3D");
  wifiSSID.replace("?", "%3F");
  wifiSSID.replace("@", "%40");
  wifiSSID.replace("[", "%5B");
  wifiSSID.replace("]", "%5D");

  return wifiSSID;
}

int8_t getWifiStrength()
{
  int8_t rssi = WiFi.RSSI();
  Serial.println("Wifi Strength: " + String(rssi) + " dB");

  return rssi;
}

float getBatteryVoltage()
{
  float volt;

#if defined ESPink_V3

  Serial.println("Reading battery on ESPink V3 board");

  setEPaperPowerOn(true);
  pinMode(PIN_ALERT, INPUT_PULLUP);

  delay(100);

  Wire.begin (PIN_SDA, PIN_SCL);

  lipo.begin();

  //lipo.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  // Read and print the reset indicator
  Serial.print(F("Reset Indicator was: "));
  bool RI = lipo.isReset(true); // Read the RI flag and clear it automatically if it is set
  Serial.println(RI); // Print the RI
  // If RI was set, check it is now clear
  if (RI)
  {
    Serial.print(F("Reset Indicator is now: "));
    RI = lipo.isReset(); // Read the RI flag
    Serial.println(RI); // Print the RI    
  }

	lipo.setThreshold(1); // Set alert threshold to just 1% - we don't want to trigger the alert
  lipo.setVALRTMax((float)4.3); // Set high voltage threshold (Volts)
  lipo.setVALRTMin((float)2.9); // Set low voltage threshold (Volts)
  
  volt = (float)lipo.getVoltage();
  // percentage could be read like this:
  // lipo.getSOC();
  //Serial.println("Battery percentage: " + String(lipo.getSOC(), 2) + " %");

  lipo.clearAlert();
  lipo.enableHibernate();

  setEPaperPowerOn(false);

#elif defined ES3ink
  esp_adc_cal_characteristics_t adc_cal;
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 0, &adc_cal);
  adc1_config_channel_atten(vBatPin, ADC_ATTEN_DB_11);

  Serial.println("Reading battery on ES3ink board");

  digitalWrite(enableBattery, LOW);
  uint32_t raw = adc1_get_raw(vBatPin);
  //Serial.println(raw);
  uint32_t millivolts = esp_adc_cal_raw_to_voltage(raw, &adc_cal);
  //Serial.println(millivolts);
  const uint32_t upper_divider = 1000;
  const uint32_t lower_divider = 1000;
  volt = (float)(upper_divider + lower_divider) / lower_divider / 1000 * millivolts;
  digitalWrite(enableBattery, HIGH);

#elif defined ESP32S3Adapter
  Serial.println("Reading battery on ESP32-S3 DEVKIT board");
  // attach ADC input
  adc.attach(vBatPin);
  // battery voltage measurement
  volt = (float)(adc.readVoltage() * dividerRatio);

#elif defined M5StackCoreInk
  analogSetPinAttenuation(vBatPin, ADC_11db);
  esp_adc_cal_characteristics_t *adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 3600, adc_chars);
  uint16_t ADCValue = analogRead(vBatPin);

  uint32_t BatVolmV = esp_adc_cal_raw_to_voltage(ADCValue, adc_chars);
  volt = float(BatVolmV) * 25.1 / 5.1 / 1000;
  free(adc_chars);

#elif defined MakerBadge_revB
  volt = (BATT_V_CAL_SCALE * 2.0 * (2.50 * analogRead(vBatPin) / 8192));

#elif defined MakerBadge_revD
  // Borrowed from @Yourigh
  // Battery voltage reading
  // can be read right after High->Low transition of IO_BAT_meas_disable
  // Here, pin should not go LOW, so intentionally digitalWrite called as first.
  // First write output register (PORTx) then activate output direction (DDRx). Pin will go from highZ(sleep) to HIGH without LOW pulse.
  digitalWrite(enableBattery, HIGH);
  pinMode(enableBattery, OUTPUT);

  digitalWrite(enableBattery, LOW);
  delayMicroseconds(150);
  volt = (BATT_V_CAL_SCALE * 2.0 * (2.50 * analogRead(vBatPin) / 8192));
  digitalWrite(enableBattery, HIGH);

#elif defined TTGO_T5_v23
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize((adc_unit_t)ADC_UNIT_1, (adc_atten_t)ADC_ATTEN_DB_2_5, (adc_bits_width_t)ADC_WIDTH_BIT_12, 1100, &adc_chars);
  
  float measurement = (float) analogRead(vBatPin);
  volt = (float)(measurement / 4095.0) * 7.05;

#else
  // attach ADC input
  adc.attach(vBatPin);
  // battery voltage measurement
  volt = (float)(adc.readVoltage() * dividerRatio);
#endif

  Serial.println("Battery voltage: " + String(volt) + " V");

  return volt;
}

void drawQrCode(const char *qrStr, int qrSize, int yCord, int xCord, byte qrSizeMulti = 1)
{
  uint8_t qrcodeData[qrcode_getBufferSize(qrSize)];
  qrcode_initText(&qrcode, qrcodeData, qrSize, ECC_LOW, qrStr);

  int qrSizeTemp = (4 * qrSize) + 17;
  // QR Code Starting Point
  int offset_x = xCord - (qrSizeTemp * 2);
  int offset_y = yCord - (qrSizeTemp * 2);

  for (int y = 0; y < qrcode.size; y++)
  {
    for (int x = 0; x < qrcode.size; x++)
    {
      int newX = offset_x + (x * qrSizeMulti);
      int newY = offset_y + (y * qrSizeMulti);

      display.fillRect(newX, newY, qrSizeMulti, qrSizeMulti,
                       qrcode_getModule(&qrcode, x, y) ? 0x0 : 0xf);
    }
  }
}

void centeredText(const String &text, int xCord, int yCord)
{
  BBEPRECT rect;
  display.getStringBox(text.c_str(), &rect);
  display.drawString(text.c_str(), xCord - rect.w / 2, yCord);
}

// This is called if the WifiManager is in config mode (AP open)
// and draws information screen
void configModeCallback(WiFiManager *myWiFiManager)
{
  // Iterate notConnectedToAPCount counter
  if(notConnectedToAPCount < 255)
  {
    notConnectedToAPCount++;
  }

  /*
    QR code hint
    Common format: WIFI:S:<SSID>;T:<WEP|WPA|nopass>;P:<PASSWORD>;H:<true|false|blank>;;
    Sample: WIFI:S:MySSID;T:WPA;P:MyPassW0rd;;
  */
  const String hostname = WiFi.softAPSSID();
  const String qrString = "WIFI:S:" + hostname + ";T:WPA;P:" + wifiPassword + ";;";
  //Serial.println(qrString);

  const String urlWeb = "http://" + WiFi.softAPIP().toString();

  timestamp = 0; // set timestamp to 0 to force update because we changed screen to this info

  delay(500);

  display.clearWhite();
  display.setTextColor(BBEP_BLACK);
  // display.setFont(&OpenSansSB_24px);

  centeredText("No Wi-Fi configured OR connection lost", display.width() / 2, 28);
  centeredText("Retries in a few minutes if lost.", display.width() / 2, 64);
  centeredText("To setup or change Wi-Fi configuration", display.width() / 2, 120);
  centeredText("(with mobile data turned off):", display.width() / 2, 145);
  centeredText("1) Connect to this AP:", display.width() / 4, (display.height() / 2) - 50);
  centeredText("2) Open in web browser:", display.width() * 3 / 4, (display.height() / 2) - 50);

  drawQrCode(qrString.c_str(), 4, (display.height() / 2) + 40, display.width() / 4, 4);
  display.drawLine(display.width() / 2 - 1, (display.height() / 2) - 60, display.width() / 2 - 1, (display.height() / 2) + 170, 0x0);
  display.drawLine(display.width() / 2, (display.height() / 2) - 60, display.width() / 2, (display.height() / 2) + 170, 0x0);
  drawQrCode(urlWeb.c_str(), 4, (display.height() / 2) + 40, display.width() * 3 / 4, 4);

  centeredText("SSID: " + hostname, display.width() / 4, (display.height() / 2) + 130);
  centeredText("Password: " + wifiPassword, display.width() / 4, (display.height() / 2) + 155);
  centeredText(urlWeb, display.width() * 3 / 4, (display.height() / 2) + 130);

  display.fillRect(0, display.height() - 40, display.width(), 40, 0x0);
  display.setTextColor(0xf);
  centeredText("Documentation: " + urlWiki, display.width() / 2, display.height() - 22);


  display.fullUpdate();
}

void displayNoWiFiError()
{
  timestamp = 0; // set timestamp to 0 to force update because we changed screen to this info

  // displayInit();
  // setEPaperPowerOn(true);
  // delay(500);

  // display.setFullWindow();
  // display.firstPage();
  // do
  // {
  display.fillRect(0, 0, DISPLAY_RESOLUTION_X, DISPLAY_RESOLUTION_Y, 0xf);
  // display.setTextColor(0x0, 0xf);
  // display.setFont(&OpenSansSB_20px);
  centeredText("Cannot connect to Wi-Fi", DISPLAY_RESOLUTION_X / 2, DISPLAY_RESOLUTION_Y / 2 - 15);
  // display.setFont(&OpenSansSB_16px);
  centeredText("Retries in a " + String(deepSleepTime) + " minutes.", DISPLAY_RESOLUTION_X / 2, DISPLAY_RESOLUTION_Y / 2 + 15);
  // display.setFont(&OpenSansSB_14px);
  centeredText("Docs: " + urlWiki, DISPLAY_RESOLUTION_X / 2, DISPLAY_RESOLUTION_Y - 20);
  // } while (false);
  // } while (display.nextPage());

  // setEPaperPowerOn(false);
  display.fullUpdate();
}

void WiFiInit()
{
  // Connecting to WiFi
  Serial.println();
  Serial.print("Connecting... ");
  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  wm.setWiFiAutoReconnect(true);
  wm.setConnectRetries(5);
  wm.setDarkMode(true);
  wm.setConnectTimeout(5);
  wm.setSaveConnectTimeout(5);

  // Set network name to wi-fi mac address
  String hostname = "INK_";
  hostname += WiFi.macAddress();
  // Replace colon with nothing
  hostname.replace(":", "");

  // reset settings - wipe stored credentials for testing
  //wm.resetSettings();

  wm.setConfigPortalTimeout(240); // set portal time to 4 min, then sleep/try again.
  wm.setAPCallback(configModeCallback);
  wm.autoConnect(hostname.c_str(), wifiPassword.c_str());

  // Check if Wi-Fi is connected
  if (WiFi.status() == WL_CONNECTED)
  {
    // Reset counter    
    notConnectedToAPCount = 0;
  }
}

uint32_t read8n(WiFiClient &client, uint8_t *buffer, int32_t bytes)
{
  int32_t remain = bytes;
  uint32_t start = millis();
  while ((client.connected() || client.available()) && (remain > 0))
  {
    if (client.available())
    {
      int16_t v = (int16_t)client.read();
      if (buffer) *buffer++ = uint8_t(v);
      remain--;
    }
    else delay(1);
    if (millis() - start > 2000) break; // don't hang forever
  }
  return bytes - remain;
}

uint32_t skip(WiFiClient &client, int32_t bytes)
{
  return read8n(client, NULL, bytes);
}

// read one byte safely from WiFiClient, wait a while if data are not available immediately
uint8_t safe_read(WiFiClient &client)
{
  uint8_t ret;
  read8n(client, &ret, 1);
  return ret;
}

// read one byte safely from WiFiClient, wait a while if data are not available immediately
// if byte is not read, set valid to false
uint8_t safe_read_valid(WiFiClient &client,bool *valid)
{
  uint8_t ret;
  *valid=read8n(client, &ret, 1) == 1;  // signalize not valid reading when not all bytes are read
  return ret;
}

uint16_t read16(WiFiClient &client)
{
  // BMP data is stored little-endian, same as Arduino.
  uint16_t result;
  ((uint8_t *)&result)[0] = safe_read(client); // LSB
  ((uint8_t *)&result)[1] = safe_read(client); // MSB
  return result;
}

uint32_t read32(WiFiClient &client)
{
  // BMP data is stored little-endian, same as Arduino.
  uint32_t result;
  ((uint8_t *)&result)[0] = safe_read(client); // LSB
  ((uint8_t *)&result)[1] = safe_read(client);
  ((uint8_t *)&result)[2] = safe_read(client);
  ((uint8_t *)&result)[3] = safe_read(client); // MSB
  return result;
}

bool createHttpRequest(WiFiClient &client, bool &connStatus, bool checkTimestamp, const String &extraParams)
{
  // Make an url
  String url = "index.php?mac=" + WiFi.macAddress() +
               (checkTimestamp ? "&timestamp_check=1" : "") +
               "&rssi=" + String(rssi) +
               "&ssid=" + ssid +
               "&v=" + String(d_volt) +
               "&x=" + String(1600) +
               "&y=" + String(1200) +
              //  "&x=" + String(DISPLAY_RESOLUTION_X) +
              //  "&y=" + String(DISPLAY_RESOLUTION_Y) +
               "&c=" + String("BW") +
               "&fw=" + String(firmware) +
               "&ap_retries=" + String(notConnectedToAPCount) +
               extraParams;

  Serial.print("connecting to ");
  Serial.println(host);

  // Let's try twice
  for (uint8_t client_reconnect = 0; client_reconnect < 3; client_reconnect++)
  {
    if (!client.connect(host, 80))
    {
      Serial.println("connection failed");
      if (client_reconnect == 2)
      {
        deepSleepTime = defaultDeepSleepTime;
        if (!checkTimestamp) return false;
        delay(500);
      }
      if (!checkTimestamp) delay(200);
    }
  }

  Serial.print("requesting URL: ");
  Serial.println(String("http://") + host + "/" + url);
  client.print(String("GET ") + "/" + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "Connection: close\r\n\r\n");
  Serial.println("request sent");

  // Workaroud for timeout
  uint32_t timeout = millis();
  while (client.available() == 0)
  {
    if (millis() - timeout > 10000)
    {
      Serial.println(">>> Client Timeout !");
      client.stop();
      if (checkTimestamp) deepSleepTime = defaultDeepSleepTime;
      return false;
    }
  }

  bool gotTimestamp = false;

  while (client.connected())
  {
    String line = client.readStringUntil('\n');
    //Serial.println(line);

    if (checkTimestamp)
    {
      // If line starts with "Timestamp", put it into the timestamp variable
      if (line.startsWith("Timestamp"))
      {
        gotTimestamp = true;
        // Skipping also colon and space - also in following code for sleep, rotate, ...
        timestampNow = line.substring(11).toInt();
        Serial.print("Timestamp now: ");
        Serial.println(timestampNow);
      }

      // Let's try to get info about how long to go to deep sleep
      if (line.startsWith("Sleep"))
      {
        uint64_t sleep = line.substring(7).toInt();
        deepSleepTime = sleep;
        Serial.print("Sleep: ");
        Serial.println(sleep);
      }

      // Do we want to rotate display? (IE. upside down)
      if (line.startsWith("Rotate"))
      {
        uint8_t rotation = line.substring(8).toInt();
        display.setRotation(rotation);
        Serial.print("Rotate: ");
        Serial.println(rotation);
      }
    }

    if (!connStatus)
    {
      connStatus = line.startsWith("HTTP/1.1 200 OK");
      Serial.println(line);
    }
    if (line == "\r")
    {
      Serial.println("headers received");
      break;
    }
  }

  // Is there a problem? Fallback to default deep sleep time to try again soon
  if (!connStatus)
  {
    deepSleepTime = defaultDeepSleepTime;
    return false;
  }

  // For debug purposes - print out the whole response
  /*
  Serial.println("Byte by byte:");

  while (client.connected() || client.available()) {
    if (client.available()) {
      char c = client.read();  // Read one byte
      Serial.print(c);         // Print the byte to the serial monitor
    }
  }
  client.stop();
  /* */

  if (checkTimestamp)
  {
    if (gotTimestamp && (timestampNow == timestamp))
    {
      Serial.print("No screen reload, because we already are at current timestamp: ");
      Serial.println(timestamp);
      return false;
    }

    // Set timestamp to actual one
    timestamp = timestampNow;
  }

  return true;
}

#ifdef SENSOR
int readSensorsVal(float &sen_temp, int &sen_humi, int &sen_pres)
{
  // Check for SHT40 OR SHT41 OR SHT45
  if (sht4.begin())
  {
    Serial.println("SHT4x FOUND");
    sht4.setPrecision(SHT4X_LOW_PRECISION);
    sht4.setHeater(SHT4X_NO_HEATER);

    sensors_event_t hum, temp;
    sht4.getEvent(&hum, &temp);

    sen_temp = temp.temperature;
    sen_humi = hum.relative_humidity;
    return 1;
  }

  // Check for BME280
  if (bme.begin())
  {
    Serial.println("BME280 FOUND");

    sen_temp = bme.readTemperature();
    sen_humi = bme.readHumidity();
    sen_pres = bme.readPressure() / 100.0F;
    return 2;
  }

  // Check for SCD40 OR SCD41
  if (SCD4.begin(false, true, false))
  {
    Serial.println("SCD4x FOUND");
    SCD4.measureSingleShot();

    while (SCD4.readMeasurement() == false) // wait for a new data (approx 30s)
    {
      Serial.println("Waiting for first measurement...");
      delay(1000);
    }

    sen_temp = SCD4.getTemperature();
    sen_humi = SCD4.getHumidity();
    sen_pres = SCD4.getCO2();
    return 3;
  }

  return 0;
}
#endif

bool checkForNewTimestampOnServer(WiFiClient &client)
{
  bool connection_ok = false;
  String extraParams = "";

  // Measuring temperature and humidity?
#ifdef SENSOR
  #if (defined ESPink_V2) || (defined ESPink_V3)
  // LaskaKit ESPink 2.5 needs to power up uSup
  setEPaperPowerOn(true);
  delay(50);
  #endif

  float temperature;
  int humidity;
  int pressure;
  uint8_t sen_ret = readSensorsVal(temperature, humidity, pressure);

  if (sen_ret)
  {
    extraParams = "&temp=" + String(temperature) + "&hum=" + String(humidity);

    switch (sen_ret)
    {
      case 2:
        extraParams += "&pres=" + String(pressure); // BME280
        break;
      case 3:
        extraParams += "&co2=" + String(pressure); // SCD4x
        break;
    }
  }

  #if (defined ESPink_V2) || (defined ESPink_V3)
  // Power down for now
  setEPaperPowerOn(false);
  #endif
#endif

  return createHttpRequest(client, connection_ok, true, extraParams);
}

void print_error_reading(uint32_t bytes_read)
{
  // print error message when reading bitmap data from server failed
  Serial.print("Client got disconnected after bytes:");
  Serial.println(bytes_read);
}

void readBitmapData(WiFiClient &client)
{
  // Let's read bitmap
  static const uint16_t input_buffer_pixels = 800; // may affect performance
  static const uint16_t max_row_width = 1872; // for up to 7.8" display 1872x1404
  static const uint16_t max_palette_pixels = 256; // for depth <= 8

  // int16_t x = display.width() - DISPLAY_RESOLUTION_X;
  // int16_t y = display.height() - DISPLAY_RESOLUTION_Y;
  int16_t x = 0, y = 0;

  uint8_t input_buffer[3 * input_buffer_pixels]; // up to depth 24
  uint8_t output_row_mono_buffer[max_row_width / 8]; // buffer for at least one row of b/w bits
  uint8_t output_row_color_buffer[max_row_width / 8]; // buffer for at least one row of color bits
  uint8_t mono_palette_buffer[max_palette_pixels / 8]; // palette buffer for depth <= 8 b/w
  uint8_t color_palette_buffer[max_palette_pixels / 8]; // palette buffer for depth <= 8 c/w
  uint16_t rgb_palette_buffer[max_palette_pixels]; // palette buffer for depth <= 8 for buffered graphics, needed for 7-color display

  // Defaults - also for BW
  bool with_color = false;
  bool has_multicolors = false;
  bool grayscale = false;

  bool connection_ok = false;
  bool valid = false; // valid format to be handled
  bool flip = true; // bitmap is stored bottom-to-top

  bool whitish = false;
  bool lightgrey = false;
  bool darkgrey = false;
  bool colored = false;

  uint32_t startTime = millis();
  if ((x >= display.width()) || (y >= display.height())) return;
  if (!createHttpRequest(client, connection_ok, false, "")) return;

  // Parse header
  uint16_t header = read16(client);
  Serial.print("Header ");
  Serial.println(header, HEX);

  if (header == 0x4D42) // BMP signature
  {
    //#include <pgmspace.h>
    uint32_t fileSize = read32(client);
    uint32_t creatorBytes = read32(client);
    (void)creatorBytes; // unused
    uint32_t imageOffset = read32(client); // Start of image data
    uint32_t headerSize = read32(client);
    uint32_t width = read32(client);
    int32_t height = (int32_t)read32(client);
    uint16_t planes = read16(client);
    uint16_t depth = read16(client); // bits per pixel
    uint32_t format = read32(client);
    uint32_t bytes_read = 7 * 4 + 3 * 2; // read so far
    if ((planes == 1) && ((format == 0) || (format == 3))) // uncompressed is handled, 565 also
    {
      Serial.print("File size: ");
      Serial.println(fileSize);
      Serial.print("Image Offset: ");
      Serial.println(imageOffset);
      Serial.print("Header size: ");
      Serial.println(headerSize);
      Serial.print("Bit Depth: ");
      Serial.println(depth);
      Serial.print("Image size: ");
      Serial.print(width);
      Serial.print('x');
      Serial.println(height);
      // BMP rows are padded (if needed) to 4-byte boundary
      uint32_t rowSize = (width * depth / 8 + 3) & ~3;
      if (depth < 8) rowSize = ((width * depth + 8 - depth) / 8 + 3) & ~3;
      if (height < 0)
      {
        height = -height;
        flip = false;
      }
      uint16_t w = width;
      uint16_t h = height;
      if ((x + w - 1) >= display.width()) w = display.width() - x;
      if ((y + h - 1) >= display.height()) h = display.height() - y;

      //if (w <= max_row_width) // handle with direct drawing
      {
        valid = true;
        uint8_t bitmask = 0xFF;
        uint8_t bitshift = 8 - depth;
        uint16_t red, green, blue, rowToShow;
        whitish = false;
        lightgrey = false;
        darkgrey = false;
        colored = false;
        if (depth == 1) with_color = false;
        if (depth <= 8)
        {
          if (depth < 8) bitmask >>= depth;
          // bytes_read += skip(client, 54 - bytes_read); //palette is always @ 54
          bytes_read += skip(client, (int32_t)(imageOffset - (4 << depth) - bytes_read)); // 54 for regular, diff for colorsimportant
          for (uint16_t pn = 0; pn < (1 << depth); pn++)
          {
            blue = safe_read(client);
            green = safe_read(client);
            red = safe_read(client);
            skip(client, 1);
            bytes_read += 4;
            whitish = with_color ? ((red > 0x80) && (green > 0x80) && (blue > 0x80)) : ((red + green + blue) > 3 * 0x80); // whitish
            colored = (red > 0xF0) || ((green > 0xF0) && (blue > 0xF0)); // reddish or yellowish?
            if (0 == pn % 8)
            {
              mono_palette_buffer[pn / 8] = 0;
              color_palette_buffer[pn / 8] = 0;
            }
            mono_palette_buffer[pn / 8] |= whitish << pn % 8;
            color_palette_buffer[pn / 8] |= colored << pn % 8;
            //Serial.print("0x00"); Serial.print(red, HEX); Serial.print(green, HEX); Serial.print(blue, HEX);
            //Serial.print(" : "); Serial.print(whitish); Serial.print(", "); Serial.println(colored);
            rgb_palette_buffer[pn] = ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | ((blue & 0xF8) >> 3);
          }
        }

        uint32_t rowPosition = flip ? imageOffset + (height - h) * rowSize : imageOffset;
        //Serial.print("skip "); Serial.println(rowPosition - bytes_read);
        bytes_read += skip(client, (int32_t)(rowPosition - bytes_read));

        for (uint16_t row = 0; row < h; row++, rowPosition += rowSize) // for each line
        {
          if (!connection_ok || !(client.connected() || client.available())) break;
          delay(1); // yield() to avoid WDT
          uint32_t in_remain = rowSize;
          uint32_t in_idx = 0;
          uint32_t in_bytes = 0;
          uint8_t in_byte = 0; // for depth <= 8
          uint8_t in_bits = 0; // for depth <= 8
          uint16_t color = 0xf;
          for (uint16_t col = 0; col < w; col++) // for each pixel
          {
            yield();
            if (!connection_ok || !(client.connected() || client.available())) break;
            // Time to read more pixel data?
            if (in_idx >= in_bytes) // ok, exact match for 24bit also (size IS multiple of 3)
            {
              uint32_t get = min(in_remain, sizeof(input_buffer));
              uint32_t got = read8n(client, input_buffer, (int32_t)get);
              while ((got < get) && connection_ok)
              {
                //Serial.print("got "); Serial.print(got); Serial.print(" < "); Serial.print(get); Serial.print(" @ "); Serial.println(bytes_read);
                uint32_t gotmore = read8n(client, input_buffer + got, (int32_t)(get - got));
                got += gotmore;
                connection_ok = gotmore > 0;
              }
              in_bytes = got;
              in_remain -= got;
              bytes_read += got;
            }
            if (!connection_ok)
            {
              Serial.print("Error: got no more after ");
              Serial.print(bytes_read);
              Serial.println(" bytes read!");
              break;
            }

            whitish = false;
            lightgrey = false;
            darkgrey = false;
            colored = false;
            switch (depth)
            {
              case 32:
              case 24:
                blue = input_buffer[in_idx++];
                green = input_buffer[in_idx++];
                red = input_buffer[in_idx++];
                if (depth == 32) in_idx++; // skip alpha
                whitish = with_color ? ((red > 0x80) && (green > 0x80) && (blue > 0x80)) : ((red + green + blue) > 3 * 0x80); // whitish
                lightgrey = with_color ? ((red > 0x60) && (green > 0x60) && (blue > 0x60)) : ((red + green + blue) > 3 * 0x60); // lightgrey
                darkgrey = with_color ? ((red > 0x40) && (green > 0x40) && (blue > 0x40)) : ((red + green + blue) > 3 * 0x40); // darkgrey
                colored = (red > 0xF0) || ((green > 0xF0) && (blue > 0xF0)); // reddish or yellowish?
                color = ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | ((blue & 0xF8) >> 3);
                break;
              case 16:
              {
                uint8_t lsb = input_buffer[in_idx++];
                uint8_t msb = input_buffer[in_idx++];
                if (format == 0) // 555
                {
                  blue = (lsb & 0x1F) << 3;
                  green = ((msb & 0x03) << 6) | ((lsb & 0xE0) >> 2);
                  red = (msb & 0x7C) << 1;
                  color = ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | ((blue & 0xF8) >> 3);
                }
                else // 565
                {
                  blue = (lsb & 0x1F) << 3;
                  green = ((msb & 0x07) << 5) | ((lsb & 0xE0) >> 3);
                  red = (msb & 0xF8);
                  color = (msb << 8) | lsb;
                }
                whitish = with_color ? ((red > 0x80) && (green > 0x80) && (blue > 0x80)) : ((red + green + blue) > 3 * 0x80); // whitish
                lightgrey = with_color ? ((red > 0x60) && (green > 0x60) && (blue > 0x60)) : ((red + green + blue) > 3 * 0x60); // lightgrey
                darkgrey = with_color ? ((red > 0x40) && (green > 0x40) && (blue > 0x40)) : ((red + green + blue) > 3 * 0x40); // darkgrey
                colored = (red > 0xF0) || ((green > 0xF0) && (blue > 0xF0)); // reddish or yellowish?
              }
              break;
              case 1:
              case 2:
              case 4:
              case 8:
              {
                if (0 == in_bits)
                {
                  in_byte = input_buffer[in_idx++];
                  in_bits = 8;
                }
                uint16_t pn = (in_byte >> bitshift) & bitmask;
                whitish = mono_palette_buffer[pn / 8] & (0x1 << pn % 8);
                colored = color_palette_buffer[pn / 8] & (0x1 << pn % 8);
                in_byte <<= depth;
                in_bits -= depth;
                color = rgb_palette_buffer[pn];

                if (grayscale)
                {
                  switch (pn)
                  {
                    case 1:
                      lightgrey = true;
                      break;
                    case 2:
                    case 3:
                      darkgrey = true;
                      break;
                    case 4:
                      whitish = true;
                      break;
                  }
                }
              }
              break;
            }
            if (with_color && has_multicolors)
            {
              // keep color
            }
            else if (whitish)
            {
              color = 0xf;
            }
            else if (grayscale && lightgrey)
            {
              color = 0x9;
            }
            else if (grayscale && darkgrey)
            {
              color = 0x6;
            }
            else if (colored && with_color)
            {
              color = 0x10;
            }
            else
            {
              color = 0x0;
            }

            uint16_t yrow = y + (flip ? h - row - 1 : row);
            display.drawPixelFast(x + col, yrow, color);  // DRAWING HERE
            Serial.println(color);
          } // end col
        } // end row
        display.fullUpdate();
      } // end block
      Serial.print("bytes read ");
      Serial.println(bytes_read);
    }
  }
  else if (header == 0x315A || header == 0x325A || header == 0x335A) // ZivyObraz RLE data Z1 or Z3
  {
    // Z1 - 1 byte for color, 1 byte for number of repetition
    // Z2 - 2 bits for color, 6 bits for number of repetition
    // Z3 - 3 bits for color, 5 bits for number of repetition
    if (header == 0x315A) Serial.println("Got format Z1, processing");
    else if (header == 0x325A) Serial.println("Got format Z2, processing");
    else Serial.println("Got format Z3, processing");

    uint32_t bytes_read = 2; // read so far
    uint16_t w = display.width();
    uint16_t h = display.height();
    uint8_t pixel_color, count, compressed;
    uint16_t color = 0x1;
    valid = true;

    uint16_t color2 = 0x10;
    uint16_t color3 = 0x4;

#if (defined TYPE_BW) || (defined TYPE_GRAYSCALE)
    color2 = 0x10;
    color3 = 0x4;
#endif
    display.fillScreen(0xf);
    for (uint16_t row = 0; row < h; row++) // for each line
    {
      if (!connection_ok || !(client.connected() || client.available())) break;

      for (uint16_t col = 0; col < w; col++) // for each pixel
      {
        yield();

        if (!connection_ok)
        {
          Serial.print("Error: got no more after ");
          Serial.print(bytes_read);
          Serial.println(" bytes read!");
          break;
        }

        if (!(client.connected() || client.available()))
        {
          print_error_reading(bytes_read);
          break;
        }

        // Z1
        if (header == 0x315A)
        {
          pixel_color = safe_read_valid(client,&valid);
          if (!valid)
          {
            print_error_reading(bytes_read);
            break;
          }
          count = safe_read_valid(client,&valid);
          if (!valid)
          {
            print_error_reading(bytes_read);
            break;
          }
          bytes_read += 2;
        }
        else if (header == 0x325A)
        {
          // Z2
          compressed = safe_read_valid(client,&valid);
          if (!valid)
          {
            print_error_reading(bytes_read);
            break;
          }
          count = compressed & 0b00111111;
          pixel_color = (compressed & 0b11000000) >> 6;
          bytes_read++;
        }
        else if (header == 0x335A)
        {
          // Z3
          compressed = safe_read_valid(client,&valid);
          if (!valid)
          {
            print_error_reading(bytes_read);
            break;
          }
          count = compressed & 0b00011111;
          pixel_color = (compressed & 0b11100000) >> 5;
          bytes_read++;
        }

        // color picker (for Z2 only and 4BP mode)
        switch (pixel_color) {
          case 0:  // white
            color = 0xf;
            break;
          case 1:  // black
            color = 0x0;
            break;
          case 2:  // light gray
            color = 0x9;
            break;
          case 3:  // dark gray
            color = 0x6;
            break;
        }

        // if (color != 0)
        // {
        //   Serial.print("count: "); Serial.print(count); Serial.print(" pixel: "); Serial.println(color);
        // }

        for (uint8_t i = 0; i < count - 1; i++)
        {
          yield();

          display.drawPixelFast(col, row, color);
          if ((count > 1) && (++col == w))
          {
            col = 0;
            row++;
          }
        }
        display.drawPixelFast(col, row, color);
      } // end col
    } // end row
    display.fullUpdate();

    Serial.print("bytes read ");
    Serial.println(bytes_read);
  }

  Serial.print("loaded in ");
  Serial.print(millis() - startTime);
  Serial.println(" ms");

  client.stop();
  if (!valid)
  {
    Serial.print("Format not handled, got: ");
    Serial.println(header);
    deepSleepTime = defaultDeepSleepTime;
    timestamp = 0;
  }
}

void testDraw()
{
  for (int row = 0; row < 1200; row++) {
    for (int col = 0; col < 1200; col++) {
      if (abs(row - col) < 50) {
        display.drawPixelFast(row, col, 0x0);
      }
    }
  }
}

void setup()
{
  display.initPanel(BB_PANEL_EPDIY_V7);
  display.setPanelSize(1600, 1200);
  display.setMode(BB_MODE_4BPP);

  // display.drawString("ahoj", 100, 100);
  // display.fullUpdate();

  Serial.begin(115200);
  Serial.println("Starting firmware for Zivy Obraz service");
  
  // Battery voltage measurement
  // d_volt = getBatteryVoltage();

  // Wifi init
  WiFiInit();


  // WiFi strength - so you will know how good your signal is
  rssi = getWifiStrength();

  // WiFi SSID - get connected ssid
  ssid = getWifiSSID();

  // Use WiFiClient class to create TCP connections
  WiFiClient client;

  // Successfully connected to Wi-Fi?
  if(notConnectedToAPCount == 0)
  {
    // Do we need to update the screen?
    if (checkForNewTimestampOnServer(client))
    {
      // Enable power supply for ePaper
      // setEPaperPowerOn(true);
      delay(500);

      // Get that lovely bitmap and put it on your gorgeous grayscale ePaper screen!

      timestamp = 0;
      readBitmapData(client);

      delay(100);
    }
  }
  else
  {
    Serial.println("No Wi-Fi connection, will sleep for a while and try again. Failure no.: " + String(notConnectedToAPCount));

    // Determine how long we will sleep determined by number of notConnectedToAPCount
    if(notConnectedToAPCount <= 3) deepSleepTime = defaultDeepSleepTime;
    else if(notConnectedToAPCount <= 10) deepSleepTime = 10;
    else if(notConnectedToAPCount <= 20) deepSleepTime = 30;
    else if(notConnectedToAPCount <= 50) deepSleepTime = 60;
    else deepSleepTime = 720;

    // Enable power supply for ePaper
    delay(500);

    // Display error message
    displayNoWiFiError();

    delay(100);
    // Disable power supply for ePaper
  }

  // Deep sleep mode
  Serial.print("Going to sleep now for (minutes): ");
  Serial.println(deepSleepTime);
  esp_sleep_enable_timer_wakeup(deepSleepTime * 5 * 1000000);  // 5 seconds
  delay(100);
  esp_deep_sleep_start();
}

void loop()
{
  Serial.println("test");
  delay(1000);
}

// 0x0 -> black
// 0xf -> white
