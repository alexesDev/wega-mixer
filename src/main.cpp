#include <Arduino.h>

///////////////////////////////////////////////////////////////////
// main code - don't change if you don't know what you are doing //
///////////////////////////////////////////////////////////////////
const char FW_version[] PROGMEM = "2.1.5";

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP8266HTTPClient.h>
#include <Wire.h>

#include <HX711.h>
#include <Adafruit_MCP23017.h>

#include <config.h>

#ifdef SSD1306
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#else
#include <LiquidCrystal_I2C.h>
#endif

class Display {
  #ifdef SSD1306
  Adafruit_SSD1306 lcd = Adafruit_SSD1306(128, 32, &Wire);
  #else
  LiquidCrystal_I2C lcd(0x27, 16, 2); // Check I2C address of LCD, normally 0x27 or 0x3F // SDA = D1, SCL = D2
  #endif

  public:
  void init();
  void setCursor(int x, int y);
  void print(const __FlashStringHelper *text);
  void print(const Printable &text);
  void print(const char *text);
  void print(char ch);
  void printf(const char *format, ...);
  void printf_P(PGM_P format, ...);
  void clear();
  void display();
};

void Display::init() {
  #ifdef SSD1306
  lcd.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  lcd.setTextSize(1);
  lcd.setTextColor(SSD1306_WHITE);
  clear();
  #else
  lcd.init(); 
  lcd.backlight();
  #endif
}

void Display::display() {
  #ifdef SSD1306
  lcd.display();
  #endif
}

void Display::setCursor(int x, int y) {
  #ifdef SSD1306
  lcd.setCursor(x, y * 8);
  #else
  lcd.setCursor(x, y);
  #endif
}

void Display::print(const __FlashStringHelper *text) {
  lcd.print(text);
  display();
}

void Display::print(const char *text) {
  lcd.print(text);
  display();
}

void Display::print(const Printable &text) {
  lcd.print(text);
  display();
}

void Display::print(char ch) {
  lcd.print(ch);
  display();
}

void Display::clear() {
  #ifdef SSD1306
  lcd.clearDisplay();
  #else
  lcd.clear();
  #endif
}

void Display::printf(const char *format, ...) {
  va_list argptr;
  va_start(argptr, format);
  lcd.printf(format, argptr);
  va_end(argptr);
  display();
}

void Display::printf_P(PGM_P format, ...) {
  va_list argptr;
  va_start(argptr, format);
  lcd.printf_P(format, argptr);
  va_end(argptr);
  display();
}

// Assign ports names
// Here is the naming convention:
// A0=0 .. A7=7
// B0=8 .. B7=15
// That means if you have A0 == 0 B0 == 8 (this is how it's on the board B0/A0 == 8/0)
// one more example B3/A3 == 11/3
#define A0 0
#define A1 1
#define A2 2
#define A3 3
#define A4 4
#define A5 5
#define A6 6
#define A7 7
#define B0 8
#define B1 9
#define B2 10
#define B3 11
#define B4 12
#define B5 13
#define B6 14
#define B7 15

class Kalman { // https://github.com/denyssene/SimpleKalmanFilter
  private:
  float err_measure, err_estimate, q, last_estimate;  

  public:
  Kalman(float _err_measure, float _err_estimate, float _q) {
      err_measure = _err_measure;
      err_estimate = _err_estimate;
      q = _q;
  }

  float updateEstimation(float measurement) {
    float gain = err_estimate / (err_estimate + err_measure);
    float current_estimate = last_estimate + gain * (measurement - last_estimate);
    err_estimate =  (1.0 - gain) * err_estimate + fabs(last_estimate - current_estimate) * q;
    last_estimate = current_estimate;
    return last_estimate;
  }
  float getEstimation() {
    return last_estimate;
  }
};

Kalman displayFilter = Kalman(1400, 80, 0.15); // плавный
Kalman filter        = Kalman(1000, 80, 0.4);  // резкий

#define PUMPS_NO 8
const char* names[PUMPS_NO]        = {pump1n, pump2n, pump3n, pump4n, pump5n, pump6n, pump7n, pump8n};
const byte pinForward[PUMPS_NO]    = {pump1,  pump2,  pump3,  pump4,  pump5,  pump6,  pump7,  pump8};
const byte pinReverse[PUMPS_NO]    = {pump1r, pump2r, pump3r, pump4r, pump5r, pump6r, pump7r, pump8r};
const long staticPreload[PUMPS_NO] = {pump1p, pump2p, pump3p, pump4p, pump5p, pump6p, pump7p, pump8p};
const char* stateStr[]             = {"Ready", "Working", "Busy"};
enum State {STATE_READY, STATE_WORKING, STATE_BUSY};


float goal[PUMPS_NO];
float curvol[PUMPS_NO];
float sumA, sumB;
byte pumpWorking = -1;
unsigned long sTime, eTime;


Adafruit_MCP23017 mcp;
Display lcd;
HX711 scale;
ESP8266WebServer server(80);
State state;
void setState(State s);

#include <func.h>


void setup() {
  #ifdef SDA_PIN
  Wire.begin(SDA_PIN, SCL_PIN);
  #else
  Wire.begin(D1, D2);
  #endif
  
  lcd.init();
  lcd.setCursor(0, 0);
  lcd.print(F("ver: "));
  lcd.print(FPSTR(FW_version));

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {delay(500);}
  lcd.clear();
  lcd.setCursor(0, 1); 
  lcd.print(WiFi.localIP()); 

  MDNS.begin("mixer");
  MDNS.addService("http", "tcp", 80);
  server.on("/rest/events",  handleSubscribe);
  server.on("/rest/meta",    handleMeta);
  server.on("/rest/start",   handleStart);
  server.on("/rest/tare",    handleTare);
  server.on("/rest/measure", handleMeasure);  
  server.on("/rest/test",    handleTest);
  server.on("/",             mainPage);
  server.on("/calibration",  calibrationPage);
  server.on("/style.css",    cssPage);
  server.begin();
  ArduinoOTA.onStart([]() {});
  ArduinoOTA.onEnd([]() {});
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {});
  ArduinoOTA.onError([](ota_error_t error) {});
  ArduinoOTA.begin();
  
  mcp.begin();
  for (byte i = 0; i < PUMPS_NO; i++) {
    mcp.pinMode(pinForward[i], OUTPUT); 
    mcp.pinMode(pinReverse[i], OUTPUT);
  }

  #ifdef SCALE_DOUT_PIN
  scale.begin(SCALE_DOUT_PIN, SCALE_SCK_PIN);
  #else
  scale.begin(D5, D6); // DOUT = D5 SCK = D6;
  #endif
  scale.set_scale(scale_calibration_A);
  scale.power_up();
  
  lcd.setCursor(0, 0);
  lcd.print("Tare scales...");

  tareScalesWithCheck(255);
  delay(3000);
  
  lcd.clear();
  setState(STATE_READY);
}

void loop() {
  readScales(16);
  printStatus(stateStr[state]); 
  printProgressValueOnly(rawToUnits(displayFilter.getEstimation()));
  server.handleClient();
  ArduinoOTA.handle();
  MDNS.update();
  if (lastSentTime + 1000 < millis()) sendScalesValue();
}
