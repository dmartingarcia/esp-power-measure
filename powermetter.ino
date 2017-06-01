#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <ACROBOTIC_SSD1306.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h> 
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>


#define AC_FREQ 50
#define SAMPLES 500

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

const int sclPin = D2;
const int sdaPin = D1;

void setup()
{
  WiFiManager wifiManager;
  wifiManager.setTimeout(180);
  
  Wire.begin(sdaPin, sclPin);
  oled.init();                      // Initialze SSD1306 OLED display
  oled.clearDisplay();              // Clear screen
  oled.setTextXY(0,0);              // Set cursor position, start of line 0
  oled.putString("INIT");
  if (!wifiManager.autoConnect("ESP")) {
    Serial.println("failed to connect, we should reset as see if it connects");
    delay(3000);
    ESP.reset();
    delay(5000);
  }
  
  ads.setGain(GAIN_ONE);  // 0.125mV (default)
  ads.begin();
  ArduinoOTA.setHostname("esp-watts");
  ArduinoOTA.begin();
  oled.setTextXY(0,0);              // Set cursor position, start of line 0
  oled.putString("Starting");
  Serial.begin(115200);
  delay(1000);
}

void loop() {  
  ArduinoOTA.handle(); 

  int16_t adc0 = 0;
  float volts = 0;

  for (int i = 0; i < SAMPLES; i++) {
    volts =  _max(volts, ads.readADC_SingleEnded(3));
  }
   
  volts = volts * 0.125 / 1000; 
  
  float vcc = ads.readADC_SingleEnded(0) * 0.125 / 1000;
  float offset = vcc / 2 + 0.03;
  float diff_volts = volts - offset;
  float current = abs(diff_volts) / 0.066;
  float currentRMS = 0.707 * current;
  float power = 230.0 * currentRMS;

  oled.setTextXY(0,0);              // Set cursor position, start of line 0
  oled.putString("Vcc:  ");
  oled.putFloat(vcc);
  oled.putString("           ");
  oled.setTextXY(1,0);              // Set cursor position, start of line 1
  oled.putString("Vdif: ");
  oled.putFloat(diff_volts);
  oled.putString("          ");
  oled.setTextXY(2,0);              // Set cursor position, start of line 2
  oled.putString("Curr: ");
  oled.putFloat(current);
  oled.putString("           ");
  oled.setTextXY(3,0);             // Set cursor position, line 2 10th character
  oled.putString("Watt: ");
  oled.putFloat(power);
  oled.putString("        ");
  oled.setTextXY(5,0);             // Set cursor position, line 2 10th character
  oled.putString("Offs: ");
  oled.putFloat(offset);
  oled.putString("        ");
}
