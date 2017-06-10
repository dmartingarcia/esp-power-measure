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
#include <ESP8266HTTPClient.h>
#define VERSION 1
#define SENS 0.1875
#define AC_FREQ 50
#define SAMPLES 100

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

const int sclPin = D2;
const int sdaPin = D1;
char buffer[100];

void setup()
{
  WiFiManager wifiManager;
  wifiManager.setTimeout(180);
  
  Wire.begin(sdaPin, sclPin);
  oled.init();                      // Initialze SSD1306 OLED display
  oled.clearDisplay();              // Clear screen
  oled.setTextXY(0,0);              // Set cursor position, start of line 0
  oled.putString("INIT");
  if (!wifiManager.autoConnect("ESP-watts")) {
    Serial.println("failed to connect, we should reset as see if it connects");
    delay(3000);
    ESP.reset();
    delay(5000);
  }
  
  ads.setGain(GAIN_TWOTHIRDS);  // 0.1875mV (default)
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
  float volts_max = 0.0;

  for (int i = 0; i < SAMPLES; i++) {
    adc0 = ads.readADC_SingleEnded(3);
    volts_max =  _max(volts_max, adc0);
  }
   
  volts_max = volts_max * SENS / 1000; 
  //volts_min = volts_min * SENS / 1000;
  float vcc = ads.readADC_SingleEnded(0) * SENS / 1000;
  float offset = 2.5; /*vcc / 2 + 0.03;*/
  float diff_volts = abs(volts_max - offset);
  float current = diff_volts / 0.066;
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
  oled.setTextXY(4,0);             // Set cursor position, line 2 10th character
  oled.putString("Offs: ");
  oled.putFloat(offset);
  oled.putString("        ");
  oled.setTextXY(5,0);             // Set cursor position, line 2 10th character
  oled.putString("Vread: ");
  oled.putFloat(volts_max);
  
  char power_string[10];
  add_headers();
  add_measurement("consumption", power);
  add_measurement("vcc", vcc);
  add_measurement("current", current);
  add_measurement("vread", volts_max);
  oled.setTextXY(6,0);
  oled.putString("HTTP req: ");
  oled.putNumber(post_measurements());
}

void add_headers(){
  sprintf(buffer, "wattmetter,host=%d version=%d", ESP.getChipId(), VERSION);
}

void add_measurement(char* name, float measurement){
  char measurement_buffer[10];
  dtostrf(measurement, 4, 6, measurement_buffer);
  sprintf(buffer, "%s,%s=%s", buffer, name, measurement_buffer);
}

int post_measurements(){
  HTTPClient http;
  http.begin("http://192.168.1.134:8086/write?db=sensors");
  return http.POST(buffer);
}

