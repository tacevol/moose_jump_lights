/**************************************************************************
  ESP32-S3 Reverse TFT Feather
  Built-in NeoPixel blink + OTA + BNO085 UART-RVC (one-wire receive)
 **************************************************************************/

#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Adafruit_NeoPixel.h>
#include "Adafruit_BNO08x_RVC.h"
#include "secrets.h"

// ----- Pins (one-wire RVC: sensor TX -> ESP32 RX) -----
#define BNO_RX_PIN  38   // Feather pin wired to BNO085 SDA (sensor TX)
#define BNO_TX_PIN  -1  // Unused in RVC

Adafruit_BNO08x_RVC rvc;

// ----- NeoPixel -----
Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
const uint8_t BRIGHT = 16;

// ----- Wi-Fi / OTA -----
const char* WIFI_SSID     = SECRET_WIFI_SSID;
const char* WIFI_PASS     = SECRET_WIFI_PASS;
const char* OTA_HOSTNAME  = SECRET_OTA_HOSTNAME;
const char* OTA_PASS_HASH = SECRET_OTA_PASS_HASH;

static void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) { delay(200); Serial.print("."); }
  Serial.print("\nWiFi connected: "); Serial.println(WiFi.localIP());
}

static void setupOTA() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  if (OTA_PASS_HASH && strlen(OTA_PASS_HASH) == 64) {
    ArduinoOTA.setPasswordHash(OTA_PASS_HASH);
  }
  ArduinoOTA.begin();
  Serial.println("OTA ready");
}

void setup() {
  Serial.begin(115200);
  delay(100);

#ifdef NEOPIXEL_POWER
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);
#endif
  pixel.begin();
  pixel.setBrightness(BRIGHT);
  pixel.fill(pixel.Color(0,255,0)); // start ON (green)
  pixel.show();

  connectWiFi();
  setupOTA();

  // --- BNO085 (one-wire UART receive) ---
  Serial1.begin(115200, SERIAL_8N1, BNO_RX_PIN, BNO_TX_PIN); // specify pins!
  if (!rvc.begin(&Serial1)) {
    Serial.println("BNO085 RVC init failed (check wiring and P0=3V).");
  } else {
    Serial.println("BNO085 RVC ready.");
  }

  Serial.println("Yaw,Pitch,Roll,X,Y,Z");
}

void loop() {
  ArduinoOTA.handle();

  // Blink NeoPixel (non-blocking)
  static uint32_t lastBlink = 0;
  static bool ledOn = true;
  if (millis() - lastBlink > 1000) {
    lastBlink = millis();
    ledOn = !ledOn;
    if (ledOn) pixel.fill(pixel.Color(0,255,0)); else pixel.clear();
    pixel.show();
  }

  // Read IMU if a fresh packet is available
  BNO08x_RVC_Data d;
  if (rvc.read(&d)) {
    Serial.print(d.yaw);      Serial.print(',');
    Serial.print(d.pitch);    Serial.print(',');
    Serial.print(d.roll);     Serial.print(',');
    Serial.print(d.x_accel);  Serial.print(',');
    Serial.print(d.y_accel);  Serial.print(',');
    Serial.print(d.z_accel);  Serial.println();
  }

  delay(1);
}
