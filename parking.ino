#include <Wire.h>
#include <VL53L0X.h>
#include "mcp3021.h"
#include <MGS_FR403.h>

MGS_FR403 Fire;
float visib = 0;
byte ADDR = 0b011;
MCP3021 mcp3021;
VL53L0X lox;

const float air_value = 561.0;
const float water_value = 293.0;
const float moisture_0 = 0.0;
const float moisature_100 = 100.0;
bool occupied = 0;

bool isCharging = false;
unsigned long chargeStartTime = 0;
const int chargeDuration = 60000;
int initialCharge = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mcp3021.begin(ADDR);
  lox.init();
  lox.setTimeout(500);
  lox.setMeasurementTimingBudget(200000);
  randomSeed(analogRead(0));
}

struct Data {
  bool occupied;
  byte level;
};

Data toSend;

void loop() {
  Fire.get_ir_and_vis();
  visib = Fire.vis_data;
  float dist = lox.readRangeSingleMillimeters();
  float adc0 = mcp3021.readADC();
  float h = map(adc0, air_value, water_value, moisture_0, moisture_100);

  if (h > 30) {
    Serial.println("Протечка!");
  }
  if (visib >= 8000) {
    Serial.println("Пожар");
  }

  if (dist <= 60 && !isCharging) {
    occupied = 1;
    isCharging = true;
    chargeStartTime = millis();
    initialCharge = random(0, 21);
    Serial.println("Начало зарядки с " + String(initialCharge) + "%");
  }

  if (isCharging) {
    unsigned long elapsed = millis() - chargeStartTime;
    byte progress = initialCharge + ((elapsed * (100 - initialCharge)) / chargeDuration);

    if (progress > 100) progress = 100;
    
    Serial.println("Зарядка: " + String(progress) + "%");
    
    if (progress >= 100) {
      isCharging = false;
      occupied = 0;
      Serial.println("Зарядка завершена");
    }
  }

  toSend.occupied = occupied;
  toSend.level = progress;

  if (WiFi.status() == WL_CONNECTED) {
    udp.beginPacket(udpServerIP, udpPort);
    udp.write((uint8_t*)&toSend, sizeof(Data));
    udp.endPacket();

    Serial.println("Data sent");
  }
}

//udp.begin(listenPort);

//void loop() {
// int packetSize = udp.parsePacket();
// if (packetSize == sizeof(Data)) {
// Data receivedData;
// udp.read((uint8_t*)&receivedData, sizeof(Data));
//
// // Теперь можно использовать receivedData
// Serial.print("State1: "); Serial.println(receivedData.state1);
// Serial.print("Conjestion1: "); Serial.println(receivedData.conjest1);
// // и так далее
// }
//}
