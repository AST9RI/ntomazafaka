#include <FastLED.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#define LED_PIN1     5
#define LED_PIN2     18
#define NUM_LEDS     3

#define SENSOR1_XSHUT 32
#define SENSOR2_XSHUT 33

#define MAX_CHANNEL 8          // Добавьте, если отсутствует
#define I2C_HUB_ADDR 0x70     // Адрес I2C мультиплексора PCA9547 (пример)
#define EN_MASK 0x04          // Пример маски, замените на вашу если нужно

CRGB leds1[NUM_LEDS];
CRGB leds2[NUM_LEDS];

VL53L0X lox1;
VL53L0X lox2;
Adafruit_BME280 bme280;

// Время базового сигнала (в мс)
const int baseRedTime = 5000;
const int baseYellowTime = 2000;
const int baseGreenTime = 5000;
const char* ssid = "Wi-Fi"; // Замените на вашу сеть
const char* password = "password"; // Замените на ваш пароль
WiFiUDP udp;
const char* udpServerIP = "192.168.66.240"; // IP приёмника
const uint16_t udpPort = 8000;

// Порог расстояния для определения машины
const int distanceThreshold = 50;

// Максимальное количество машин для регулировки скорости
const int maxCars = 5;

// Время подсчёта машин (в мс)
const unsigned long countInterval = 60000;

// Состояния светофора
enum TrafficState {
  RED,
  RED_YELLOW,
  GREEN,
  GREEN_BLINK,
  YELLOW
};

struct TrafficLight {
  TrafficState state;
  unsigned long stateStartTime;
  unsigned long carCountStartTime;
  int carsDetected;
  int carCountTemp;
  bool countingCars;
  unsigned long lastCarCountTime;  // Для неблокирующего подсчёта машин
};

struct Data {
  byte state1;
  byte state2;
  byte conjest1;
  byte conjest2;
  float humid;
  float temp;
  float pressure;
};

TrafficLight tl1 = {RED, 0, 0, 0, 0, true, 0};
TrafficLight tl2 = {RED, 0, 0, 0, 0, true, 0};

void setTrafficLightColor(CRGB* leds, bool red, bool yellow, bool green) {
  leds[0] = red ? CRGB::Red : CRGB::Black;
  leds[1] = yellow ? CRGB::Yellow : CRGB::Black;
  leds[2] = green ? CRGB::Green : CRGB::Black;
  FastLED.show();
}

bool setBusChannel(uint8_t i2c_channel)
{
  if (i2c_channel >= MAX_CHANNEL)
  {
    return false;
  }
  else
  {
    Wire.beginTransmission(I2C_HUB_ADDR);
    Wire.write(i2c_channel | EN_MASK); // для микросхемы PCA9547
    // Wire.write(0x01 << i2c_channel); // Для микросхемы PW548A
    Wire.endTransmission();
    return true;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());

  // Инициализация датчиков VL53L0X с переключением каналов I2C мультиплексора
  setBusChannel(0x05);
  delay(10);
  lox1.init();
  lox1.setTimeout(500);
  lox1.setMeasurementTimingBudget(200000);

  setBusChannel(0x04);
  delay(10);
  lox2.init();
  lox2.setTimeout(500);
  lox2.setMeasurementTimingBudget(200000);

  // --- Инициализация двух лент ---
  FastLED.addLeds<WS2812, LED_PIN1, GRB>(leds1, NUM_LEDS);
  FastLED.addLeds<WS2812, LED_PIN2, GRB>(leds2, NUM_LEDS);
  FastLED.setBrightness(100);

  unsigned long now = millis();
  tl1.stateStartTime = now;
  tl1.carCountStartTime = now;
  tl1.lastCarCountTime = 0;
  tl2.stateStartTime = now;
  tl2.carCountStartTime = now;
  tl2.lastCarCountTime = 0;

  udp.begin(udpPort); // Запускаем UDP на локальном порту
  setBusChannel(0x06);
  bool bme_status = bme280.begin();
  if (!bme_status) {
    Serial.println("Не найден по адресу 0х77, пробую другой...");
    setBusChannel(0x06);
    bme_status = bme280.begin(0x76);
    if (!bme_status)
      Serial.println("Датчик не найден, проверьте соединение");
  }
}

void processTrafficLight(TrafficLight &tl, int leng, CRGB* leds) {
  unsigned long currentMillis = millis();

  // Подсчёт машин (без delay)
  if (tl.countingCars) {
    int sensorValue = leng;

    if (sensorValue > distanceThreshold) {
      if (currentMillis - tl.lastCarCountTime >= 500) {
        tl.carCountTemp++;
        tl.lastCarCountTime = currentMillis;
      }
    }

    if (currentMillis - tl.carCountStartTime >= countInterval) {
      tl.carsDetected = tl.carCountTemp;
      if (tl.carsDetected > maxCars) tl.carsDetected = maxCars;
      Serial.print("Cars detected: ");
      Serial.println(tl.carsDetected);
      tl.carCountTemp = 0;
      tl.carCountStartTime = currentMillis;
      tl.countingCars = false;
      tl.stateStartTime = currentMillis;
    }
    return;
  }

  int greenTime = baseGreenTime + tl.carsDetected * 1000;
  int redTime = baseRedTime - tl.carsDetected * 800;
  if (redTime < 1000) redTime = 1000;

  switch (tl.state) {
    case RED:
      setTrafficLightColor(leds, true, false, false);
      if (currentMillis - tl.stateStartTime >= (unsigned long)redTime) {
        tl.state = RED_YELLOW;
        tl.stateStartTime = currentMillis;
      }
      break;

    case RED_YELLOW:
      setTrafficLightColor(leds, true, true, false);
      if (currentMillis - tl.stateStartTime >= baseYellowTime) {
        tl.state = GREEN;
        tl.stateStartTime = currentMillis;
      }
      break;

    case GREEN:
      setTrafficLightColor(leds, false, false, true);
      if (currentMillis - tl.stateStartTime >= (unsigned long)greenTime) {
        tl.state = GREEN_BLINK;
        tl.stateStartTime = currentMillis;
      }
      break;

    case GREEN_BLINK: {
      unsigned long elapsed = currentMillis - tl.stateStartTime;
      int blinkCycle = (elapsed / 500) % 2;
      if (blinkCycle == 0) {
        setTrafficLightColor(leds, false, false, false);
      } else {
        setTrafficLightColor(leds, false, false, true);
      }
      if (elapsed >= 3000) {
        tl.state = YELLOW;
        tl.stateStartTime = currentMillis;
      }
      break;
    }

    case YELLOW:
      setTrafficLightColor(leds, false, true, false);
      if (currentMillis - tl.stateStartTime >= baseYellowTime) {
        tl.countingCars = true;
        tl.carCountStartTime = currentMillis;
        tl.lastCarCountTime = 0;
      }
      break;
  }
}

int leng1, leng2;

Data toSend;

void loop() {
  setBusChannel(0x06);
  float t = bme280.readTemperature();
  setBusChannel(0x06);
  float h = bme280.readHumidity();
  setBusChannel(0x06);
  float p = bme280.readPressure() / 100.0F;
  toSend.temp = t;
  toSend.humid = h;
  toSend.pressure = p;
  setBusChannel(0x05);
  delay(5); // Небольшая задержка для переключения канала
  leng1 = lox1.readRangeSingleMillimeters();

  setBusChannel(0x04);
  delay(5);
  leng2 = lox2.readRangeSingleMillimeters();

  processTrafficLight(tl1, leng1, leds1);
  processTrafficLight(tl2, leng2, leds2);

  // Формируем данные для отправки
  if (tl1.state == RED) toSend.state1 = 2;
  else if (tl1.state == GREEN) toSend.state1 = 0;
  else if (tl1.state == YELLOW) toSend.state1 = 1;
  else toSend.state1 = 3; // для RED_YELLOW и GREEN_BLINK

  if (tl2.state == RED) toSend.state2 = 2;
  else if (tl2.state == GREEN) toSend.state2 = 0;
  else if (tl2.state == YELLOW) toSend.state2 = 1;
  else toSend.state2 = 3;

  toSend.conjest1 = tl1.carCountTemp;
  toSend.conjest2 = tl2.carCountTemp;

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
