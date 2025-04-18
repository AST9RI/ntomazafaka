#include <Wire.h>
#include <VL53L0X.h>
#include <FastLED.h>
#include "SparkFun_SGP30_Arduino_Library.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <WiFiUdp.h>
Adafruit_BME280 TVASensor;
SGP30 CO2Sensor;
// Настройки светофоров
#define LED_PIN_1  15
#define LED_PIN_2  14
#define NUM_LEDS   3
#define BRIGHTNESS 50
CRGB leds1[NUM_LEDS], leds2[NUM_LEDS];
WiFiUDP udp;
float t;
float h;
float p;
// Настройки мультиплексора I2C
#define I2C_HUB_ADDR 0x70
#define CHANNEL_1 0x06
#define CHANNEL_2 0x07
VL53L0X sensor1, sensor2;

struct data {
  int light1;
  int light2;
  int carcount1;
  int carcount2;
  int co2;
  float humid;
  float temp;
  float pressure;
  int light1fs;
  int tvoc;
};

// Параметры трафика
struct {
  int carCount = 0;
  unsigned long lastReset = 0;
  unsigned long lastDetection = 0;
} street1, street2;

const unsigned long RESET_INTERVAL = 60000;  // Сброс счетчика каждые 60 сек
const unsigned long CAR_COOLDOWN = 3000;     // 3 сек между обнаружением машин
const unsigned long YELLOW_TIME = 2000;      // 2 сек желтого
const unsigned long MIN_GREEN_TIME = 15000;   // 5 сек мин. зеленого
const unsigned long MAX_TOTAL_CYCLE = 50000; // 50 сек полный цикл

// Состояния светофоров
enum {GREEN1_RED2, YELLOW1_RED2, RED1_GREEN2, RED1_YELLOW2} state = GREEN1_RED2;
unsigned long stateStartTime;
unsigned long greenTime1 = MIN_GREEN_TIME;
unsigned long greenTime2 = MIN_GREEN_TIME;

void setBusChannel(uint8_t channel) {
  Wire.beginTransmission(I2C_HUB_ADDR);
  Wire.write(channel | 0x08);  // Для PCA9547
  Wire.endTransmission();
}

data datik;

void setup() {
  Serial.begin(115200);
  WiFi.begin("TP-Link_4F90", "NTOContest202324");
  delay(1000);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println(".");
  }
  Serial.println("Connected to: ");
  Serial.print(WiFi.localIP());
  // Инициализация светодиодов
  FastLED.addLeds<NEOPIXEL, LED_PIN_1>(leds1, NUM_LEDS);
  FastLED.addLeds<NEOPIXEL, LED_PIN_2>(leds2, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  
  // Инициализация датчиков
  Wire.begin();
  setBusChannel(CHANNEL_1);
  if(!sensor1.init()) Serial.println("Ошибка датчика 1!");
  sensor1.setTimeout(500);
  
  setBusChannel(CHANNEL_2);
  if(!sensor2.init()) Serial.println("Ошибка датчика 2!");
  sensor2.setTimeout(500);
  setBusChannel(0x03);
  CO2Sensor.begin();

  CO2Sensor.initAirQuality();
  bool bme_status = TVASensor.begin();
  if (!bme_status) {
    Serial.println("Не найден по адресу 0х77, пробую другой...");
    bme_status = TVASensor.begin(0x76);
    if (!bme_status)
      Serial.println("Датчик не найден, проверьте соединение");
  }
  // Начальное состояние
  stateStartTime = millis();
  updateLights();
}

void updateLights() {
  fill_solid(leds1, NUM_LEDS, CRGB::Black);
  fill_solid(leds2, NUM_LEDS, CRGB::Black);
  
  switch(state) {
    case GREEN1_RED2:
      leds1[2] = CRGB::Green; leds2[0] = CRGB::Red; break;
    case YELLOW1_RED2:
      leds1[1] = CRGB::Yellow; leds2[0] = CRGB::Red; break;
    case RED1_GREEN2:
      leds1[0] = CRGB::Red; leds2[2] = CRGB::Green; break;
    case RED1_YELLOW2:
      leds1[0] = CRGB::Red; leds2[1] = CRGB::Yellow; break;
  }
  FastLED.show();
}

void checkTraffic() {
  unsigned long now = millis();

  float t = TVASensor.readTemperature();
  float h = TVASensor.readHumidity();
  float p = TVASensor.readPressure() / 100.0F;
  setBusChannel(0x03);  
  CO2Sensor.measureAirQuality();
  // Проверка датчика 1
  setBusChannel(CHANNEL_1);
  if(now - street1.lastDetection > CAR_COOLDOWN) {
    if(sensor1.readRangeSingleMillimeters() < 60) { // 60 см для машин
      street1.carCount = min(street1.carCount + 1, 20);
      street1.lastDetection = now;
        Serial.print("CO2: ");
  Serial.print(CO2Sensor.CO2);  Serial.print(" ppm\tTVOC: ");
  Serial.print(CO2Sensor.TVOC);
  Serial.println(" ppb");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print(" *C\tHumidity: ");
  Serial.print(h);
  Serial.print(" %\tPressure: ");
  Serial.print(p);
  Serial.println(" hPa");
  Serial.println("======================");
      Serial.print("Улица 1: машин=");
      Serial.println(street1.carCount);
    }
  }
  
  // Проверка датчика 2
  setBusChannel(CHANNEL_2);
  if(now - street2.lastDetection > CAR_COOLDOWN) {
    if(sensor2.readRangeSingleMillimeters() < 60) {
      street2.carCount = min(street2.carCount + 1, 20);
      street2.lastDetection = now;
      Serial.print("Улица 2: машин=");
      Serial.println(street2.carCount);
    }
  }
  
  // Сброс счетчиков
  if(now - street1.lastReset >= RESET_INTERVAL) {
    street1.carCount = 0;
    street1.lastReset = now;
  }
  if(now - street2.lastReset >= RESET_INTERVAL) {
    street2.carCount = 0;
    street2.lastReset = now;
  }
}

void calculateGreenTimes() {
  unsigned long totalAvailable = MAX_TOTAL_CYCLE - 2*YELLOW_TIME;
  unsigned long remaining = totalAvailable - 2*MIN_GREEN_TIME;
  
  if(street1.carCount > street2.carCount) {
    greenTime1 = MIN_GREEN_TIME + remaining-10000;
    greenTime2 = MIN_GREEN_TIME;
  } 
  else if(street2.carCount > street1.carCount) {
    greenTime1 = MIN_GREEN_TIME;
    greenTime2 = MIN_GREEN_TIME + remaining-10000;
  } 
  else {
    greenTime1 = MIN_GREEN_TIME + remaining/2;
    greenTime2 = MIN_GREEN_TIME + remaining/2;
  }

  Serial.print("Времена: Улица1=");
  Serial.print(greenTime1/1000);
  Serial.print("с, Улица2=");
  Serial.print(greenTime2/1000);
  Serial.println("с");
}

void loop() {
  unsigned long currentTime = millis();
  datik.co2 = CO2Sensor.CO2;
  datik.tvoc = CO2Sensor.TVOC;
  checkTraffic();
  
  if((state == GREEN1_RED2 && currentTime-stateStartTime >= greenTime1) || 
     (state == RED1_GREEN2 && currentTime-stateStartTime >= greenTime2)) {
    calculateGreenTimes();
  }
  
  switch(state) {
    case GREEN1_RED2:
      if(currentTime-stateStartTime >= greenTime1) {
        state = YELLOW1_RED2;
        datik.light1 = 1;
        datik.light2 = 2;
        stateStartTime = currentTime;
        updateLights();
      }
      break;
      
    case YELLOW1_RED2:
      if(currentTime-stateStartTime >= YELLOW_TIME) {
        state = RED1_GREEN2;
        datik.light1 = 2;
        datik.light2 = 0;
        stateStartTime = currentTime;
        updateLights();
      }
      break;
      
    case RED1_GREEN2:
      if(currentTime-stateStartTime >= greenTime2) {
        state = RED1_YELLOW2;
        datik.light1 = 2;
        datik.light2 = 1;
        stateStartTime = currentTime;
        updateLights();
      }
      break;
      
    case RED1_YELLOW2:
      if(currentTime-stateStartTime >= YELLOW_TIME) {
        state = GREEN1_RED2;
        datik.light1 = 0;
        datik.light2 = 2;
        stateStartTime = currentTime;
        updateLights();
      }
      break;
  }
  datik.humid = TVASensor.readHumidity();
  datik.temp = TVASensor.readTemperature();
  datik.pressure = TVASensor.readPressure() / 100.F;
  datik.carcount1 = street1.carCount;
  datik.carcount2 = street2.carCount;

  if (WiFi.status() == WL_CONNECTED) {
    udp.beginPacket("192.168.0.102", 8000);
    udp.write((uint8_t*)&datik, sizeof(data));
    udp.endPacket();
    Serial.println("Data sent");
  }

  delay(100);
}