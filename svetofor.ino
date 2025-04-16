#include <FastLED.h>
#include <Wire.h>
#define LED_PIN     5       // Пин подключения светодиодной ленты
#define NUM_LEDS    3       // Количество светодиодов (красный, жёлтый, зелёный)
#define SENSOR_PIN  34      // Аналоговый пин для датчика MGS-D20

CRGB leds[NUM_LEDS];
#include <VL53L0X.h>
VL53L0X lox;

// Время базового сигнала (в мс)
const int baseRedTime = 5000;
const int baseYellowTime = 2000;
const int baseGreenTime = 5000;

// Порог расстояния для определения машины (настраивается под датчик)
const int distanceThreshold = 50; // примерное значение АЦП для машины

// Максимальное количество машин для регулировки скорости
const int maxCars = 5;

// Время подсчёта машин (в мс)
const unsigned long countInterval = 3000;

// Состояния светофора
enum TrafficState {
  RED,
  RED_YELLOW,
  GREEN,
  GREEN_BLINK,
  YELLOW
};

TrafficState state = RED;

unsigned long stateStartTime = 0;
unsigned long lastCarCheckTime = 0;

int carsDetected = 0;

// Для подсчёта машин
unsigned long carCountStartTime = 0;
int carCountTemp = 0;
bool countingCars = false;

void setup() {
  Serial.begin(115200);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(100);
  pinMode(SENSOR_PIN, INPUT);
  stateStartTime = millis();
  carCountStartTime = millis();
  countingCars = true;
  Wire.begin();
  lox.init();
  lox.setTimeout(500);
  // параметры для режима высокой точности
  lox.setMeasurementTimingBudget(200000);
}
void setTrafficLightColor(bool red, bool yellow, bool green) {
  leds[0] = red ? CRGB::Red : CRGB::Black;
  leds[1] = yellow ? CRGB::Yellow : CRGB::Black;
  leds[2] = green ? CRGB::Green : CRGB::Black;
  FastLED.show();
}

void loop() {
  unsigned long currentMillis = millis();

  // Подсчёт машин в течение countInterval
  if (countingCars) {
    int sensorValue = lox.readRangeSingleMillimeters();
    if (sensorValue > distanceThreshold) {
      carCountTemp++;
      // Ждём 500 мс, чтобы не считать одну машину несколько раз
      delay(500); // Здесь можно заменить на неблокирующий аналог, но для простоты оставим
    }
    if (currentMillis - carCountStartTime >= countInterval) {
      carsDetected = carCountTemp;
      if (carsDetected > maxCars) carsDetected = maxCars;
      Serial.print("Cars detected: ");
      Serial.println(carsDetected);
      carCountTemp = 0;
      carCountStartTime = currentMillis;
      countingCars = false; // Подсчёт завершён, переходим к работе светофора
      stateStartTime = currentMillis;
    }
    return; // Пока считаем машины, не переключаем свет
  }

  // Рассчитываем время для каждого состояния с учётом количества машин
  int greenTime = baseGreenTime + carsDetected * 1000; // +1 сек на машину
  int redTime = baseRedTime - carsDetected * 800;      // -0.8 сек на машину
  if (redTime < 1000) redTime = 1000;

  switch (state) {
    case RED:
      setTrafficLightColor(true, false, false);
      if (currentMillis - stateStartTime >= (unsigned long)redTime) {
        state = RED_YELLOW;
        stateStartTime = currentMillis;
      }
      break;

    case RED_YELLOW:
      setTrafficLightColor(true, true, false);
      if (currentMillis - stateStartTime >= baseYellowTime) {
        state = GREEN;
        stateStartTime = currentMillis;
      }
      break;

    case GREEN:
      setTrafficLightColor(false, false, true);
      if (currentMillis - stateStartTime >= (unsigned long)greenTime) {
        state = GREEN_BLINK;
        stateStartTime = currentMillis;
      }
      break;

    case GREEN_BLINK: {
      // Мигающий зелёный 3 раза по 500 мс (всего 3000 мс)
      unsigned long elapsed = currentMillis - stateStartTime;
      int blinkCycle = (elapsed / 500) % 2; // 0 или 1
      if (blinkCycle == 0) {
        setTrafficLightColor(false, false, false);
      } else {
        setTrafficLightColor(false, false, true);
      }
      if (elapsed >= 3000) {
        state = YELLOW;
        stateStartTime = currentMillis;
      }
      break;
    }

    case YELLOW:
      setTrafficLightColor(false, true, false);
      if (currentMillis - stateStartTime >= baseYellowTime) {
        // После полного цикла снова считаем машины
        countingCars = true;
        carCountStartTime = currentMillis;
      }
      break;
  }
}
