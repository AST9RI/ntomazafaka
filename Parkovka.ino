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
const float moisture_100 = 100.0;
void setup() {

  Serial.begin(115200);

  Wire.begin();

  mcp3021.begin(ADDR);
  lox.init();
  lox.setTimeout(500);

  lox.setMeasurementTimingBudget(200000);
}

void loop() {
  Fire.get_ir_and_vis();
  visib = Fire.vis_data;
  float dist = lox.readRangeSingleMillimeters();
  //Serial.println("Distance  = " + String(dist, 0) + " mm  ");
  float adc0 = mcp3021.readADC();
  float h = map(adc0, air_value, water_value, moisture_0, moisture_100);
  //Serial.println("Water level = " + String(h, 1) + " %");
  if(h>30){
    Serial.println("Протечка!");}

  if (visib >= 8000){
    Serial.println("Пожар");
  }
  if (dist <= 60){

    int value = 0;
    for(int i = random(0, 20); i<= 60; i++){
      float adc0 = mcp3021.readADC();
      float h = map(adc0, air_value, water_value, moisture_0, moisture_100);
      Fire.get_ir_and_vis();
      visib = Fire.vis_data;
      if(h>30){
          Serial.println("Протечка!");
      }
      if (visib >= 8000){
          Serial.println("Пожар!!!");
      }
      value = i * 1.66666667;
      Serial.println("Зарядка:" + String(value) + "%");
      delay(1000);

    }
  }
  delay(250);
}