#include <Wire.h>
#include <VL53L0X.h>
VL53L0X lox;
#include <PCA9634.h>
#include <ESP32Servo.h> // ШПИНГАЛЕТЫ ЛОПУНЛИ :((((((((((((
#include <WiFi.h>
#include <WiFiUdp.h>
Servo myservo;
PCA9634 testModule(0x1C);

WiFiUDP udp;

int count = 0;
void setup() {
  Serial.begin(115200);
  myservo.attach(14);
  Wire.begin();
  lox.init();
  lox.setTimeout(500);
  lox.setMeasurementTimingBudget(200000);
  testModule.begin();
  for (int channel = 0; channel < testModule.channelCount(); channel++)
  {
    testModule.setLedDriverMode(channel, PCA9634_LEDOFF);
  }
  WiFi.begin("TP-Link_4F90", "NTOContest202324");
  delay(1000);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("Connected to: " + WiFi.localIP().toString());
  udp.begin(8000);
}

bool train = false;


void loop() {
    float dist = lox.readRangeSingleMillimeters();

     if (dist <= 60){
      train = true;
      if (count < 3){
      
      for (int i = 0; i<=3;i++){
      testModule.setLedDriverMode(3, PCA9634_LEDOFF);
      testModule.setLedDriverMode(2, PCA9634_LEDOFF);
      testModule.setLedDriverMode(5, PCA9634_LEDOFF);
      delay(1000);
      testModule.setLedDriverMode(3, PCA9634_LEDON);
      testModule.setLedDriverMode(2, PCA9634_LEDOFF);
      testModule.setLedDriverMode(5, PCA9634_LEDOFF);
      delay(1000);
      count += 1;
      }}
  
      myservo.write(110);
      count = 4;
     }else{
    Serial.print("Выключен");
    train = false;
    if (count == 4){
      testModule.setLedDriverMode(3, PCA9634_LEDOFF);
      testModule.setLedDriverMode(2, PCA9634_LEDOFF);
      testModule.setLedDriverMode(5, PCA9634_LEDOFF); 
    delay(500);
      testModule.setLedDriverMode(3, PCA9634_LEDON);
      testModule.setLedDriverMode(2, PCA9634_LEDOFF);
      testModule.setLedDriverMode(5, PCA9634_LEDOFF);  
    count += 1;
    }delay(500);

        testModule.write1(3, 0x00);
    testModule.write1(2, 0x90);
    testModule.write1(5, 0x00);
    myservo.write(180);
    }

    if (WiFi.status() == WL_CONNECTED) {
    udp.beginPacket(IPAddress(192,168,0,102), 8000);
    udp.write((uint8_t*)&train, sizeof(bool));
    udp.endPacket();

    Serial.println("Data sent");
    }
    delay(500);


  Serial.println("Distance  = " + String(dist, 0) + " mm  ");
  delay(250);
}