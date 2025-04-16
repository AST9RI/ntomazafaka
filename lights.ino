#include <BH1750.h>
#include <PCA9634.h>
#include <Wire.h>
/*Обнаружен модуль MGB-RGB3 по адресу: 0x08 !
Обнаружен модуль MGB-RGB3 по адресу: 0x0C !
Обнаружен модуль MGB-RGB3 по адресу: 0x18 !*/
BH1750 LightSensor_1; // экземпляры пяти датчиков освещенности
BH1750 LightSensor_2;
BH1750 LightSensor_3;
BH1750 LightSensor_4;

bool light1 = false;
bool light2 = false;
bool light3 = false;
bool light4 = false;

#define I2C_HUB_ADDR        0x70
#define EN_MASK             0x08
#define DEF_CHANNEL         0x00
#define MAX_CHANNEL         0x08

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

PCA9634 testModule(0x1C);
PCA9634 testModule1(0x1C);
PCA9634 testModule2(0x1C);
PCA9634 testModule3(0x1C);
/*
  I2C порт 0x07 - выводы GP16 (SDA), GP17 (SCL)
  I2C порт 0x06 - выводы GP4 (SDA), GP13 (SCL)
  I2C порт 0x05 - выводы GP14 (SDA), GP15 (SCL)
  I2C порт 0x04 - выводы GP5 (SDA), GP23 (SCL)
  I2C порт 0x03 - выводы GP18 (SDA), GP19 (SCL)
*/
void setup() {
  // Инициализация последовательного порта
  Serial.begin(115200);







  Wire.begin();
  delay(1000);

  // Инициализация датчика #1
  setBusChannel(0x04); // функция смены I2C-порта
  testModule.begin();
  setBusChannel(0x04);
  LightSensor_1.begin();

  // Инициализация датчика #2
  setBusChannel(0x05); // функция смены I2C-порта
  testModule1.begin();
  setBusChannel(0x05);
  LightSensor_2.begin();

  // Инициализация датчика #3
  setBusChannel(0x06); // функция смены I2C-порта
  testModule2.begin();
  setBusChannel(0x06);
  LightSensor_3.begin();

  setBusChannel(0x07); // функция смены I2C-порта
    testModule3.begin();
    setBusChannel(0x07);
  LightSensor_4.begin();
setBusChannel(0x04);
    for (int channel = 0; channel < testModule.channelCount(); channel++)
  {
    setBusChannel(0x04);
    testModule.setLedDriverMode(channel, PCA9634_LEDON);
    Serial.println(channel);
    delay(500);
    setBusChannel(0x04);
    testModule.setLedDriverMode(channel, PCA9634_LEDOFF);
    delay(500);
  }
  setBusChannel(0x04);
  for (int channel = 0; channel < testModule.channelCount(); channel++)
  {
    testModule.setLedDriverMode(channel, PCA9634_LEDPWM); // установка режима ШИМ (0-255)
  }
    setBusChannel(0x05);
    for (int channel = 0; channel < testModule1.channelCount(); channel++)
  {
    setBusChannel(0x05);
    testModule1.setLedDriverMode(channel, PCA9634_LEDON);
    Serial.println(channel);
    delay(500);
    setBusChannel(0x05);
    testModule1.setLedDriverMode(channel, PCA9634_LEDOFF);
    delay(500);
  }
  setBusChannel(0x05);
  for (int channel = 0; channel < testModule1.channelCount(); channel++)
  {
    setBusChannel(0x05);
    testModule1.setLedDriverMode(channel, PCA9634_LEDPWM); // установка режима ШИМ (0-255)
  }
  setBusChannel(0x06);
    for (int channel = 0; channel < testModule2.channelCount(); channel++)
  {
    setBusChannel(0x06);
    testModule2.setLedDriverMode(channel, PCA9634_LEDON);
    Serial.println(channel);
    delay(500);
    setBusChannel(0x06);
    testModule2.setLedDriverMode(channel, PCA9634_LEDOFF);
    delay(500);
  }
  setBusChannel(0x06);
  for (int channel = 0; channel < testModule2.channelCount(); channel++)
  {
    setBusChannel(0x06);
    testModule2.setLedDriverMode(channel, PCA9634_LEDPWM); // установка режима ШИМ (0-255)
  }
  setBusChannel(0x07);
    for (int channel = 0; channel < testModule3.channelCount(); channel++)
  {
    setBusChannel(0x07);
    testModule3.setLedDriverMode(channel, PCA9634_LEDON);
    Serial.println(channel);
    delay(500);
    setBusChannel(0x07);
    testModule3.setLedDriverMode(channel, PCA9634_LEDOFF);
    delay(500);
  }
  setBusChannel(0x07);
  for (int channel = 0; channel < testModule3.channelCount(); channel++)
  {
    setBusChannel(0x07);
    testModule3.setLedDriverMode(channel, PCA9634_LEDPWM); // установка режима ШИМ (0-255)
  }
}
void loop() {
  // Считывание датчиков
  setBusChannel(0x04);
  float light1 = LightSensor_1.readLightLevel();
  setBusChannel(0x05);
  float light2 = LightSensor_2.readLightLevel();
  setBusChannel(0x06);
  float light3 = LightSensor_3.readLightLevel();
  setBusChannel(0x07);
  float light4 = LightSensor_4.readLightLevel();


  if (light1 < 250){
    setBusChannel(0x04);
  testModule.write1(3, 0x90);
    setBusChannel(0x04);
  testModule.write1(2, 0x90);
    setBusChannel(0x04);
  testModule.write1(5, 0x00);
  }else{
    setBusChannel(0x04);
  testModule.write1(3, 0x00);
  setBusChannel(0x04);
  testModule.write1(2, 0x00);
  setBusChannel(0x04);
  testModule.write1(5, 0x00);
  }
  if (light2 < 250){
  setBusChannel(0x05);
  testModule1.write1(3, 0x90);
  setBusChannel(0x05);
  testModule1.write1(2, 0x90);
  setBusChannel(0x05);
  testModule1.write1(5, 0x00);
  }else{
    setBusChannel(0x05);
  testModule1.write1(3, 0x00);
  setBusChannel(0x05);
  testModule1.write1(2, 0x00);
  setBusChannel(0x05);
  testModule1.write1(5, 0x00);
  }
  if (light3 < 250){
    setBusChannel(0x06);
  testModule2.write1(3, 0x90);
  setBusChannel(0x06);
  testModule2.write1(2, 0x90);
  setBusChannel(0x06);
  testModule2.write1(5, 0x00);
  }  else{
    setBusChannel(0x06);
  testModule2.write1(3, 0x00);
  setBusChannel(0x06);
  testModule2.write1(2, 0x00);
  setBusChannel(0x06);
  testModule2.write1(5, 0x00);
  }
  if (light4 < 250){
    setBusChannel(0x07);
  testModule3.write1(3, 0x90);
  setBusChannel(0x07);
  testModule3.write1(2, 0x90);
  setBusChannel(0x07);
  testModule3.write1(5, 0x00);
  } else{
    setBusChannel(0x07);
  testModule3.write1(3, 0x00);
  setBusChannel(0x07);
  testModule3.write1(2, 0x00);
  setBusChannel(0x07);
  testModule3.write1(5, 0x00);
  }
  Serial.println("-------------------------------------------------------");
  Serial.println("Ambient light intensity 1: " + String(light1, 1) + " lx");
  Serial.println("Ambient light intensity 2: " + String(light2, 1) + " lx");
  Serial.println("Ambient light intensity 3: " + String(light3, 1) + " lx");
  Serial.println("Ambient light intensity 4: " + String(light3, 1) + " lx");
  delay(1000);
}

