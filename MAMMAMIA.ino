#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>

WebServer server(80);
WiFiUDP udp;

// Структуры данных
struct Data {
  bool state1;
  bool state2;
  bool state3;
  bool state4;
  float light1;
  float light2;
  float light3;
  float light4;
};

struct svetofor {
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

struct parking {
  bool occupied;
  byte level;
  int len;
  bool water;
  bool fire;
};

// Глобальные переменные для хранения последних данных
Data lampData = {0};
parking parkingData = {0};
svetofor trafficData = {0};
int CO2stat = 0;
bool schlagData;
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>ESP32 Dashboard</title></head><body>";
  html += "<h1>Smart City Dashboard</h1>";

  // Фонари
  html += "<h2>Street Lights</h2>";
  html += "<ul>";
  html += "<li>State1: " + String(lampData.state1 ? "ON" : "OFF") + "</li>";
  html += "<li>Light1: " + String(map(lampData.light1, 0, 30000, 0, 100)) + " %</li>";
  html += "<li>State2: " + String(lampData.state2 ? "ON" : "OFF") + "</li>";
  html += "<li>Light2: " + String(map(lampData.light2, 0, 30000, 0, 100)) + " %</li>";
  html += "<li>State3: " + String(lampData.state3 ? "ON" : "OFF") + "</li>";
  html += "<li>Light3: " + String(map(lampData.light3, 0, 30000, 0, 100)) + " %</li>";
  html += "<li>State4: " + String(lampData.state4 ? "ON" : "OFF") + "</li>";
  html += "<li>Light4: " + String(map(lampData.light4, 0, 30000, 0, 100)) + " %</li>";
  html += "</ul>";

  // Паркинг
  html += "<h2>Parking</h2>";
  html += "<ul>";
  html += "<li>Occupied: " + String(parkingData.occupied ? "Yes" : "No") + "</li>";
  html += "<li>Level: " + String(parkingData.level) + "</li>";
  html += "<li>Length: " + String(parkingData.len) + "</li>";
  html += "<li>Water: " + String(parkingData.water ? "Yes" : "No") + "</li>";
  html += "</ul>";

  // Светофор
  html += "<h2>Traffic Light</h2>";
  html += "<ul>";
  html += "<li>Light1: " + String(trafficData.light1) + "</li>";
  html += "<li>Light2: " + String(trafficData.light2) + "</li>";
  html += "<li>Car Count 1: " + String(trafficData.carcount1) + "</li>";
  html += "<li>Car Count 2: " + String(trafficData.carcount2) + "</li>";
  html += "<li>CO2: " + String(trafficData.co2) + " ppm</li>";
  html += "<li>Humidity: " + String(trafficData.humid) + " %</li>";
  html += "<li>Temperature: " + String(trafficData.temp) + " °C</li>";
  html += "<li>Pressure: " + String(trafficData.pressure) + " hPa</li>";
  html += "<li>Light1FS: " + String(trafficData.light1fs) + "</li>";
  html += "<li>TVOC: " + String(trafficData.tvoc) + "</li>";
  html += "</ul>";
  if (CO2stat == 0) html += "<p>Вы можете спокойно посещать наш район - с воздухом всё в порядке.</p>";
  if (CO2stat == 1) html += "<p>Будьте бдительны, следите за своим самочувствием в нашем районе - уровень CО2 выше нормы.</p>";
  if (CO2stat == 2) html += "<p>Воздержитесь от посещения нашего района - уровень СО2 приближается к опасным значениям.</p>";
  if (CO2stat == 3) html += "<p>Посещение нашего района запрещено - уровни СО2 слишком высокие.</p>";
  if (schlagData) html += "<p>Поезд едет.</p>";
  if (!schlagData) html += "<p>Поезда нет.</p>"; 
  
  html += "<p><small>Обновите страницу для получения последних данных</small></p>";

  html += "</body></html>";

  server.send(200, "text/html", html);
}



void setup() {
  Serial.begin(115200);
  WiFi.begin("TP-Link_4F90", "NTOContest202324");
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  udp.begin(8000);

  server.on("/", handleRoot);
  server.begin();
}



void loop() {
  server.handleClient();

  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    IPAddress remoteIP = udp.remoteIP();

    if (packetSize == sizeof(Data) && remoteIP == IPAddress(192, 168, 0, 218)) {
      udp.read((uint8_t*)&lampData, sizeof(Data));
      Serial.println("Received Data from 192.168.0.218 (lampData)");
    } else if (packetSize == sizeof(parking) && remoteIP == IPAddress(192, 168, 0, 246)) {
      udp.read((uint8_t*)&parkingData, sizeof(parking));
      Serial.println("Received Data from 192.168.0.247 (parkingData)");
    } else if (packetSize == sizeof(svetofor) && remoteIP == IPAddress(192, 168, 0, 138)) {
      udp.read((uint8_t*)&trafficData, sizeof(svetofor));
      Serial.println("Received Data from 192.168.0.138 (trafficData)");
    } else if (packetSize == sizeof(bool) && remoteIP == IPAddress(192, 168, 0, 98)) {
      udp.read((uint8_t*)&schlagData, sizeof(bool));
      Serial.println("Recieved Data from 192.168.0.98 (schlag)");
    } else {
      // Неизвестный пакет - можно прочитать и проигнорировать
      uint8_t buffer[255];
      udp.read(buffer, min(packetSize, 255));
      Serial.println("Received unknown UDP packet");
    }
  }
  if (trafficData.co2 >= 0 && trafficData.co2 <= 400) CO2stat = 0;
  if (trafficData.co2 > 400 && trafficData.co2 <= 600) CO2stat = 1;
  if (trafficData.co2 > 600 && trafficData.co2 <= 1000) CO2stat = 2;
  if (trafficData.co2 > 1000) CO2stat = 3;

  Serial.println("lampData: ");
  Serial.println("\nLamp 1 state: " + String(lampData.state1));
  Serial.println("Lamp 2 state: " + String(lampData.state2));
  Serial.println("Lamp 3 state: " + String(lampData.state3));
  Serial.println("Lamp 4 state: " + String(lampData.state4));
  Serial.println("Light Level 1: " + String(map(lampData.light1, 0, 30000, 0, 100)));
  Serial.println("Light level 2: " + String(map(lampData.light2, 0, 30000, 0, 100)));
  Serial.println("Light level 3: " + String(map(lampData.light3, 0, 30000, 0, 100)));
  Serial.println("Light level 4: " + String(map(lampData.light4, 0, 30000, 0, 100)));

  Serial.println("parkingData: ");
  Serial.println("\nOccupied: " + String(parkingData.occupied));
  Serial.println("Charge level: " + String(parkingData.level));
  Serial.println("Distance: " + String(parkingData.len));
  Serial.println("Water: " + String(parkingData.water));
  Serial.println("Fire: " + String(parkingData.fire));

  Serial.println("trafficData: ");
  Serial.println("\nTRLight1: " + String(trafficData.light1));
  Serial.println("TRLight2: " + String(trafficData.light2));
  Serial.println("Carcount1: " + String(trafficData.carcount1));
  Serial.println("Carcount2: " + String(trafficData.carcount2));
  Serial.println("CO2: " + String(trafficData.co2));
  Serial.println("Humidity: " + String(trafficData.humid));
  Serial.println("Temperature: " + String(trafficData.temp));
  Serial.println("Pressure: " + String(trafficData.pressure));
  Serial.println("TVOC: " + String(trafficData.tvoc)); 

  Serial.println("\nSchlag: " + String(schlagData));
  delay(250);
}
