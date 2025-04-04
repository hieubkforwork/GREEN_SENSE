#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include "DHT20.h"
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>
#include <LiquidCrystal_I2C.h>
#include <sstream>

/*CONFIG GPIO PIN*/
#define FAN 5            // P16
#define PUMP 19          // P14
#define LIGHT_SENSOR 33  // P1
#define SERVO 15         // P4
#define SOIL 39          // P2
#define PIN_NEO_PIXEL 32 // P0
#define NUM_PIXELS 4
LiquidCrystal_I2C lcd(0x21, 16, 2);

/*GLOBAL VARIABLE*/
String tempStr;
String humidStr;
String soilStr;
String lightStr;
String RGBStr;

bool light;

/*CONFIG MQTT CONNECTION*/
const char *ssid = "ACLAB";
const char *password = "ACLAB2023";
const char *mqttServer = "io.adafruit.com";
const int mqttPort = 1883;
const char *mqttUser = "hieuduongk22bk";
const char *mqttPassword = "aio_CgOe77qUN0Gdvfnb0sqMCTXErAdS";

WiFiClient espClient;
PubSubClient client(espClient);

DHT20 dht20;
Servo servo1;

Adafruit_NeoPixel NeoPixel(NUM_PIXELS, PIN_NEO_PIXEL, NEO_GRB + NEO_KHZ800);

/*FEED*/
const char *mqttTopic1 = "hieuduongk22bk/feeds/fan";
const char *mqttTopic2 = "hieuduongk22bk/feeds/temp";
const char *mqttTopic3 = "hieuduongk22bk/feeds/humid";
const char *mqttTopic4 = "hieuduongk22bk/feeds/pump";
const char *mqttTopic5 = "hieuduongk22bk/feeds/light_sensor";
const char *mqttTopic6 = "hieuduongk22bk/feeds/servo";
const char *mqttTopic7 = "hieuduongk22bk/feeds/soil";
const char *mqttTopic8 = "hieuduongk22bk/feeds/rgbled";

//------------------- WIFI ----------------------------------
void wifi_connect()
{
  Serial.print("Starting connecting WiFi.");
  delay(10);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

//------------------ MQTT ----------------------------------
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  String byteRead = "";
  Serial.print("Message: ");
  for (int i = 0; i < length; i++)
  {
    byteRead += (char)payload[i];
  }
  Serial.println(byteRead);
  Serial.println();
  Serial.println("------------------");

  if (strcmp(topic, mqttTopic1) == 0)
  {
    int fanSpeed = byteRead.toInt();
    analogWrite(FAN, fanSpeed);
    Serial.print("FAN Speed: ");
    Serial.println(fanSpeed);
  }

  if (strcmp(topic, mqttTopic4) == 0)
  {
    if (byteRead == "1")
    {
      analogWrite(PUMP, 255);
      Serial.println("PUMP ON");
    }
    else if (byteRead == "0")
    {
      analogWrite(PUMP, 0);
      Serial.println("PUMP OFF");
    }
  }

  if (strcmp(topic, mqttTopic6) == 0)
  {
    int ServoRadius = byteRead.toInt();
    Serial.print("ServoRadius: ");
    Serial.println(ServoRadius);
    servo1.write(ServoRadius);
    delay(100);
  }

    if (strcmp(topic, mqttTopic8) == 0)
    {
      int value[13];

      int pos = 0;
      for (int i = 0; i < 12; i++)
      {
        int space_pos = byteRead.indexOf(' ', pos);
        if (space_pos == -1)
          space_pos = byteRead.length();
        value[i] = byteRead.substring(pos, space_pos).toInt();
        pos = space_pos + 1;
      }

      int pos_a = 1;
      int pos_b = 2;
      int pos_c = 3;

      NeoPixel.clear();
      if (value[0] <= NUM_PIXELS)
      {
        for (int pixel = 0; pixel < value[0]; pixel++)
        {
          NeoPixel.setPixelColor(pixel, NeoPixel.Color(value[pos_a], value[pos_b], value[pos_c]));
          NeoPixel.show();
          pos_a = pos_a + 3;
          pos_b = pos_b + 3;
          pos_c = pos_c + 3;
        }
      }
    }
}
void mqtt_connect()
{
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  Serial.println("Connecting to MQTT…");
  while (!client.connected())
  {
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), mqttUser, mqttPassword))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed with state  ");
      Serial.println(client.state());
      delay(2000);
    }
  }
  client.subscribe(mqttTopic1);
  client.subscribe(mqttTopic4);
  client.subscribe(mqttTopic6);
  client.subscribe(mqttTopic8);
}

/*SCHEDULER TASKs*/
void TaskTemperature_Humidity(void *pvParameters)
{

  while (1)
  {
    Wire.begin();
    dht20.begin();
    dht20.read();

    double temperature = dht20.getTemperature();
    double humidity = dht20.getHumidity();

    Serial.print("Temp: ");
    Serial.print(temperature);
    Serial.print(" *C ");
    Serial.print(" Humidity: ");
    Serial.print(humidity);
    Serial.print(" %");

    Serial.println();
    if (temperature && humidity)
    {
      tempStr = String(temperature);
      client.publish(mqttTopic2, tempStr.c_str());
      humidStr = String(humidity);
      client.publish(mqttTopic3, humidStr.c_str());
    }
    vTaskDelay(60000);
  }
}

void TaskLight_Sensor(void *pvParameters)
{
  while (1)
  {
    double sensorLight = analogRead(LIGHT_SENSOR);
    double translate = (sensorLight) * (100) / (4095);
    double lux = round(translate);
    Serial.print("Light is ");
    Serial.print(lux);
    Serial.print(" Lx\t");
    Serial.println();
    lightStr = String(lux);
    client.publish(mqttTopic5, lightStr.c_str());
    vTaskDelay(60000);
  }
}

void TaskSoil(void *pvParameters)
{
  while (1)
  {
    double soil = analogRead(SOIL);
    double translate = (soil) * (100) / (4095);
    double per = round(translate);
    Serial.print("Soil Mosture is ");
    Serial.print(per);
    Serial.print(" %\t");
    Serial.println();
    soilStr = String(per);
    client.publish(mqttTopic7, soilStr.c_str());
    vTaskDelay(60000);
  }
}

// void TaskServo(void *pvParameters) {
//   servo1.attach(SERVO);  // Gắn servo vào GPIO

//   while (1) {
//       // Quay từ 0° -> 180°
//       for (int pos = 0; pos <= 180; pos++) {
//           servo1.write(pos);
//           vTaskDelay(10 / portTICK_PERIOD_MS);  // Thêm độ trễ để servo có thời gian quay
//       }
//       Serial.println("Servo đạt 180 độ");
//       vTaskDelay(2000 / portTICK_PERIOD_MS);  // Dừng 2 giây

//       // Quay từ 180° -> 0°
//       for (int pos = 180; pos >= 0; pos--) {
//           servo1.write(pos);
//           vTaskDelay(10 / portTICK_PERIOD_MS);
//       }
//       Serial.println("Servo trở về 0 độ");
//       vTaskDelay(2000 / portTICK_PERIOD_MS);  // Dừng 2 giây
//   }
// }

// void TaskLight(void *pvParameters)
// {
//   while (1)
//   {
//     NeoPixel.clear();
//     for (int pixel = 0; pixel < NUM_PIXELS; pixel++)
//     {
//       NeoPixel.setPixelColor(pixel, NeoPixel.Color(0, 255, 0));
//       NeoPixel.show();
//       vTaskDelay(500);
//     }
//     vTaskDelay(5000);
//   }
// }
void TaskLCD(void *pvParameters)
{
  while (1)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: " + tempStr);
    lcd.setCursor(0, 1);
    lcd.print("Humid: " + humidStr);
    vTaskDelay(10000);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Light: " + lightStr);
    lcd.setCursor(0, 1);
    lcd.print("Soil: " + soilStr);
    vTaskDelay(5000);
  }
}
/*SETUP*/
void setup()
{
  Serial.begin(115200);

  Serial.println(__FILE__);
  Serial.print("DHT20 LIBRARY VERSION: ");
  Serial.println(DHT20_LIB_VERSION);
  Serial.println();

  servo1.attach(SERVO);
  NeoPixel.begin();

  lcd.init();
  lcd.backlight();

  xTaskCreate(TaskTemperature_Humidity, "Temp_Humid Control", 2048, NULL, 2, NULL);
  xTaskCreate(TaskLight_Sensor, "Light_Sensor Control", 2048, NULL, 2, NULL);
  // xTaskCreate(TaskServo, "Servo Control", 2048, NULL, 2, NULL);
  xTaskCreate(TaskSoil, "Soil Control", 2048, NULL, 2, NULL);
  //xTaskCreate(TaskLight, "Light Control", 2048, NULL, 2, NULL);
  xTaskCreate(TaskLCD, "LCD Control", 2048, NULL, 2, NULL);

  pinMode(FAN, OUTPUT);
  pinMode(PUMP, OUTPUT);
  pinMode(LIGHT_SENSOR, INPUT);

  wifi_connect(); // Kết nối WiFi
  mqtt_connect(); // Kết nối MQTT
}

void loop()
{
  if (!client.connected())
  {
    mqtt_connect();
  }
  client.loop(); // Duy trì kết nối MQTT
}
