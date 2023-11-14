#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#include <WiFi.h>
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#define WIFI_SSID "LucasR"
#define WIFI_PASSWORD "lucasromary3"

// #define MQTT_HOST IPAddress(192, 168, 1, 10)
#define MQTT_HOST "mqtt.ci-ciad.utbm.fr"
#define MQTT_PORT 1883

#define MQTT_TOPIC "IMP3D/U3_01/Leds/State"

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 3600, 60000);

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

int commande_heures = 0;
int commande_minutes = 0;
int timer_minutes = 0;
int alarme_minutes = 0;
int alarme_heures = 0;
int preparation_time = 0;

// NEOPIXEL
#define PIN D7       // On Trinket or Gemma, suggest changing this to 1
#define NUMPIXELS 16 // Popular NeoPixel ring size
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void setLedGreen()
{
  pixels.clear(); // Set all pixel colors to 'off'
  for (int i = 0; i < NUMPIXELS; i++)
  {
    pixels.setPixelColor(i, pixels.Color(0, 20, 0));
  }
  pixels.show(); // Send the updated pixel colors to the hardware.
}

void setLedRed()
{
  pixels.clear(); // Set all pixel colors to 'off'
  for (int i = 0; i < NUMPIXELS; i++)
  {
    pixels.setPixelColor(i, pixels.Color(20, 0, 0));
  }
  pixels.show(); // Send the updated pixel colors to the hardware.
}

void setLedYellow()
{
  pixels.clear(); // Set all pixel colors to 'off'
  for (int i = 0; i < NUMPIXELS; i++)
  {
    pixels.setPixelColor(i, pixels.Color(20, 20, 0));
  }
  pixels.show(); // Send the updated pixel colors to the hardware.
}

void setLedBlue()
{
  pixels.clear(); // Set all pixel colors to 'off'
  for (int i = 0; i < NUMPIXELS; i++)
  {
    pixels.setPixelColor(i, pixels.Color(0, 0, 20));
  }
  pixels.show(); // Send the updated pixel colors to the hardware.
}

void setLedConnectingMQTT()
{
  pixels.clear();
  for (int i = 0; i < NUMPIXELS - 1; i += 2)
  {
    pixels.setPixelColor(i, pixels.Color(0, 0, 20));
  }
  pixels.show(); // Send the updated pixel colors to the hardware.
}

void connectToWifi()
{
  randomSeed(micros());
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt()
{
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
  setLedGreen();
}

void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event)
  {
  case SYSTEM_EVENT_STA_GOT_IP:
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    connectToMqtt();

    timeClient.begin();
    timeClient.update();
    Serial.println(timeClient.getFormattedTime());

    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    WiFi.disconnect();
    WiFi.reconnect();
    Serial.println("WiFi lost connection");
    xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
    xTimerStart(wifiReconnectTimer, 0);
    break;
  }
}

void onMqttConnect(bool sessionPresent)
{
  Serial.println("Connected to MQTT.");
  mqttClient.subscribe(MQTT_TOPIC, 2);
  // mqttClient.publish("test/lol", 1, true, "test 2");
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  Serial.println("Disconnected from MQTT.");
  setLedConnectingMQTT();
  if (WiFi.isConnected())
  {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos)
{
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId)
{
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
  String messageTemp = "";
  for (int i = 0; i < len; i++)
  {
    // Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }

  Serial.print("  message: ");
  Serial.println(messageTemp);
  Serial.println();

  if (String(topic) == MQTT_TOPIC && messageTemp == "-1") // On allume en bleue
  {
    setLedBlue();
  }
  if (String(topic) == MQTT_TOPIC && messageTemp.toInt() > 0) // On allume en vert
  {
    Serial.print("Timer de :");
    Serial.print(messageTemp);
    Serial.println("minutes");
    timer_minutes = messageTemp.toInt() + preparation_time;
    timeClient.update();

    if(timer_minutes>60){
      commande_heures = timer_minutes/60;
    }
    Serial.print(commande_heures);
    Serial.print("h");
    commande_minutes = timer_minutes%60;
    Serial.println(commande_minutes);

    Serial.println("Reveil à :");

    if(timeClient.getMinutes()+commande_minutes >60){
      commande_heures++;
    }

    alarme_minutes = (timeClient.getMinutes()+commande_minutes)%60;
    alarme_heures = (timeClient.getHours()+commande_heures)%24;

    Serial.print(alarme_heures);
    Serial.print(" ");
    Serial.print(alarme_minutes);


    setLedRed();
  }
  
  if (String(topic) == MQTT_TOPIC && messageTemp == "0") // On allume en rouge
  {
    alarme_heures = 0;
    alarme_minutes = 0;
    setLedGreen();
  }
}

void onMqttPublish(uint16_t packetId)
{
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup()
{
  Serial.begin(115200);
  delay(3000);
  Serial.println("init");

  // init neopixel
  pixels.begin();
  pixels.clear();
  pixels.show();

  setLedConnectingMQTT();

  // init WIFI / MQTT
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  connectToWifi();
}

void loop()
{

  timeClient.update();
  Serial.println(timeClient.getFormattedTime());
  delay(1000);

  if(timeClient.getMinutes() == alarme_minutes && alarme_heures == timeClient.getHours()){
    Serial.println("FINI !");
    setLedGreen();
    alarme_heures =-1;
    alarme_minutes =-1;
  }

  /*
if(timer_minutes > 0){
  if(){

  }
}
*/
}