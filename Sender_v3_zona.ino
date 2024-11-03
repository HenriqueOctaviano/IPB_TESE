#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <DHT.h>

#define DHT11_PIN  32 // GPIO32 for DHT11 sensor

// Define ESP-NOW receiver address
constexpr uint8_t ESP_NOW_RECEIVER[] = {0x10, 0x06, 0x1C, 0x97, 0x9C, 0x3C};
  
// WiFi credentials
const char* WIFI_AP_SSID = "Vodafone-71C302";
const char* WIFI_AP_PASS = "JCb8TXb85x";

// Global variables
esp_now_peer_info_t peerInfo;
uint8_t channel = 1;

DHT dht(DHT11_PIN, DHT11);

void connect_esp32_wifi_network(const char* ssid, const char* password) {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.print("WiFi channel: ");
  Serial.println(WiFi.channel());
}

void initEspNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW failed to initialize");
    while (1);
  }

  memcpy(peerInfo.peer_addr, ESP_NOW_RECEIVER, 6);
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ESP-NOW pairing failure");
    while (1);
  }

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
}

void tryNextChannel() {
  channel = (channel % 13) + 1;
  Serial.print("Changing channel to ");
  Serial.println(channel);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
}

void onDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("Delivery failed. Trying next channel...");
    tryNextChannel();
  } else {
    Serial.println("Delivery successful!");
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(WIFI_PS_NONE);
  connect_esp32_wifi_network(WIFI_AP_SSID, WIFI_AP_PASS);
  esp_wifi_set_ps(WIFI_PS_NONE);
  
  dht.begin();
  initEspNow();
}

void loop() {
  static uint32_t lastSendTime = 0;
  if (millis() - lastSendTime > 60000) {  // 60 seconds = 60000 milliseconds
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    
    // Create a struct to send
    struct {
      float temp;
      float hum;
    } data = { temperature, humidity };

    esp_now_send(ESP_NOW_RECEIVER, (uint8_t*)&data, sizeof(data));
    Serial.printf("Sent Temperature: %.2f, Humidity: %.2f on channel: %u\n", temperature, humidity, WiFi.channel());
    lastSendTime = millis();
  }
}

