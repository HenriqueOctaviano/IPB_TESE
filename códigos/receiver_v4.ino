#include <esp_now.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_wifi.h>
#include <DHT.h>

#define DHT11_PIN 32 // ESP32 pin GPIO32 connected to DHT11 sensor

// MQTT server configuration
#define MQTT_SERVER "broker.mqtt-dashboard.com"
#define MQTT_PORT 1883

// MQTT topics for each sender
#define MQTT_TOPIC1 "IPB/Tese/a61450/dataz1"
#define MQTT_TOPIC2 "IPB/Tese/a61450/dataz2"
#define MQTT_TOPIC3 "IPB/Tese/a61450/dataz3"
#define MQTT_TOPIC4 "IPB/Tese/a61450/dataz4"
#define MQTT_TOPIC5 "IPB/Tese/a61450/dataz5"  // Local DHT11 sensor

// MAC addresses of the senders
constexpr uint8_t MAC_ADDR1[] = {0x10, 0x06, 0x1C, 0x97, 0x9C, 0x34}; // Sender 1
constexpr uint8_t MAC_ADDR2[] = {0x10, 0x06, 0x1C, 0x98, 0x51, 0x04}; // Sender 2
constexpr uint8_t MAC_ADDR3[] = {0xCC, 0x7B, 0x5C, 0x9A, 0xF2, 0xB4}; // Sender 3
constexpr uint8_t MAC_ADDR4[] = {0x, 0x, 0x, 0x, 0x, 0x}; // Sender 4

// Define the struct for received data
typedef struct struct_message {
  float temp;
  float hum;
} struct_message;

struct_message incomingReadings;
WiFiClient espClient;
PubSubClient client(espClient);

// Initialize DHT sensor
DHT dht11(DHT11_PIN, DHT11);

// Flag to indicate when new data has been received
bool newDataAvailable = false;
uint8_t incomingMacAddr[6];

void OnDataRecv(const esp_now_recv_info *esp_now_info, const uint8_t *incomingData, int len) {
  if (len == sizeof(incomingReadings)) {
    memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
    memcpy(incomingMacAddr, esp_now_info->src_addr, 6);
    Serial.print("Received data from MAC: ");
    for (int i = 0; i < 6; i++) {
      Serial.printf("%02X", incomingMacAddr[i]);
      if (i < 5) Serial.print(":");
    }
    Serial.println();
    newDataAvailable = true; // Set flag to indicate new data is available
  } else {
    Serial.println("Data length mismatch");
  }
}

void setup_wifi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin("Vodafone-71C302", "JCb8TXb85x");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  dht11.begin();
  
  // Setup Wi-Fi
  setup_wifi();

  // Scan for the network to find its channel
  int chan = 1; // Default channel if network is not found
  int32_t n = WiFi.scanNetworks();
  for (uint8_t i = 0; i < n; i++) {
    if (strcmp("Vodafone-71C302", WiFi.SSID(i).c_str()) == 0) {
      chan = WiFi.channel(i);
      break;
    }
  }

  // Set Wi-Fi channel
  esp_wifi_set_channel(chan, WIFI_SECOND_CHAN_NONE);
  Serial.print("WiFi channel set to: ");
  Serial.println(chan);

  // Setup MQTT client
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback([](char* topic, byte* payload, unsigned int length) {}); // Empty callback

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}

// Function to read and publish DHT11 sensor data
void readAndSendDHTData() {
  float hum5 = dht11.readHumidity();
  float temp5 = dht11.readTemperature();

  if (isnan(temp5) || isnan(hum5)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  Serial.print("Local DHT11 Sensor - Temp: ");
  Serial.print(temp5);
  Serial.print(" °C, Hum: ");
  Serial.println(hum5);

  if (!client.connected()) {
    reconnect();
  }
  
  char DataZ5[50];
  snprintf(DataZ5, sizeof(DataZ5), "{\"temperature\":%.2f,\"humidity\":%.2f}", temp5, hum5);
  
  if (client.publish(MQTT_TOPIC5, DataZ5)) {
    Serial.println("Local data published to MQTT_TOPIC5 successfully");
  } else {
    Serial.print("Failed to publish local data to MQTT_TOPIC5, error code: ");
    Serial.println(client.state());
  }
}

void loop() {
  // Handle incoming ESP-NOW data and publish to the corresponding MQTT topic
  if (newDataAvailable) {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();

    char data[50];
    snprintf(data, sizeof(data), "{\"temperature\":%.2f,\"humidity\":%.2f}", incomingReadings.temp, incomingReadings.hum);
    
    if (memcmp(incomingMacAddr, MAC_ADDR1, 6) == 0) {
      if (client.publish(MQTT_TOPIC1, data)) {
        Serial.println("Data from Sender 1 published to MQTT_TOPIC1 successfully");
      }
    } else if (memcmp(incomingMacAddr, MAC_ADDR2, 6) == 0) {
      if (client.publish(MQTT_TOPIC2, data)) {
        Serial.println("Data from Sender 2 published to MQTT_TOPIC2 successfully");
      }
    } else if (memcmp(incomingMacAddr, MAC_ADDR3, 6) == 0) {
      if (client.publish(MQTT_TOPIC3, data)) {
        Serial.println("Data from Sender 3 published to MQTT_TOPIC3 successfully");
      }
    } else if (memcmp(incomingMacAddr, MAC_ADDR4, 6) == 0) {
      if (client.publish(MQTT_TOPIC4, data)) {
        Serial.println("Data from Sender 4 published to MQTT_TOPIC4 successfully");
      }
    } else {
      Serial.println("Unknown sender");
    }

    newDataAvailable = false; // Reset flag after sending data
  }
  
  // Read and publish local DHT11 sensor data every 30 seconds
  static unsigned long lastDHTRead = 0;
  if (millis() - lastDHTRead > 30000) { // 30 seconds interval
    readAndSendDHTData();
    lastDHTRead = millis();
  }
}
