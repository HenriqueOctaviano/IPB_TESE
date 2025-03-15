#include <esp_now.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_wifi.h>  // Include esp_wifi.h to use esp_wifi_set_channel

#define MQTT_SERVER "broker.mqtt-dashboard.com"
#define MQTT_PORT 1883
#define MQTT_TOPIC "IPB/Tese/a61450/dataz1"


// Define the struct for received data
typedef struct struct_message {
  float temp1;
  float hum1;
} struct_message;


struct_message incomingReadings;
WiFiClient espClient;
PubSubClient client(espClient);

// Flag to indicate when new data has been received
bool newDataAvailable = false;

// Callback when data is received via ESP-NOW
void OnDataRecv(const esp_now_recv_info *esp_now_info, const uint8_t *incomingData, int len) {
  if (len == sizeof(incomingReadings)) {
    memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
    Serial.print("Temperature: ");
    Serial.println(incomingReadings.temp1);
    Serial.print("Humidity: ");
    Serial.println(incomingReadings.hum1);
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

void loop() {
  if (newDataAvailable) {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();

    // Format and publish data
    char DataZ1[50];
  
    snprintf(DataZ1, sizeof(DataZ1), "{\"temperature\":%.2f,\"humidity\":%.2f}", incomingReadings.temp1, incomingReadings.hum1);

    
    if (client.publish(MQTT_TOPIC, DataZ1)) {
      Serial.println("Data published successfully");
    } else {
      Serial.println("Failed to publish data");
    }

    newDataAvailable = false; // Reset flag after sending data
  }

}
