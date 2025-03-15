#include <esp_now.h>
#include <WiFi.h>
#include <DHT.h>

#define DHT11_PIN 32 // ESP32 pin GPIO32 connected to DHT11 sensor
uint8_t broadcastAddress[] = {0x10, 0x06, 0x1C, 0x97, 0x9C, 0x3C};

// Define the structure for sending data
typedef struct struct_message {
  float temp;
  float hum;
} struct_message;

struct_message Readings;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// Initialize DHT sensor
DHT dht11(DHT11_PIN, DHT11);

// Setup ESP-NOW and WiFi
void setup() {
  Serial.begin(115200);
  dht11.begin();

  WiFi.mode(WIFI_STA);
  Serial.println("Setting up ESP-NOW");

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("Peer added successfully");
}

// Function to read sensor data
void readAndSendData() {
  float humi = dht11.readHumidity();
  float tempC = dht11.readTemperature();

  if (isnan(tempC) || isnan(humi)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  Readings.temp = tempC;
  Readings.hum = humi;

  Serial.print("Sending data: Temp: ");
  Serial.print(Readings.temp);
  Serial.print(" °C, Hum: ");
  Serial.println(Readings.hum);

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Readings, sizeof(Readings));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }
}

void loop() {
  readAndSendData();
  delay(30000); // Send every 30 seconds
}
