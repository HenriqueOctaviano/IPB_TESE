//Libraries for LoRa
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <PubSubClient.h>
#include <ctime>  // Include time library

//Libraries for OLED Display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// WiFi credentials
const char* WIFI_AP_SSID = "Vodafone-71C302";
const char* WIFI_AP_PASS = "JCb8TXb85x";

#define MQTT_SERVER "broker.mqtt-dashboard.com"
#define MQTT_PORT 1883
#define MQTT_TOPIC "IPB/Tese/a61450/data"

// Global variables
esp_now_peer_info_t peerInfo;
uint8_t channel = 1;
constexpr uint8_t MAC_ADDR1[] = {0x10, 0x06, 0x1C, 0x97, 0x9C, 0x34};
constexpr uint8_t MAC_ADDR2[] = {0x10, 0x06, 0x1C, 0x98, 0x51, 0x04};

// Define structure for received messages
struct struct_message {
    float temp;
    float hum;
};

//define the pins used by the LoRa transceiver module
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

//433E6 for Asia
//866E6 for Europe
//915E6 for North America
#define BAND 866E6

//OLED pins
#define OLED_SDA 4
#define OLED_SCL 15 
#define OLED_RST 16
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

//packet counter
int counter = 0;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

struct_message incomingReadings;
uint8_t incomingMacAddr[6];
bool newDataAvailable = false;
WiFiClient espClient;
PubSubClient client(espClient);
String messageBuffer = "";
String pendingMessage = ""; // Armazena a mensagem pendente
unsigned long lastReceivedTime = 0;
const unsigned long delayTime = 60000;  // 60 segundos

void connect_esp32_wifi_network() {
    Serial.println("Connecting to WiFi...");
    WiFi.begin(WIFI_AP_SSID, WIFI_AP_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected!");
    Serial.print("IP Address: "); Serial.println(WiFi.localIP());
    Serial.print("MAC Address: "); Serial.println(WiFi.macAddress());
    Serial.print("WiFi channel: "); Serial.println(WiFi.channel());
}

void initEspNow() {
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW initialization failed!");
        return;
    }
    esp_now_register_recv_cb(OnDataRecv);
    
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, MAC_ADDR1, 6); // Use valid MAC
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
    }
}

void OnDataRecv(const uint8_t *macAddr, const uint8_t *incomingData, int len) {
    if (len == sizeof(incomingReadings)) {
        memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
        memcpy(incomingMacAddr, macAddr, 6);
        newDataAvailable = true;
        lastReceivedTime = millis();

        String macID = "UNKNOWN";
        if (memcmp(incomingMacAddr, MAC_ADDR1, 6) == 0) macID = "M1";
        else if (memcmp(incomingMacAddr, MAC_ADDR2, 6) == 0) macID = "M2";
        
        Serial.print("Received from "); Serial.print(macID);
        Serial.print(" | Temp: "); Serial.print(incomingReadings.temp);
        Serial.print(" °C, Hum: "); Serial.println(incomingReadings.hum);

        if (!messageBuffer.isEmpty()) messageBuffer += " | ";
        messageBuffer += macID + ", Temp: " + String(incomingReadings.temp, 2) + "°C, Hum: " + String(incomingReadings.hum, 2) + "%";
    }
}

void setupTime() {
    configTime(0, 0, "pool.ntp.org"); // Set NTP server
    Serial.println("Waiting for NTP time sync...");

    time_t now = time(nullptr);
    int attempt = 0;
    while (now < 24 * 3600) { // Wait until time updates from NTP
        delay(500);
        now = time(nullptr);
        Serial.print(".");
        attempt++;
        if (attempt > 30) {  // Timeout after ~15 seconds
            Serial.println("\nFailed to sync NTP time. Restarting...");
            ESP.restart();
        }
    }
    Serial.println("\nNTP time synchronized!");
}

void setup_wifi() {
    Serial.print("Connecting to WiFi...");
    WiFi.begin(WIFI_AP_SSID, WIFI_AP_PASS);

    int attempt = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        attempt++;
        if (attempt > 30) {  // Timeout after ~15 seconds
            Serial.println("\nWiFi connection failed. Restarting...");
            ESP.restart();
        }
    }
    Serial.println("\nWiFi connected");
    Serial.print("IP Address: "); Serial.println(WiFi.localIP());
}

void reconnectMQTT() {
    if (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect("ESP32Client")) {
            Serial.println("Connected to MQTT broker");

            // Tenta enviar a mensagem pendente assim que a conexão for restabelecida
            if (!pendingMessage.isEmpty()) {
                Serial.println("Sending pending message...");
                if (client.publish(MQTT_TOPIC, pendingMessage.c_str())) {
                    Serial.println("Pending message published successfully");
                    pendingMessage = ""; // Limpa a mensagem pendente após o envio
                } else {
                    Serial.println("Failed to send pending message");
                }
            }
        } else {
            Serial.print("Failed, rc="); Serial.println(client.state());
        }
    }
}

void sendMQTTData() {
    if (!messageBuffer.isEmpty()) {
        time_t now = time(nullptr);
        struct tm *timeinfo = localtime(&now);
        char timeStr[20];
        strftime(timeStr, sizeof(timeStr), "%d/%m/%Y %H:%M:%S", timeinfo);
        messageBuffer += " | Timestamp: ";
        messageBuffer += timeStr;

        Serial.print("Publishing to MQTT: "); Serial.println(messageBuffer);
        
        if (client.publish(MQTT_TOPIC, messageBuffer.c_str())) {
            Serial.println("Message published successfully");
            messageBuffer = "";
            counter++;

        } else {
            Serial.println("MQTT Publish failed, storing message...");
            pendingMessage = messageBuffer; // Armazena a mensagem pendente
            messageBuffer = "";
        }
    }
}

void setup() {

  //initialize Serial Monitor
  Serial.begin(115200);
  delay(20);
  
  //reset OLED display via software
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);

  //initialize Serial Monitor (again, as in the working version)
  Serial.begin(115200);
  delay(20);

  //initialize OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  setup_wifi();
  setupTime();
  initEspNow();

  // Scan for the network to find its channel
  int chan = 1; // Default channel if network is not found
  int32_t n = WiFi.scanNetworks();
  for (uint8_t i = 0; i < n; i++) {
    if (strcmp("Vodafone-71C302", WiFi.SSID(i).c_str()) == 0) {
      chan = WiFi.channel(i);
      break;
    }
  }
  
  client.setServer(MQTT_SERVER, MQTT_PORT);
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    esp_now_register_recv_cb(OnDataRecv);

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("LORA SENDER ");
  display.display();
  
  Serial.println("LoRa Sender Test");

  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);
  
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Initializing OK!");
  display.setCursor(0,10);
  display.print("LoRa Initializing OK!");
  display.display();
  delay(2000);
}

void updateDisplay() {

  time_t now = time(nullptr);
  struct tm *timeinfo = localtime(&now);
  char timeStr[20];
  strftime(timeStr, sizeof(timeStr), "%H:%M:%S", timeinfo);  // Format as HH:MM:SS

  // Update OLED display
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("LORA SENDER");
  display.setCursor(0, 20);
  display.setTextSize(1);
  display.print("Time: ");
  display.print(timeStr);
  display.setCursor(0, 30);
  display.print("Counter: ");
  display.print(counter);
  display.display();

}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
       Serial.println("WiFi disconnected, reconnecting...");
       setup_wifi();
  }

    if (!client.connected()) reconnectMQTT();
    client.loop();

    if (newDataAvailable && millis() - lastReceivedTime >= delayTime) {
        sendMQTTData();
        newDataAvailable = false;
    }

  //Serial.print("Sending packet: ");
  //Serial.println(counter);
  
  // Send LoRa packet
  LoRa.beginPacket();
  LoRa.print("Counter: ");
  LoRa.print(counter);
  LoRa.endPacket();

  // Update OLED with time and counter
  updateDisplay();

  //delay(10000); 
}
