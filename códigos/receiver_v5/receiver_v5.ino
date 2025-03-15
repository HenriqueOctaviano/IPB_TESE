#include <esp_now.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_wifi.h>
#include <DHT.h>
#include <ctime>

#define DHT11_PIN 32
#define MQTT_SERVER "broker.mqtt-dashboard.com"
#define MQTT_PORT 1883
#define MQTT_TOPIC "IPB/Tese/a61450/data"

constexpr uint8_t MAC_ADDR1[] = {0x10, 0x06, 0x1C, 0x97, 0x9C, 0x34};
constexpr uint8_t MAC_ADDR2[] = {0x10, 0x06, 0x1C, 0x98, 0x51, 0x04};

typedef struct struct_message {
  float temp;
  float hum;
} struct_message;

struct_message incomingReadings;
WiFiClient espClient;
PubSubClient client(espClient);
DHT dht11(DHT11_PIN, DHT11);
bool newDataAvailable = false;
uint8_t incomingMacAddr[6];
String messageBuffer = "";
String pendingMessage = ""; // Armazena a mensagem pendente
unsigned long lastReceivedTime = 0;
const unsigned long delayTime = 60000;  // 60 segundos

// Função para configurar e sincronizar o horário com o NTP
void setupTime() {
    configTime(0, 0, "pool.ntp.org"); // Configura o NTP
    Serial.println("Waiting for NTP time sync...");

    time_t now = time(nullptr);
    int attempt = 0;
    while (now < 24 * 3600) { // Espera até que o horário seja sincronizado
        delay(500);
        now = time(nullptr);
        attempt++;
        if (attempt > 30) { // Tenta por 15 segundos
            Serial.println("Failed to sync NTP time. Restarting...");
            ESP.restart();
        }
    }

    Serial.println("NTP time synchronized!");
}

void OnDataRecv(const esp_now_recv_info *esp_now_info, const uint8_t *incomingData, int len) {
    if (len == sizeof(incomingReadings)) {
        memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
        memcpy(incomingMacAddr, esp_now_info->src_addr, 6);
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

void setup_wifi() {
    Serial.print("Connecting to WiFi...");
    WiFi.begin("Vodafone-71C302", "JCb8TXb85x");
    
    int attempt = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        attempt++;
        if (attempt > 30) {  // Tenta por 15 segundos
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

void readLocalDHT11() {
    float temp = dht11.readTemperature();
    float hum = dht11.readHumidity();

    if (isnan(temp) || isnan(hum)) {
        Serial.println("Failed to read from DHT11 sensor!");
        return;
    }

    Serial.print("Local DHT11 - Temp: "); Serial.print(temp);
    Serial.print(" °C, Hum: "); Serial.println(hum);

    if (!messageBuffer.isEmpty()) messageBuffer += " | ";
    messageBuffer += "ML, Temp: " + String(temp, 2) + "°C, Hum: " + String(hum, 2) + "%";
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
        } else {
            Serial.println("MQTT Publish failed, storing message...");
            pendingMessage = messageBuffer; // Armazena a mensagem pendente
            messageBuffer = "";
        }
    }
}

void setup() {
    Serial.begin(115200);
    dht11.begin();
    setup_wifi();
    setupTime(); // Configura e sincroniza o horário com o NTP
    client.setServer(MQTT_SERVER, MQTT_PORT);
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi disconnected, reconnecting...");
        setup_wifi();
    }

    if (!client.connected()) reconnectMQTT();
    client.loop();

    if (newDataAvailable && millis() - lastReceivedTime >= delayTime) {
        readLocalDHT11();  // Adiciona leitura do DHT11 local antes de enviar
        sendMQTTData();
        newDataAvailable = false;
    }
}