#include <Arduino.h>
#include <WiFi.h>
#include <DHT.h>
#include <PubSubClient.h>

#define DHT11_PIN  32 // GPIO32 para o sensor DHT11

// Credenciais WiFi
const char* WIFI_AP_SSID = "Vodafone-71C302";
const char* WIFI_AP_PASS = "JCb8TXb85x";

#define MQTT_SERVER "broker.mqtt-dashboard.com"
#define MQTT_PORT 1883
#define MQTT_TOPIC "IPB/Tese/a61450/data"

DHT dht(DHT11_PIN, DHT11);
WiFiClient espClient;
PubSubClient client(espClient);
String pendingMessage = "";

void connect_wifi() {
    Serial.print("Conectando ao WiFi...");
    WiFi.begin(WIFI_AP_SSID, WIFI_AP_PASS);
    
    int attempt = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        attempt++;
        if (attempt > 30) {
            Serial.println("\nFalha na conexão WiFi. Reiniciando...");
            ESP.restart();
        }
    }
    Serial.println("\nWiFi conectado!");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
}

void reconnectMQTT() {
    while (!client.connected()) {
        Serial.print("Tentando conectar ao MQTT...");
        if (client.connect("ESP32Client")) {
            Serial.println("Conectado ao broker MQTT");
            if (!pendingMessage.isEmpty()) {
                client.publish(MQTT_TOPIC, pendingMessage.c_str());
                Serial.println("Mensagem pendente publicada");
                pendingMessage = "";
            }
        } else {
            Serial.print("Falha, rc="); Serial.println(client.state());
            delay(5000); // Espera antes de tentar novamente
        }
    }
}

void sendMQTTData(float temperature, float humidity) {
    if (!client.connected()) {
        reconnectMQTT();
    }
    
    String payload = "{\"temperature\": " + String(temperature, 2) + ", \"humidity\": " + String(humidity, 2) + "}";
    Serial.print("Enviando MQTT: "); Serial.println(payload);
    
    if (client.publish(MQTT_TOPIC, payload.c_str())) {
        Serial.println("Mensagem enviada com sucesso");
    } else {
        Serial.println("Falha no envio MQTT, armazenando mensagem...");
        pendingMessage = payload;
    }
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    connect_wifi();
    
    client.setServer(MQTT_SERVER, MQTT_PORT);
    dht.begin();
}

void loop() {
    static uint32_t lastSendTime = 0;
    if (millis() - lastSendTime > 300000) { // 5 minutos
        float temperature = dht.readTemperature();
        float humidity = dht.readHumidity();
        
        if (isnan(temperature) || isnan(humidity)) {
            Serial.println("Falha ao ler do sensor DHT!");
            return;
        }
        
        sendMQTTData(temperature, humidity);
        lastSendTime = millis();
    }
    client.loop();
}
