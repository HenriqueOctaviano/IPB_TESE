#include <Arduino.h>
#include <WiFi.h>
#include <DHT.h>
#include <PubSubClient.h>

#define DHT11_PIN  32 // GPIO32 para o sensor DHT11

// Credenciais WiFi
const char* WIFI_AP_SSID = "agents";
const char* WIFI_AP_PASS = "QgC9O8VucAByqvVu5Rruv1zdpqM66cd23KG4ElV7vZiJND580bzYvaHqz5k07G2";

// Configuração do MQTT
#define MQTT_SERVER "broker.emqx.io"
#define MQTT_PORT 1883
#define MQTT_TOPIC "IPB/Tese/a61450/data"

// Inicialização do DHT11 e MQTT
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
        if (attempt > 30) {  // Se demorar mais de 15s, reinicia o ESP32
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

        // Se o WiFi caiu, reconectar
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi desconectado, tentando reconectar...");
            connect_wifi();
        }

        if (client.connect("ESP32Client")) {
            Serial.println("Conectado ao broker MQTT!");
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

    String payload = "M1, Temp: " + String(temperature, 2) + "°C, Hum: " + String(humidity, 2) + "%";
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
    dht.begin();
    client.setServer(MQTT_SERVER, MQTT_PORT);
}

void loop() {
    static uint32_t lastSendTime = 0;

    // Mantém o MQTT ativo
    if (!client.connected()) {
        reconnectMQTT();
    }
    client.loop();

    // Enviar dados a cada 30 segundos
    if (millis() - lastSendTime > 30000) {
        float temperature = dht.readTemperature();
        float humidity = dht.readHumidity();
        
        if (isnan(temperature) || isnan(humidity)) {
            Serial.println("Falha ao ler do sensor DHT!");
            return;
        }
        
        sendMQTTData(temperature, humidity);
        lastSendTime = millis();
    }
}
