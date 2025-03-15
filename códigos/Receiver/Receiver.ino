#include <esp_now.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

#define DHT11_PIN  32 // ESP32 pin GPIO32 connected to DHT11 sensor

// Define Wi-Fi credentials
const char* ssid = "Vodafone-71C302";
const char* password = "JCb8TXb85x";

// Define MQTT broker details
const char* mqtt_server = "broker.mqtt-dashboard.com";
const int mqtt_port = 1883; // Default MQTT port

WiFiClient espClient;
PubSubClient client(espClient);

// Buffer to store data as string
char tempCString[8];
char humiString[8];

typedef struct struct_message {
  float temp;
  float hum;
} struct_message;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;

// MQTT callback function
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensagem recebida no tópico: ");
  Serial.print(topic);
  Serial.print(". Mensagem: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

// Function to setup Wi-Fi connection
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando-se a ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA); // Set Wi-Fi to station mode
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.println("Endereço IP: ");
  Serial.println(WiFi.localIP());

  // Set ESP-NOW channel to match Wi-Fi channel
  int channel = WiFi.channel();
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
}

// Function to reconnect to MQTT broker
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Tentando conexão MQTT...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("conectado");
      // Subscribe to the desired topic
      // client.subscribe("IPB/Tese/a61450/Temperature");
    } else {
      Serial.print("falhou, rc=");
      Serial.print(client.state());
      Serial.println(" tentando novamente em 5 segundos");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// Callback when data is received via ESP-NOW
void OnDataRecv(const esp_now_recv_info *esp_now_info, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes recebidos: ");
  Serial.println(len);

  // Convert float variable to string
  dtostrf(incomingReadings.temp, 6, 2, tempCString);
  dtostrf(incomingReadings.hum, 6, 2, humiString);

  // Publish the float variable converted to string
  Serial.print("Publicando Temperatura: ");
  Serial.println(tempCString);
  if (client.publish("IPB/Tese/a61450/Temperature", tempCString)) {
    Serial.println("Publicação de temperatura bem-sucedida");
  } else {
    Serial.println("Falha na publicação de temperatura");
  }

  Serial.print("Publicando Umidade: ");
  Serial.println(humiString);
  if (client.publish("IPB/Tese/a61450/Humidity", humiString)) {
    Serial.println("Publicação de umidade bem-sucedida");
  } else {
    Serial.println("Falha na publicação de umidade");
  }
}

void setup() {
  Serial.begin(115200);

  // Setup Wi-Fi
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
