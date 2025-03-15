#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

#define DHT11_PIN  32 // ESP32 pin GPIO32 connected to DHT11 sensor

// Defina as credenciais da rede Wi-Fi
const char* ssid = "Vodafone-71C302";
const char* password = "JCb8TXb85x";

// Defina o endereço do broker MQTT
const char* mqtt_server = "broker.mqtt-dashboard.com";
const int mqtt_port = 1883; // Porta padrão para MQTT

WiFiClient espClient;
PubSubClient client(espClient);
DHT dht11(DHT11_PIN, DHT11);

// Buffer para armazenar dados como string
char tempCString[8];
char humiString[8];

// Função de callback que será chamada quando uma mensagem MQTT for recebida
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensagem recebida no tópico: ");
  Serial.print(topic);
  Serial.print(". Mensagem: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando-se a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.println("Endereço IP: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop até que estejamos conectados
  while (!client.connected()) {
    Serial.print("Tentando conexão MQTT...");
    // Tentativa de conexão
    if (client.connect("ESP32Client")) {
      Serial.println("conectado");
      // Subscreve ao tópico desejado
      // client.subscribe("IPB/Tese/a61450/Temperature");
    } else {
      Serial.print("falhou, rc=");
      Serial.print(client.state());
      Serial.println(" tentando novamente em 5 segundos");
      // Aguarda 5 segundos antes de tentar novamente
      delay(5000);
    }
  }
}

void readAndPublishDHT() {
  // Leitura da umidade
  float humi = dht11.readHumidity();
  // Leitura da temperatura em Celsius
  float tempC = dht11.readTemperature();
  // Leitura da temperatura em Fahrenheit
  float tempF = dht11.readTemperature(true);

  // Verifica se a leitura foi bem-sucedida
  if (isnan(tempC) || isnan(tempF) || isnan(humi)) {
    Serial.println("Falha ao ler do sensor DHT11!");
  } else {
    Serial.print("Umidade: ");
    Serial.print(humi);
    Serial.print("%  |  Temperatura: ");
    Serial.print(tempC);
    Serial.print("°C  ~  ");
    Serial.print(tempF);
    Serial.println("°F");

    // Converte a variável float para string
    dtostrf(tempC, 6, 2, tempCString);
    dtostrf(humi, 6, 2, humiString);

    // Publica a variável float convertida em string
    client.publish("IPB/Tese/a61450/Temperature", tempCString);
    client.publish("IPB/Tese/a61450/Humidity", humiString);
  }
}

void setup() {
  Serial.begin(115200);
  dht11.begin(); // Inicializa o sensor DHT11
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  // Leitura e publicação do DHT a cada 30 segundos
  static unsigned long lastDHTReadTime = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - lastDHTReadTime >= 30000) {
    lastDHTReadTime = currentMillis;
    readAndPublishDHT();
  }
}
