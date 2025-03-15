/* ============================================================================
 *
 *   Code to Gateway
 *   This code went developer to last work in IPB - BRAGANÇA 
 * 
 *   Autor: Mateus Costa de Araujo 
 *   Data:  February, 2024
 *
============================================================================ */
// Lybrares
// #include <WiFi.h>
#include <Arduino.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFiManager.h> 
#include <esp_task_wdt.h>
#include <HardwareSerial.h>  // LoRa
//==========================================================================
// Defines
#define rxLoRa 16
#define txLoRA 17
#define led_to_receive 25
#define pin_led 12
//---------------------------------------------------------
// Wifi and MQTT
WiFiClient espClient;
int32_t rssi; // RSSI wifi
const char *mqttServer = "broker.mqtt-dashboard.com";
bool flag_mqtt = false; // confirmation flag package.
const short mqttPort = 1883;
const char *mqttUser = "IPB";
const char *mqttPassword = "noiottracker";
const char *topic = "IPB/TESTE/TRACKER/01";
const char *topic2 = "IPB/TESTE/GATWAY/01";
unsigned long  time_to_try_mqtt_reconnect = 10 * 1000;

PubSubClient client(espClient);
short time_to_resend_msg_status_gatway = 30000; // Time to resend gateway status (ms) (EDITABLE)
char *RSSI_LoRA = 0;

String jsonData = "";
String jsonStatus = "";       // (gateway status)
DynamicJsonDocument doc(256); // Package buffer
DynamicJsonDocument gat(64);  // gatway vuffer status
//---------------------------------------------------------
// Lora - Uart
HardwareSerial lora(1);
String incomingString;
char end_to_send = '1'; // Device Address
char markers[12]; // = "ABCDEFGHIJK";  // J -> end to string
char *data;                    // raw data
char extractedStrings[12][20]; // Vector to package
char utctime[25];              // vetor que vai receber time          ******
char latlon[2][10];
char last_data[] = "K";        // Character which packet is memory 
char find_gatway[] = "O";      // Character that indicates that the device is looking for the gateway
char type_data = 0;            //  [0] - current; [1] - memory
//---------------------------------------------------------
 
// FLOW
unsigned short break_line = 60000;        
unsigned short time_to_show_point = 1000; 
unsigned long time_break_line;          
unsigned long time_show_msg;            
//bool serialEnabled = true; // Variável de controle para a comunicação serial
//---------------------------------------------------------
// FREERTOS
static uint8_t taskCoreZero = 0;
static uint8_t taskCoreOne = 1;
//==========================================================================
// Functions
void LORA_SETTINGS();
void WIFI_SETTINGS();
void TASKS();
void zerar_extractedStrings(); 
void publishMQTTstatus (void *pcParameters);
void debug_led (void *pvParameters);
void processing(void *pvParameters);
void separate_lat_and_lon();
void zerar_extractedStrings();
void toggleSerial(bool enable);
void reconnect();
void send_test_json(); // Nova função para enviar o JSON de teste
//==========================================================================
void setup(){                   
  LORA_SETTINGS();
  WIFI_SETTINGS();      
  TASKS();
} 
//==========================================================================
void loop(){} 

void TASKS(){
  xTaskCreatePinnedToCore(
        publishMQTTstatus,   /* função que implementa a tarefa */
        "coreTaskOne", /* task name */
        2000,          /* words */
        NULL,          /* entrance parameter */
        1,             /* Priority */
        NULL,          /* Reference */
        taskCoreOne);  /* Core */

    delay(500); // tempo para a tarefa iniciar
  //------------------------------------
    xTaskCreatePinnedToCore(
        debug_led,     
        "coreTaskTwo", 
        1000,          
        NULL,          
        1,             
        NULL,          
        taskCoreZero);  
    delay(500); 
  //------------------------------------
    xTaskCreatePinnedToCore(
        processing,   
        "coreTaskThree", 
        10000,          
        NULL,          
        2,             
        NULL,          
        taskCoreOne); 
    delay(500);   
}

void publishMQTTstatus (void *pcParameters){
  while(true){
    if (!client.connected()){
      reconnect();
    }
    client.loop();           
    rssi = WiFi.RSSI();      
    gat["RSSI_WIFI"] = rssi; 

    jsonStatus = "";                
    serializeJson(gat, jsonStatus); 

    client.publish(topic2, jsonStatus.c_str());            
    Serial.print("\nSTATUS GATWAY ENVIADO - RSSI WIFI: "); 
    Serial.println(rssi);                                  
    vTaskDelay(time_to_resend_msg_status_gatway);         
  }
}

void debug_led (void *pvParameters){

  String taskMessage = "Task running on core ";
  taskMessage = taskMessage + xPortGetCoreID();

  int control = 0;
  while (true){
    if(WiFi.status() != WL_CONNECTED){
      digitalWrite(pin_led, HIGH);
      vTaskDelay(500);
      digitalWrite(pin_led, LOW);
    }
    else{
      digitalWrite(pin_led, LOW);
    }
    vTaskDelay(1000);
  }
}

void processing(void *pvParameters){
  String taskMessage = "Task running on core ";
  taskMessage = taskMessage + xPortGetCoreID();
  short i = 0;
  short n_try_send_package_mqtt = 3;
  while (true){
    //----------------------------------------------
    //reseta whatchdogs para indicar que o sistema está funcionando normalmente
    //esp_task_wdt_reset();
    //----------------------------------------------
    // mostra "." para indicar que está aguardando pacote LORA
    if (millis() - time_break_line >= break_line){
      // Reinicie o contador millis()
      Serial.print("\n"); 
      time_break_line = millis();
    }
    if (millis() - time_show_msg >= time_to_show_point){
      Serial.print(".");
      time_show_msg = millis();
    }
    if (lora.available()){

      Serial.println("\n========[PACOTE CHEGOU]=========="); // debug serial
      incomingString = lora.readString();
      Serial.println(incomingString);
      //----------------------------------------------
      toggleSerial(false); // Desliga a comunicação serial para não haver interrupções indesejadas
      //----------------------------------------------
      char dataArray[100];                        // vetor que vai receber string
      incomingString.toCharArray(dataArray, 100); // método para atribui string a vetor
      data = strtok(dataArray, ",");
      data = strtok(NULL, ",");
      data = strtok(NULL, ",");
      RSSI_LoRA = &*data; // para pegar o RSSI do sinal lora
      RSSI_LoRA = strtok(NULL, ",");
      //Serial.println(data);  // imprime pacote recebido (sem outros detalhes)

      //----------------------------------------------
      // Dividir data em pacotes
      type_data = 0; // tipo de dado pre estabelecido
      int i, j = 0, startPos = -1, count = 0;
      // conferir se há T no vetor

      for (i = 0; data[i] != '\0'; i++){
        if (strchr(find_gatway, data[i])){                // se device estver procurando o gatway
          type_data = 1;                                  // flag para indicar que o pacote é passado
          break;
        }
        if (strchr(last_data, data[i])){ // exsitir "Z" no pacote o dado é passado
          sprintf(markers, "ABCDEFGHIK"); // atribui K como ultimo caracter (depois do horario)
          type_data = 2;                  // flag para indicar que o pacote é passado
          break;
        }
        else{

          type_data = 3;                 // flag para pacote em tempo real
          sprintf(markers, "ABCDEFGHI"); // caso contrario o pacote é em tempo real
        }
      }
      //----------------------------------------------
      if (type_data == 1){
        Serial.println("TIPO -> [Device procurando gatway]");
      }
      else // debug
        if (type_data == 2){
          Serial.println("TIPO -> [Pacote da memoria]");
        }
        else // debug
          if (type_data == 3){
            Serial.println("TIPO -> [Pacote atual]");
          } // debug
      //----------------------------------------------
      // pré processamento - lógica para quebrar o pacote em partes se pacote requiser isso
      if (type_data == 1){
        int size_data = strlen(data); // confere o tamanho do pacote
        data[size_data - 1] = '\0';   // quebra o ultimo valor ( caractere "O")

        for (i = 0; i < size_data; i++){    // laço para preencger extractedStrings com a informação de interesse                
          extractedStrings[0][i] = data[i];   // Método de Preenchimento para cada string no array
          if (data[i] == 'T'){
            break; // quebra aqui se encontrar T
          }
        }
        Serial.println(extractedStrings[0]);
      }
      else if (type_data == 2){  // se dados virem em memória
        delay(10);               // para que não haja tempo de interrupção para coleta
        continue;                // 
      }
      //----------------------------------------------
      zerar_extractedStrings(); 
    }
  }
}
//==========================================================================

void zerar_extractedStrings(){
  for (int i = 0; i < 12; i++){
    for (int j = 0; j < 20; j++){
      extractedStrings[i][j] = '\0';
    }
  }
}

void toggleSerial(bool enable){
  if(enable){
    Serial.begin(115200);
  }
  else{
    Serial.end();
  }
}

void LORA_SETTINGS(){
  lora.begin(9600, SERIAL_8N1, rxLoRa, txLoRA);
  delay(2000);
  Serial.println("LoRa Configurado!");
}

void WIFI_SETTINGS(){
  WiFiManager wifiManager;
  wifiManager.autoConnect("AutoConnectAP");
  Serial.println("Conectado ao WiFi!");
}

void send_test_json(){
  // Criar um JSON de teste
  DynamicJsonDocument testDoc(256);
  
  testDoc["status"] = "connected";
  testDoc["device"] = "ESP32_Gateway";
  testDoc["timestamp"] = millis();
  testDoc["message"] = "Test message from gateway";

  // Serializa o JSON para enviar via MQTT
  String jsonString;
  serializeJson(testDoc, jsonString);

  // Enviar o JSON de teste para o tópico de teste MQTT
  if (client.connected()) {
    if (client.publish(topic2, jsonString.c_str())) {
      Serial.println("Test JSON sent successfully");
    } else {
      Serial.println("Failed to send test JSON");
    }
  } else {
    Serial.println("MQTT not connected. Cannot send test JSON");
  }
}

void reconnect(){
  short cont_to_reset = 0;
  unsigned long now = millis();
  while (!client.connected()){
    client.setServer(mqttServer, mqttPort);
    Serial.println("\nConectando ao broker MQTT...");

    if (client.connect("ESP32_Gateway_IPB_Tracker", mqttUser, mqttPassword)){
      Serial.println("Conectado");
      send_test_json();  // Enviar o JSON de teste após conectar
      break;
    }
    else{
      Serial.print("Falha na conexão - Estado: ");
      Serial.print(client.state());
      Serial.println(" Tentando novamente");
      cont_to_reset++;
      if (cont_to_reset >= 2){
        Serial.println("Não foi possível reconectar ao Broker, reiniciando sistema");
        Serial.println("=======================================");
        ESP.restart();
      }
    }
    delay(80);
    if(now >= now + time_to_try_mqtt_reconnect) break;
  }
}

