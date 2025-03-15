#include <Wire.h>               // Biblioteca para I2C
#include <Adafruit_Sensor.h>     // Biblioteca base para sensores
#include <Adafruit_ADXL345_U.h>  // Biblioteca para o ADXL345
#include <TinyGPS++.h>           // Biblioteca do GPS (TinyGPS++)

// Definindo pinos para o GPS (RX/TX)
#define RX_PIN 16
#define TX_PIN 17

// Criando o objeto do sensor ADXL345
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Criando o objeto GPS
TinyGPSPlus gps;

void setup() {
  // Inicializa a comunicação serial para o monitor serial
  Serial.begin(115200);        // Monitor serial com baud rate de 115200
  Wire.begin();                // Inicializando o barramento I2C
  
  // Inicializando o ADXL345
  if (!accel.begin()) {
    Serial.println("Não foi possível encontrar o ADXL345. Verifique a conexão.");
    while (1); // Se não encontrar o sensor, para o programa aqui
  }

  // Inicializando o GPS (com baud rate de 9600)
  Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);  // GPS com Serial1 (9600 bps)
  
  // Configurando o sensor ADXL345
  accel.setRange(ADXL345_RANGE_2_G);  // Definindo a faixa de ±2g
  accel.setDataRate(ADXL345_DATARATE_100_HZ); // Definindo a taxa de dados para 100 Hz
  
  Serial.println("Inicialização completa.");
}

void loop() {
  // Exibindo dados do GPS
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
    
    if (gps.location.isUpdated()) {
      Serial.println("=== Dados do GPS ===");
      Serial.print("Latitude: ");
      Serial.print(gps.location.lat(), 6);
      Serial.print(" Longitude: ");
      Serial.println(gps.location.lng(), 6);
    }
  }

  // Exibindo dados do ADXL345
  sensors_event_t event;
  accel.getEvent(&event);
  
  Serial.println("=== Dados do Acelerômetro ADXL345 ===");
  Serial.print("Aceleração X: ");
  Serial.print(event.acceleration.x);
  Serial.print(" m/s², Y: ");
  Serial.print(event.acceleration.y);
  Serial.print(" m/s², Z: ");
  Serial.print(event.acceleration.z);
  Serial.println(" m/s²");

  // Pausa para o próximo ciclo de leitura
  delay(1000); // Pausa de 1 segundo entre as leituras
}

