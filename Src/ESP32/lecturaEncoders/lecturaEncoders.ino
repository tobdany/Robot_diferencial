#include <Arduino.h>
#include <ESP32Encoder.h>
#include <WiFi.h>

// --- Configuración WiFi ---
const char* ssid = "TU_SSID";
const char* password = "TU_PASSWORD";
WiFiServer server(5000); 
WiFiClient client;
bool wifiConectado = false; // Bandera para saber si tenemos red

// --- Estructuras --- 
struct Motor_t {
  int IN_A;
  int IN_B;
};

struct __attribute__((packed)) MsgDatos_t {
  uint8_t header = 0x3A;
  uint8_t globalBuffer[8]; 
  uint8_t byteReserva;
  uint8_t crcFinal;
};

// --- Variables Globales ---
Motor_t mtIzq, mtDer;
ESP32Encoder encoderIzq;
ESP32Encoder encoderDer;
QueueHandle_t colaTelemetria;

// --- Funciones de Control ---
void moverMotor(Motor_t &objMotor, int velocidad) {
  int velProtegida = constrain(velocidad, -255, 255);
  if (velProtegida < 0) {
    ledcWrite(objMotor.IN_A, abs(velProtegida)); 
    ledcWrite(objMotor.IN_B, 0);
  } else {
    ledcWrite(objMotor.IN_A, 0);
    ledcWrite(objMotor.IN_B, velProtegida); 
  }
}

void frenarRobot() {
  moverMotor(mtIzq, 0);
  moverMotor(mtDer, 0);
  Serial.println("[DEBUG] MOTORES FRENADOS");
}

// --- Tarea: Muestreo (CORE 1) ---
void tareaMuestreo(void * parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriodo = pdMS_TO_TICKS(10); 
  
  for (;;) {
    int32_t pasosIzq = (int32_t)encoderIzq.getCount();
    int32_t pasosDer = (int32_t)encoderDer.getCount();

    MsgDatos_t clusterLocalMsg;
    memcpy(&clusterLocalMsg.globalBuffer[0], &pasosIzq, 4);
    memcpy(&clusterLocalMsg.globalBuffer[4], &pasosDer, 4);
    clusterLocalMsg.byteReserva = 0;
  
    uint16_t sumaCrc = clusterLocalMsg.header;
    for(int i = 0; i < 8; i++) sumaCrc += clusterLocalMsg.globalBuffer[i];
    clusterLocalMsg.crcFinal = (uint8_t)(sumaCrc & 0xFF);

    xQueueSend(colaTelemetria, &clusterLocalMsg, 0);
    vTaskDelayUntil(&xLastWakeTime, xPeriodo);
  }
}

// --- Tarea: Comunicación TCP y Serial (CORE 0) ---
void tareaComunicacion(void * parameter){
  MsgDatos_t msg; 
  for(;;){
    // Solo intentamos manejar clientes si el WiFi se conectó en el setup
    if (wifiConectado) {
      if (!client || !client.connected()) {
        client = server.available();
      }
    }

    if (xQueueReceive(colaTelemetria, &msg, 10 / portTICK_PERIOD_MS) == pdPASS) {
      // DEBUG SERIAL SIEMPRE ACTIVO
      int32_t pI, pD;
      memcpy(&pI, &msg.globalBuffer[0], 4);
      memcpy(&pD, &msg.globalBuffer[4], 4);
      Serial.printf("DEBUG | I: %d | D: %d | CRC: %d\n", pI, pD, msg.crcFinal);

      // ENVIAR A LABVIEW SOLO SI HAY CLIENTE
      if (wifiConectado && client && client.connected()) {
        client.write((const uint8_t*)&msg, sizeof(MsgDatos_t));
      }
    }

    // Leer comandos de ambas fuentes (Serial siempre, WiFi opcional)
    if (Serial.available() > 0) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd.startsWith("MOV:")) {
          // ... lógica de parsing ...
          int firstColon = cmd.indexOf(':');
          int secondColon = cmd.indexOf(':', firstColon + 1);
          if (secondColon != -1) {
            int velI = cmd.substring(firstColon + 1, secondColon).toInt();
            int velD = cmd.substring(secondColon + 1).toInt();
            moverMotor(mtIzq, velI);
            moverMotor(mtDer, velD);
          }
      } else if (cmd == "STOP") frenarRobot();
    }

    if (wifiConectado && client && client.available() > 0) {
        String cmd = client.readStringUntil('\n');
        cmd.trim();
        if (cmd.startsWith("MOV:")) {
            int firstColon = cmd.indexOf(':');
            int secondColon = cmd.indexOf(':', firstColon + 1);
            if (secondColon != -1) {
              int velI = cmd.substring(firstColon + 1, secondColon).toInt();
              int velD = cmd.substring(secondColon + 1).toInt();
              moverMotor(mtIzq, velI);
              moverMotor(mtDer, velD);
            }
        } else if (cmd == "STOP") frenarRobot();
    }
    vTaskDelay(1);
  }
}

void setup() {
  Serial.begin(115200);

  // --- Intento de Conexión WiFi con Timeout ---
  WiFi.begin(ssid, password);
  Serial.print("Conectando WiFi");
  int timeoutCounter = 0;
  while (WiFi.status() != WL_CONNECTED && timeoutCounter < 20) { // 10 segundos
    delay(500);
    Serial.print(".");
    timeoutCounter++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[OK] WiFi Conectado!");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
    server.begin();
    wifiConectado = true;
  } else {
    Serial.println("\n[WARN] WiFi no encontrado. Iniciando en MODO LOCAL.");
    wifiConectado = false;
  }

  // --- Configuración Motores y Encoders ---
  mtIzq = {18, 19}; mtDer = {21, 22};
  ledcAttach(mtIzq.IN_A, 5000, 8); ledcAttach(mtIzq.IN_B, 5000, 8);
  ledcAttach(mtDer.IN_A, 5000, 8); ledcAttach(mtDer.IN_B, 5000, 8);

  pinMode(13, INPUT_PULLUP); pinMode(25, INPUT_PULLUP);
  pinMode(26, INPUT_PULLUP); pinMode(27, INPUT_PULLUP);

  encoderIzq.attachHalfQuad(13, 25);
  encoderDer.attachHalfQuad(26, 27);
  
  colaTelemetria = xQueueCreate(10, sizeof(MsgDatos_t));

  xTaskCreatePinnedToCore(tareaMuestreo, "Muestreo", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(tareaComunicacion, "Comms", 8192, NULL, 1, NULL, 0);
}

void loop() { vTaskDelete(NULL); }