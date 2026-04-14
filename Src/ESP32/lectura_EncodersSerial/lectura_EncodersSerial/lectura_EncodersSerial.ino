#include <Arduino.h>
#include <ESP32Encoder.h>

// Cambia esto a false cuando vayas a conectar con LabVIEW
bool debugMode = true; 

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
  if(debugMode) Serial.println(">>> MOTORES FRENADOS");
}

// --- Tarea: Muestreo ---
void tareaMuestreo(void * parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriodo = pdMS_TO_TICKS(50); // Bajamos a 20Hz para que no sature la consola en debug
  
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

// --- Tarea: Escritura (Dual: Debug/Binario) ---
void WriteSerial(void * parameter){
  MsgDatos_t msg; 
  for(;;){
    if (xQueueReceive(colaTelemetria, &msg, portMAX_DELAY) == pdPASS) {
      if (debugMode) {
        // Formato legible para humanos
        int32_t pI, pD;
        memcpy(&pI, &msg.globalBuffer[0], 4);
        memcpy(&pD, &msg.globalBuffer[4], 4);
        Serial.printf("DEBUG | EncI: %d | EncD: %d | CRC: %d\n", pI, pD, msg.crcFinal);
      } else {
        // Formato binario para LabVIEW
        Serial.write(msg.header); 
        Serial.write(msg.globalBuffer, 8); 
        Serial.write(msg.byteReserva); 
        Serial.write(msg.crcFinal);
      }
    }
  }
}

// --- Tarea: Leer Comandos ---
void ReadSerial(void * parameter) {
  for (;;) {
    if (Serial.available() > 0) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim(); // Limpiar espacios o saltos de línea extra

      if (cmd.startsWith("MOV:")) {
        int firstColon = cmd.indexOf(':');
        int secondColon = cmd.indexOf(':', firstColon + 1);
        
        if (secondColon != -1) {
          int velI = cmd.substring(firstColon + 1, secondColon).toInt();
          int velD = cmd.substring(secondColon + 1).toInt();
          moverMotor(mtIzq, velI);
          moverMotor(mtDer, velD);
          if(debugMode) Serial.printf(">>> MOVIMIENTO: Izq=%d, Der=%d\n", velI, velD);
        }
      } else if (cmd == "STOP") {
        frenarRobot();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000); 

  mtIzq = {18, 19};
  mtDer = {21, 22};
  const int frecuencia = 5000;
  const int resolucion = 8;

  ledcAttach(mtIzq.IN_A, frecuencia, resolucion);
  ledcAttach(mtIzq.IN_B, frecuencia, resolucion);
  ledcAttach(mtDer.IN_A, frecuencia, resolucion);
  ledcAttach(mtDer.IN_B, frecuencia, resolucion);

 /* pinMode(13, INPUT_PULLUP);
  pinMode(25, INPUT_PULLUP);
  pinMode(26, INPUT_PULLUP);
  pinMode(27, INPUT_PULLUP);*/

  encoderIzq.attachHalfQuad(13, 25);
  encoderDer.attachHalfQuad(26, 27);
  
  colaTelemetria = xQueueCreate(20, sizeof(MsgDatos_t));

  xTaskCreatePinnedToCore(tareaMuestreo, "Muestreo", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(ReadSerial, "Read", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(WriteSerial, "Write", 4096, NULL, 1, NULL, 0);

  if(debugMode) Serial.println("SISTEMA LISTO - MODO DEBUG");
}

void loop() {
  vTaskDelete(NULL);
}