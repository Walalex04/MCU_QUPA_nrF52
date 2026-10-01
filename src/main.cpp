#include <Arduino.h>
#include <Wire.h>
#include <cstdint>
#include <cstring>
#include <FreeRTOS.h>

#include "Protocol.h"
#include "Hw617IRSensor.h"
#include "MPU9250.h"
#include "Hw171Manager.h"
#include "DriverPololu.h"
#include "Protocol.h"


static PROTOCOLUART_SEND dataSensor;        //store fixed in RAM

/*
// ============================================================================
// DEFINICIONES DE DIRECCIONES I2C Y PINES
// ============================================================================
// 1. Expansor Digital HW-171 (PCF8574)
#define PCF8574_ADDR        0x20  // Dirección I2C por defecto (A0,A1,A2 a GND)

// Máscaras de Bits del PCF8574 para el TB6612FNG
#define BIT_AIN1            (1 << 0) // P0
#define BIT_AIN2            (1 << 1) // P1
#define BIT_BIN1            (1 << 2) // P2
#define BIT_BIN2            (1 << 3) // P3
#define BIT_STBY            (1 << 4) // P4

// 2. Multiplexor HW-617 (TCA9548A)
#define TCA9548A_ADDR       0x70  
#define NUM_CANALES_IR      8

// 3. Sensor IR de Distancia Sharp GP2Y0E03
#define GP2Y0E03_ADDR       0x40  
#define REG_DISTANCIA_HIGH  0x5E  

// 4. MPU9250 Registros I2C directos
#define MPU9250_ADDR        0x68  // Cambiar a 0x69 si AD0 está conectado a VCC
#define MPU9250_PWR_MGMT_1  0x6B
#define MPU9250_ACCEL_XOUT_H 0x3B

// Escalas por defecto del MPU9250 (±2g y ±250 deg/s)
const float ACCEL_SCALE = 16384.0f; // LSB/g
const float GYRO_SCALE  = 131.0f;   // LSB/(deg/s)

// 5. PWM de Motores directos desde la XIAO nRF52840
const uint8_t PIN_PWMA = D0;  // PWM para Motor A
const uint8_t PIN_PWMB = D1;  // PWM para Motor B

// 6. Asignación de Pines para Encoders Pololu (Pines con Interrupción Externa)
const uint8_t PIN_ENC_A_PHASEA = D2; // Canal A - Motor A
const uint8_t PIN_ENC_A_PHASEB = D3; // Canal B - Motor A
const uint8_t PIN_ENC_B_PHASEA = D4; // Canal A - Motor B
const uint8_t PIN_ENC_B_PHASEB = D5; // Canal B - Motor B

// Constante de reducción del motor Pololu (Ejemplo: 12 CPR * caja 75.81:1 ≈ 909.72 CPR)
const float TICKS_POR_REVOLUCION = 909.72f;

// Variables globales
uint16_t distancias_IR[NUM_CANALES_IR];
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float temp = 0; // Variable para almacenar la temperatura
uint8_t estadoPinosPCF = 0x00; // Estado del puerto del HW-171

// Variables globales para Encoders (Atómicas para ISR)
volatile long contadorPulsosMotorA = 0;
volatile long contadorPulsosMotorB = 0;

// Variables para cálculo de RPM
long pulsosAnterioresA = 0;
long pulsosAnterioresB = 0;
float rpmMotorA = 0.0f;
float rpmMotorB = 0.0f;

// Consignas de control para simulación opcional en seco
int16_t consignaPWMA = 0;
int16_t consignaPWMB = 0;

// Temporizador de muestreo rápido
unsigned long ultimoMuestreoMs = 0;
const unsigned long INTERVALO_MUESTREO_MS = 20; // ~50 Hz de tasa de refresco

// ============================================================================
// RUTINAS DE INTERRUPCIÓN (ISR)
// ============================================================================
void ISR_Encoder_MotorA() {
  if (digitalRead(PIN_ENC_A_PHASEB) == HIGH) {
    contadorPulsosMotorA++;
  } else {
    contadorPulsosMotorA--;
  }
}

void ISR_Encoder_MotorB() {
  if (digitalRead(PIN_ENC_B_PHASEB) == HIGH) {
    contadorPulsosMotorB++;
  } else {
    contadorPulsosMotorB--;
  }
}

// ============================================================================
// PROTOTIPOS DE FUNCIONES
// ============================================================================
void initPerifericos(void);
void initEncoders(void);
void initHW171(void);
void initIMU(void);

void simularMovimientoEncoders(float dt);
void calcularVelocidadesRPM(float dt);

void actualizarPCF8574(uint8_t mascara);
void seleccionarCanalTCA(uint8_t canal);
uint16_t leerDistanciaGP2Y0E03(void);
void muestrearTodosSensoresIR(void);

void leerIMUInterno(void);
void mostrarDatosMonitorSerial(void);
void procesarComandosUART(void);

void controlarMotorA(int16_t velocidad);
void controlarMotorB(int16_t velocidad);
void detenerMotores(void);

// ============================================================================
// SETUP Y MAIN LOOP
// ============================================================================
void setup() {
  initPerifericos();
  initEncoders();
  
  // Confirmación inicial en el Monitor Serial
  Serial.println("XIAO nRF52840 lista y puerto CDC activo.");
  Serial.println("Se comienza a inicializar todo");
  
  initHW171();
  initIMU();
  
  // Habilitar driver TB6612FNG vía HW-171 (STBY = HIGH)
  estadoPinosPCF |= BIT_STBY;
  actualizarPCF8574(estadoPinosPCF);
  detenerMotores();
}

void loop() {
  // --- 1. Muestreo de sensores a alta velocidad ---
  if (millis() - ultimoMuestreoMs >= INTERVALO_MUESTREO_MS) {
    unsigned long deltaMs = millis() - ultimoMuestreoMs;
    ultimoMuestreoMs = millis();
    float dt = (float)deltaMs / 1000.0f; // Delta de tiempo en segundos

    // Simulación de incrementos cuando los motores no generan ticks reales físicamente
    simularMovimientoEncoders(dt);

    // Cálculo periódico de velocidad angular (RPM)
    calcularVelocidadesRPM(dt);

    muestrearTodosSensoresIR();        // Lee los 8 IRs en HW-617
    leerIMUInterno();                  // Lee el MPU9250 vía Wire
    mostrarDatosMonitorSerial();       // Visualización continua en el Monitor Serial
  }

  // --- 2. Recepción y control de motores vía Serial / UART ---
  procesarComandosUART();

  // --- 3. Pequeña pausa activa para estabilidad del scheduler de FreeRTOS ---
  delay(1); 
}

// ============================================================================
// FUNCIONES DE INICIALIZACIÓN
// ============================================================================
void initPerifericos(void) {
  // Monitor Serial por USB CDC
  Serial.begin(115200);
  
  // Esperar hasta 3 segundos para que el puerto serie USB sea enumerado por Linux
  uint32_t startMs = millis();
  while (!Serial && (millis() - startMs < 3000)) {
    delay(10);
  }
  
  // UART Física opcional (Pines D6/D7)
  Serial1.begin(115200);

  // Configurar pines de PWM
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT);
  analogWrite(PIN_PWMA, 0);
  analogWrite(PIN_PWMB, 0);

  // I2C a 400kHz (Modo rápido)
  Wire.begin();
  Wire.setClock(400000);
}

void initEncoders(void) {
  pinMode(PIN_ENC_A_PHASEA, INPUT_PULLUP);
  pinMode(PIN_ENC_A_PHASEB, INPUT_PULLUP);
  pinMode(PIN_ENC_B_PHASEA, INPUT_PULLUP);
  pinMode(PIN_ENC_B_PHASEB, INPUT_PULLUP);

  // Adjuntar interrupciones al flanco de subida del Canal A de cada motor
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A_PHASEA), ISR_Encoder_MotorA, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B_PHASEA), ISR_Encoder_MotorB, RISING);
}

void initHW171(void) {
  // Apagar todas las salidas del PCF8574 al iniciar
  estadoPinosPCF = 0x00;
  actualizarPCF8574(estadoPinosPCF);
}

void initIMU(void) {
  // Despertar el MPU9250 escribiendo 0x00 en el registro PWR_MGMT_1 (0x6B)
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(MPU9250_PWR_MGMT_1);
  Wire.write(0x00); 
  if (Wire.endTransmission() != 0) {
    Serial.println("Error: No se pudo comunicar con el MPU9250 (I2C 0x68).");
  } else {
    Serial.println("MPU9250 despierto e inicializado correctamente vía I2C directo.");
  }
}

// ============================================================================
// CÁLCULOS Y SIMULACIÓN DE ENCODERS
// ============================================================================
void simularMovimientoEncoders(float dt) {
  // Simulación: Asume velocidad máxima de 200 RPM a PWM = 255
  float rpmEstimadaA = ((float)consignaPWMA / 255.0f) * 200.0f;
  float rpmEstimadaB = ((float)consignaPWMB / 255.0f) * 200.0f;

  float deltaTicksSimA = (rpmEstimadaA / 60.0f) * TICKS_POR_REVOLUCION * dt;
  float deltaTicksSimB = (rpmEstimadaB / 60.0f) * TICKS_POR_REVOLUCION * dt;

  noInterrupts();
  contadorPulsosMotorA += (long)deltaTicksSimA;
  contadorPulsosMotorB += (long)deltaTicksSimB;
  interrupts();
}

void calcularVelocidadesRPM(float dt) {
  if (dt <= 0.0f) return;

  // Copia atómica de los contadores incrementados por las ISRs
  noInterrupts();
  long pulsosA = contadorPulsosMotorA;
  long pulsosB = contadorPulsosMotorB;
  interrupts();

  long deltaTicksA = pulsosA - pulsosAnterioresA;
  long deltaTicksB = pulsosB - pulsosAnterioresB;

  pulsosAnterioresA = pulsosA;
  pulsosAnterioresB = pulsosB;

  // Cálculo de velocidad angular en RPM
  rpmMotorA = ((float)deltaTicksA / TICKS_POR_REVOLUCION) * (60.0f / dt);
  rpmMotorB = ((float)deltaTicksB / TICKS_POR_REVOLUCION) * (60.0f / dt);
}

// ============================================================================
// CONTROLADOR I2C HW-171 (PCF8574)
// ============================================================================
void actualizarPCF8574(uint8_t mascara) {
  Wire.beginTransmission(PCF8574_ADDR);
  Wire.write(mascara);
  Wire.endTransmission();
}

// ============================================================================
// CONTROLADOR I2C HW-617 (TCA9548A) Y SENSORES IR
// ============================================================================
void seleccionarCanalTCA(uint8_t canal) {
  if (canal > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << canal);
  Wire.endTransmission();
}

uint16_t leerDistanciaGP2Y0E03(void) {
  uint8_t msb = 0, lsb = 0;
  Wire.beginTransmission(GP2Y0E03_ADDR);
  Wire.write(REG_DISTANCIA_HIGH);
  if (Wire.endTransmission() != 0) return 0;

  Wire.requestFrom((uint8_t)GP2Y0E03_ADDR, (uint8_t)2);
  if (Wire.available() >= 2) {
    msb = Wire.read();
    lsb = Wire.read();
  }
  return ((uint16_t)msb << 4) | (lsb & 0x0F);
}

void muestrearTodosSensoresIR(void) {
  for (uint8_t i = 0; i < NUM_CANALES_IR; i++) {
    seleccionarCanalTCA(i);
    distancias_IR[i] = leerDistanciaGP2Y0E03();
  }
}

// ============================================================================
// LECTURA DEL MPU9250 VÍA I2C DIRECTO (SIN LIBRERÍA)
// ============================================================================
void leerIMUInterno(void) {
  // Iniciar lectura desde el registro ACCEL_XOUT_H (0x3B)
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(MPU9250_ACCEL_XOUT_H);
  if (Wire.endTransmission() != 0) return;

  // Solicitar 14 bytes consecutivos: Accel (6) + Temp (2) + Gyro (6)
  Wire.requestFrom((uint8_t)MPU9250_ADDR, (uint8_t)14);
  if (Wire.available() >= 14) {
    int16_t rawAx   = (Wire.read() << 8) | Wire.read();
    int16_t rawAy   = (Wire.read() << 8) | Wire.read();
    int16_t rawAz   = (Wire.read() << 8) | Wire.read();
    int16_t rawTemp = (Wire.read() << 8) | Wire.read();
    int16_t rawGx   = (Wire.read() << 8) | Wire.read();
    int16_t rawGy   = (Wire.read() << 8) | Wire.read();
    int16_t rawGz   = (Wire.read() << 8) | Wire.read();

    // Conversión a unidades físicas (Gs, °C, dps)
    ax = (float)rawAx / ACCEL_SCALE;
    ay = (float)rawAy / ACCEL_SCALE;
    az = (float)rawAz / ACCEL_SCALE;

    // Fórmula del datasheet MPU9250 para la temperatura: ((Temp_out - RoomTemp_Offset)/Temp_Sensitivity) + 21
    temp = ((float)rawTemp - 21.0f) / 333.87f + 21.0f;

    gx = (float)rawGx / GYRO_SCALE;
    gy = (float)rawGy / GYRO_SCALE;
    gz = (float)rawGz / GYRO_SCALE;
  }
}

// ============================================================================
// CONTROL DE MOTORES COMBINANDO HW-171 (I2C) Y XIAO (PWM)
// ============================================================================
void controlarMotorA(int16_t velocidad) {
  consignaPWMA = velocidad;

  // Limpiar bits de dirección AIN1 y AIN2
  estadoPinosPCF &= ~(BIT_AIN1 | BIT_AIN2);

  if (velocidad > 0) {
    estadoPinosPCF |= BIT_AIN1; // Adelante
  } else if (velocidad < 0) {
    estadoPinosPCF |= BIT_AIN2; // Atrás
    velocidad = -velocidad;
  }
  
  // Enviar cambio de dirección por I2C al HW-171
  actualizarPCF8574(estadoPinosPCF);

  // Enviar magnitud de velocidad PWM directa desde la XIAO
  analogWrite(PIN_PWMA, constrain(velocidad, 0, 255));
}

void controlarMotorB(int16_t velocidad) {
  consignaPWMB = velocidad;

  // Limpiar bits de dirección BIN1 y BIN2
  estadoPinosPCF &= ~(BIT_BIN1 | BIT_BIN2);

  if (velocidad > 0) {
    estadoPinosPCF |= BIT_BIN1; // Adelante
  } else if (velocidad < 0) {
    estadoPinosPCF |= BIT_BIN2; // Atrás
    velocidad = -velocidad;
  }
  
  actualizarPCF8574(estadoPinosPCF);
  analogWrite(PIN_PWMB, constrain(velocidad, 0, 255));
}

void detenerMotores(void) {
  controlarMotorA(0);
  controlarMotorB(0);
}

// ============================================================================
// VISUALIZACIÓN Y COMUNICACIÓN
// ============================================================================
void mostrarDatosMonitorSerial(void) {
  // Telemetría de Encoders (RPM y Ticks)
  Serial.print("RPM [A,B]: ");
  Serial.print(rpmMotorA, 1); Serial.print(", ");
  Serial.print(rpmMotorB, 1);
  Serial.print("\t| Ticks: ");
  Serial.print(contadorPulsosMotorA); Serial.print(", ");
  Serial.print(contadorPulsosMotorB);

  // Imprimir canal IR
  Serial.print("\t| IR [0-7]: ");
  for (uint8_t i = 0; i < NUM_CANALES_IR; i++) {
    Serial.print(distancias_IR[i]);
    Serial.print("\t");
  }

  // Imprimir MPU9250
  Serial.print("| Acc: ");
  Serial.print(ax, 1); Serial.print(",");
  Serial.print(ay, 1); Serial.print(",");
  Serial.print(az, 1);
  Serial.print(" | Giro: ");
  Serial.print(gx, 1); Serial.print(",");
  Serial.print(gy, 1); Serial.print(",");
  Serial.print(gz, 1);
  Serial.print(" | Temp: ");
  Serial.print(temp, 1); Serial.println("C");
}

void procesarComandosUART(void) {
  // Acepta comandos desde la Consola USB o Serial1
  Stream* puertoSerie = NULL;

  if (Serial.available() > 0) puertoSerie = &Serial;
  else if (Serial1.available() > 0) puertoSerie = &Serial1;

  if (puertoSerie != NULL) {
    String comando = puertoSerie->readStringUntil('\n');
    comando.trim();

    // Formato de comando: "M,150,-150"
    if (comando.startsWith("M,")) {
      int idx1 = comando.indexOf(',');
      int idx2 = comando.indexOf(',', idx1 + 1);
      
      if (idx1 != -1 && idx2 != -1) {
        int16_t velA = comando.substring(idx1 + 1, idx2).toInt();
        int16_t velB = comando.substring(idx2 + 1).toInt();
        
        controlarMotorA(velA);
        controlarMotorB(velB);
      }
    } else if (comando.equalsIgnoreCase("STOP")) {
      detenerMotores();
    }
  }
}

*/

static float directionValues[8] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f, 0.0f};
unsigned long ultimoMuestreoMs = 0;
const unsigned long INTERVALO_MUESTREO_MS = 20; // ~50 Hz de tasa de refresco

Hw617IRSensor IRsens(Wire, 8, dataSensor.IRDistance, directionValues);
MPU9250 MPsensor(Wire, dataSensor.IMUACC, dataSensor.IMUGIR, &dataSensor.IMUTemp);
Hw171Manager Hw171(Wire);
DriverPololu motorLeft(DriverPololu::MOTORID::LEFT, Hw171);
Protocol protocol(Serial1);

SemaphoreHandle_t g_sensorDataMutex;



void updateIRsensorsTask(void *pvParameters){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(16.6);     //16.6ms  60HZ
  for (;;){
    IRsens.updateValues(g_sensorDataMutex, xFrequency);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void updateIMUSensor(void *pvParameters){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(5);     //5  100HZ
  for (;;){
    MPsensor.updateData(g_sensorDataMutex, xFrequency);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}



void sendTelemetryTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);      // 50hz 

    for (;;) {
        protocol.sendData(&dataSensor, g_sensorDataMutex, xFrequency);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void printTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000); // Frecuencia de 1 Hz (1s)

    while (1) {
        // Intentar tomar el Mutex con un timeout corto (10 ms)
        if (xSemaphoreTake(g_sensorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            
            // --- 1. IMPRESIÓN DE SENSORES IR ---
            Serial.print("IRs [cm]:\t");
            for (int i = 0; i < 8; i++) {
                Serial.print(dataSensor.IRDistance[i], 2);
                if (i < 7) Serial.print("\t");
            }
            Serial.println();

            // --- 2. IMPRESIÓN DE IMU (Acelerómetro, Giroscopio, Temp) ---
            Serial.print("ACC [g]:\tX: ");
            Serial.print(dataSensor.IMUACC[0], 2);
            Serial.print("\tY: ");
            Serial.print(dataSensor.IMUACC[1], 2);
            Serial.print("\tZ: ");
            Serial.println(dataSensor.IMUACC[2], 2);

            Serial.print("GYR [deg/s]:\tX: ");
            Serial.print(dataSensor.IMUGIR[0], 2);
            Serial.print("\tY: ");
            Serial.print(dataSensor.IMUGIR[1], 2);
            Serial.print("\tZ: ");
            Serial.println(dataSensor.IMUGIR[2], 2);

            Serial.print("IMU Temp:\t");
            Serial.print(dataSensor.IMUTemp, 1);
            Serial.println(" °C");

            Serial.println("--------------------------------------------------");

            // Liberar el Mutex inmediatamente después de la lectura/copia
            xSemaphoreGive(g_sensorDataMutex);
        }

        // Mantener la cadencia exacta de 1 segundo
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}



void setup() {
  
  //init memory space
  std::memset(&dataSensor,0, sizeof(dataSensor));
  g_sensorDataMutex = xSemaphoreCreateMutex();


  Serial.begin(115200);
  Serial1.begin(115200);

  Serial.println("inicializando ...");
  
  Wire.begin();
  Wire.setClock(400000);

  MPsensor.beginMPU();
  //Hw171.begin();

  xTaskCreate(
    updateIRsensorsTask,
    "Task_IR",
    2048,
    NULL,
    2,
    NULL
  );


  xTaskCreate(
    updateIMUSensor,
    "updateIMUSensor",
    2048,
    NULL,
    2,
    NULL
  );

  xTaskCreate(
    printTask,
    "printTask",
    2048,
    NULL,
    3,
    NULL
  );

 

  xTaskCreate(
      sendTelemetryTask,
      "Task_Telemetry",
      2048,
      NULL,
      2,
      NULL
  );

}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
  /*
  if (millis() - ultimoMuestreoMs >= INTERVALO_MUESTREO_MS) {
    Serial.println("se envia");
    unsigned long deltaMs = millis() - ultimoMuestreoMs;
    ultimoMuestreoMs = millis();
    float dt = (float)deltaMs / 1000.0f; // Delta de tiempo en segundos
    
    /*
    MPsensor.updateData();  //update data

    const IRsensors& datosIR = IRsens.getDistance();

    for (size_t i = 0; i < datosIR.values.size(); i++) {
        Serial.print("SensorIR ");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(datosIR.values[i]);
        Serial.print(" cm  | ");
    }
    
    const std::vector<float>& acc = MPsensor.getAcceleration();
    const std::vector<float>& gir = MPsensor.getGiroscope();

    for (size_t i = 0; i < acc.size(); i++) {
        Serial.print("a");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(acc[i]);
        Serial.print("-g");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(gir[i]);
        Serial.print("  **  ");
    }
    Serial.println('/n');
    Serial.println(motorLeft.currentCounter());
  
   

  } */
}