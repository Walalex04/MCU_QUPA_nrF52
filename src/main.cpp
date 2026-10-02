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
static EncodedControlFrame rxBufferDMA;
static PROTOCOLUART_RECIVE g_receiveData;
static TaskHandle_t xRxTaskHandle = NULL;

static float directionValues[8] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f, 0.0f};
unsigned long ultimoMuestreoMs = 0;
const unsigned long INTERVALO_MUESTREO_MS = 20; // ~50 Hz de tasa de refresco

Hw617IRSensor IRsens(Wire, 8, dataSensor.IRDistance, directionValues);
MPU9250 MPsensor(Wire, dataSensor.IMUACC, dataSensor.IMUGIR, &dataSensor.IMUTemp);
Hw171Manager Hw171(Wire);
DriverPololu motorLeft(DriverPololu::MOTORID::LEFT, Hw171, &dataSensor.DriverVelocityA, &dataSensor.DirectionA);
DriverPololu motorRigth(DriverPololu::MOTORID::RIGHT, Hw171, &dataSensor.DriverVelocityB, &dataSensor.DirectionB);
Protocol protocol(Serial1);

SemaphoreHandle_t g_sensorDataMutex;


void ISR_UART_RxPin();


void updateIRsensorsTask(void *pvParameters){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(16.6);     //16.6ms  60HZ
  for (;;){
    IRsens.updateValues(g_sensorDataMutex, pdMS_TO_TICKS(5));
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void updateIMUSensor(void *pvParameters){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(5);     //5  100HZ
  for (;;){
    MPsensor.updateData(g_sensorDataMutex, pdMS_TO_TICKS(5));
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void updateVelocityMotors(void *PvParameters){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10);   // 50HZ
  for(;;){
    motorRigth.updateVelocity(g_sensorDataMutex, pdMS_TO_TICKS(5));
    motorLeft.updateVelocity(g_sensorDataMutex, pdMS_TO_TICKS(5));
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}



void sendTelemetryTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);      // 50hz 

    for (;;) {
        //Serial.println("Se envia los valores");
        protocol.sendData(&dataSensor, g_sensorDataMutex, pdMS_TO_TICKS(5));
        //Serial.println("Se enviaron los valores");
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
    }
}

void printTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5000); // Frecuencia de 1 Hz (1s)

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
            Serial.println(motorLeft.currentCounter());
            // Liberar el Mutex inmediatamente después de la lectura/copia
            xSemaphoreGive(g_sensorDataMutex);
        }

        // Mantener la cadencia exacta de 1 segundo
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void nativeRxTask(void *pvParameters)
{
    xRxTaskHandle = xTaskGetCurrentTaskHandle();
    EncodedControlFrame rawFrame;
    
    for (;;)
    {   
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        detachInterrupt(digitalPinToInterrupt(PIN_SERIAL1_RX));

        size_t bytesRead = Serial1.readBytes(reinterpret_cast<char*>(&rawFrame), sizeof(EncodedControlFrame));

        if (bytesRead == sizeof(EncodedControlFrame)) 
        {
            if (protocol.decodeFrame(&rawFrame, &g_receiveData, g_sensorDataMutex, pdMS_TO_TICKS(5))) 
            {
                Serial.print("PWM Motor A recibido: ");
                Serial.println(g_receiveData.PWMPorcentA);

                motorLeft.setVelocity(g_receiveData.PWMPorcentA); //this can be improved if only update the register
                                                                  //and the duty point to g_receiveData directly
                motorRigth.setVelocity(g_receiveData.PWMPorcentB);
            } 
            else 
            {
                Serial.println("Error: Trama corrupta o Header/CRC invalido");
            }
        } 
        else 
        {
            while (Serial1.available()) Serial1.read();
        }
        attachInterrupt(digitalPinToInterrupt(PIN_SERIAL1_RX), ISR_UART_RxPin, FALLING);
    }
}

void ISR_UART_RxPin() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (xRxTaskHandle != NULL) {
        vTaskNotifyGiveFromISR(xRxTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}




void setup() {

  
  //init fixed memory space  --see protocol file
  std::memset(&dataSensor,0, sizeof(dataSensor));


  g_sensorDataMutex = xSemaphoreCreateMutex();

  Serial.begin(115200);     // Serial Monitor

  Serial.println("Initializing components");
  Serial1.begin(115200);   //UART RASP
  pinMode(PIN_SERIAL1_RX, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_SERIAL1_RX), ISR_UART_RxPin, FALLING);


  Wire.begin();
  Wire.setClock(400000);

  MPsensor.beginMPU();
  Hw171.begin();

  initPWMMotors();
  xTaskCreate(updateIRsensorsTask, "Task_IR", 2048,
              NULL, 2, NULL);

  xTaskCreate( updateIMUSensor, "updateIMUSensor", 
              2048, NULL, 2,NULL ); 
  
  xTaskCreate(updateVelocityMotors, "updateVelocity", 2048,
            NULL, 2, NULL);
  
  xTaskCreate( printTask, "printTask", 2048,
              NULL, 3, NULL );

  xTaskCreate( sendTelemetryTask, "Task_Telemetry", 2048,
               NULL, 2, NULL);
  
  xTaskCreate( nativeRxTask, "NativeRxTask", 2048,
        NULL, 1, NULL);
  
  Serial.print("Components was initilized");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}