
#ifndef PROTOCOL_H
#define PROTOCOL_H


#include <Arduino.h>
#include <vector>




struct PROTOCOLUART_SEND
{
    float                   IRDistance[8];         //0.00    80.00
    float                   IMUACC[3];             // MPU9250             
    float                   IMUGIR[3];             //MPU9250
    float                   IMUTemp;            // 0.00 50.00
    float                   DriverVelocityA;    // RPM de velocidad de un pololu 
    uint8_t                 DirectionA;         // 1 byte (1 o 0)
    float                   DriverVelocityB;
    uint8_t                 DirectionB;

    /**
     * TO DO: ADD COLOR SENSOR
     */
};


struct PROTOCOLUART_RECIVE
{
    float                   PWMPorcentA;    //0.00 - 100.00
    char                    directionA;      // 1 0
    float                   PWMPorcentB;        
    char                    directionB;
    uint8_t                 StateColor[3];    // rgb leds
    char                    ONPheromones;     // 1 0 on off
};



#pragma pack(push, 1)
struct EncodedTelemetryFrame
{
    uint8_t  header = 0xAA;
    uint16_t irDistance[8];     // 0-8000 (0.00 - 80.00 cm)
    int16_t  imuAcc[3];         // +-16000 (+-16.000 g)
    int16_t  imuGir[3];         // +-20000 (+-2000.0 deg/s)
    uint16_t imuTemp;           // 0-5000 (0.00 - 50.00 °C)
    int16_t  driverVelA;        // +-5000 (+-500.0 RPM)
    int16_t  driverVelB;        // +-5000 (+-500.0 RPM)
    uint8_t  directions;        // Bit 0: DirectionA, Bit 1: DirectionB
    uint8_t  crc;               // Checksum XOR básico
};

struct EncodedControlFrame
{
    uint8_t  header = 0xBB;     // Delimitador de inicio de recepción
    uint16_t pwmA;              // 0-10000 (0.00 - 100.00 %)
    uint16_t pwmB;              // 0-10000 (0.00 - 100.00 %)
    uint8_t  stateColor[3];     // R, G, B
    uint8_t  flags;             // Bit 0: directionA, Bit 1: directionB, Bit 2: ONPheromones
    uint8_t  crc;               // Checksum XOR
};

#pragma pack(pop)


class Protocol
{

private:
    HardwareSerial*      _serial;
    uint8_t calculateCRC(const uint8_t* data, size_t length);
public:
    Protocol(HardwareSerial& serial);
    ~Protocol();


    void sendData(const PROTOCOLUART_SEND* dataStruct, SemaphoreHandle_t mutex, TickType_t waitTicks = pdMS_TO_TICKS(5));
    bool decodeFrame(const EncodedControlFrame* rawFrame, PROTOCOLUART_RECIVE* receiveStruct,
                     SemaphoreHandle_t mutex, TickType_t waitTicks = pdMS_TO_TICKS(5));
};


#endif