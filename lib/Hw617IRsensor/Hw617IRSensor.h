#ifndef HW617IRSENSOR_H
#define HW617IRSENSOR_H

#include <Arduino.h>
#include <vector>
#include <Wire.h>


#define TCA9548A_ADDR           0x70  
#define NUM_CHANNELS_IR_MAX     8
#define GP2Y0E03_ADDR           0x40  
#define REG_DISTANCIA_HIGH      0x5E  


// define constans for second order aproximation
// A2 * x**2 + A1 * x + A0
#define A2                      0.00001198f
#define A1                      -0.0583f
#define A0                      71.80f



/**
 * struct for store all data of sensors
 */


class Hw617IRSensor
{
private:
    void        readAllSensors(SemaphoreHandle_t sensorMutex, TickType_t maxWaitTicks);
    uint16_t    readSensor(uint8_t channel);
    float       mapRealDistance(uint16_t analogValue);

    float       *_values;
    float       *_orientation;
    TwoWire     *_wire;
    uint8_t     _numIrChannels;

public:

    Hw617IRSensor(TwoWire &wireBus, uint8_t numIrChannels, float *addressValues, float* addressOrientation);
    ~Hw617IRSensor();

    void updateValues(SemaphoreHandle_t sensorMutex, TickType_t maxWaitTicks);
};  





#endif
