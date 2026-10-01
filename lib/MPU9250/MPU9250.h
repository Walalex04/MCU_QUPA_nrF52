#ifndef MPU9250_H
#define MPU9250_H

#include <Arduino.h>
#include <Wire.h>

#define MPU9250_ADDR            0x68  
#define MPU9250_PWR_MGMT_1      0x6B
#define MPU9250_ACCEL_XOUT_H    0x3B


#define ACCEL_SCALE             16384.0f; 
#define GYRO_SCALE              131.0f; 

class MPU9250
{
private:

    void requestData(SemaphoreHandle_t sensorMutex, TickType_t maxWaitTicks);

    float* _acc;
    float* _gir;
    float* _temp;

    TwoWire *_wire;
    
public:
    MPU9250(TwoWire &wireBus, float* addressAcc, float* addressGir, float* addressTemp);
    ~MPU9250();


    char beginMPU();


    void updateData(SemaphoreHandle_t sensorMutex, TickType_t maxWaitTicks);
};





#endif 