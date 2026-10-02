#ifndef DRIVERPOLOLU_H
#define DRIVERPOLOLU_H


#include "Hw171Manager.h"
#include <Arduino.h>
#include <Wire.h>

#include "nrf.h"
#include "nrf_gpiote.h"
#include "nrf_ppi.h"
#include "nrf_timer.h"

#define BIT_AIN1            (1 << 0) 
#define BIT_AIN2            (1 << 1) 
#define BIT_BIN1            (1 << 2) 
#define BIT_BIN2            (1 << 3) 
#define BIT_STBY            (1 << 4) 

#define PIN_ENCODER_A       3           //CP0.03 SoC
#define CHANNEL_GPIOTE_A    0
#define CHANNEL_PPI_A       0

#define PIN_ENCODER_B       29          // P0.29
#define CHANNEL_GPIOTE_B    1 
#define CHANNEL_PPI_B       1       

#define FREQVELOCITY        1           // must be change in the programming
#define PULSE_PER_REV       909.72f


#define PIN_PWMA            14  // P1.14
#define PIN_PWMB            15  // P1.15

#define PWM_TOP_VALUE       1000


const float TICKS_POR_REVOLUCION = 909.72f;

alignas(4) static uint16_t pwmDutyCycleBuffer[4] = {0, 0, 0, 0};

// utils for class
void initPWMMotors();
void updatePWM(float porcentajeM1, float porcentajeM2);

class DriverPololu
{
public: enum class MOTORID{
        LEFT,
        RIGHT
};


private:

    void initCounterEncoder();


    MOTORID         _idMotor; 
    float*          _sensVelocity;
    uint8_t*        _sensDirection;
    Hw171Manager*   _Hw171manager;
    u_int32_t       _counterCurrentEncoder;
    u_int32_t       _couterLastEnconder;

    NRF_TIMER_Type* _HW_TIMER;
    u_int8_t        _CHANNEL_GPIOTE;
    u_int8_t        _CHANNEL_PPI;
    u_int8_t        _PIN_ENCODER;

public:
    
    DriverPololu(MOTORID idMotor,  Hw171Manager &hw171manager, float* addressVelocity, uint8_t* addressDirecction);
    ~DriverPololu();

    void updateVelocity(SemaphoreHandle_t sensorMutex, TickType_t maxWaitTicks);

    void setVelocity(float porcent);

    u_int32_t currentCounter();

   
};



#endif