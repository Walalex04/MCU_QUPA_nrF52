
#include "Hw617IRSensor.h"
#include <Arduino.h>
#include <FreeRTOS.h>
#include <cstdint>
#include <cstring>


Hw617IRSensor::Hw617IRSensor(TwoWire &wireBus, uint8_t numIrChannels, float *addressValues, float* addressOrientation):
                         _wire(&wireBus), _numIrChannels(numIrChannels), _values(addressValues), _orientation(addressOrientation)
{

}

Hw617IRSensor::~Hw617IRSensor()
{

}

float Hw617IRSensor::mapRealDistance(uint16_t analogValue){
    return  A2 * (float)(analogValue*analogValue) +
            A1 * (float)(analogValue) + A0;
}

void Hw617IRSensor::readAllSensors(SemaphoreHandle_t sensorMutex, TickType_t maxWaitTicks){
    float TempValues[8];
    uint16_t analogValue;
    for (uint8_t i = 0; i < _numIrChannels; i++) {
        analogValue = Hw617IRSensor::readSensor(i); 
        TempValues[i] = Hw617IRSensor::mapRealDistance(analogValue);
    }

    if((xSemaphoreTake(sensorMutex, maxWaitTicks) == pdTRUE)){
        std::memcpy(_values, TempValues, sizeof(float) * _numIrChannels);
        xSemaphoreGive(sensorMutex);
    }
}
uint16_t Hw617IRSensor::readSensor(uint8_t channel){

    uint8_t msb = 0, lsb = 0;

    // select the channel for stating with transmision
    if (channel > _numIrChannels) return 0;
    _wire->beginTransmission(TCA9548A_ADDR);
    _wire->write(1 << channel);
    _wire->endTransmission();


    // send request for data 

    _wire->beginTransmission(GP2Y0E03_ADDR);
    _wire->write(REG_DISTANCIA_HIGH);
    if (_wire->endTransmission() != 0) return 0;

    _wire->requestFrom((uint8_t)GP2Y0E03_ADDR, (uint8_t)2);
    if (_wire->available() >= 2) {
        msb = _wire->read();
        lsb = _wire->read();
    }

    return ((uint16_t)msb << 4) | (lsb & 0x0F);
}


void Hw617IRSensor::updateValues(SemaphoreHandle_t sensorMutex, TickType_t maxWaitTicks){
    Hw617IRSensor::readAllSensors(sensorMutex, maxWaitTicks);
}