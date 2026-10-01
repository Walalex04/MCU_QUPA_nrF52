#include "MPU9250.h"


MPU9250::MPU9250(TwoWire &wireBus, float* addressAcc, float* addressGir, float* addressTemp):
     _wire(&wireBus), _acc(addressAcc), _gir(addressGir), _temp(addressTemp)
{   

}

MPU9250::~MPU9250()
{
}


void MPU9250::requestData(SemaphoreHandle_t sensorMutex, TickType_t maxWaitTicks) {

  _wire->beginTransmission(MPU9250_ADDR);
  _wire->write(MPU9250_ACCEL_XOUT_H);
  if (_wire->endTransmission() != 0) return;
  
  _wire->requestFrom((uint8_t)MPU9250_ADDR, (uint8_t)14);
  if (_wire->available() >= 14) {

    // check dataasheet
    int16_t rawAx   = (_wire->read() << 8) | _wire->read();
    int16_t rawAy   = (_wire->read() << 8) | _wire->read();
    int16_t rawAz   = (_wire->read() << 8) | _wire->read();
    int16_t rawTemp = (_wire->read() << 8) | _wire->read();
    int16_t rawGx   = (_wire->read() << 8) | _wire->read();
    int16_t rawGy   = (_wire->read() << 8) | _wire->read();
    int16_t rawGz   = (_wire->read() << 8) | _wire->read();

    if((xSemaphoreTake(sensorMutex, maxWaitTicks) == pdTRUE)){
        _acc[0] = (float)rawAx / ACCEL_SCALE;
        _acc[1] = (float)rawAy / ACCEL_SCALE;
        _acc[2] = (float)rawAz / ACCEL_SCALE;

        *_temp = ((float)rawTemp - 21.0f) / 333.87f + 21.0f;

        _gir[0] = (float)rawGx / GYRO_SCALE;
        _gir[1] = (float)rawGy / GYRO_SCALE;
        _gir[2] = (float)rawGz / GYRO_SCALE;
        xSemaphoreGive(sensorMutex);
    }
  }
}


char MPU9250::beginMPU(){
    _wire->beginTransmission(MPU9250_ADDR);
    _wire->write(MPU9250_PWR_MGMT_1);
    _wire->write(0x00); 
    if (_wire->endTransmission() != 0) {
        return -1;
    }
}





void MPU9250::updateData(SemaphoreHandle_t sensorMutex, TickType_t maxWaitTicks){
    MPU9250::requestData(sensorMutex, maxWaitTicks);
}