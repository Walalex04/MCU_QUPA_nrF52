
#include "Hw171Manager.h"


Hw171Manager::Hw171Manager(TwoWire &wireBus): _wire(&wireBus)
{
}

Hw171Manager::~Hw171Manager()
{
}


void Hw171Manager::begin(){
    uint8_t estadoPinosPCF = 0x00;
    Hw171Manager::sendByte(estadoPinosPCF);
}


void Hw171Manager::sendByte(uint8_t byte){
    _wire->beginTransmission(PCF8574_ADDR);
    _wire->write(byte);
    _wire->endTransmission();
}