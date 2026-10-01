#ifndef HW171MANAGER_H
#define HW171MANAGER_H

#include <Arduino.h>
#include <Wire.h>

#define PCF8574_ADDR        0x20  // A0 = A1 = A2 = LOW


class Hw171Manager
{
private:
    TwoWire *_wire;
public:
    Hw171Manager(TwoWire &wireBus);
    ~Hw171Manager();

    void begin();

    void sendByte(uint8_t byte);
};





#endif