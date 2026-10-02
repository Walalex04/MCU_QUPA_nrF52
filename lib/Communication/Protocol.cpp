#include "Protocol.h"
#include <cstring>

Protocol::Protocol(HardwareSerial& serial) : _serial(&serial) {}

Protocol::~Protocol() {}

uint8_t Protocol::calculateCRC(const uint8_t* data, size_t length)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i];
    }
    return crc;
}

void Protocol::sendData(const PROTOCOLUART_SEND* dataStruct, SemaphoreHandle_t mutex, TickType_t waitTicks)
{
    if (dataStruct == nullptr || _serial == nullptr) return;

    PROTOCOLUART_SEND snapshot;

    if (xSemaphoreTake(mutex, waitTicks) == pdTRUE)
    {
        std::memcpy(&snapshot, dataStruct, sizeof(PROTOCOLUART_SEND));
        xSemaphoreGive(mutex);

        EncodedTelemetryFrame frame;


        for (int i = 0; i < 8; i++)
        {
            frame.irDistance[i] = static_cast<uint16_t>(snapshot.IRDistance[i] * 100.0f);
        }

        for (int i = 0; i < 3; i++)
        {
            frame.imuAcc[i] = static_cast<int16_t>(snapshot.IMUACC[i] * 1000.0f);
            frame.imuGir[i] = static_cast<int16_t>(snapshot.IMUGIR[i] * 10.0f);
        }

        frame.imuTemp = static_cast<uint16_t>(snapshot.IMUTemp * 100.0f);
        frame.driverVelA = static_cast<int16_t>(snapshot.DriverVelocityA * 10.0f);
        frame.driverVelB = static_cast<int16_t>(snapshot.DriverVelocityB * 10.0f);

        frame.directions = 0;
        if (snapshot.DirectionA) frame.directions |= (1 << 0);
        if (snapshot.DirectionB) frame.directions |= (1 << 1);

        frame.crc = calculateCRC(reinterpret_cast<const uint8_t*>(&frame), sizeof(EncodedTelemetryFrame) - 1);

        _serial->write(reinterpret_cast<const uint8_t*>(&frame), sizeof(EncodedTelemetryFrame));
    }
}