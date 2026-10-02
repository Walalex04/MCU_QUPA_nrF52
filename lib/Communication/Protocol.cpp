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


bool Protocol::decodeFrame(const EncodedControlFrame* rawFrame, PROTOCOLUART_RECIVE* receiveStruct, SemaphoreHandle_t mutex, TickType_t waitTicks)
{
    if (rawFrame == nullptr || receiveStruct == nullptr) return false;

    if (rawFrame->header != 0xBB) return false;

    uint8_t computedCRC = calculateCRC(reinterpret_cast<const uint8_t*>(rawFrame), sizeof(EncodedControlFrame) - 1);
    if (computedCRC != rawFrame->crc) return false;

    PROTOCOLUART_RECIVE decodedData;
    decodedData.PWMPorcentA = static_cast<float>(rawFrame->pwmA) / 100.0f;
    decodedData.PWMPorcentB = static_cast<float>(rawFrame->pwmB) / 100.0f;

    decodedData.StateColor[0] = rawFrame->stateColor[0];
    decodedData.StateColor[1] = rawFrame->stateColor[1];
    decodedData.StateColor[2] = rawFrame->stateColor[2];

    decodedData.directionA   = (rawFrame->flags & (1 << 0)) ? 1 : 0;
    decodedData.directionB   = (rawFrame->flags & (1 << 1)) ? 1 : 0;
    decodedData.ONPheromones = (rawFrame->flags & (1 << 2)) ? 1 : 0;

    if (xSemaphoreTake(mutex, waitTicks) == pdTRUE)
    {
        std::memcpy(receiveStruct, &decodedData, sizeof(PROTOCOLUART_RECIVE));
        xSemaphoreGive(mutex);
        return true;
    }

    return false;
}