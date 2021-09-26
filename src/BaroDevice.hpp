#pragma once
#include "Baro.hpp"
#include "MS5611/MS5611.hpp"

enum BaroType
{
    MS5611,
    BMP180,
};

// Don't Try to Copy between two baro Device
// Old Class will close fd and cause file descripter unusable
class BaroDevice
{
public:
    BaroDevice(const BaroDevice &other) = delete;
    BaroDevice(BaroType Type, const char *I2CChannel, uint8_t I2CAddress);
    BaroData BaroRead();
    ~BaroDevice();

private:
    BaroType bType;
    MS5611Baro *MS5611Device;
};

BaroDevice::BaroDevice(BaroType Type, const char *I2CChannel, uint8_t I2CAddress)
{
    bType = Type;
    switch (Type)
    {
    case BaroType::MS5611:
        MS5611Device = new MS5611Baro(I2CChannel, I2CAddress);
        break;
    case BaroType::BMP180:
        break;
    }
}

BaroData BaroDevice::BaroRead()
{
    switch (bType)
    {
    case BaroType::MS5611:
    {
        return MS5611Device->MS5611Read();
    }
    case BaroType::BMP180:
        break;
    }

    BaroData Data;
    Data.IsDataCorrect = false;
    return Data;
};

BaroDevice::~BaroDevice()
{
    switch (bType)
    {
    case BaroType::MS5611:
        delete MS5611Device;
        break;
    case BaroType::BMP180:
        break;
    }
}
