#include <iostream>
#include "src/BaroDevice.hpp"

int main(int, char **)
{
    // Don't Try to Copy between two baro Device
    BaroDevice *Baro;
    try
    {
        Baro = new BaroDevice(BaroType::BMP280, "/dev/i2c-1", 0x77);
    }
    catch (int error)
    {
        switch (error)
        {
        case -1:
            std::cout << "open I2C device faild\n";
            break;
        case -2:
            std::cout << "Set I2C device prop faild\n";
            break;
        default:
            std::cout << "Set I2C Command faild\n";
            break;
        }
    }

    float filterA = 0;
    float filterP = DEFAULT_SEA_PRESSURE;
    float filterT = 0;
    BaroData BData;
    while (true)
    {
        BData = Baro->BaroRead();
        if (BData.IsDataCorrect)
        {
            // filterP = BData.PressureHPA; // if mode is bmp2800
            // filterT = BData.TemperatureC;

            // filterA = filterA * 0.8 + BData.AltitudeM * 0.2;
            // filterP = filterP * 0.8 + BData.PressureHPA * 0.2;
            // filterT = filterT * 0.8 + BData.TemperatureC * 0.2;
            // std::cout << "Baro Altitude CM :" << (int)(filterA * 100.f) << "                                      \n";
            // std::cout << "Baro Pressure HPA:" << (int)(filterP * 100.f) / 100.f << "                                      \n";
            // std::cout << "Baro TemperatureC:" << (int)(filterT * 100.f) / 100.f << "                                      \n\n";
            // std::cout << "\033[4A";
            // std::cout << "\033[K";
        }
        else
        {
            std::cout << "I2C frame Error"
                      << "\n";
        }
    }
}
