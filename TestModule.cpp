#include <iostream>
#include "src/BaroDevice.hpp"
#include <sys/time.h>


inline int GetTimestamp()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * (uint64_t)1000000 + tv.tv_usec;
}

int main(int, char **)
{
    int TimeStart;
    int TimeEnd;
    // Don't Try to Copy between two baro Device
    BaroDevice *Baro;
    try
    {
        Baro = new BaroDevice(BaroType::BMP280, "/dev/i2c-0", 0x76);
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
        TimeStart = GetTimestamp();
        BData = Baro->BaroRead();
        if (BData.IsDataCorrect)
        {
            filterP = BData.PressureHPA; // if mode is bmp2800
            filterT = BData.TemperatureC;

            filterA = filterA * 0.8 + BData.AltitudeM * 0.2;
            filterP = filterP * 0.8 + BData.PressureHPA * 0.2;
            filterT = filterT * 0.8 + BData.TemperatureC * 0.2;
            usleep(25000);
            TimeEnd = GetTimestamp();
            std::cout << "Baro Altitude CM :" << (int)(filterA * 100.f) << "                                      \n";
            std::cout << "Baro Pressure HPA:" << (int)(filterP * 100.f) / 100.f << "                                      \n";
            std::cout << "Baro TemperatureC:" << (int)(filterT * 100.f) / 100.f << "                                      \n";
            std::cout<<" tim:"<<TimeEnd-TimeStart<<"                                      \n\n";
            std::cout << "\033[5A";
            std::cout << "\033[K";
        }
        else
        {
            std::cout << "I2C frame Error"
                      << "\n";
        }
    }
}
