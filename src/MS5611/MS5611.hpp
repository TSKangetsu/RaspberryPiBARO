#pragma once
#include <mutex>
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <linux/i2c-dev.h>
#include "../Baro.hpp"

#define CMD_PROM_READ 0xA2
#define MS5611_ADDRESS 0x77
#define CONV_D1_4096 0x48
#define CONV_D2_4096 0x58

#define MS5611PreWait4096 9800

#define DEFAULT_SEA_PRESSURE 1013.25f

class MS5611Baro
{
public:
    MS5611Baro(const MS5611Baro &other) = delete;
    inline MS5611Baro(const char *I2CChannel = "/dev/i2c-1", uint8_t I2CAddress = 0x77)
    {
        if ((MS5611FD = open(I2CChannel, O_RDWR)) < 0)
            throw - 1;
        if (ioctl(MS5611FD, I2C_SLAVE, I2CAddress) < 0)
            throw - 2;
        if (ioctl(MS5611FD, I2C_TIMEOUT, 0x01) < 0) // set to 10ms?
            throw - 2;

        if (write(MS5611FD, &MS5611RESET, 1) != 1)
            throw - 3;
        usleep(10000);
        for (int i = 0; i < 6; i++)
        {
            uint16_t ret = 0;
            uint8_t r8b[] = {0, 0};
            char AddressOffset = (CMD_PROM_READ + (i * 2));
            if (write(MS5611FD, &AddressOffset, 1) != 1)
                throw - 4;
            if (read(MS5611FD, r8b, 2) != 2)
                throw - 5;
            ret = r8b[0] * 256 + r8b[1];
            C[i] = ret;
            usleep(1000);
        }
    }

    inline BaroData MS5611Read()
    {
        BaroData Data;

        //D2 TemperatureC Raw
        {
            long ret = 0;
            uint8_t D[] = {0, 0, 0};
            int h;
            char zero = 0x00;
            char output = 0x00;
            output = CONV_D2_4096;
            h = write(MS5611FD, &output, 1);
            usleep(MS5611PreWait4096);
            h = write(MS5611FD, &zero, 1);
            h = read(MS5611FD, &D, 3);
            if (h != 3)
                Data.IsDataCorrect = false;
            else
            {
                D2 = D[0] * (unsigned long)65536 + D[1] * (unsigned long)256 + D[2];
                dT = D2 - (uint32_t)C[4] * 256;
                TEMP = (2000 + (dT * (int64_t)C[5] / 8388608));
                //
                Data.TemperatureC = TEMP / 100.f;
                //
                OFF = (int64_t)C[1] * 65536 + (int64_t)dT * C[3] / 128;
                SENS = (int32_t)C[0] * 32768 + (int64_t)dT * C[2] / 256;
                if (TEMP < 2000)
                {
                    TEMP -= (dT * dT) / (2 << 30);
                    int64_t OFF1 = 0;
                    int64_t SENS1 = 0;
                    OFF1 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
                    SENS1 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
                    if (TEMP < -1500)
                    {
                        OFF1 = OFF1 + 7 * ((TEMP + 1500) * (TEMP + 1500));
                        SENS1 = SENS1 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
                    }
                    OFF -= OFF1;
                    SENS -= SENS1;
                }
            }
        }
        //D1 Pressure Raw
        {
            long ret = 0;
            uint8_t D[] = {0, 0, 0};
            int h;
            char zero = 0x00;
            char output = 0x00;
            //
            output = CONV_D1_4096;
            h = write(MS5611FD, &output, 1);
            usleep(MS5611PreWait4096);
            h = write(MS5611FD, &zero, 1);
            h = read(MS5611FD, &D, 3);
            if (h != 3)
                Data.IsDataCorrect = false;
            else
            {
                D1 = D[0] * (unsigned long)65536 + D[1] * (unsigned long)256 + D[2];
                P = ((((int64_t)D1 * SENS) / 2097152.0 - OFF) / 32768.0);
                Data.PressureHPA = (double)P / (double)100;
                Data.IsDataCorrect = true;
            }
        }
        Data.AltitudeM = 44330.0f * (1.0f - pow((Data.PressureHPA / DEFAULT_SEA_PRESSURE), 0.1902949f));
        return Data;
    };

    inline BaroData MS5611Read(std::mutex *I2CDeviceLock)
    {
        BaroData Data;

        //D2 TemperatureC Raw
        {
            long ret = 0;
            uint8_t D[] = {0, 0, 0};
            int h;
            char zero = 0x00;
            char output = 0x00;
            output = CONV_D2_4096;
            I2CDeviceLock->lock();
            h = write(MS5611FD, &output, 1);
            I2CDeviceLock->unlock();

            usleep(MS5611PreWait4096);

            I2CDeviceLock->lock();
            h = write(MS5611FD, &zero, 1);
            h = read(MS5611FD, &D, 3);
            I2CDeviceLock->unlock();

            if (h != 3)
                Data.IsDataCorrect = false;
            else
            {
                D2 = D[0] * (unsigned long)65536 + D[1] * (unsigned long)256 + D[2];
                dT = D2 - (uint32_t)C[4] * 256;
                TEMP = (2000 + (dT * (int64_t)C[5] / 8388608));
                //
                Data.TemperatureC = TEMP / 100.f;
                //
                OFF = (int64_t)C[1] * 65536 + (int64_t)dT * C[3] / 128;
                SENS = (int32_t)C[0] * 32768 + (int64_t)dT * C[2] / 256;
                if (TEMP < 2000)
                {
                    TEMP -= (dT * dT) / (2 << 30);
                    int64_t OFF1 = 0;
                    int64_t SENS1 = 0;
                    OFF1 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
                    SENS1 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
                    if (TEMP < -1500)
                    {
                        OFF1 = OFF1 + 7 * ((TEMP + 1500) * (TEMP + 1500));
                        SENS1 = SENS1 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
                    }
                    OFF -= OFF1;
                    SENS -= SENS1;
                }
            }
        }
        //D1 Pressure Raw
        {
            long ret = 0;
            uint8_t D[] = {0, 0, 0};
            int h;
            char zero = 0x00;
            char output = 0x00;
            //
            output = CONV_D1_4096;
            I2CDeviceLock->lock();
            h = write(MS5611FD, &output, 1);
            I2CDeviceLock->unlock();

            usleep(MS5611PreWait4096);

            I2CDeviceLock->lock();
            h = write(MS5611FD, &zero, 1);
            h = read(MS5611FD, &D, 3);
            I2CDeviceLock->unlock();

            if (h != 3)
                Data.IsDataCorrect = false;
            else
            {
                D1 = D[0] * (unsigned long)65536 + D[1] * (unsigned long)256 + D[2];
                P = ((((int64_t)D1 * SENS) / 2097152.0 - OFF) / 32768.0);
                Data.PressureHPA = (double)P / (double)100;
                Data.IsDataCorrect = true;
            }
        }
        Data.AltitudeM = 44330.0f * (1.0f - pow((Data.PressureHPA / DEFAULT_SEA_PRESSURE), 0.1902949f));
        return Data;
    };

    ~MS5611Baro()
    {
        close(MS5611FD);
    };

private:
    uint16_t C[7];
    uint32_t D1;
    uint32_t D2;
    //cac tmp
    int64_t dT;
    int32_t TEMP;
    int64_t OFF;
    int64_t SENS;
    int32_t P;
    int MS5611FD;
    char MS5611RESET = 0x1E;
};