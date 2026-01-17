#pragma once
#include <iostream>
#include <mutex>
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <asm/ioctls.h>
#include <sys/types.h>
#include <linux/i2c-dev.h>
#include "../Baro.hpp"

#define BMP280_ADDRESS 0x76
#define BMP280_ID_REG 0xD0

#define BMP280_DIG_T1_LSB_REG 0x88
#define BMP280_CTRLMEAS_REG 0xF4        /*Ctrl Measure Register */
#define BMP280_CONFIG_REG 0xF5          /*Configuration Register */
#define BMP280_PRESSURE_MSB_REG 0xF7    /*Pressure MSB Register */
#define BMP280_TEMPERATURE_MSB_REG 0xFA /*Temperature MSB Reg */

#define DEFAULT_SEA_PRESSURE 1013.25f
#define BMP280_MODE1 ((BMP280_MODE_8X << 2) | (BMP280_MODE_1X << 5) | (BMP280_FORCED_MODE))
#define BMP280_FLITER ((BMP280_FILTER_8X << 2))

typedef long signed int BMP280_S32_t;
class BMP280Baro
{
public:
    BMP280Baro(const BMP280Baro &other) = delete;
    inline BMP280Baro(const char *I2CChannel = "/dev/i2c-7", uint8_t I2CAddress = BMP280_ADDRESS)
    {
        if ((BMP280FD = open(I2CChannel, O_RDWR)) < 0)
            throw -1;
        if (ioctl(BMP280FD, I2C_SLAVE, I2CAddress) < 0)
            throw -2;
        if (ioctl(BMP280FD, I2C_TIMEOUT, 0x01) < 0) // set to 10ms?
            throw -2;

        for (int i = 0; i < 24; i += 2)
        {
            uint16_t ret = 0;
            uint8_t r8b[] = {0, 0};
            char AddressOffset = BMP280_DIG_T1_LSB_REG + i;
            if (write(BMP280FD, &AddressOffset, 1) != 1) // correct data
                throw -3;
            if (read(BMP280FD, r8b, 2) != 2)
                throw -4;
            ret = r8b[1] * 256 + r8b[0];
            C[i / 2] = ret;
            usleep(1000);
        }
        // reset
        char input[2] = {0x00, 0x00};
        // read id
        input[0] = BMP280_ID_REG;
        write(BMP280FD, &input, 1);
        read(BMP280FD, &iddata, 1);
        usleep(500);
        if (iddata == 88)
        {
            input[0] = BMP280_CTRLMEAS_REG;
            input[1] = BMP280_MODE1;
            write(BMP280FD, input, 2);
            usleep(500);
            input[0] = BMP280_CONFIG_REG;
            input[1] = BMP280_FLITER;
            write(BMP280FD, input, 2);
        }
        else
            throw -4;
    }

    inline BaroData BMP280Read()
    {
        return BMP280Read(NULL);
    }

    inline BaroData BMP280Read(std::mutex *I2CDeviceLock)
    {
        BaroData Data;
        Data.IsDataCorrect = false;
        //
        const char input[2] = {BMP280_CTRLMEAS_REG, BMP280_MODE1};
        I2CDeviceLock != NULL ? I2CDeviceLock->lock() : void();
        write(BMP280FD, input, 2);
        I2CDeviceLock != NULL ? I2CDeviceLock->unlock() : void();
        //
        usleep(30000);
        //
        int h;
        uint8_t D[6] = {0};
        const char output = BMP280_PRESSURE_MSB_REG;

        I2CDeviceLock != NULL ? I2CDeviceLock->lock() : void();
        write(BMP280FD, &output, 1);
        h = read(BMP280FD, D, sizeof(D));
        I2CDeviceLock != NULL ? I2CDeviceLock->unlock() : void();

        if (h == 6)
        {
            // Temperature Raw
            {
                bmp280RawTemperature = (int32_t)((((uint32_t)(D[3])) << 12) | (((uint32_t)(D[4])) << 4) | ((uint32_t)D[5] >> 4));
                double var1, var2;
                var1 = (((double)bmp280RawTemperature) / 16384.0 - ((double)(uint16_t)C[0]) / 1024.0) * ((double)C[1]);
                var2 = ((((double)bmp280RawTemperature) / 131072.0 - ((double)(uint16_t)C[0]) / 8192.0) *
                        (((double)bmp280RawTemperature) / 131072.0 - ((double)(uint16_t)C[0]) / 8192.0)) *
                       ((double)C[2]);

                t_fine = (int32_t)(var1 + var2);
                Data.TemperatureC = (var1 + var2) / 5120.0;
            }
            // Pressure Raw
            {
                bmp280RawPressure = (int32_t)((((uint32_t)(D[0])) << 12) | (((uint32_t)(D[1])) << 4) | ((uint32_t)D[2] >> 4));

                double var1, var2;
                var1 = ((double)t_fine / 2.0) - 64000.0;
                var2 = var1 * var1 * ((double)C[8]) / 32768.0;
                var2 = var2 + var1 * ((double)C[7]) * 2.0;
                var2 = (var2 / 4.0) + (((double)C[6]) * 65536.0);
                var1 = (((double)C[5]) * var1 * var1 / 524288.0 + ((double)C[4]) * var1) / 524288.0;
                var1 = (1.0 + var1 / 32768.0) * ((double)(uint16_t)C[3]);
                if (var1 != 0.0)
                {
                    Data.PressureHPA = 1048576.0 - (double)bmp280RawPressure;
                    Data.PressureHPA = (Data.PressureHPA - (var2 / 4096.0)) * 6250.0 / var1;
                    var1 = ((double)C[11]) * Data.PressureHPA * Data.PressureHPA / 2147483648.0;
                    var2 = Data.PressureHPA * ((double)C[10]) / 32768.0;
                    double pressure_Pa = Data.PressureHPA + (var1 + var2 + ((double)C[9])) / 16.0;

                    Data.PressureHPA = pressure_Pa / 100.0;
                    Data.AltitudeM = 44330.0 * (1.0 - pow((Data.PressureHPA / DEFAULT_SEA_PRESSURE), 0.1902949));
                    Data.AltitudeM = (float)((int)(Data.AltitudeM * 100.0f + 0.5f)) / 100.0f;

                    Data.IsDataCorrect = true;
                }
                else
                {
                    Data.PressureHPA = 0;
                    Data.AltitudeM = 0;
                    Data.IsDataCorrect = false;
                }
            }
        }
        return Data;
    }

    ~BMP280Baro()
    {
        close(BMP280FD);
    };

    typedef enum
    {
        BMP280_SLEEP_MODE = 0x0,
        BMP280_FORCED_MODE = 0x1,
        BMP280_NORMAL_MODE = 0x3
    } BMP280_WORK_MODE;

    typedef enum
    {
        BMP280_MODE_SKIP = 0x0,
        BMP280_MODE_1X,
        BMP280_MODE_2X,
        BMP280_MODE_4X,
        BMP280_MODE_8X,
        BMP280_MODE_16X
    } BMP280_OVERSAMPLING;

    typedef enum
    {
        BMP280_FILTER_OFF = 0x0,
        BMP280_FILTER_2X,
        BMP280_FILTER_4X,
        BMP280_FILTER_8X,
        BMP280_FILTER_16X
    } BMP280_FILTER_COEFFICIENT;

    typedef enum
    {
        BMP280_T_SB1 = 0x0, /*0.5ms*/
        BMP280_T_SB2,       /*62.5ms*/
        BMP280_T_SB3,       /*125ms*/
        BMP280_T_SB4,       /*250ms*/
        BMP280_T_SB5,       /*500ms*/
        BMP280_T_SB6,       /*1000ms*/
        BMP280_T_SB7,       /*2000ms*/
        BMP280_T_SB8,       /*4000ms*/
    } BMP280_T_SB;

private:
    uint8_t wdata[2];
    uint8_t rdata[24];
    uint8_t iddata;
    int16_t C[12];
    uint32_t bmp280RawPressure = 0;
    uint32_t bmp280RawTemperature = 0;
    int BMP280FD;
    BMP280_S32_t t_fine;
};
