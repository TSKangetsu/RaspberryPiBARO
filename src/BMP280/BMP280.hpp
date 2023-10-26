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

#define BMI280_ADDRESS 0x76
#define MS5611RESETADDR 0xe0
#define MS5611RESET 0xb6
#define BMP280_ID_REG 0xD0
#define BMP280_STATUS_REG 0xF3 /*Status Register */

#define BMP280_DIG_T1_LSB_REG 0x88
#define BMP280_DIG_T1_MSB_REG 0x89
#define BMP280_DIG_T2_LSB_REG 0x8A
#define BMP280_DIG_T2_MSB_REG 0x8B
#define BMP280_DIG_T3_LSB_REG 0x8C
#define BMP280_DIG_T3_MSB_REG 0x8D
#define BMP280_DIG_P1_LSB_REG 0x8E
#define BMP280_DIG_P1_MSB_REG 0x8F
#define BMP280_DIG_P2_LSB_REG 0x90
#define BMP280_DIG_P2_MSB_REG 0x91
#define BMP280_DIG_P3_LSB_REG 0x92
#define BMP280_DIG_P3_MSB_REG 0x93
#define BMP280_DIG_P4_LSB_REG 0x94
#define BMP280_DIG_P4_MSB_REG 0x95
#define BMP280_DIG_P5_LSB_REG 0x96
#define BMP280_DIG_P5_MSB_REG 0x97
#define BMP280_DIG_P6_LSB_REG 0x98
#define BMP280_DIG_P6_MSB_REG 0x99
#define BMP280_DIG_P7_LSB_REG 0x9A
#define BMP280_DIG_P7_MSB_REG 0x9B
#define BMP280_DIG_P8_LSB_REG 0x9C
#define BMP280_DIG_P8_MSB_REG 0x9D
#define BMP280_DIG_P9_LSB_REG 0x9E
#define BMP280_DIG_P9_MSB_REG 0x9F

#define BMP280_SLEEP_MODE 0x00
#define BMP280_FORCED_MODE 0x01
#define BMP280_NORMAL_MODE 0x03

#define BMP280_OVERSAMP_SKIPPED 0x00
#define BMP280_OVERSAMP_1X 0x01
#define BMP280_OVERSAMP_2X 0x02
#define BMP280_OVERSAMP_4X 0x03
#define BMP280_OVERSAMP_8X 0x04
#define BMP280_OVERSAMP_16X 0x05

#define BMP280_FILTER_OFF 0x00                  /*filter off*/
#define BMP280_FILTER_MODE_1 0x01 /*0.223*ODR*/ /*x1*/
#define BMP280_FILTER_MODE_2 0x02 /*0.092*ODR*/ /*x2*/
#define BMP280_FILTER_MODE_3 0x03 /*0.042*ODR*/ /*x4*/
#define BMP280_FILTER_MODE_4 0x04 /*0.021*ODR*/ /*x8*/
#define BMP280_FILTER_MODE_5 0x05 /*0.021*ODR*/ /*x16*/

#define BMP280_T_SB1 0x00 /*0.5ms*/
#define BMP280_T_SB2 0x01 /*62.5ms*/
#define BMP280_T_SB3 0x02 /*125ms*/
#define BMP280_T_SB4 0x03 /*250ms*/
#define BMP280_T_SB5 0x04 /*500ms*/
#define BMP280_T_SB6 0x05 /*1000ms*/
#define BMP280_T_SB7 0x06 /*2000ms*/
#define BMP280_T_SB8 0x07 /*4000ms*/

#define BMP280_CTRLMEAS_REG 0xF4         /*Ctrl Measure Register */
#define BMP280_CONFIG_REG 0xF5           /*Configuration Register */
#define BMP280_PRESSURE_MSB_REG 0xF7     /*Pressure MSB Register */
#define BMP280_PRESSURE_LSB_REG 0xF8     /*Pressure LSB Register */
#define BMP280_PRESSURE_XLSB_REG 0xF9    /*Pressure XLSB Register */
#define BMP280_TEMPERATURE_MSB_REG 0xFA  /*Temperature MSB Reg */
#define BMP280_TEMPERATURE_LSB_REG 0xFB  /*Temperature LSB Reg */
#define BMP280_TEMPERATURE_XLSB_REG 0xFC /*Temperature XLSB Reg */

#define BMP280_MODE1 (BMP280_OVERSAMP_16X << 2 | BMP280_OVERSAMP_16X << 5 | BMP280_NORMAL_MODE)
#define BMP280_MODE2 (BMP280_FILTER_MODE_5 << 2 | BMP280_T_SB2 << 5)

#define BMP280_MEASURING 0x01
#define BMP280_IM_UPDATE 0x08

typedef long signed int BMP280_S32_t;
typedef long unsigned int BMP280_U32_t;
typedef long long signed int BMP280_S64_t;
BMP280_S32_t t_fine;

class BMP280Baro
{
public:
    BMP280Baro(const BMP280Baro &other) = delete;
    inline BMP280Baro(const char *I2CChannel = "/dev/i2c-7", uint8_t I2CAddress = BMI280_ADDRESS)
    {
        if ((BMP280FD = open(I2CChannel, O_RDWR)) < 0)
            throw -1;
        if (ioctl(BMP280FD, I2C_SLAVE, I2CAddress) < 0)
            throw -2;
        if (ioctl(BMP280FD, I2C_TIMEOUT, 0x01) < 0) // set to 10ms?
            throw -2;

        wdata[0] = BMP280_DIG_T1_LSB_REG;
        write(BMP280FD, wdata, 1); // 读取温度矫正值
        read(BMP280FD, rdata, 6);
        bmp280Cal.dig_T1 = (rdata[1] << 8) + rdata[0];
        bmp280Cal.dig_T2 = (rdata[3] << 8) + rdata[2];
        bmp280Cal.dig_T3 = (rdata[5] << 8) + rdata[4];

        wdata[0] = BMP280_DIG_P1_LSB_REG; // 读取气压矫正值
        write(BMP280FD, wdata, 1);
        read(BMP280FD, rdata, 18);
        bmp280Cal.dig_P1 = (rdata[1] << 8) + rdata[0];
        bmp280Cal.dig_P2 = (rdata[3] << 8) + rdata[2];
        bmp280Cal.dig_P3 = (rdata[5] << 8) + rdata[4];
        bmp280Cal.dig_P4 = (rdata[7] << 8) + rdata[6];
        bmp280Cal.dig_P5 = (rdata[9] << 8) + rdata[8];
        bmp280Cal.dig_P6 = (rdata[11] << 8) + rdata[10];
        bmp280Cal.dig_P7 = (rdata[13] << 8) + rdata[12];
        bmp280Cal.dig_P8 = (rdata[15] << 8) + rdata[14];
        bmp280Cal.dig_P9 = (rdata[17] << 8) + rdata[16];

        wdata[0] = MS5611RESETADDR;
        wdata[1] = MS5611RESET;
        if (write(BMP280FD, wdata, 2) != 2) // 复位
            throw -3;
        usleep(500);
        wdata[0] = BMP280_ID_REG;
        write(BMP280FD, wdata, 1); // 读取id
        read(BMP280FD, rdata, 1);
        usleep(500);
        if (int(rdata[0]) == 88)
        {

            wdata[0] = BMP280_CTRLMEAS_REG;
            wdata[1] = BMP280_MODE1; // 16倍率，normal
            write(BMP280FD, wdata, 2);
            usleep(500);
            wdata[0] = BMP280_CONFIG_REG;
            wdata[1] = BMP280_MODE2; // iir16倍率，65ms
            write(BMP280FD, wdata, 2);

            // std::cout << "dig_T1: " << bmp280Cal.dig_T1 << "\r\n";
            // std::cout << "dig_T2: " << bmp280Cal.dig_T2 << "\r\n";
            // std::cout << "dig_T3: " << bmp280Cal.dig_T3 << "\r\n";
            // std::cout << "bmp280Cal.dig_P1: " << bmp280Cal.dig_P1 << "\r\n";
            // std::cout << "bmp280Cal.dig_P2: " << bmp280Cal.dig_P2 << "\r\n";
            // std::cout << "bmp280Cal.dig_P3: " << bmp280Cal.dig_P3 << "\r\n";
            // std::cout << "bmp280Cal.dig_P4: " << bmp280Cal.dig_P4 << "\r\n";
            // std::cout << "bmp280Cal.dig_P5: " << bmp280Cal.dig_P5 << "\r\n";
            // std::cout << "bmp280Cal.dig_P6: " << bmp280Cal.dig_P6 << "\r\n";
            // std::cout << "bmp280Cal.dig_P7: " << bmp280Cal.dig_P7 << "\r\n";
            // std::cout << "bmp280Cal.dig_P8: " << bmp280Cal.dig_P8 << "\r\n";
            // std::cout << "bmp280Cal.dig_P9: " << bmp280Cal.dig_P9 << "\r\n";
            // std::cout << "BMP280_MODE: " << std::hex << BMP280_MODE2 << "\r\n";
        }
        else
            throw -4;
    }

    inline BaroData BMP280Read()
    {
        BaroData Data;

        // wdata[0] = BMP280_CTRLMEAS_REG;
        // wdata[1] = BMP280_MODE1;
        // write(BMP280FD, wdata, 2); //进入force模式才使用
        while (BMP280_GetStatus(BMP280_MEASURING) != 0)
            ;
        while (BMP280_GetStatus(BMP280_IM_UPDATE) != 0)
            ; // 检测数据是否传输完成
        wdata[0] = BMP280_PRESSURE_MSB_REG;
        write(BMP280FD, wdata, 1); // 读数据
        read(BMP280FD, rdata, 6);
        bmp280RawPressure = (uint32_t)rdata[0] << 12 + (uint32_t)rdata[1] << 4 + (uint32_t)rdata[2] >> 4;
        bmp280RawTemperature = (uint32_t)rdata[3] << 12 + (uint32_t)rdata[4] << 4 + (uint32_t)rdata[5] >> 4;
        Data.PressureHPA = bmp280_compensate_P_double(bmp280RawPressure); // 带入数据手册的公式
        Data.TemperatureC = bmp280_compensate_T_double(bmp280RawTemperature);

        std::cout << "Data.PressureHPA: " << Data.PressureHPA << "\r\n";
        std::cout << "Data.TemperatureC: " << Data.TemperatureC << "\r\n";
        return Data;
    }

    ~BMP280Baro()
    {
        close(BMP280FD);
    };

    BMP280_S32_t bmp280_compensate_T_int32(BMP280_S32_t adc_T)
    {
        BMP280_S32_t var1, var2, T;
        var1 = ((((adc_T >> 3) - ((BMP280_S32_t)bmp280Cal.dig_T1 << 1))) * ((BMP280_S32_t)bmp280Cal.dig_T2)) >> 11;
        var2 = (((((adc_T >> 4) - ((BMP280_S32_t)bmp280Cal.dig_T1)) * ((adc_T >> 4) - ((BMP280_S32_t)bmp280Cal.dig_T1))) >> 12) *
                ((BMP280_S32_t)bmp280Cal.dig_T3)) >>
               14;
        t_fine = var1 + var2;
        T = (t_fine * 5 + 128) >> 8;
        return T;
    }

    BMP280_U32_t bmp280_compensate_P_int64(BMP280_S32_t adc_P)
    {
        BMP280_S64_t var1, var2, p;
        var1 = ((BMP280_S64_t)t_fine) - 128000;
        var2 = var1 * var1 * (BMP280_S64_t)bmp280Cal.dig_P6;
        var2 = var2 + ((var1 * (BMP280_S64_t)bmp280Cal.dig_P5) << 17);
        var2 = var2 + (((BMP280_S64_t)bmp280Cal.dig_P4) << 35);
        var1 = ((var1 * var1 * (BMP280_S64_t)bmp280Cal.dig_P3) >> 8) + ((var1 * (BMP280_S64_t)bmp280Cal.dig_P2) << 12);
        var1 = (((((BMP280_S64_t)1) << 47) + var1)) * ((BMP280_S64_t)bmp280Cal.dig_P1) >> 33;
        if (var1 == 0)
        {
            return 0; // avoid exception caused by division by zero
        }
        p = 1048576 - adc_P;
        p = (((p << 31) - var2) * 3125) / var1;
        var1 = (((BMP280_S64_t)bmp280Cal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        var2 = (((BMP280_S64_t)bmp280Cal.dig_P8) * p) >> 19;
        p = ((p + var1 + var2) >> 8) + (((BMP280_S64_t)bmp280Cal.dig_P7) << 4);
        return (BMP280_U32_t)p;
    }

    double bmp280_compensate_T_double(BMP280_S32_t adc_T)
    {
        double var1, var2, T;
        var1 = (((double)adc_T) / 16384.0 - ((double)bmp280Cal.dig_T1) / 1024.0) * ((double)bmp280Cal.dig_T2);
        var2 = ((((double)adc_T) / 131072.0 - ((double)bmp280Cal.dig_T1) / 8192.0) *
                (((double)adc_T) / 131072.0 - ((double)bmp280Cal.dig_T1) / 8192.0)) *
               ((double)bmp280Cal.dig_T3);
        t_fine = (BMP280_S32_t)(var1 + var2);
        T = (var1 + var2) / 5120.0;
        return T;
    }

    // Returns pressure in Pa as double. Output value of “96386.2” equals 96386.2 Pa = 963.862 hPa
    double bmp280_compensate_P_double(BMP280_S32_t adc_P)
    {
        double var1, var2, p;
        var1 = ((double)t_fine / 2.0) - 64000.0;
        var2 = var1 * var1 * ((double)bmp280Cal.dig_P6) / 32768.0;
        var2 = var2 + var1 * ((double)bmp280Cal.dig_P5) * 2.0;
        var2 = (var2 / 4.0) + (((double)bmp280Cal.dig_P4) * 65536.0);
        var1 = (((double)bmp280Cal.dig_P3) * var1 * var1 / 524288.0 + ((double)bmp280Cal.dig_P2) * var1) / 524288.0;
        var1 = (1.0 + var1 / 32768.0) * ((double)bmp280Cal.dig_P1);
        if (var1 == 0.0)
        {
            return 0; // avoid exception caused by division by zero
        }
        p = 1048576.0 - (double)adc_P;
        p = (p - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = ((double)bmp280Cal.dig_P9) * p * p / 2147483648.0;
        var2 = p * ((double)bmp280Cal.dig_P8) / 32768.0;
        p = p + (var1 + var2 + ((double)bmp280Cal.dig_P7)) / 16.0;
        return p;
    }

    uint8_t BMP280_GetStatus(uint8_t status_flag)
    {
        uint8_t flag;
        wdata[0] = BMP280_STATUS_REG;
        write(BMP280FD, wdata, 1);
        read(BMP280FD, rdata, 1);
        if (rdata[0] & status_flag)
            return 1;
        else
            return 0;
    }

private:
    uint8_t wdata[2];
    uint8_t rdata[18];
    uint32_t bmp280RawPressure = 0;
    uint32_t bmp280RawTemperature = 0;
    int BMP280FD;
    BMP280_S32_t t_fine;
    typedef struct
    {
        uint16_t dig_T1; /* calibration T1 data */
        int16_t dig_T2;  /* calibration T2 data */
        int16_t dig_T3;  /* calibration T3 data */
        uint16_t dig_P1; /* calibration P1 data */
        int16_t dig_P2;  /* calibration P2 data */
        int16_t dig_P3;  /* calibration P3 data */
        int16_t dig_P4;  /* calibration P4 data */
        int16_t dig_P5;  /* calibration P5 data */
        int16_t dig_P6;  /* calibration P6 data */
        int16_t dig_P7;  /* calibration P7 data */
        int16_t dig_P8;  /* calibration P8 data */
        int16_t dig_P9;  /* calibration P9 data */
        int16_t t_fine;  /* calibration t_fine data */
    } bmp280Calib;
    bmp280Calib bmp280Cal;
};
