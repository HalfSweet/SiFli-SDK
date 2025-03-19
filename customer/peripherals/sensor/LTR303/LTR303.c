/**
  ******************************************************************************
  * @file   LTR303.c
  * @author Sifli software development team
  * @brief   This file includes the LTR303 driver functions
  *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2025,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "LTR303.h"
#include <cstdint>
#include <rtthread.h>
#include "board.h"

#define DRV_DEBUG
#define LOG_TAG              "drv.als"
#include <drv_log.h>

// TODO: This parameters should be defined by board, move to configure?
//#define LTR303_I2C_NAME        "i2c2"

static struct rt_i2c_bus_device *LTR303_bus;

// Add delay ms
void DEV_Delay_ms(uint32_t delay)
{
    rt_thread_delay(delay);
}

// TODO:  gpio read
int DEV_Digital_Read(uint8_t pin)
{
    return 0;
}

// i2c bus initial
int LTR303_I2C_Init()
{
    /* get i2c bus device */
    LTR303_bus = rt_i2c_bus_device_find(LTR303_I2C_NAME);
    if (LTR303_bus)
    {
        LOG_D("Find i2c bus device %s\n", LTR303_I2C_NAME);
    }
    else
    {
        LOG_E("Can not found i2c bus %s, init fail\n", LTR303_I2C_NAME);
        return -1;
    }

    return 0;
}


/******************************************************************************
function:   Read one byte of data to LTR303 via I2C
parameter:
            Addr: Register address
Info:
******************************************************************************/
static uint8_t LTR303_Read_Byte(uint8_t Addr)
{
    //Addr = Addr | COMMAND_BIT;
    struct rt_i2c_msg msgs[2];
    uint8_t RegAddr = Addr | COMMAND_BIT;
    uint8_t value = 0;
    uint32_t res;

    if (LTR303_bus)
    {
        msgs[0].addr  = LTR303_ADDRESS;    /* Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = &RegAddr;         /* Slave register address */
        msgs[0].len   = 1;                /* Number of bytes sent */

        msgs[1].addr  = LTR303_ADDRESS;    /* Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = &value;             /* Read data pointer */
        msgs[1].len   = 1;              /* Number of bytes read */

        res = rt_i2c_transfer(LTR303_bus, msgs, 2);
        if (res != 2)
        {
            LOG_I("LTR303_Read_Byte fail with res %d\n", res);
        }
    }

    return value;
}

/******************************************************************************
function:   Read one word of data to LTR303 via I2C
parameter:
            Addr: Register address
Info:
******************************************************************************/
static uint32_t LTR303_Read_Word(uint8_t Addr)
{
    //Addr = Addr | COMMAND_BIT;
    struct rt_i2c_msg msgs[2];
    uint8_t RegAddr = Addr | COMMAND_BIT;
    uint32_t value = 0;
    uint8_t buf[4];
    uint32_t res;

    if (LTR303_bus)
    {
        msgs[0].addr  = LTR303_ADDRESS;    /* Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = &RegAddr;         /* Slave register address */
        msgs[0].len   = 1;                /* Number of bytes sent */

        msgs[1].addr  = LTR303_ADDRESS;    /* Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = buf;             /* Read data pointer */
        msgs[1].len   = 4;              /* Number of bytes read */

        res = rt_i2c_transfer(LTR303_bus, msgs, 2);
        if (res != 2)
        {
            LOG_I("LTR303_Read_Byte fail with res %d\n", res);
        }
    }

    value = (bfu[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];
    return value;
}

/******************************************************************************
function:   Send one byte of data to LTR303 via I2C
parameter:
            Addr: Register address
           Value: Write to the value of the register
Info:
******************************************************************************/
static void LTR303_Write_Byte(uint8_t Addr, uint8_t Value)
{
    //Addr = Addr | COMMAND_BIT;
    struct rt_i2c_msg msgs[2];
    uint8_t value[2];
    uint32_t res;

    if (LTR303_bus)
    {
        value[0] = Addr | COMMAND_BIT;
        value[1] = Value;

        msgs[0].addr  = LTR303_ADDRESS;    /* Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = value;             /* Slave register address */
        msgs[0].len   = 2;                /* Number of bytes sent */

        res = rt_i2c_transfer(LTR303_bus, msgs, 1);
        if (res != 1)
        {
            LOG_I("LTR303_Write_Byte FAIL %d\n", res);
        }
    }
}

void LTR303_PowerOn(void)
{
    uint8_t Reg = LTR303_Read_Byte(LTR303_ALS_CTRL);
    Reg |= 0x01;
    LTR303_Write_Byte(LTR303_ALS_CTRL, Reg);
}

void LTR303_PowerOff(void)
{
    uint8_t Reg = LTR303_Read_Byte(LTR303_ALS_CTRL);
    Reg &= 0xfe;
    LTR303_Write_Byte(LTR303_ALS_CTRL, Reg);
}

void LTR303_SetGain(ltr303_gain_t Gain)
{
    uint8_t Reg = LTR303_Read_Byte(LTR303_ALS_CTRL);
    // Gain(4:2)
    Reg &= 0xe3;
    Reg |= (Gain << 2);
    LTR303_Write_Byte(LTR303_ALS_CTRL, Reg);
}

ltr303_gain_t LTR303_GetGain(void)
{
    uint8_t data;
    data = LTR303_Read_Byte(LTR303_ALS_CTRL);
    uint8_t LTR303_Gain = (data & 0x1c) >> 2;
    return LTR303_Gain;
}

void LT303_SetIntegrationTime(ltr303_time_t Time)
{
    uint8_t Reg = LTR303_Read_Byte(LTR303_MEAS_RATE);
    // Time(5:3)
    Reg &= 0xc7;
    Reg |= (Time << 3);
    LTR303_Write_Byte(LTR303_ALS_CTRL, Reg);
}

ltr303_time_t LTR303_GetIntegrationTime(void)
{
    uint8_t data;
    data = LTR303_Read_Byte(LTR303_MEAS_RATE);
    uint8_t LTR303_Time = (data & 0x38) >> 3;
    return LTR303_Time;
}

void LT303_SetMeasurementRate(ltr303_rate_t Rate)
{
    uint8_t Reg = LTR303_Read_Byte(LTR303_MEAS_RATE);
    // Rate(2:0)
    Reg &= 0xf8;
    Reg |= Rate;
    LTR303_Write_Byte(LTR303_ALS_CTRL, Reg);
}

ltr303_rate_t LTR303_GetMeasurementRate(void)
{
    uint8_t data;
    data = LTR303_Read_Byte(LTR303_MEAS_RATE);
    uint8_t LTR303_Rate = data & 0x07;
    return LTR303_Rate;
}

rt_err_t LTR303_Init()
{
    LTR303_I2C_Init();

    LTR303_SetGain(LTR3XX_GAIN_1);
    LT303_SetIntegrationTime(LTR3XX_INTEGTIME_50);
    LT303_SetMeasurementRate(LTR3XX_MEASRATE_50);

    return RT_EOK;
}

uint16_t LTR303_ReadVisible(void)
{
    uint32_t data;
    data = LTR303_Read_Word(LTR303_CH1DATA);
    return data >> 16;
}


#include <string.h>

int cmd_als(int argc, char *argv[])
{
    uint32_t value;
    // if (argc >= 2)
    // {
    //     if (strcmp(argv[1], "-open") == 0)
    //     {
    //         int res = LTR303_Init();
    //         LOG_I("Open ALS %d\n", res);
    //         //LTR303_Enable();
    //     }
    //     else if (strcmp(argv[1], "-close") == 0)
    //     {
    //         //LTR303_Disable();
    //         LOG_I("Close als\n");
    //     }
    //     else if (strcmp(argv[1], "-visi") == 0)
    //     {
    //         value = LTR303_Read_Visible();
    //         LOG_I("get visible %d\n", value);
    //     }
    //     else if (strcmp(argv[1], "-inf") == 0)
    //     {
    //         value = LTR303_Read_Infrared();
    //         LOG_I("get infrared %d\n", value);
    //     }
    //     else if (strcmp(argv[1], "-full") == 0)
    //     {
    //         value = LTR303_Read_FullSpectrum();
    //         LOG_I("get fullspectrum %d\n", value);
    //     }
    //     else if (strcmp(argv[1], "-lux") == 0)
    //     {
    //         value = LTR303_Read_Lux();
    //         LOG_I("get lux %d\n", value);
    //     }
    //     else
    //     {
    //         LOG_I("Invalid parameter\n");
    //     }
    // }
    // else
    // {
    //     LOG_I("Invalid parameter\n");
    // }
    return 0;
}

FINSH_FUNCTION_EXPORT_ALIAS(cmd_als, __cmd_als, Test hw als);
