/**-------------------------------------------------------------------------
@file	MotionSensor.cpp

@brief	Motion sensor manager

This file implements motion sensors management initialization, interrupt
handling, data handling,..  


@author	Hoang Nguyen Hoan
@date	Dec. 27, 2024

@license

MIT License

Copyright (c) 2024, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include "iopinctrl.h"
#include "coredev/i2c.h"
#include "coredev/spi.h"
#include "coredev/uart.h"
#include "sensors/agm_invn_icm20948.h"
#include "sensors/ag_bmi323.h"
#include "sensors/mag_bmm350.h"

#include "SlimeTrackerESB.h"

#include "board.h"

extern UART g_Uart;

typedef struct {
	DeviceIntrf *pIntrf;
	Sensor *pSensor;
} MotionDevice_t;

static const AccelSensorCfg_t s_AccelCfg = {
	.DevAddr = 0,// SPI CS index,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 50000,
	.Scale = 2,
	.FltrFreq = 0,
	.bInter = true,
	.IntPol = DEVINTR_POL_LOW,
};

static const GyroSensorCfg_t s_GyroCfg = {
	.DevAddr = 0,//BMI323_I2C_7BITS_DEVADDR,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 50000,
	.Sensitivity = 10,
	.FltrFreq = 200,
};

static const MagSensorCfg_t s_MagCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,//SENSOR_OPMODE_SINGLE,
	.Freq = 50000,
	.Precision = MAGSENSOR_PRECISION_HIGH,
};


//#if BOARD == BLUEIO_TAG_EVIM

AgmInvnIcm20948 g_Icm20948;

//#elif BOARD == BLYST_MOTION

AgBmi323 g_Bmi323;
MagBmm350 g_Bmi350;

//#else
//#error "No board defined"
//#endif
MotionDevice_t g_MotionSensor[] = {
};

AccelSensor *g_pAccel = nullptr;
GyroSensor *g_pGyro = nullptr;
MagSensor *g_pMag = nullptr;

void ImuIntHandler(int IntNo, void *pCtx)
{

}

bool InitSensors(DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	// IMU init
	bool res = g_Icm20948.Init(s_AccelCfg, pIntrf, pTimer);

	if (res)
	{
		// Found ICM20948
		g_Uart.printf("Found ICM20948\r\n");

		g_pAccel = &g_Icm20948;
		res = g_Icm20948.Init(s_GyroCfg, pIntrf, pTimer);
		{
			g_pGyro = &g_Icm20948;
			res = g_Icm20948.Init(s_MagCfg, pIntrf, pTimer);
			if (res)
			{
				g_pMag = &g_Icm20948;

				return true;
			}
		}
	}
	else
	{
		res = g_Bmi323.Init(s_AccelCfg, pIntrf, pTimer);
		if (res)
		{
			g_Uart.printf("Found BMI323\r\n");
			g_pAccel = &g_Bmi323;
			res = g_Bmi323.Init(s_GyroCfg, pIntrf, pTimer);
			{
				g_pGyro = &g_Bmi323;
				res = g_Bmi350.Init(s_MagCfg, pIntrf, pTimer);
				if (res)
				{
					g_pMag = &g_Bmi350;
				}

				return true;
			}
		}
	}

	return g_pAccel != nullptr;
}
