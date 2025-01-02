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

static const AccelSensorCfg_t s_AccelCfg = {
	.DevAddr = 0,// SPI CS index,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 50000,
	.Scale = 2,
	.FltrFreq = 0,
	.Inter = 1,
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

#if 0
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
#endif

AccelSensor *g_pAccel = nullptr;
GyroSensor *g_pGyro = nullptr;
MagSensor *g_pMag = nullptr;
Imu *g_pImu;

void ImuEvtHandler(Device * const pDev, DEV_EVT Evt);

static const ImuCfg_t s_ImuCfg = {
	.EvtHandler = ImuEvtHandler
};

void ImuIntHandler(int IntNo, void *pCtx)
{

}

void ImuEvtHandler(Device * const pDev, DEV_EVT Evt)
{

}

bool InitSensors(const MotionDevice_t * const pMotDev, size_t Count, Timer * const pTimer)//DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (pMotDev == nullptr || Count == 0)
	{
		return false;
	}

	bool res = false;

	for (int i = 0; i < Count; i++)
	{
		if (pMotDev[i].pAccel == nullptr || pMotDev[i].pGyro == nullptr)
		{
			continue;
		}

		res = pMotDev[i].pAccel->Init(s_AccelCfg, pMotDev[i].pAccIntrf, pTimer);
		if (res == true)
		{
			res = pMotDev[i].pGyro->Init(s_GyroCfg, pMotDev[i].pGyroIntrf, pTimer);
			if (res == true && pMotDev[i].pMag)
			{
				g_pGyro = pMotDev[i].pGyro;
				res = pMotDev[i].pMag->Init(s_MagCfg, pMotDev[i].pMagIntrf, pTimer);
				if (res)
				{
					g_pMag = pMotDev[i].pMag;
				}
			}
			if (res &&  pMotDev[i].pImuDev)
			{
				g_pAccel = pMotDev[i].pAccel;
				g_pGyro = pMotDev[i].pGyro;
				g_pMag = pMotDev[i].pMag;
				res = pMotDev[i].pImuDev->Init(s_ImuCfg, g_pAccel, g_pGyro, g_pMag);
				if (res)
				{
					g_pImu = pMotDev[i].pImuDev;
					break;
				}
			}
		}
	}

	return res;
}
