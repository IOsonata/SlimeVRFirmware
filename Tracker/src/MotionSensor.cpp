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
#include "sensors/agm_icm20948.h"
#include "sensors/agm_invn_icm20948.h"
#include "sensors/ag_bmi323.h"
#include "sensors/mag_bmm350.h"

#include "Fusion/Fusion.h"
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
	.FltrFreq = 0,
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

FusionAhrs g_Fusion;

void ImuEvtHandler(Device * const pDev, DEV_EVT Evt);

static const ImuCfg_t s_ImuCfg = {
	.EvtHandler = ImuEvtHandler
};


void ImuIntHandler(int IntNo, void *pCtx)
{
	if (g_pImu)
	{
		g_pImu->IntHandler();
	}
	else
	{
		g_pAccel->IntHandler();

		AccelSensorData_t accdata;
		GyroSensorData_t gyrodata;
		MagSensorData_t magdata;

		g_pAccel->Read(accdata);
		g_pGyro->Read(gyrodata);

		FusionVector gyroscope = {gyrodata.X, gyrodata.Y, gyrodata.Z}; // replace this with actual gyroscope data in degrees/s
		FusionVector accelerometer = {accdata.X, accdata.Y, accdata.Z}; // replace this with actual accelerometer data in g

		if (g_pMag)
		{
			g_pMag->Read(magdata);
			FusionVector magnetometer = {magdata.X, magdata.Y, magdata.Z};
			FusionAhrsUpdate(&g_Fusion, gyroscope, accelerometer, magnetometer, 0.02);
		}
		else
		{
			FusionAhrsUpdateNoMagnetometer(&g_Fusion, gyroscope, accelerometer, 0.02);
		}

		FusionQuaternion fq = FusionAhrsGetQuaternion(&g_Fusion);

		int16_t q[4];
		q[0] = fq.array[0] * (1 << 15);
		q[1] = fq.array[1] * (1 << 15);
		q[2] = fq.array[2] * (1 << 15);
		q[3] = fq.array[3] * (1 << 15);

		EsbPktUpdateImu(accdata, q);
	//	printf("Quat %d: %d %d %d\r\n", q[0], q[1], q[2], q[3]);
		EsbSendPacket(ESBPKT_TYPE_PRECISE_QUAT);
	}
}

void ImuEvtHandler(Device * const pDev, DEV_EVT Evt)
{
	int16_t q[4];
	AccelSensorData_t accdata;
	GyroSensorData_t gyrodata;
	ImuQuat_t quat;

	switch (Evt)
	{
		case DEV_EVT_DATA_RDY:
			g_pImu->Read(accdata);
			g_pImu->Read(gyrodata);
			g_pImu->Read(quat);

			q[0] = quat.Q[0] * (1 << 15);
			q[1] = quat.Q[1] * (1 << 15);
			q[2] = quat.Q[2] * (1 << 15);
			q[3] = quat.Q[3] * (1 << 15);
			EsbPktUpdateImu(accdata, q);
		//	printf("Quat %d: %d %d %d\r\n", q[0], q[1], q[2], q[3]);
			EsbSendPacket(ESBPKT_TYPE_PRECISE_QUAT);
			break;
	}
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
			g_Uart.printf("Accel found\r\n");

			res = pMotDev[i].pGyro->Init(s_GyroCfg, pMotDev[i].pGyroIntrf, pTimer);
			if (res == true && pMotDev[i].pMag)
			{
				res = pMotDev[i].pMag->Init(s_MagCfg, pMotDev[i].pMagIntrf, pTimer);
				if (res)
				{
					g_Uart.printf("Mag found\r\n");
					g_pMag = pMotDev[i].pMag;
				}
				if (pMotDev[i].pImuDev)
				{
					g_pAccel = pMotDev[i].pAccel;
					g_pGyro = pMotDev[i].pGyro;
					res = pMotDev[i].pImuDev->Init(s_ImuCfg, g_pAccel, g_pGyro, g_pMag);
					if (res)
					{
						g_pImu = pMotDev[i].pImuDev;
						g_pImu->Enable();
						res = true;
						break;
					}
				}
				else
				{
					g_pAccel = pMotDev[i].pAccel;
					g_pGyro = pMotDev[i].pGyro;

					g_pAccel->Enable();
					g_pGyro->Enable();
					if (g_pMag)
					{
						g_pMag->Enable();
					}
					FusionAhrsInitialise(&g_Fusion);
					res = true;
					break;
				}
			}
		}
	}

	return res;
}
