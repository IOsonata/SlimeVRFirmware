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
#include "imu/imu.h"

#include "Fusion/Fusion.h"
#include "SlimeTracker.h"

//extern UART g_Uart;

static const AccelSensorCfg_t s_AccelCfg = {
	.DevAddr = 0,// SPI CS index,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 50000,
	.Scale = 8,
	.FltrFreq = 0,
	.Inter = 1,
	.IntPol = DEVINTR_POL_LOW,
};

static const GyroSensorCfg_t s_GyroCfg = {
	.DevAddr = 0,//BMI323_I2C_7BITS_DEVADDR,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 50000,
	.Sensitivity = 4000,
	.FltrFreq = 0,
};

static const MagSensorCfg_t s_MagCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,//SENSOR_OPMODE_SINGLE,
	.Freq = 50000,
	.Precision = MAGSENSOR_PRECISION_HIGH,
	.bFifoEn = false,
};

const float s_AccelBias[3] = {
   -0.058f,   // X
   -0.001f,   // Y
   -0.112f    // Z
};

const float s_AccelSoftiron[3][3] = {
    {  0.6569f, -0.4138f, -0.2773f },
    { -0.4138f,  0.4748f,  0.1109f },
    { -0.2773f,  0.1109f,  0.2663f }
};

static const float s_GyroBias[3] = {
	 0.154f,   // X axis
	-0.768f,   // Y axis
	 0.596f    // Z axis
};

static const float s_MagGain[3][3] = {
#if 0
	{ 0.0454f,  0.0108f,  0.0124f },
	{ 0.0108f,  0.0504f,  0.0063f },
	{ 0.0124f,  0.0063f,  0.0522f }
#else
    { 0.0504f, -0.0108f,  0.0063f },
    {-0.0108f,  0.0454f, -0.0124f },
    { 0.0063f, -0.0124f,  0.0522f }
#endif
};

static const float s_MagBias[3] = {
#if 0
	48.46, -65.60, 43.76
#else
    65.60f,   // X' = -(-65.60)
   -48.46f,   // Y' = 48.46
    43.76f    // Z unchanged
#endif
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

Timer *g_pTimer = nullptr;

void ImuEvtHandler(Device * const pDev, DEV_EVT Evt);

static const ImuCfg_t s_ImuCfg = {
	.EvtHandler = ImuEvtHandler
};

uint32_t dt = 0;
void ImuIntHandler(int IntNo, void *pCtx)
{
	int16_t q[4];
	AccelSensorData_t accdata;
	GyroSensorData_t gyrodata;
	MagSensorData_t magdata;
	uint64_t t = g_pTimer->uSecond();

	if (g_pImu)
	{
		ImuQuat_t quat;
#if 1
		//g_Uart.printf("g_pImu->IntHandler()\r\n");
		g_pImu->IntHandler();
		//g_Uart.printf("g_pImu->IntHandler() - exit\r\n");
		g_pImu->Read(quat);
		g_pImu->Read(accdata);
		//g_pImu->Read(gyrodata);

		if (g_pMag)
		{
			g_pImu->Read(magdata);
		}

		q[0] = quat.Q[0] * (1 << 15);
		q[1] = quat.Q[1] * (1 << 15);
		q[2] = quat.Q[2] * (1 << 15);
		q[3] = quat.Q[3] * (1 << 15);

#else
		g_pAccel->IntHandler();
		g_pGyro->IntHandler();

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
/*
		q[0] = fq.array[0] * (1 << 15);
		q[1] = fq.array[1] * (1 << 15);
		q[2] = fq.array[2] * (1 << 15);
		q[3] = fq.array[3] * (1 << 15);
*/
		q[0] = g_Fusion.quaternion.array[0] * (1 << 15);
		q[1] = g_Fusion.quaternion.array[1] * (1 << 15);
		q[2] = g_Fusion.quaternion.array[2] * (1 << 15);
		q[3] = g_Fusion.quaternion.array[3] * (1 << 15);
#endif
		//g_Uart.printf("%d %d %d %d\r\n", q[0], q[1], q[2], q[3]);


	}
	else
	{
		g_pAccel->IntHandler();
		g_pGyro->IntHandler();
		if (g_pMag)
		{
			//g_pMag->IntHandler();
		}
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

		q[0] = fq.array[0] * (1 << 15);
		q[1] = fq.array[1] * (1 << 15);
		q[2] = fq.array[2] * (1 << 15);
		q[3] = fq.array[3] * (1 << 15);
	}

	dt = g_pTimer->uSecond() - t;

	SendMotionData(accdata, q);
	if (g_pMag)
	{
		SendMotionDataMag(magdata, q);
	}

	cli_printf("ITS:%d T:%.2f A:%.4f %.4f %.4f, G:%.4f %.4f %.4f, MTS: %d T:%.2f M:%.4f %.4f %.4f\n", (uint32_t)(accdata.Timestamp / 1000), accdata.Temp, accdata.X, accdata.Y, accdata.Z, gyrodata.X, gyrodata.Y, gyrodata.Z, (uint32_t)(magdata.Timestamp / 1000.0), magdata.Temp, magdata.X, magdata.Y, magdata.Z);
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

			SendMotionData(accdata, q);

			break;
	}
}

void MagIntHandler(int IntNo, void *pCtx)
{
	MagSensorData_t d;
	MagSensorRawData_t raw;

	if (g_pMag)
	{
		g_pMag->IntHandler();

		g_pMag->Read(raw);
		g_pMag->Read(d);

	}


	//cli_printf("Mag int\n");
	//cli_printf("%.4f %.4f %.4f\n", d.X, d.Y, d.Z);
}

bool InitSensors(const MotionDevice_t * const pMotDev, size_t Count, Timer * const pTimer)//DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (pMotDev == nullptr || Count == 0)
	{
		return false;
	}

	g_pTimer = pTimer;

	bool res = false;

	for (int i = 0; i < Count; i++)
	{
		if (pMotDev[i].pAccel == nullptr || pMotDev[i].pGyro == nullptr)
		{
			continue;
		}
		//g_Uart.printf("Iter %d\r\n", i);
#if 0
		switch (i)
		{
		case BMI270_BMM350_SPI:
			g_Uart.printf("Looking for BMI270 and BMM 350 via SPI interface\r\n");
			break;
		case BMI270_BMM350_I2C:
			g_Uart.printf("Looking for BMI270 and BMM 350 via I2C interface\r\n");
			break;
		case BMI323_NONE_SPI:
			g_Uart.printf("Looking for BMI323 via SPI interface\r\n");
			break;
		case ICM20948_ICM20498_SPI:
			g_Uart.printf("Looking for ICM20948 via SPI interface\r\n");
			break;
		case ICM20948_ICM20498_I2C:
			g_Uart.printf("Looking for I2M20948 via SPI interface\r\n");
			break;
		default:
			g_Uart.printf("Unknown motion sensor\r\n");
		}
#else
		cli_printf(pMotDev[i].pDesc);
		cli_printf("\r\n");
#endif
		res = pMotDev[i].pAccel->Init(s_AccelCfg, pMotDev[i].pAccIntrf, pTimer);
		if (res == true)
		{
			cli_printf("Accel found\r\n");

			res = pMotDev[i].pGyro->Init(s_GyroCfg, pMotDev[i].pGyroIntrf, pTimer);
			if (res == true)
			{
				cli_printf("Gyro found\r\n");

				if (pMotDev[i].pMag)
				{
					//pMotDev[i].pAccel->Enable();
					pMotDev[i].pMagIntrf->Enable();
					res = pMotDev[i].pMag->Init(s_MagCfg, pMotDev[i].pMagIntrf, pTimer);
					if (res)
					{
						cli_printf("Mag found\r\n");
						g_pMag = pMotDev[i].pMag;
						g_pMag->SetCalibration(s_MagGain, s_MagBias);
					}
				}
				if (pMotDev[i].pImuDev)
				{
					g_pAccel = pMotDev[i].pAccel;
					g_pAccel->SetCalibration(s_AccelSoftiron, s_AccelBias);

					g_pGyro = pMotDev[i].pGyro;
					g_pGyro->SetCalibrationOffset(s_GyroBias);

					res = pMotDev[i].pImuDev->Init(s_ImuCfg, g_pAccel, g_pGyro, g_pMag);
					if (res)
					{
						g_pImu = pMotDev[i].pImuDev;
						g_pImu->Enable();
						res = true;
						FusionAhrsInitialise(&g_Fusion);
						break;
					}
				}
				else
				{
					cli_printf("No imu\r\n");

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
					cli_printf("Sensor found\r\n");
					break;
				}
			}
		}
	}

	return res;
}
