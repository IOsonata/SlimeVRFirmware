/**-------------------------------------------------------------------------
@file	SlimeTracker.h

@brief	Slime tracker definition file.

Implementation based on SlimenRF protocol

@author	Nguyen Hoan Hoang
@date	Dec. 24, 2024

@license

MIT License

Copyright (c) 2024 I-SYST inc. All rights reserved.

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
#ifndef __SLIMETRACKER_H__
#define __SLIMETRACKER_H__

#include <stdarg.h>

#include "nrf_cli.h"

#include "coredev/timer.h"
#include "coredev/i2c.h"
#include "coredev/spi.h"
#include "sensors/accel_sensor.h"
#include "sensors/gyro_sensor.h"
#include "sensors/mag_sensor.h"
#include "imu/imu.h"

#define BUILD_DAY			(BUILDN - 100 * (BUILDN / 100))
#define BUILD_MONTH			(BUILDN / 100 - 100 * (BUILDN / 10000))
#define BUILD_YEAR			(BUILDN / 10000 - 100 * (BUILDN / 1000000))


// ImuId - SlimeVR Displayed.  Does not correspond to those listed in Slime-nRF
// 1 - MPU9250
// 2 - MPU6500
// 3 - BNO080
// 4 - BNO085
// 5 - BNO055
// 6 - MPU6050
// 7 - BNO086
// 8 - BMI160
// 9 - ICM20948
// 10 - ICM42688

#define IMU_ID_MPU9250			1
#define IMU_ID_MPU6500   		2
#define IMU_ID_BNO080    		3
#define IMU_ID_BNO085    		4
#define IMU_ID_BNO055    		5
#define IMU_ID_MPU6050   		6
#define IMU_ID_BNO086    		7
#define IMU_ID_BMI160    		8
#define IMU_ID_ICM20948  		9
#define IMU_ID_ICM42688 		10

typedef enum {
	BMI270_BMM350_SPI = 0,
	BMI270_BMM350_I2C,
	BMI323_NONE_SPI,
	ICM20948_ICM20498_SPI,
	ICM20948_ICM20498_I2C,
} MotionDevInfo_e;

typedef enum __ADC_Pins {
	AIN0,
	AIN1,
	AIN2,
	AIN3,
	AIN4,
	AIN5,
	AIN6,
	AIN7,
	AVdd,
} ADC_PINS;

#define MAX_BATTERY_VOLTAGE 4.1 //[V]

#pragma pack(push, 4)
typedef struct {
	Imu * const pImuDev;
	AccelSensor * const pAccel;
	DeviceIntrf * const pAccIntrf;
	GyroSensor * const pGyro;
	DeviceIntrf * const pGyroIntrf;
	MagSensor * const pMag;
	DeviceIntrf * const pMagIntrf;
//	uint8_t NbAxes;
	const char * const pDesc;
} MotionDevice_t;
#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

bool InitSensors(const MotionDevice_t * const pMotDev, size_t Count, Timer * const pTimer);
SPI * const GetSpi(void);
I2C * const GetI2c(void);
void ImuIntHandler(int IntNo, void *pCtx);
void MagIntHandler(int IntNo, void *pCtx);
void SendMotionData(AccelSensorData_t &Accel, int16_t Quat[4]);
void ClearPairingInfo();
void ADVEventHandler(Device *pDevObj, DEV_EVT Evt);
void MeasureBatteryVoltage();

#define cli_printf(Format, ...) nrf_cli_fprintf(&s_Cli, NRF_CLI_DEFAULT, Format, ##__VA_ARGS__)

extern nrf_cli_t const s_Cli;

#ifdef __cplusplus
}
#endif

#endif // __SLIMETRACKER_H__
