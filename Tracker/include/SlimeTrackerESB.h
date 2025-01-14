/**-------------------------------------------------------------------------
@file	SlimeTrackerESB.h

@brief	ESB definition file.

Implementation based on SlimenRF ESB protocol

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
#ifndef __SLIMETRACKERESB_H__
#define __SLIMETRACKERESB_H__

#include "nrf_esb.h"
#include "coredev/timer.h"
#include "coredev/i2c.h"
#include "coredev/spi.h"
#include "sensors/accel_sensor.h"
#include "sensors/gyro_sensor.h"
#include "sensors/mag_sensor.h"
#include "imu/imu.h"


#pragma pack(push, 1)

typedef struct {
	uint8_t Id;					//!< Packet id = 0
	uint8_t TrackerId;
	uint8_t BatLevel;
	uint8_t BatVolt;
	uint8_t SensorTemp;			//!< Sensor temperature
	uint8_t BoardId;			//!< Board id
	uint8_t McuId;				//!< MCU id
	uint16_t FwBuild;
	uint8_t FwMajor;
	uint8_t FwMinor;
	uint8_t FwPatch;
	uint8_t Rssi;
} EsbPktDevInfo_t;

typedef struct {
	uint8_t Id;					//!< Packet id = 1
	uint8_t TrackerId;
	uint16_t Quat[4];			//!< Fixed point 15 format
	uint16_t Acc[3];			//!< Fixed point 15 format
} EsbPktPrecisionAccQuat_t;

typedef struct {
	uint8_t Id;					//!< Packet id = 2
	uint8_t TrackerId;
	uint8_t BatLevel;
	uint8_t BatVolt;
	uint8_t SensorTemp;			//!< Sensor temperature
	uint32_t Q;	// ???
	uint16_t Acc[3];
	uint8_t Rssi;
} EsbPktAccQuat_t;

typedef struct {
	uint8_t Id;					//!< Packet id = 2
	uint8_t TrackerId;
	uint8_t TrakerSvrStatus;
	uint8_t TrackerStatus;
	uint8_t Rssi;
} EsbPktStatus_t;

#pragma pack(pop)

#define RECEIVER_ID_LENGTH		8

#pragma pack(push, 4)
typedef struct {
	Imu * const pImuDev;
	AccelSensor * const pAccel;
	DeviceIntrf * const pAccIntrf;
	GyroSensor * const pGyro;
	DeviceIntrf * const pGyroIntrf;
	MagSensor * const pMag;
	DeviceIntrf * const pMagIntrf;
	uint8_t NbAxes;
} MotionDevice_t;
#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

bool IsPaired(void);
void SetReceiverMacAddr(uint64_t Id);
uint64_t GetReceiverMacAddr();
void SetTrackerId(uint8_t Id);
bool EsbInit(void);
bool EsbSendPairing(void);
bool EsbSend(uint8_t *pData, size_t Len);
bool InitSensors(const MotionDevice_t * const pMotDev, size_t Count, Timer * const pTimer);
SPI * const GetSpi(void);
I2C * const GetI2c(void);
void ImuIntHandler(int IntNo, void *pCtx);

#ifdef __cplusplus
}
#endif

#endif // __SLIMETRACKERESB_H__
