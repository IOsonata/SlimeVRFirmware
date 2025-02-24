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

#include "SlimeTracker.h"

#define FIRMWARE_VERSION	0

#define MAX_BATTERY_VOLTAGE 4.15 //[V]

#pragma pack(push, 1)

typedef enum {
	ESBPKT_TYPE_DEVINFO,
	ESBPKT_TYPE_PRECISE_QUAT,
	ESBPKT_TYPE_QUAT,
	ESBPKT_TYPE_STATUS
} ESBPKT_TYPE;

typedef struct {
	uint8_t Id;					//!< Packet id
	uint8_t TrackerId;
} EWsbPktHdr_t;

typedef struct {
	uint8_t Id;					//!< Packet id = 0
	uint8_t TrackerId;
	uint8_t BatLevel;
	uint8_t BatVolt;
	uint8_t SensorTemp;			//!< Sensor temperature
	uint8_t BoardId;			//!< Board id
	uint8_t McuId;				//!< MCU id
	uint8_t Rsv;
	uint8_t ImuId;
	uint8_t MagId;
	uint16_t FwBuild;
	uint8_t FwMajor;
	uint8_t FwMinor;
	uint8_t FwPatch;
	uint8_t Rssi;
	uint8_t Pad;
} EsbPktDevInfo_t;

typedef struct {
	uint8_t Id;					//!< Packet id = 1
	uint8_t TrackerId;
	uint16_t Quat[4];			//!< Fixed point 15 format
	uint16_t Acc[3];			//!< Fixed point 7 format
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
	uint8_t Id;					//!< Packet id = 3
	uint8_t TrackerId;
	uint8_t TrakerSvrStatus;
	uint8_t TrackerStatus;
	uint8_t Rssi;
} EsbPktStatus_t;

typedef struct {
	size_t PktLen;
	union {
		EWsbPktHdr_t Hdr;
		EsbPktDevInfo_t DevInfo;
		EsbPktPrecisionAccQuat_t PreciseQuat;
		EsbPktAccQuat_t Quat;
		EsbPktStatus_t Status;
		uint8_t Data[16];
	};
} EsbPacket_t;

#pragma pack(pop)

#define RECEIVER_ID_LENGTH		8

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
void SetEsbPktTrackerId(uint8_t TrakerId);
bool EsbSendPacket(ESBPKT_TYPE PktType);
static inline bool EsbSendDeviceInfo() { return EsbSendPacket(ESBPKT_TYPE_DEVINFO); }
static inline bool EsbSendPreciseQuat() { return EsbSendPacket(ESBPKT_TYPE_PRECISE_QUAT); }
static inline bool EsbSendQuat() { return EsbSendPacket(ESBPKT_TYPE_QUAT); }
static inline bool EsbSendStatus() { return EsbSendPacket(ESBPKT_TYPE_STATUS); }
void EsbPktUpdateImu(AccelSensorData_t &Accel, int16_t Quat[4]);
float GetBattVolt();
void UpdateBattLevel();

#ifdef __cplusplus
}
#endif

#endif // __SLIMETRACKERESB_H__
