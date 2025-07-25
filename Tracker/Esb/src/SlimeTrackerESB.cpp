/**-------------------------------------------------------------------------
@file	SlimeTrackerESB.cpp

@brief	Slime protocol ESB implementation.

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
copies or substantial portions of the Softare.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/

#include "sdk_common.h"
#include "nrf_error.h"

#include "idelay.h"
//#include "crc.h"
#include "coredev/uart.h"
#include "nrf_mac.h"

#include "SlimeTrackerESB.h"

//extern UART g_Uart;

static const uint8_t DiscBaseAddr0[4] = {0x62, 0x39, 0x8A, 0xF2};
static const uint8_t DiscBaseAddr1[4] = {0x28, 0xFF, 0x50, 0xB8};
static const uint8_t DiscAddrPrefix[8] = {0xFE, 0xFF, 0x29, 0x27, 0x09, 0x02, 0xB2, 0xD6};
static const uint8_t crc8_ccitt_small_table[16] = {
	0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15,
	0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d
};

static uint8_t s_PairCrc = 0; // This is to validate pairing reply from receiver

/** Definition of packet that the SlimeVR tracker sends over ESB */
static EsbPacket_t s_EsbPacket[] = {
	{
		.PktLen = 16/*sizeof(EsbPktDevInfo_t)*/,
		.DevInfo = {
			.Id = 0,
			.ImuId = 16,
			.MagId = 2,
			.FwBuild = (uint16_t)((BUILD_YEAR - 20) << 9) | (BUILD_MONTH << 5) | BUILD_DAY,
			.FwMajor = FIRMWARE_VERSION >> 8,
			.FwMinor = FIRMWARE_VERSION & 0xFF,
		}
	},
	{
		.PktLen = 16/*sizeof(EsbPktPrecisionAccQuat_t)*/,
		.PreciseQuat =  {
			.Id = 1,
		}
	},
	{
		.PktLen = 16/*sizeof(EsbPktAccQuat_t)*/,
		.Quat =  {
			.Id = 2,
		}
	},
	{
		.PktLen = 16/*sizeof(EsbPktStatus_t)*/,
		.Status =  {
			.Id = 3,
		}
	},
	{
		.PktLen = 16/*sizeof(EsbPktStatus_t)*/,
		.MagQuat =  {
			.Id = 4,
		}
	},
};

EsbPktDevInfo_t &g_EsbPktDevInfo = s_EsbPacket[ESBPKT_TYPE_DEVINFO].DevInfo;

EsbPktPrecisionAccQuat_t &g_EsbPktPrecisionAccQuat = s_EsbPacket[ESBPKT_TYPE_PRECISE_QUAT].PreciseQuat;

EsbPktQuat_t &g_EsbPktQuat = s_EsbPacket[ESBPKT_TYPE_ACCQUAT].Quat;

EsbPktStatus_t &g_EsbPktStatus = s_EsbPacket[ESBPKT_TYPE_STATUS].Status;

EsbPktMagQuat_t &g_EsbPktMagQuat = s_EsbPacket[ESBPKT_TYPE_MAGQUAT].MagQuat;

/** Calculate CRC compliant with SlimeVR project */
uint8_t slime_crc8_ccitt(uint8_t val, const void *buf, size_t cnt)
{
	size_t i;
	const uint8_t *p = (uint8_t*)buf;

	for (i = 0; i < cnt; i++) {
		val ^= p[i];
		val = (val << 4) ^ crc8_ccitt_small_table[val >> 4];
		val = (val << 4) ^ crc8_ccitt_small_table[val >> 4];
	}
	return val;
}

/** Event handler for ESB */
void EsbEventHandler(nrf_esb_evt_t const * pEvt)
{
	nrf_esb_payload_t payload;

    switch (pEvt->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
//        	g_Uart.printf("Tx Success\r\n");
            break;
        case NRF_ESB_EVENT_TX_FAILED:
//        	g_Uart.printf("Tx failed\r\n");
        	(void) nrf_esb_flush_tx();
            break;
        case NRF_ESB_EVENT_RX_RECEIVED:
            // Get the most recent element from the RX FIFO.
            while (nrf_esb_read_rx_payload(&payload) == NRF_SUCCESS)
            {
				cli_printf("len = %d, rx_payload.data[0] = %02x ", payload.length, payload.data[0]);
				for (int i = 1; i < payload.length; i++)
				{
					cli_printf("%02x ", payload.data[i]);
				}
				cli_printf("\r\n");
				if (!IsPaired())
				{
					if (payload.length == 8)
					{
						// This is the pairing packet which contains the receiver MAC address

						cli_printf("rx_payload.data[0] = %02x ", payload.data[0]);
						for (int i = 1; i < 8; i++)
						{
							cli_printf("%02x ", payload.data[i]);
						}
						cli_printf("\r\n");

						if (payload.data[0] == s_PairCrc)
						{
							cli_printf("Paired tracker id:%d\r\n", payload.data[1]);
							SetTrackerId(payload.data[1]);
							// Save receiver MAC address
							uint64_t mac = 0;
							memcpy(&mac, payload.data, 8);
							SetReceiverMacAddr(mac);
						}
						else
						{
						}
					}
				}
				else
				{
					if (payload.length == 4)
					{
						// unexpected packet
						cli_printf("Error\r\n");
					}
				}
            }
			break;
    }
}

/** Init Enhanced ShockBurst (ESB) protocol */
bool EsbInit(void)
{
    uint32_t err_code;
    uint8_t base_addr_0[4] = {0x62, 0x39, 0x8A, 0xF2};
    uint8_t base_addr_1[4] = {0x28, 0xFF, 0x50, 0xB8};
    uint8_t addr_prefix[8] = {0xFE, 0xFF, 0x29, 0x27, 0x09, 0x02, 0xB2, 0xD6};

    if (IsPaired() == false)
    {
    	// Set the discovery address
    	memcpy(base_addr_0, DiscBaseAddr0, sizeof(base_addr_0));
    	memcpy(base_addr_1, DiscBaseAddr1, sizeof(base_addr_1));
    	memcpy(addr_prefix, DiscAddrPrefix, sizeof(addr_prefix));
    }
    else
    {
    	// Already paired, set the normal operating address
    	// Don't know the reason why Slime nRF is doing it this way
    	// Recreate receiver address

    	uint8_t addr_buffer[16] = {0};
    	uint64_t rxid = GetReceiverMacAddr();
    	uint8_t *prxid = (uint8_t*)&rxid;

    	for (int i = 0; i < 4; i++)
    	{
    		addr_buffer[i] = prxid[i + 2];
    		addr_buffer[i + 4] = prxid[i + 2] + prxid[6];
    	}
    	for (int i = 0; i < 8; i++)
    		addr_buffer[i + 8] = prxid[7] + i;
    	for (int i = 0; i < 16; i++)
    	{
    		if (addr_buffer[i] == 0x00 || addr_buffer[i] == 0x55 || addr_buffer[i] == 0xAA) // Avoid invalid addresses (see nrf datasheet)
    		{
    			addr_buffer[i] += 8;
    		}
    	}
    	memcpy(base_addr_0, addr_buffer, sizeof(base_addr_0));
    	memcpy(base_addr_1, addr_buffer + 4, sizeof(base_addr_1));
    	memcpy(addr_prefix, addr_buffer + 8, sizeof(addr_prefix));
    }

#ifndef NRF_ESB_LEGACY
    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
#else // NRF_ESB_LEGACY
    nrf_esb_config_t nrf_esb_config         = NRF_ESB_LEGACY_CONFIG;
#endif // NRF_ESB_LEGACY
#ifdef NRF52832_XXAA
    nrf_esb_config.tx_output_power 			= NRF_ESB_TX_POWER_4DBM;
#else
    nrf_esb_config.tx_output_power 			= NRF_ESB_TX_POWER_8DBM;
#endif
    nrf_esb_config.retransmit_count         = 6;
    nrf_esb_config.selective_auto_ack       = false;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.event_handler            = EsbEventHandler;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PTX;

    err_code = nrf_esb_init(&nrf_esb_config);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, 8);
    VERIFY_SUCCESS(err_code);

    return true;
}

/** Send data over ESB */
bool EsbSendData(uint8_t *pData, size_t Len)
{
	nrf_esb_payload_t txpayload = {
		.length = (uint8_t)Len,
		.pipe = 1,
		.noack = 0,
	};

	memcpy(txpayload.data, pData, Len);

	uint32_t res = nrf_esb_write_payload(&txpayload);

	return res == NRF_SUCCESS;
}

/** Send pairing info over ESB */
bool EsbSendPairing(void)
{
	nrf_esb_payload_t txpayload = {
		.length = 8,
		.pipe = 0,
		.noack = 0,
		.data = { 0, },
	};

	uint64_t mac = nrf_get_mac_address();
	memcpy(&txpayload.data[2], &mac, 6);

	s_PairCrc = slime_crc8_ccitt(7, &txpayload.data[2], 6);
	if (s_PairCrc == 0)
		s_PairCrc = 8;

	txpayload.data[0] = s_PairCrc;

	uint32_t res = nrf_esb_write_payload(&txpayload);

	return res == NRF_SUCCESS;
}

void SetEsbPktTrackerId(uint8_t TrackerId)
{
	g_EsbPktDevInfo.TrackerId = TrackerId;
	g_EsbPktPrecisionAccQuat.TrackerId = TrackerId;
	g_EsbPktQuat.TrackerId = TrackerId;
	g_EsbPktStatus.TrackerId = TrackerId;
	g_EsbPktMagQuat.TrackerId = TrackerId;
}

/** Send packet containing sensor's raw data over ESB*/
bool EsbSendPacket(ESBPKT_TYPE PktType)
{
	return EsbSendData((uint8_t*)s_EsbPacket[PktType].Data, s_EsbPacket[PktType].PktLen);
}

/** Update the sensor's data to the ESB packet */
void EsbPktUpdateImu(AccelSensorData_t &Accel, int16_t Quat[4])
{
	g_EsbPktPrecisionAccQuat.Acc[0] = Accel.X * (1<<7);
	g_EsbPktPrecisionAccQuat.Acc[1] = Accel.Y * (1<<7);
	g_EsbPktPrecisionAccQuat.Acc[2] = Accel.Z * (1<<7);
	g_EsbPktPrecisionAccQuat.Quat[0] = Quat[1];
	g_EsbPktPrecisionAccQuat.Quat[1] = Quat[2];
	g_EsbPktPrecisionAccQuat.Quat[2] = Quat[3];
	g_EsbPktPrecisionAccQuat.Quat[3] = Quat[0];
}

void EsbPktUpdateMagImu(AccelSensorData_t &Accel, MagSensorData_t &Mag, int16_t Quat[4])
{
	g_EsbPktPrecisionAccQuat.Acc[0] = Accel.X * (1<<7);
	g_EsbPktPrecisionAccQuat.Acc[1] = Accel.Y * (1<<7);
	g_EsbPktPrecisionAccQuat.Acc[2] = Accel.Z * (1<<7);
	g_EsbPktPrecisionAccQuat.Quat[0] = Quat[1];
	g_EsbPktPrecisionAccQuat.Quat[1] = Quat[2];
	g_EsbPktPrecisionAccQuat.Quat[2] = Quat[3];
	g_EsbPktPrecisionAccQuat.Quat[3] = Quat[0];

	g_EsbPktMagQuat.Mag[0] = Mag.X * (1<<10);
	g_EsbPktMagQuat.Mag[1] = Mag.Y * (1<<10);
	g_EsbPktMagQuat.Mag[2] = Mag.Z * (1<<10);
	g_EsbPktMagQuat.Quat[0] = Quat[1];
	g_EsbPktMagQuat.Quat[1] = Quat[2];
	g_EsbPktMagQuat.Quat[2] = Quat[3];
	g_EsbPktMagQuat.Quat[3] = Quat[0];
}

void SendMotionData(AccelSensorData_t &Accel, int16_t Quat[4])
{
	g_EsbPktPrecisionAccQuat.Acc[0] = Accel.X * (1 << 7);
	g_EsbPktPrecisionAccQuat.Acc[1] = Accel.Y * (1 << 7);
	g_EsbPktPrecisionAccQuat.Acc[2] = Accel.Z * (1 << 7);
	g_EsbPktPrecisionAccQuat.Quat[0] = Quat[1];
	g_EsbPktPrecisionAccQuat.Quat[1] = Quat[2];
	g_EsbPktPrecisionAccQuat.Quat[2] = Quat[3];
	g_EsbPktPrecisionAccQuat.Quat[3] = Quat[0];

//	EsbSendPacket(ESBPKT_TYPE_PRECISE_QUAT);
	EsbSendData((uint8_t*) s_EsbPacket[ESBPKT_TYPE_PRECISE_QUAT].Data,
			s_EsbPacket[ESBPKT_TYPE_PRECISE_QUAT].PktLen);
}

void SendMotionDataMag(MagSensorData_t &Mag, int16_t Quat[4])
{
	g_EsbPktMagQuat.Quat[0] = Quat[1];
	g_EsbPktMagQuat.Quat[1] = Quat[2];
	g_EsbPktMagQuat.Quat[2] = Quat[3];
	g_EsbPktMagQuat.Quat[3] = Quat[0];

	g_EsbPktMagQuat.Mag[0] = Mag.X * 0.01 * (float)(1 << 10);
	g_EsbPktMagQuat.Mag[1] = Mag.Y * 0.01 * (float)(1 << 10);
	g_EsbPktMagQuat.Mag[2] = Mag.Z * 0.01 * (float)(1 << 10);

//	EsbSendPacket(ESBPKT_TYPE_PRECISE_QUAT);
	EsbSendData((uint8_t*) s_EsbPacket[ESBPKT_TYPE_MAGQUAT].Data,
			s_EsbPacket[ESBPKT_TYPE_MAGQUAT].PktLen);
}

extern volatile float g_battVolt;
void UpdateBattLevel()
{
	g_EsbPktDevInfo.BatLevel = (int)(100 * g_battVolt / MAX_BATTERY_VOLTAGE);
//	g_Uart.printf("Bat level = %d \r\n", g_EsbPktDevInfo.BatLevel);
}

