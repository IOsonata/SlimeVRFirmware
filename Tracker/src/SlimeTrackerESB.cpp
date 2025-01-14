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
#include "SlimeTrackerESB.h"
#include "sdk_common.h"
#include "nrf_error.h"

#include "crc.h"
#include "coredev/uart.h"
#include "nrf_mac.h"

extern UART g_Uart;

static const uint8_t DiscBaseAddr0[4] = {0x62, 0x39, 0x8A, 0xF2};
static const uint8_t DiscBaseAddr1[4] = {0x28, 0xFF, 0x50, 0xB8};
static const uint8_t DiscAddrPrefix[8] = {0xFE, 0xFF, 0x29, 0x27, 0x09, 0x02, 0xB2, 0xD6};

static const uint8_t crc8_ccitt_small_table[16] = {
	0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15,
	0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d
};

// non standard
uint8_t slime_crc8_ccitt(uint8_t val, const void *buf, size_t cnt)
{
	size_t i;
	const uint8_t *p = (const uint8_t *)buf;

	for (i = 0; i < cnt; i++) {
		val ^= p[i];
		val = (val << 4) ^ crc8_ccitt_small_table[val >> 4];
		val = (val << 4) ^ crc8_ccitt_small_table[val >> 4];
	}
	return val;
}

void EsbEventHandler(nrf_esb_evt_t const * pEvt)
{
	nrf_esb_payload_t payload;

    switch (pEvt->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
            break;
        case NRF_ESB_EVENT_TX_FAILED:
            (void) nrf_esb_flush_tx();
            break;
        case NRF_ESB_EVENT_RX_RECEIVED:
            // Get the most recent element from the RX FIFO.
            while (nrf_esb_read_rx_payload(&payload) == NRF_SUCCESS) ;

            if (!IsPaired())
			{
				if (payload.length == 8)
				{
					// This is the pairing packet which contains the receiver MAC address

					printf("rx_payload.data[0] = %02x ", payload.data[0]);
					for (int i = 1; i < 8; i++)
					{
						printf("%02x ", payload.data[i]);
					}
					printf("\r\n");

					// Save receiver MAC address
					uint64_t mac;
					memcpy(&mac, payload.data, 8);
					SetReceiverMacAddr(mac);
				}
			}
			else
			{
				if (payload.length == 4)
				{
					// unexpected packet
					printf("Error\r\n");
				}
			}
			break;
    }
}

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
    			addr_buffer[i] += 8;
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

bool EsbSendData(uint8_t *pData, size_t Len)
{
	nrf_esb_payload_t txpayload = {
		.length = 0,
		.pipe = 1,
		.noack = 0,
	};

	memcpy(txpayload.data, pData, Len);

	uint32_t res = nrf_esb_write_payload(&txpayload);

	return res == NRF_SUCCESS;
}

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

	uint8_t cs = slime_crc8_ccitt(7, &txpayload.data[2], 6);
	if (cs == 0)
		cs = 8;

	txpayload.data[0] = cs;

	uint32_t res = nrf_esb_write_payload(&txpayload);

	return res == NRF_SUCCESS;
}
