/**-------------------------------------------------------------------------
@file	main.cpp

@brief	Main firmware file

Implementation of SlimeVR tracker ESB

@author	Nguyen Hoan Hoang
@date	Dec. 11, 2024

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
#include "nrf_esb.h"
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_error.h"
#include "nrf_esb_error_codes.h"
#include "app_util.h"
#include "fds.h"

#include "istddef.h"
#include "idelay.h"
#include "crc.h"
#include "coredev/i2c.h"
#include "coredev/spi.h"
#include "coredev/uart.h"
#include "coredev/timer.h"
#include "miscdev/led.h"
#include "iopinctrl.h"
#include "sensors/ag_bmi323.h"
#include "sensors/mag_bmm350.h"

#define INVN

#ifdef INVN
#include "sensors/agm_invn_icm20948.h"
#include "imu/imu_invn_icm20948.h"
#else
#include "sensors/agm_icm20948.h"
#include "imu/imu_icm20948.h"
#endif

#include "SlimeTrackerESB.h"

#include "board.h"


#define FIRMWARE_VERSION	0
#define DEVICE_NAME			"BLYST-MOTION"

#define RESET_MEMORY_TEST_BYTE  (0x0DUL)        /**< Known sequence written to a special register to check if this wake up is from System OFF. */
#define RAM_RETENTION_OFF       (0x00000003UL)  /**< The flag used to turn off RAM retention on nRF52. */

#define BTN_PRESSED     0                       /**< Value of a pressed button. */
#define BTN_RELEASED    1                       /**< Value of a released button. */

#define FDS_DATA_FILE     (0xF010)
#define FDS_DATA_REC_KEY  (0x7010)

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

typedef struct __App_Data {
	uint8_t Cs;				// Checksum
	uint8_t TrackerId;		// Receiver assigned tracker id
	uint64_t ReceiverMacAddr;
	bool bPairMode;
} AppData_t;

__attribute__ ((section(".Version"), used))
const AppInfo_t g_AppInfo = {
	DEVICE_NAME, {FIRMWARE_VERSION, 0, BUILDN},
	{'I', 'O', 's', 'o', 'n', 'a', 't', 'a', 'I', '-', 'S', 0x55, 0xA5, 0x5A, 0xA5, 0x5A},
};

AppData_t g_AppData = { 0, (uint8_t)-1, (uint64_t)-1LL};

//static uint8_t s_TrackerId;

alignas(4) static fds_record_t const g_AppDataRecord =
{
    .file_id           = FDS_DATA_FILE,
    .key               = FDS_DATA_REC_KEY,
    .data = {
		.p_data = &g_AppData,
    /* The length of a record is always expressed in 4-byte units (words). */
		.length_words = (sizeof(AppData_t) + 3) >> 2,
    }
};

volatile bool g_FdsInitialized = false;
volatile bool g_FdsCleaned = false;

Led g_LedPair;
Led g_LedRun;

/// I2C configuration
static const IOPinCfg_t s_I2cPins[] = I2C_PINS;

static const I2CCfg_t s_I2cCfg = {
	.DevNo = I2C_DEVNO,			// I2C device number
	.Type = I2CTYPE_STANDARD,
	.Mode = I2CMODE_MASTER,
	.pIOPinMap = s_I2cPins,
	.NbIOPins = sizeof(s_I2cPins) / sizeof(IOPinCfg_t),
	.Rate = 200000,				// Rate in Hz
	.MaxRetry = 5,				// Retry
	.AddrType = I2CADDR_TYPE_NORMAL,
	.NbSlaveAddr = 0,			// Number of slave addresses
	.SlaveAddr = {0,},			// Slave addresses
	.bDmaEn = true,
	.bIntEn = false,
	.IntPrio = 7,				// Interrupt prio
	.EvtCB = nullptr			// Event callback
};

I2C g_I2c;

/// SPI configuration
static const IOPinCfg_t s_Spi1Pins[] = SPI_PINS;

static const SPICfg_t s_SpiCfg = {
	.DevNo = SPI_DEVNO,
	.Phy = SPIPHY_NORMAL,
    .Mode = SPIMODE_MASTER,
	.pIOPinMap = s_Spi1Pins,
    .NbIOPins = sizeof(s_Spi1Pins) / sizeof(IOPinCfg_t),
    .Rate = 4000000,   						// Speed in Hz
    .DataSize = 8,      					// Data Size
    .MaxRetry = 5,      					// Max retries
    .BitOrder = SPIDATABIT_MSB,
    .DataPhase = SPIDATAPHASE_FIRST_CLK, 	// Data phase
    .ClkPol = SPICLKPOL_HIGH,         		// clock polarity
    .ChipSel = SPICSEL_AUTO,
	.bDmaEn = true,						// DMA
	.bIntEn = false,
    .IntPrio = APP_IRQ_PRIORITY_LOW,    	// Interrupt priority
	.DummyByte = 0xff,
};

SPI g_Spi;


//int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);

#define UARTFIFOSIZE			CFIFO_MEMSIZE(256)

alignas(4) static uint8_t s_UartRxFifo[UARTFIFOSIZE];
alignas(4) static uint8_t s_UartTxFifo[UARTFIFOSIZE];


static const IOPinCfg_t s_UartPins[] = UART_PINS;

// UART configuration data
static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = 115200,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = 1,
	//.EvtCallback = nRFUartEvthandler,
	.bFifoBlocking = true,
	.RxMemSize = UARTFIFOSIZE,
	.pRxMem = s_UartRxFifo,
	.TxMemSize = UARTFIFOSIZE,
	.pTxMem = s_UartTxFifo,
	.bDMAMode = true,
};

UART g_Uart;

void TimerHandler(TimerDev_t *pTimer, uint32_t Evt);

// Timer configureation
const static TimerCfg_t s_TimerCfg = {
    .DevNo = 0,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,			// 0 => Default highest frequency
	.IntPrio = 1,
	.EvtHandler = TimerHandler,
};

Timer g_Timer;

// Device list
#ifdef INVN
ImuInvnIcm20948 g_Imu20948;
AgmInvnIcm20948 g_Icm20948;
#else
ImuIcm20948 g_Imu20948;
AgmIcm20948 g_Icm20948;
#endif

AgBmi323 g_Bmi323;
MagBmm350 g_Bmm350;

static const MotionDevice_t s_MotionDevices[] = {
	{/*&g_Imu20948*/nullptr, &g_Icm20948, &g_Spi, &g_Icm20948, &g_Spi, &g_Icm20948, &g_Spi, 9},
	{/*&g_Imu20948*/nullptr, &g_Icm20948, &g_I2c, &g_Icm20948, &g_I2c, &g_Icm20948, &g_I2c, 9},
	{nullptr, &g_Bmi323, &g_Spi, &g_Bmi323, &g_Spi, &g_Bmm350, &g_I2c, 9},
};

static const size_t s_NbMotionDevices = sizeof(s_MotionDevices) / sizeof(MotionDevice_t);

// Need this for icm20948
uint64_t inv_icm20948_get_time_us(void)
{
	return g_Timer.uSecond();
}

void system_off( void )
{
#ifdef NRF51
    NRF_POWER->RAMON |= (POWER_RAMON_OFFRAM0_RAM0Off << POWER_RAMON_OFFRAM0_Pos) |
                        (POWER_RAMON_OFFRAM1_RAM1Off << POWER_RAMON_OFFRAM1_Pos);
#endif //NRF51
#ifdef NRF52
    NRF_POWER->RAM[0].POWER = RAM_RETENTION_OFF;
    NRF_POWER->RAM[1].POWER = RAM_RETENTION_OFF;
    NRF_POWER->RAM[2].POWER = RAM_RETENTION_OFF;
    NRF_POWER->RAM[3].POWER = RAM_RETENTION_OFF;
    NRF_POWER->RAM[4].POWER = RAM_RETENTION_OFF;
    NRF_POWER->RAM[5].POWER = RAM_RETENTION_OFF;
    NRF_POWER->RAM[6].POWER = RAM_RETENTION_OFF;
    NRF_POWER->RAM[7].POWER = RAM_RETENTION_OFF;
#endif //NRF52

    // Turn off LEDs before sleeping to conserve energy.
    //bsp_board_leds_off();

    // Set nRF5 into System OFF. Reading out value and looping after setting the register
    // to guarantee System OFF in nRF52.
    NRF_POWER->SYSTEMOFF = 0x1;
    (void) NRF_POWER->SYSTEMOFF;
    while (true);
}

static void fds_evt_handler(fds_evt_t const * p_evt)
{
    switch (p_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_evt->result == NRF_SUCCESS)
            {
            	g_FdsInitialized = true;
            }
            break;

        case FDS_EVT_WRITE:
        {
            if (p_evt->result == NRF_SUCCESS)
            {
            }
        } break;

        case FDS_EVT_DEL_RECORD:
        {
            if (p_evt->result == NRF_SUCCESS)
            {
            }
        } break;

        case FDS_EVT_GC:
        	g_FdsCleaned = true;
        	break;

        default:
            break;
    }
}

void UpdateRecord()
{
    fds_record_desc_t desc = {0};
    fds_find_token_t  tok  = {0};

    uint32_t rc = fds_record_find(FDS_DATA_FILE, FDS_DATA_REC_KEY, &desc, &tok);

    if (rc == NRF_SUCCESS)
    {
    	rc = fds_record_update(&desc, &g_AppDataRecord);

    	if (rc == FDS_ERR_NO_SPACE_IN_FLASH)
    	{
    		// Remove deleted record to make space
    		g_FdsCleaned = false;
    		fds_gc();

    		while (g_FdsCleaned == false) __WFE();

        	rc = fds_record_update(&desc, &g_AppDataRecord);
        	APP_ERROR_CHECK(rc);
    	}
    }
}


void clocks_start( void )
{
    // Start HFCLK and wait for it to start.
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}

bool IsPaired(void)
{
	return g_AppData.ReceiverMacAddr != -1LL;
}

void SetReceiverMacAddr(uint64_t Addr)
{
	uint8_t *p = (uint8_t*)&g_AppData;

	g_AppData.ReceiverMacAddr = Addr;
	g_AppData.Cs = 0;

	for (int i = 0; i < sizeof(AppData_t); i++, p++)
	{
		g_AppData.Cs += *p;
	}

	g_AppData.Cs = 0 - g_AppData.Cs;

	UpdateRecord();

	g_LedPair.Off();

	msDelay(100);

    NVIC_SystemReset();
}

uint64_t GetReceiverMacAddr()
{
	return g_AppData.ReceiverMacAddr;
}

void SetTrackerId(uint8_t Id)
{
	g_AppData.TrackerId = Id;
}

void TimerHandler(TimerDev_t *pTimer, uint32_t Evt)
{

}

SPI * const GetSpi(void)
{
	return &g_Spi;
}

I2C * const GetI2c(void)
{
	return &g_I2c;
}

void HardwareInit()
{
    g_Uart.Init(s_UartCfg);

    g_Uart.printf("\nSlimeVR-Tracker nRF Vers: %d.%d.%d\n\r", g_AppInfo.Vers.Major, g_AppInfo.Vers.Minor, g_AppInfo.Vers.Build);

    g_LedPair.Init(LED_RED_PORT, LED_RED_PIN, LED_RED_LOGIC);
	g_LedRun.Init(LED_GREEN_PORT, LED_GREEN_PIN, LED_GREEN_LOGIC);

	g_LedPair.Off();
	g_LedRun.Off();

	(void) fds_register(fds_evt_handler);
    uint32_t rc = fds_init();
    APP_ERROR_CHECK(rc);

    while (g_FdsInitialized != true)
    {
        __WFE();
    }

    fds_record_desc_t desc = {0};
    fds_find_token_t  tok  = {0};

    rc = fds_record_find(FDS_DATA_FILE, FDS_DATA_REC_KEY, &desc, &tok);

    if (rc == NRF_SUCCESS)
    {
    	AppData_t data;

    	do {
			/* A config file is in flash. Let's update it. */
			fds_flash_record_t appdata = {0};

			/* Open the record and read its contents. */
			rc = fds_record_open(&desc, &appdata);
			APP_ERROR_CHECK(rc);

			memcpy(&data, appdata.p_data, sizeof(AppData_t));
			/* Close the record when done reading. */
			rc = fds_record_close(&desc);
			APP_ERROR_CHECK(rc);

			rc = fds_record_find(FDS_DATA_FILE, FDS_DATA_REC_KEY, &desc, &tok);
    	} while (rc == NRF_SUCCESS);

    	uint8_t *p = (uint8_t *)&data;
		uint8_t cs = 0;

		for (int i = 0; i < sizeof(AppData_t); i++, p++)
		{
			cs += *p;
		}

		if (cs == 0)
		{
			memcpy(&g_AppData, &data, sizeof(AppData_t));
		}
		else
		{
			UpdateRecord();
		}

    }
    else
    {
        rc = fds_record_write(&desc, &g_AppDataRecord);
        APP_ERROR_CHECK(rc);
    }

    // Timer init
	g_Timer.Init(s_TimerCfg);

	// SPI init
	g_Spi.Init(s_SpiCfg);
	g_I2c.Init(s_I2cCfg);

	bool res = InitSensors(s_MotionDevices, s_NbMotionDevices, &g_Timer);

	if (res == false)
	{
		g_Uart.printf("No sensor found\r\n");
	}

	if (res == true)
	{
		IOPinConfig(IMU_INT_PORT, IMU_INT_PIN, IMU_INT_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL);
		IOPinEnableInterrupt(IMU_INT_NO, IMU_INT_PRIO, IMU_INT_PORT, IMU_INT_PIN, IOPINSENSE_LOW_TRANSITION, ImuIntHandler, nullptr);
	}
}

//
// Print a greeting message on standard output and exit.
//
// On embedded platforms this might require semi-hosting or similar.
//
// For example, for toolchains derived from GNU Tools for Embedded,
// to enable semi-hosting, the following was added to the linker:
//
// `--specs=rdimon.specs -Wl,--start-group -lgcc -lc -lc -lm -lrdimon -Wl,--end-group`
//
// Adjust it for other toolchains.
//
// If functionality is not required, to only pass the build, use
// `--specs=nosys.specs`.
//

int main()
{
    uint32_t err_code;
    // Initialize
    clocks_start();

	// Update default checksum
	uint8_t *p = (uint8_t*)&g_AppData;

	g_AppData.Cs = 0;

	for (int i = 0; i < sizeof(AppData_t); i++, p++)
	{
		g_AppData.Cs += *p;
	}

	g_AppData.Cs = 0 - g_AppData.Cs;

    HardwareInit();

    EsbInit();

    msDelay(100);

    if (IsPaired() == false)
    {
    	g_Uart.printf("Paring mode\r\n");
    	g_LedPair.On();

    	EsbSendPairing();
    }
    else
    {
    	g_Uart.printf("Run mode\r\n");
    	g_LedRun.On();

    	g_Icm20948.Enable();

    }

    while (true)
    {
    	__WFE();
    }
}
