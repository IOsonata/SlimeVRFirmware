//============================================================================
// Name        : main.cpp
// Author      : Nguyen Hoan Hoang
// Version     :
// Copyright   : Copyright 2024, I-SYST inc. All rights reserved.
// Description : Hello World in C++
//============================================================================

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
#include "sensors/agm_invn_icm20948.h"
#include "sensors/ag_bmi323.h"
#include "sensors/mag_bmm350.h"
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
	.bIntEn = true,
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
    .Rate = 1000000,   						// Speed in Hz
    .DataSize = 8,      					// Data Size
    .MaxRetry = 5,      					// Max retries
    .BitOrder = SPIDATABIT_MSB,
    .DataPhase = SPIDATAPHASE_SECOND_CLK, 	// Data phase
    .ClkPol = SPICLKPOL_LOW,         		// clock polarity
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

#if BOARD == BLUEIO_TAG_EVIM

AgmInvnIcm20948 g_MotSensor;
MagSensor &g_Mag = g_MotSensor;

#elif BOARD == BLYST_MOTION

AgBmi323 g_MotSensor;
//MagBmm350 g_Mag;

#else
#error "No board defined"
#endif

uint64_t inv_icm20948_get_time_us(void)
{
	return g_Timer.uSecond();
}

void inv_icm20948_sleep(int ms)
{
	msDelay(ms);
}

void inv_icm20948_sleep_us(int us)
{
	usDelay(us);
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


void recover_state()
{
    uint32_t            loop_count = 0;
    if ((NRF_POWER->GPREGRET >> 4) == RESET_MEMORY_TEST_BYTE)
    {
        // Take the loop_count value.
        loop_count          = (uint8_t)(NRF_POWER->GPREGRET & 0xFUL);
        NRF_POWER->GPREGRET = 0;
    }

    loop_count++;
    NRF_POWER->GPREGRET = ( (RESET_MEMORY_TEST_BYTE << 4) | loop_count);

   // tx_payload.data[1] = loop_count << 4;
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

void ImuIntEvtHandler(int IntNo, void *pCtx)
{

}

void TimerHandler(TimerDev_t *pTimer, uint32_t Evt)
{

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

	// IMU init
	IOPinConfig(IMU_INT_PORT, IMU_INT_PIN, IMU_INT_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL);
	IOPinEnableInterrupt(IMU_INT_NO, IMU_INT_PRIO, IMU_INT_PORT, IMU_INT_PIN, IOPINSENSE_LOW_TRANSITION, ImuIntEvtHandler, nullptr);

	bool res = g_MotSensor.Init(s_AccelCfg, &g_Spi, &g_Timer);
	if (res == true)
	{
		res = g_MotSensor.Init(s_GyroCfg, &g_Spi, &g_Timer);
		if (res == true)
		{
			res = g_MotSensor.Init(s_MagCfg, &g_Spi, &g_Timer);
			if (res == false)
			{
				// device does not have connected mag. Try external mag
				res = g_Mag.Init(s_MagCfg, &g_I2c, &g_Timer);
			}
		}
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
    }

    while (true)
    {
    	__WFE();
    }
}
