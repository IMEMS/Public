/*
 * GTBEA_basic.c
 */
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_uart.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"

// Tivaware
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/flash.h"
#include "driverlib/ssi.h"
#include "driverlib/uart.h"

// GTBE Lib
#include "tw_extension.h"
#include "pga_pga2311.h"
#include "dac_ad5754.h"

#include "GTBEA_basic.h"

/**************
 * Parameters *
 **************/
#define UART_BAUD 115200

#define DAC_AMP_SET_OFFSET 	0.005
#define DAC_PI_K_OFFSET		0.000

#define GTBEA_ERR_LED_GPIO_PERIPH	SYSCTL_PERIPH_GPIOK
#define GTBEA_ERR_LED_GPIO_BASE		GPIO_PORTK_BASE
#define GTBEA_ERR_LED_PIN			GPIO_PIN_7

/********************
 * Global Variables *
 ********************/
int g_gainR2 = 0;
int g_gainL2 = 0;

/*******************
 * Board Functions *
 *******************/
void GTBEA_initErrLED(void) {
	MAP_SysCtlPeripheralEnable(GTBEA_ERR_LED_GPIO_PERIPH);
	MAP_GPIOPinTypeGPIOOutput(GTBEA_ERR_LED_GPIO_BASE, GTBEA_ERR_LED_PIN);
	MAP_GPIOPinWrite(GTBEA_ERR_LED_GPIO_BASE, GTBEA_ERR_LED_PIN, 0); // turn off err LED
}
/********
 * Main *
 ********/
int main(void) {

	// Control Parameters
	float amp_set_v = 0.2;
	float pi_k = -1;

	float Q1H_V = -1;
	float Q1L_V = 2;
	float Q2H_V = -3;
	float Q2L_V = 4;
	int32_t P_gain2 = 10; // 2 * ALC Proportional Gain
	int32_t I_gain2 = -62; // 2 * ALC Integral Gain
	//int32_t P_gain2 = 2 * PGA_GAIN_MUTE; // Mute
	//int32_t I_gain2 = 2 * PGA_GAIN_MUTE; // Mute

	/* System initialization */
	uint32_t SysClkFreq;
	#ifdef PART_TM4C123GH6PM
		// Set system clock to 80 MHz (400MHz main PLL (divided by 5 - uses DIV400 bit)  [16MHz external xtal drives PLL]
		SysClkFreq = MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	#endif
	#ifdef PART_TM4C1294NCPDT
		// Set system clock to 120 MHz
		SysClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);
	#endif

	/* Error LED Initialization */
	GTBEA_initErrLED();

	/* PGA Initialization */
	//pga_init(SysClkFreq, false, true); // unmuted, zero crossing mode enabled

	/* UART Comm. Initialization */
	//twe_initVCOM(SysClkFreq, UART_BAUD);

	/* DAC Initialization */
	DACd_initDAC(DAC_RANGE_PM5V | DAC_RANGE_PM5V_EH, DAC_PWR_PU_AMP_SET | DAC_PWR_PU_PI_K | DAC_PWR_PUALL_EH, SysClkFreq);

	DACd_updateDataVolt(DAC_ADDR_NONE_AD | DAC_ADDR_Q1H, DAC_RANGE_PM5V | DAC_RANGE_PM5V_EH, OFFSET_BINARY, OFFSET_BINARY, 0, Q1H_V);
	MAP_SysCtlDelay(4000);
	DACd_updateDataVolt(DAC_ADDR_NONE_AD | DAC_ADDR_Q1L, DAC_RANGE_PM5V | DAC_RANGE_PM5V_EH, OFFSET_BINARY, OFFSET_BINARY, 0, Q1L_V);
	MAP_SysCtlDelay(4000);
	DACd_updateDataVolt(DAC_ADDR_AMP_SET | DAC_ADDR_Q2H, DAC_RANGE_PM5V | DAC_RANGE_PM5V_EH, OFFSET_BINARY, OFFSET_BINARY, amp_set_v - DAC_AMP_SET_OFFSET, Q2H_V);
	MAP_SysCtlDelay(4000);
	DACd_updateDataVolt(DAC_ADDR_PI_K | DAC_ADDR_Q2L, DAC_RANGE_PM5V | DAC_RANGE_PM5V_EH, OFFSET_BINARY, OFFSET_BINARY, pi_k - DAC_PI_K_OFFSET, Q2L_V);
	MAP_SysCtlDelay(4000);
	DACd_loadDACs_AH();
	//DACd_loadDACsPin_EH();
	//pga_setGainInt(P_gain2, I_gain2);
	//pga_setGainInt(2 * PGA_GAIN_MUTE, 2 * PGA_GAIN_MUTE);  // Mute while testing other circuits...

	while(1) {
		// wait...
	}
}
