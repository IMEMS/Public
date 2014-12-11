/********************************************************************************
 *
 * pga_pga2311.c - Drivers for the TI PGA2311 programmmable gain amplifier (PGA).
 *  Interfaces the TI tiva C Launchpad with the TI PGA2311.  Includes two
 *  channels (L and R).  Designed for audio volume control.  The gain is
 *  programmable in 0.5dB steps from +31.5dB to -95.5dB.
 *
 *  Author: Curtis Mayberry
 *  Georgia Tech IMEMS
 *
 *  Originally written for the MRIG uHRG gyroscope project
 *
 *  This work is licensed under the Creative Commons Attribution-ShareAlike 3.0
 *  Unported License. To view a copy of this license, visit
 *  http://creativecommons.org/licenses/by-sa/3.0/ or send a letter to Creative
 *  Commons, 444 Castro Street, Suite 900, Mountain View, California, 94041, USA.
 *
 ********************************************************************************/

#ifndef PGA_PGA2311_H_
#define PGA_PGA2311_H_

// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
#ifdef __cplusplus
extern "C"
{
#endif


/************************
 * Serial Port Settings *
 ************************/
#define PGA_FREQ_SSI_DATA	6000000

/*******************
 * Port Assignment *
 *******************/
// Assigns port,pin and peripheral definitions depending on the MCU in use

	#ifdef PART_TM4C1294NCPDT // EK-TM4C1294XL

	#define PGA_SSI_PERIPH			SYSCTL_PERIPH_SSI2
	#define PGA_SSI_BASE			SSI2_BASE
	#define PGA_SSI_GPIO_PERIPH		SYSCTL_PERIPH_GPIOD
	#define PGA_SSI_GPIO_BASE		GPIO_PORTD_BASE
	#define PGA_SSI_TX_PIN_CONFIG	GPIO_PD1_SSI2XDAT0
	#define PGA_SSI_FSS_PIN_CONFIG	GPIO_PD2_SSI2FSS
	#define PGA_SSI_CLK_PIN_CONFIG	GPIO_PD3_SSI2CLK
	#define PGA_SSI_TX_PIN			GPIO_PIN_1
	#define PGA_SSI_FSS_PIN			GPIO_PIN_2
	#define PGA_SSI_CLK_PIN			GPIO_PIN_3

	#define PGA_MUTE_GPIO_PERIPH	SYSCTL_PERIPH_GPIOL
	#define PGA_MUTE_GPIO_BASE		GPIO_PORTL_BASE
	#define PGA_MUTE_PIN			GPIO_PIN_1

	#define PGA_ZCEN_GPIO_PERIPH	SYSCTL_PERIPH_GPIOL
	#define PGA_ZCEN_GPIO_BASE		GPIO_PORTL_BASE
	#define PGA_ZCEN_PIN			GPIO_PIN_2

#endif


/******************
 * PGA Parameters *
 ******************/
#define PGA_GAIN_MUTE -96

/**************
 * Prototypes *
 **************/
extern void pga_init(uint32_t SysClkFreq, bool muteEN, bool ZcenEN);
extern void pga_initSSI(uint32_t SysClkFreq);
extern void pga_initCtlMute(bool muteEN);
extern void pga_initCtlZcen(bool ZcenEN);
extern void pga_setMute(bool muteEn);
extern void pga_setZcen(bool ZcenEN);
extern void pga_setGain(float gainR, float gainL);
extern void pga_setGainInt(int32_t gainR2, int32_t gainL2);


#ifdef __cplusplus
}
#endif
#endif /* PGA_PGA2311_H_ */

