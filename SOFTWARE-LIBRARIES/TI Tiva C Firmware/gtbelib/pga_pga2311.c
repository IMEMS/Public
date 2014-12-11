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

//*****************************************************************************
//
//! \defgroup pga_pga2311 Programmable Gain Amplifier
//! \brief Drivers for the TI PGA2311 PGA.
//! Interfaces the TI tiva C Launchpad with the TI PGA2311 Programmable gain
//!  amplifier
//! Files:
//!  pga_pga2311.c
//!  pga_pga2311.h
//!
//! \author Curtis Mayberry
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup pga_pga2311
//! @{
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ssi.h"

// Tivaware
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/ssi.h"

// GTBE Lib
#include "pga_pga2311.h"

/*****************
 * PGA Functions *
 *****************/

/**
 * Initializes the PGA
 *
 * \param SysClkFreq Clocck frequency of the system clock
 *
 * \param muteEN Initializes the zero crossing mode to be
 *  enabled if true, disabled if false
 *
 * \param ZcenEN sets the control to enable zero crossing mode if true, disable if false
 */
void pga_init(uint32_t SysClkFreq, bool muteEN, bool ZcenEN) {
	pga_initSSI(SysClkFreq);  // Initialize Serial Communication
	pga_initCtlMute(muteEN);
	pga_initCtlZcen(ZcenEN);

}

/**
 * Initializes the SSI for use with the PGA.
 *
 *  \note SSI peripheral settings are defined in the header file
 */
void pga_initSSI(uint32_t SysClkFreq) {
 	uint32_t trashBin[1] = {0};

 	// Enable Peripherals
 	MAP_SysCtlPeripheralEnable(PGA_SSI_PERIPH);
 	MAP_SysCtlPeripheralEnable(PGA_SSI_GPIO_PERIPH);

 	// Set the pin muxing to SSI0 on port A
 	MAP_GPIOPinConfigure(PGA_SSI_CLK_PIN_CONFIG);
 	MAP_GPIOPinConfigure(PGA_SSI_FSS_PIN_CONFIG);
 	MAP_GPIOPinConfigure(PGA_SSI_TX_PIN_CONFIG);
 	MAP_GPIOPinTypeSSI(PGA_SSI_GPIO_BASE,PGA_SSI_TX_PIN|PGA_SSI_FSS_PIN|PGA_SSI_CLK_PIN);

 	// SPI Mode1
 	MAP_SSIConfigSetExpClk(PGA_SSI_BASE, SysClkFreq, SSI_FRF_MOTO_MODE_0,
 						   SSI_MODE_MASTER, PGA_FREQ_SSI_DATA, 8);

 	// Enable SSI uDMA
 	MAP_SSIEnable(PGA_SSI_BASE);
 	MAP_SSIAdvModeSet(PGA_SSI_BASE, SSI_ADV_MODE_WRITE);
 	MAP_SSIAdvFrameHoldEnable(PGA_SSI_BASE);
 	//SSIDMAEnable(DAC_SSI_BASE, SSI_DMA_TX);
 	while (MAP_SSIDataGetNonBlocking(PGA_SSI_BASE, &trashBin[0])) {
 	     }
}
/**
 * Initializes the mute control pin.
 *
 * \param muteEN Initializes the control to be muted if true, unmuted if false
 */
void pga_initCtlMute(bool muteEN) {
	MAP_SysCtlPeripheralEnable(PGA_MUTE_GPIO_PERIPH);
	MAP_GPIOPinTypeGPIOOutput(PGA_MUTE_GPIO_BASE, PGA_MUTE_PIN);
	MAP_GPIOPinWrite(PGA_MUTE_GPIO_BASE, PGA_MUTE_PIN, (!muteEN) << 1); // Initialize active low control to output high
}

/**
 * Initializes the zeros crossing mode control pin.
 *
 * \param muteEN Initializes the zero crossing mode to be
 *  enabled if true, disabled if false
 */
void pga_initCtlZcen(bool ZcenEN) {
	MAP_SysCtlPeripheralEnable(PGA_ZCEN_GPIO_PERIPH);
	MAP_GPIOPinTypeGPIOOutput(PGA_ZCEN_GPIO_BASE, PGA_ZCEN_PIN);
	MAP_GPIOPinWrite(PGA_ZCEN_GPIO_BASE, PGA_ZCEN_PIN, ZcenEN << 2); // Initialize active low control to output high
}

/**
 * Mutes the PGA by grounding the input.
 *
 * \param muteEN sets the control to be muted if true, unmuted if false
 */
void pga_setMute(bool muteEN) {
	MAP_GPIOPinWrite(PGA_MUTE_GPIO_BASE, PGA_MUTE_PIN, !muteEN); // active low pin
}


/**
 * Zero crossing control
 *
 * \param ZcenEN sets the control to enable zero crossing mode if true, disable if false
 */
void pga_setZcen(bool ZcenEN) {
	MAP_GPIOPinWrite(PGA_ZCEN_GPIO_BASE, PGA_ZCEN_PIN, ZcenEN); // active high pin
}

/**
 * Controls the PGA gains
 *
 * \param gainR Sets the right channel gain. Must be in increments of 0.5dB
 *  in the range -95.5 to 31.5 dB
 * \param gainL Sets the left channel gain. Must be in increments of 0.5dB
 *  in the range -95.5 to 31.5 dB
 *
 * \note Set the gain to PGA_GAIN_MUTE to mute the corresponding channel in software.
 *  This grounds the input of the PGA in the same way as the mute pin
 *
 *  \note Must make sure the floating point unit (FPU) is setup to handle
 *  floating point arithmetic in function calls (enable lazy stacking mode)
 */
void pga_setGain(float gainR, float gainL) {
	uint32_t gainRcode;
	uint32_t gainLcode;
	gainRcode = (uint32_t) 255 + 2*(gainR-31.5);
	gainLcode = (uint32_t) 255 + 2*(gainL-31.5);
	MAP_SSIDataPut(PGA_SSI_BASE, gainRcode); // right channel first
	MAP_SSIAdvDataPutFrameEnd(PGA_SSI_BASE,gainLcode); // left channel second
}

/**
 * Controls the PGA gains with integer inputs of 2*gain
 *
 * \param gainR2 Sets the right channel gain to half the gain entered.
 * The gainR2 is set to 2*gain giving a gain increment of 0.5dB
 *  in the range -95.5dB to 31.5dB (gainR2 is limited to -191 to 63)
 * \param gainL2 Sets the right channel gain to half the gain entered.
 * The gainR2 is set to 2*gain giving a gain increment of 0.5dB
 *  in the range -95.5dB to 31.5dB (gainL2 is limited to -191 to 63)
 *
 * \note Set the gain to 2*PGA_GAIN_MUTE to mute the corresponding channel in software.
 *  This grounds the input of the PGA in the same way as the mute pin
 */
void pga_setGainInt(int32_t gainR2, int32_t gainL2) {
	uint32_t gainRcode;
	uint32_t gainLcode;
	gainRcode = (uint32_t) 255 + gainR2 - 63;
	gainLcode = (uint32_t) 255 + gainL2 - 63;
	MAP_SSIDataPut(PGA_SSI_BASE, gainRcode); // right channel first
	MAP_SSIAdvDataPutFrameEnd(PGA_SSI_BASE,gainLcode); // left channel second
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

