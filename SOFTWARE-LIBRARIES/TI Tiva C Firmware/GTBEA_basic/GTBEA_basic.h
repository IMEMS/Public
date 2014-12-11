/*
 * GTBEA_basic.h
 */

#ifndef GTBEA_BASIC_H_
#define GTBEA_BASIC_H_

// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
#ifdef __cplusplus
extern "C"
{
#endif

/*************************
 * GTBE-A DAC Parameters *
 *************************/

// DAC Addresses
#define DAC_ADDR_PI_K		DAC_ADDR_A
#define DAC_ADDR_AMP_SET	DAC_ADDR_C
#define DAC_ADDR_Q1H		DAC_ADDR_H
#define DAC_ADDR_Q1L		DAC_ADDR_G
#define DAC_ADDR_Q2H		DAC_ADDR_E
#define DAC_ADDR_Q2L		DAC_ADDR_F
// DAC Power Settings
#define DAC_PWR_PU_PI_K		DAC_PWR_PUA
#define DAC_PWR_PU_AMP_SET	DAC_PWR_PUC

#ifdef __cplusplus
}
#endif
#endif /* GTBEA_BASIC_H_ */

