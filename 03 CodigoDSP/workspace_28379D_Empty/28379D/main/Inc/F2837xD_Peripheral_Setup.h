/*
 * F2837xD_Peripheral_Setup.h
 *
 *  Created on: 19 de nov de 2022
 *      Author: Jose
 */

#ifndef INC_F2837XD_PERIPHERAL_SETUP_H_
#define INC_F2837XD_PERIPHERAL_SETUP_H_

#include "F2837xD_device.h"
#include "F2837xD_Examples.h"

#define GPIO_LED_BLUE                   31
#define GPIO_LED_RED                    34

#define GPIO_PWM_S_P                    6
#define GPIO_PWM_S_N                    7
#define GPIO_PWM_EN_P                   8
#define GPIO_PWM_EN_N                   9

#define GPIO_DEBUG                      0

#define PERIPHERAL_CONNECTED_TO_CPU1    0
#define PERIPHERAL_CONNECTED_TO_CPU2    1

#define PWM_PERIPHERAL_FREQUENCY        200000000L
#define PWM_04_FREQUENCY                375000L
#define PWM_12_FREQUENCY                375000L

#define ET_3RD      0x3
#define ET_4RD      0x4
#define ET_5RD      0x5

/**
 * @brief Define qual CPU usa os DAC's
 * DAC_CPU = 0, CPU1
 * DAC_CPU = 1, CPU2
 */
#define DAC_CPU                         1
#define DAC_CPU1                       !DAC_CPU
#define DAC_CPU2                        DAC_CPU

/**
 * @brief Define se é HIL ou Real
 * HIL_ON = 0, Conversor Real
 * HIL_ON = 1, Conversor Emulado
 */
#define HIL_ON      0

/**
 * @brief Define se é HIL ou Real
 * HIL_ON = 0, Modelo medio do conversor
 * HIL_ON = 1, Modelo comutado do conversor
 */
#define HIL_MODE    0

/**
 * @brief Define resolução do ADC
 * ADC_RESOLUTION = 0, 12 bits
 * ADC_RESOLUTION = 1, 16 bits
 */
#define ADC_RESOLUTION                  1



#if ADC_RESOLUTION
    #define ADC_GAIN            65536
    #define ADC_VOLTAGE         3.3
#else
    #define ADC_GAIN            65536
    #define ADC_VOLTAGE         3.3
#endif


#if !HIL_ON
    #define CURRENT_INPUT_OFFSET        2048
    #define CURRENT_INPUT_GAIN          0.1
    #define VOLTAGE_INPUT_OFFSET        2048
    #define VOLTAGE_INPUT_GAIN          0.1
    #define VOLTAGE_OUTPUT_OFFSET       0
    #define VOLTAGE_OUTPUT_GAIN         0.1
#else
    #define CURRENT_INPUT_OFFSET        2048
    #define CURRENT_INPUT_GAIN          0.1
    #define VOLTAGE_INPUT_OFFSET        2048
    #define VOLTAGE_INPUT_GAIN          0.1
    #define VOLTAGE_OUTPUT_OFFSET       0
    #define VOLTAGE_OUTPUT_GAIN         0.1
#endif


#define ENABLE_PWMP_ON ()               GpioDataRegs.GPADAT.bit.GPIO11 = 1;
#define ENABLE_PWMP_OFF()               GpioDataRegs.GPADAT.bit.GPIO11 = 0;
#define ENABLE_PWMP_STATE(state)        GpioDataRegs.GPADAT.bit.GPIO11 = state;

#define ENABLE_PWMN_ON ()               GpioDataRegs.GPADAT.bit.GPIO8 = 1;
#define ENABLE_PWMN_OFF()               GpioDataRegs.GPADAT.bit.GPIO8 = 0;
#define ENABLE_PWMN_STATE(state)        GpioDataRegs.GPADAT.bit.GPIO8 = state;

/* ONLY CPU1*/
void Peripheral_Setup_Management(void);
void Peripheral_Setup_Memory_shared(void);
void Peripheral_Setup_InitLEDsGpio(void);


/* CPU1 AND CPU2*/
void Peripheral_Setup_ConfigEPwm1(void);
void Peripheral_Setup_ConfigEPwm4(void);
void Peripheral_Setup_ConfigEPwm12(void);

void Peripheral_Setup_ADC(void);

void Peripheral_Setup_DAC(void);

void Peripheral_Setup_configClaMemory(void);
void Peripheral_Setup_initCpu1Cla1( void(* task)(void) , PINT task_callback);


#endif /* INC_F2837XD_PERIPHERAL_SETUP_H_ */
