/**
 * @file mainCPU2.c
 * @author José Augusto Arbugeri (josearbugeri@gmail.com)
 * @brief Main CPU1 code
 * @version 1.0
 * @date 3 de dez de 2022
 *
 * @copyright Copyright (c) 2022
 *
 */

/*-------------------------------------------------------------*/
/*      Includes and dependencies                              */
/*-------------------------------------------------------------*/

#include "mainCPU2_cla_shared.h"

/*-------------------------------------------------------------*/
/*      Macros and definitions                                 */
/*-------------------------------------------------------------*/

#pragma DATA_SECTION(dataCPU1,"CPU1TOCPU2DATA")
data_cpu1_t dataCPU1 = {0.0};

#pragma DATA_SECTION(dataCPU2,"CPU2TOCPU1DATA")
data_cpu2_t dataCPU2 = {0.0};

long counter = 0;
long delay_time = 500000;

/*-------------------------------------------------------------*/
/*      Typedefs enums & structs                               */
/*-------------------------------------------------------------*/

/* COUNTER */

#pragma DATA_SECTION(WaveGenerator,"Cla1Data")
WaveGenerator_t WaveGenerator = WAVE_GENERATOR_INIT(PWM_12_FREQUENCY, 60.0, 311.0, SINE_1F);

/* COUNTER */

#pragma DATA_SECTION(counter_cla,"Cla1Data")
counter_t counter_cla = { 1000000.0 / 200000000.0, 0, 0, 0};
#pragma DATA_SECTION(counter_cpu,"Cla1Data")
counter_t counter_cpu = { 1000000.0 / 200000000.0, 0, 0, 0};

hil_t hil_boost;

/* DAC */

float * ptr_DACA = &hil_boost.dx1;
float   DACA_offset = 0.0;
float   DACA_gain   = 60.0;

float * ptr_DACB = &hil_boost.dx2;
float   DACB_offset = 0.0;
float   DACB_gain   = 8.0;

float * ptr_DACC = &hil_boost.u;
float   DACC_offset = 0.0;
float   DACC_gain   = 10.0;

/*-------------------------------------------------------------*/
/*      Function prototypes                                    */
/*-------------------------------------------------------------*/

__interrupt void isr_epwm12(void);
__interrupt void Cla1Task1_callback();

void waitCPU1(void);

int mainCPU2(void){

    waitCPU1();

    EALLOW;
    PieVectTable.EPWM12_INT = &isr_epwm12;
    EDIS;

    PieCtrlRegs.PIEIER3.bit.INTx12 = 1;

    Peripheral_Setup_ConfigEPwm12();

    Peripheral_Setup_DAC();

    Peripheral_Setup_initCpu1Cla1(&Cla1Task1, &Cla1Task1_callback);

    IER |= M_INT3;  //!< Enable group 3 interrupts (PWM)

    EINT;
    ERTM;

    while(1){

        CALCULATE_TIME_COUNTER(counter_cla);
        CALCULATE_TIME_COUNTER(counter_cpu);

        //WaveGenerator_update(&WaveGenerator, PWM_12_FREQUENCY, WaveGenerator.Fg, WaveGenerator.A);

        hil_update(&hil_boost, dataCPU1.Freq_hil, dataCPU1.Indutor, dataCPU1.Capacitor, dataCPU1.Carga);

        DELAY_US(delay_time);
        GPIO_WritePin(GPIO_LED_RED,0);
        DELAY_US(delay_time);
        GPIO_WritePin(GPIO_LED_RED,1);
    }

}

#pragma CODE_SECTION(isr_epwm12,".TI.ramfunc");
__interrupt void isr_epwm12(void)
{
    EPwm12Regs.ETCLR.bit.INT = 1; //!< Reset interrupt flag

    INIT_COUNTER(counter_cpu);

    Cla1ForceTask1();

    hil_boost.u = fabsf(WaveGenerator.a);
    #if HIL_MODE
    hil_boost.s = GpioDataRegs.GPADAT.bit.GPIO6;
    hil_switched_model_run(&hil_boost);
    #else
    hil_boost.d = dataCPU1.duty_p_pu;
    hil_average_model_run(&hil_boost);
    #endif

    dataCPU2.voltage_input  = WaveGenerator.a;
    dataCPU2.current_input  = hil_boost.dx1;
    dataCPU2.voltage_output = hil_boost.dx2;

    /* Update DAC output */
    #if HIL_ON && DAC_CPU2
        DacaRegs.DACVALS.bit.DACVALS = (Uint16) ((( (float) * ptr_DACA) * DACA_gain) + DACA_offset);
        DacbRegs.DACVALS.bit.DACVALS = (Uint16) ((( (float) * ptr_DACB) * DACB_gain) + DACB_offset);
        DaccRegs.DACVALS.bit.DACVALS = (Uint16) ((( (float) * ptr_DACC) * DACC_gain) + DACC_offset);
    #endif

    END_COUNTER(counter_cpu);
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

void waitCPU1(void){
    IpcRegs.IPCSET.bit.IPC5 = 1;            // Inform to CPU1 that the CPU2 is OK
    while (IpcRegs.IPCSTS.bit.IPC4 == 0);   // Waiting the CPU1 inform that is OK
    IpcRegs.IPCACK.bit.IPC4 = 1;            // Clears the bit flag that CPU1 used to inform
}

__interrupt void Cla1Task1_callback()
{
    // Acknowledge the end-of-task interrupt for task 1
    PieCtrlRegs.PIEACK.all = M_INT11;
    counter++;
}

