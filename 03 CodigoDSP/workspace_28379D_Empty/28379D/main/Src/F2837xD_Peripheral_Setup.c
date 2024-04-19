/*
 * F2837xD_Peripheral_Setup.c
 *
 *  Created on: 19 de nov de 2022
 *      Author: Jose
 */

#include "F2837xD_Peripheral_Setup.h"


void Peripheral_Setup_Management(void){

    #ifdef CPU1

    EALLOW;
    //!< PWM
    DevCfgRegs.CPUSEL0.bit.EPWM1  = PERIPHERAL_CONNECTED_TO_CPU1 ;
    DevCfgRegs.CPUSEL0.bit.EPWM2  = PERIPHERAL_CONNECTED_TO_CPU1 ;
    DevCfgRegs.CPUSEL0.bit.EPWM3  = PERIPHERAL_CONNECTED_TO_CPU1 ;
    DevCfgRegs.CPUSEL0.bit.EPWM4  = PERIPHERAL_CONNECTED_TO_CPU1 ;
    DevCfgRegs.CPUSEL0.bit.EPWM5  = PERIPHERAL_CONNECTED_TO_CPU1 ;
    DevCfgRegs.CPUSEL0.bit.EPWM6  = PERIPHERAL_CONNECTED_TO_CPU1 ;
    DevCfgRegs.CPUSEL0.bit.EPWM7  = PERIPHERAL_CONNECTED_TO_CPU1 ;
    DevCfgRegs.CPUSEL0.bit.EPWM8  = PERIPHERAL_CONNECTED_TO_CPU1 ;
    DevCfgRegs.CPUSEL0.bit.EPWM9  = PERIPHERAL_CONNECTED_TO_CPU1 ;
    DevCfgRegs.CPUSEL0.bit.EPWM10 = PERIPHERAL_CONNECTED_TO_CPU1 ;
    DevCfgRegs.CPUSEL0.bit.EPWM11 = PERIPHERAL_CONNECTED_TO_CPU1 ;
    DevCfgRegs.CPUSEL0.bit.EPWM12 = PERIPHERAL_CONNECTED_TO_CPU2 ;
    //!< ADC
    DevCfgRegs.CPUSEL11.bit.ADC_A = PERIPHERAL_CONNECTED_TO_CPU1 ;
    DevCfgRegs.CPUSEL11.bit.ADC_B = PERIPHERAL_CONNECTED_TO_CPU1 ;
    DevCfgRegs.CPUSEL11.bit.ADC_C = PERIPHERAL_CONNECTED_TO_CPU1 ;
    DevCfgRegs.CPUSEL11.bit.ADC_D = PERIPHERAL_CONNECTED_TO_CPU1 ;
    //!< DAC
    #if HIL_ON
        DevCfgRegs.CPUSEL14.bit.DAC_A = PERIPHERAL_CONNECTED_TO_CPU2 ;
        DevCfgRegs.CPUSEL14.bit.DAC_B = PERIPHERAL_CONNECTED_TO_CPU2 ;
        DevCfgRegs.CPUSEL14.bit.DAC_C = PERIPHERAL_CONNECTED_TO_CPU2 ;
    #else
        DevCfgRegs.CPUSEL14.bit.DAC_A = PERIPHERAL_CONNECTED_TO_CPU1 ;
        DevCfgRegs.CPUSEL14.bit.DAC_B = PERIPHERAL_CONNECTED_TO_CPU1 ;
        DevCfgRegs.CPUSEL14.bit.DAC_C = PERIPHERAL_CONNECTED_TO_CPU1 ;
    #endif

    EDIS;

    #endif
}

void Peripheral_Setup_Memory_shared(void){

    #ifdef CPU1

    EALLOW;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS0  = PERIPHERAL_CONNECTED_TO_CPU1;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS1  = PERIPHERAL_CONNECTED_TO_CPU1;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS2  = PERIPHERAL_CONNECTED_TO_CPU1;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS3  = PERIPHERAL_CONNECTED_TO_CPU1;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS4  = PERIPHERAL_CONNECTED_TO_CPU1;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS5  = PERIPHERAL_CONNECTED_TO_CPU2;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS6  = PERIPHERAL_CONNECTED_TO_CPU2;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS7  = PERIPHERAL_CONNECTED_TO_CPU2;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS8  = PERIPHERAL_CONNECTED_TO_CPU2;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS9  = PERIPHERAL_CONNECTED_TO_CPU2;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS10 = PERIPHERAL_CONNECTED_TO_CPU1;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS11 = PERIPHERAL_CONNECTED_TO_CPU1;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS12 = PERIPHERAL_CONNECTED_TO_CPU1;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS13 = PERIPHERAL_CONNECTED_TO_CPU1;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS14 = PERIPHERAL_CONNECTED_TO_CPU1;   // Transfer ownership of RAMGS14 to CPU01, CPU2 can read
    MemCfgRegs.GSxMSEL.bit.MSEL_GS15 = PERIPHERAL_CONNECTED_TO_CPU2;   // Transfer ownership of RAMGS15 to CPU02, CPU1 can read
    EDIS;

    #endif
}

void Peripheral_Setup_InitLEDsGpio(void){

    #ifdef CPU1

    EALLOW;
    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 0; //!< Set peripheral ePWM clock div to 1
    EDIS;
    //!< flag options: GPIO_ASYNC | GPIO_OPENDRAIN | GPIO_INVERT | GPIO_PULLUP

    GPIO_SetupPinOptions(GPIO_LED_BLUE, GPIO_OUTPUT, GPIO_SYNC);
    GPIO_SetupPinMux(GPIO_LED_BLUE, GPIO_MUX_CPU1, 0);

    GPIO_SetupPinOptions(GPIO_LED_RED, GPIO_OUTPUT, GPIO_SYNC);
    GPIO_SetupPinMux(GPIO_LED_RED, GPIO_MUX_CPU1, 0);

    //PWM Signal
    GPIO_SetupPinOptions(GPIO_PWM_S_P, GPIO_OUTPUT, GPIO_SYNC);
    GPIO_SetupPinMux(GPIO_PWM_S_P, GPIO_MUX_CPU1, 1);

    GPIO_SetupPinOptions(GPIO_PWM_S_N, GPIO_OUTPUT, GPIO_SYNC);
    GPIO_SetupPinMux(GPIO_PWM_S_N, GPIO_MUX_CPU1, 1);

    //PWM Enable
    GPIO_SetupPinOptions(GPIO_PWM_EN_P, GPIO_OUTPUT, GPIO_SYNC);
    GPIO_SetupPinMux(GPIO_PWM_EN_P, GPIO_MUX_CPU1, 0);
    GPIO_WritePin(GPIO_PWM_EN_P,0);
    GPIO_SetupPinOptions(GPIO_PWM_EN_N, GPIO_OUTPUT, GPIO_SYNC);
    GPIO_SetupPinMux(GPIO_PWM_EN_N, GPIO_MUX_CPU1, 0);
    GPIO_WritePin(GPIO_PWM_EN_N,0);

    //Gpio for debug
    GPIO_SetupPinOptions(GPIO_DEBUG, GPIO_OUTPUT, GPIO_SYNC);
    GPIO_SetupPinMux(GPIO_DEBUG, GPIO_MUX_CPU1, 0);
    GPIO_WritePin(GPIO_DEBUG,0);


    #endif
}

void Peripheral_Setup_ConfigEPwm1(void){
    // pg 1978 spruhm8i.pdf - Technical reference
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;
    EDIS;

    EPwm1Regs.TBPRD = 0xFFFF;                         // Set timer period
    EPwm1Regs.CMPA.bit.CMPA = EPwm1Regs.TBPRD >> 1;

    EPwm1Regs.TBPHS.bit.TBPHS = 0;                  // Phase is 0
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
    EPwm1Regs.TBCTR = 0x0000;                       // Clear counter
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;  // Count up
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;         // Disable phase loading
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;        // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;     // Load registers every ZERO
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;

    EPwm1Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
    EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // set actions for EPWM1A
    EPwm1Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;

    EPwm1Regs.AQCTLB.bit.PRD = AQ_NO_ACTION;
    EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;
    EPwm1Regs.AQCTLB.bit.CAU = AQ_CLEAR;            // set actions for EPWM1A
    EPwm1Regs.AQCTLB.bit.CAD = AQ_NO_ACTION;

    //Interrupcao epwm1
    EPwm1Regs.ETSEL.bit.INTEN  = 0;
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;
    EPwm1Regs.ETPS.bit.INTPRD  = ET_3RD;

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
}

void Peripheral_Setup_ConfigEPwm4(void){
    // pg 1978 spruhm8i.pdf - Technical reference
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    CpuSysRegs.PCLKCR2.bit.EPWM4 = 1;
    EDIS;

    EPwm4Regs.TBPRD = ((PWM_PERIPHERAL_FREQUENCY / PWM_04_FREQUENCY) - 1) >> 1;                         // Set timer period
    EPwm4Regs.CMPA.bit.CMPA = 0;

    EPwm4Regs.TBPHS.bit.TBPHS = 0;                  // Phase is 0
    EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
    EPwm4Regs.TBCTR = 0x0000;                       // Clear counter
    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // Count up DOWN
    EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;         // Disable phase loading
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;        // Clock ratio to SYSCLKOUT
    EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;         // Load registers every ZERO
    EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;         // Load registers every ZERO
    EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;   //!< load Duty A on zero and period
    EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;   //!< load Duty B on zero and period

    EPwm4Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
    EPwm4Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // set actions for EPWM4A
    EPwm4Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm4Regs.AQCTLA.bit.CBU = AQ_NO_ACTION;
    EPwm4Regs.AQCTLA.bit.CBD = AQ_NO_ACTION;

    EPwm4Regs.AQCTLB.bit.PRD = AQ_NO_ACTION;
    EPwm4Regs.AQCTLB.bit.ZRO = AQ_NO_ACTION;
    EPwm4Regs.AQCTLB.bit.CAU = AQ_NO_ACTION;            // set actions for EPWM4A
    EPwm4Regs.AQCTLB.bit.CAD = AQ_NO_ACTION;
    EPwm4Regs.AQCTLB.bit.CBU = AQ_CLEAR;            // set actions for EPWM4A
    EPwm4Regs.AQCTLB.bit.CBD = AQ_SET;

    //Interrupcao epwm1
    EPwm4Regs.ETSEL.bit.INTEN  = 1;
    EPwm4Regs.ETSEL.bit.INTSEL = ET_CTR_PRDZERO;        //!< Evento no zero e no pico da portadora
    EPwm4Regs.ETPS.bit.INTPSSEL   = 1;                  //!< 0 : ETPS.bit.INTPRD / 1 ETINTPS.bit.INTPRD2
    EPwm4Regs.ETPS.bit.INTPRD     = ET_2ND;
    EPwm4Regs.ETINTPS.bit.INTPRD2 = ET_5RD;
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
}

void Peripheral_Setup_ConfigEPwm12(void){
    // pg 1978 spruhm8i.pdf - Technical reference
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    CpuSysRegs.PCLKCR2.bit.EPWM12 = 1;
    EDIS;

    EPwm12Regs.TBPRD = (PWM_PERIPHERAL_FREQUENCY / PWM_12_FREQUENCY) - 1;                         // Set timer period
    EPwm12Regs.CMPA.bit.CMPA = 0;

    EPwm12Regs.TBPHS.bit.TBPHS = 0;                  // Phase is 0
    EPwm12Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
    EPwm12Regs.TBCTR = 0x0000;                       // Clear counter
    EPwm12Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;  // Count up
    EPwm12Regs.TBCTL.bit.PHSEN = TB_DISABLE;         // Disable phase loading
    EPwm12Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;        // Clock ratio to SYSCLKOUT
    EPwm12Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm12Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;     // Load registers every ZERO
    EPwm12Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;

    EPwm12Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
    EPwm12Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm12Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // set actions for EPWM1A
    EPwm12Regs.AQCTLA.bit.CAD = AQ_SET;

    //Interrupcao epwm12
    EPwm12Regs.ETSEL.bit.INTEN  = 1;
    EPwm12Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;
    EPwm12Regs.ETPS.bit.INTPRD  = ET_1ST;


    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
}

void Peripheral_Setup_ADC(void){
    EALLOW;
    //
    //write configurations
    //
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4

    AdcSetMode(ADC_ADCA, ADC_RESOLUTION, ADC_SIGNALMODE_SINGLE);
    //AdcaRegs.ADCCTL2.bit.SIGNALMODE = ADC_SIGNALMODE_SINGLE;
    //AdcaRegs.ADCCTL2.bit.RESOLUTION = ADC_RESOLUTION;
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION, ADC_SIGNALMODE_SINGLE);
    //AdcbRegs.ADCCTL2.bit.SIGNALMODE = ADC_SIGNALMODE_SINGLE;
    //AdcbRegs.ADCCTL2.bit.RESOLUTION = ADC_RESOLUTION;
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION, ADC_SIGNALMODE_SINGLE);
    //AdccRegs.ADCCTL2.bit.SIGNALMODE = ADC_SIGNALMODE_SINGLE;
    //AdccRegs.ADCCTL2.bit.RESOLUTION = ADC_RESOLUTION;

    //
    //Set pulse positions to late
    //
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
    //power up the ADCs
    //
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //
    //delay for 1ms to allow ADC time to power up
    //
    DELAY_US(1000);

    Uint16 acqps;

    //
    // Determine minimum acquisition window (in SYSCLKS) based on resolution
    //
    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION)
    {
        acqps = 14; //75ns
    }
    else //resolution is 16-bit
    {
        acqps = 63; //320ns
    }

    //
    //Select the channels to convert and end of conversion flag
    //ADCA
    //
#if HIL_ON
    AdcaRegs.ADCSOC0CTL.bit.CHSEL   = 1;      //SOC0 will convert pin A1 (DAC B)
#else
    AdcaRegs.ADCSOC0CTL.bit.CHSEL   = 15;      //SOC0 will convert pin A15 (PFC Vout)
#endif
    AdcaRegs.ADCSOC0CTL.bit.ACQPS   = acqps;  //sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 0;      //Trigger by software
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    //ADCB
#if HIL_ON
    AdcbRegs.ADCSOC0CTL.bit.CHSEL   = 1;      //SOC0 will convert pin B1 (DAC C)
#else
    AdcbRegs.ADCSOC0CTL.bit.CHSEL   = 5;      //SOC0 will convert pin B5 (PFC Iin)
#endif
    AdcbRegs.ADCSOC0CTL.bit.ACQPS   = acqps;  //sample window is acqps + 1 SYSCLK cycles
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 0;      //Trigger by software
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    //ADCC
#if HIL_ON
    AdccRegs.ADCSOC0CTL.bit.CHSEL   = 12;      //SOC0 will convert pin A0 (DAC A)
#else
    AdccRegs.ADCSOC0CTL.bit.CHSEL   = 5;      //SOC0 will convert pin A5 (PFC Vin)
#endif
    AdccRegs.ADCSOC0CTL.bit.ACQPS   = acqps;  //sample window is acqps + 1 SYSCLK cycles
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 0;      //Trigger by software
    AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
    AdccRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    //ADC A EOC interrupt


    EDIS;
}

void Peripheral_Setup_DAC(void){
    EALLOW;
    //DAC A
    DacaRegs.DACCTL.bit.LOADMODE    = 0;    //  Load mode by SYSCLK
    DacaRegs.DACCTL.bit.DACREFSEL   = 1;    //
    DacaRegs.DACOUTEN.bit.DACOUTEN  = 1;    //  Enable output
    DacaRegs.DACVALS.bit.DACVALS    = 0;    //  Set 0 to dac shadow register
    //DAC B
    DacbRegs.DACCTL.bit.LOADMODE    = 0;    //  Load mode by SYSCLK
    DacbRegs.DACCTL.bit.DACREFSEL   = 1;    //
    DacbRegs.DACOUTEN.bit.DACOUTEN  = 1;    //  Enable output
    DacbRegs.DACVALS.bit.DACVALS    = 0;    //  Set 0 to dac shadow register
    //DAC C
    DaccRegs.DACCTL.bit.LOADMODE    = 0;    //  Load mode by SYSCLK
    DaccRegs.DACCTL.bit.DACREFSEL   = 1;    //
    DaccRegs.DACOUTEN.bit.DACOUTEN  = 1;    //  Enable output
    DaccRegs.DACVALS.bit.DACVALS    = 0;    //  Set 0 to dac shadow register
    EDIS;
}

//
// CLA_configClaMemory - Configure CLA memory sections
//
void Peripheral_Setup_configClaMemory(void)
{
    extern uint32_t Cla1funcsRunStart, Cla1funcsLoadStart, Cla1funcsLoadSize;
    EALLOW;

#ifdef _FLASH
    //
    // Copy over code from FLASH to RAM
    //
    memcpy((uint32_t *)&Cla1funcsRunStart, (uint32_t *)&Cla1funcsLoadStart,
           (uint32_t)&Cla1funcsLoadSize);
#endif //_FLASH

    //
    // Initialize and wait for CLA1ToCPUMsgRAM
    //
    MemCfgRegs.MSGxINIT.bit.INIT_CLA1TOCPU = 1;
    while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CLA1TOCPU != 1){};

    //
    // Initialize and wait for CPUToCLA1MsgRAM
    //
    MemCfgRegs.MSGxINIT.bit.INIT_CPUTOCLA1 = 1;
    while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CPUTOCLA1 != 1){};

#ifdef CPU1
    /********************************************************************
     * Select LS4RAM and LS5RAM to be the programming space for the CLA
     * First configure the CLA to be the master for LS0 to LS2 and then
     * set the space to be a program block
    *********************************************************************/
    MemCfgRegs.LSxMSEL.bit.MSEL_LS0 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS0 = 1;
    MemCfgRegs.LSxMSEL.bit.MSEL_LS1 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS1 = 1;
    MemCfgRegs.LSxMSEL.bit.MSEL_LS2 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS2 = 1;
    /********************************************************************
     * Next configure LS0RAM and LS1RAM as data spaces for the CLA
     * First configure the CLA to be the master for LS3 to LS4 and then
     * set the spaces to be code blocks
    *********************************************************************/
    MemCfgRegs.LSxMSEL.bit.MSEL_LS3 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS3 = 0;
    MemCfgRegs.LSxMSEL.bit.MSEL_LS4 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS4 = 0;
#endif
#ifdef CPU2
    /********************************************************************
     * Select LS4RAM and LS5RAM to be the programming space for the CLA
     * First configure the CLA to be the master for LS1 to LS2 and then
     * set the space to be a program block
    *********************************************************************/
    MemCfgRegs.LSxMSEL.bit.MSEL_LS1 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS1 = 1;
    MemCfgRegs.LSxMSEL.bit.MSEL_LS2 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS2 = 1;
    /********************************************************************
     * Next configure LS0RAM and LS1RAM as data spaces for the CLA
     * First configure the CLA to be the master for LS3 to LS4 and then
     * set the spaces to be code blocks
    *********************************************************************/
    MemCfgRegs.LSxMSEL.bit.MSEL_LS3 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS3 = 0;
    MemCfgRegs.LSxMSEL.bit.MSEL_LS4 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS4 = 0;
#endif

    EDIS;
}

//
// CLA_initCpu1Cla1 - Initialize CLA1 task vectors and end of task interrupts
//

void Peripheral_Setup_initCpu1Cla1( void(* task)(void) , PINT task_callback)
{
    //
    // Compute all CLA task vectors
    // On Type-1 CLAs the MVECT registers accept full 16-bit task addresses as
    // opposed to offsets used on older Type-0 CLAs
    //
    EALLOW;
    Cla1Regs.MVECT1 = (Uint32) task;

    //
    // Enable the IACK instruction to start a task on CLA in software
    // for all  8 CLA tasks. Also, globally enable all 8 tasks (or a
    // subset of tasks) by writing to their respective bits in the
    // MIER register
    //
    Cla1Regs.MCTL.bit.IACKE = 1;
    Cla1Regs.MIER.bit.INT1 = 1;

    //
    // Configure the vectors for the end-of-task interrupt for all
    // 8 tasks
    //
    PieVectTable.CLA1_1_INT = task_callback;

    //
    // Enable CLA interrupts at the group and subgroup levels
    //
    PieCtrlRegs.PIEIER11.bit.INTx1 = 0xFFFF;

    IER |= M_INT11;

    EDIS;
}
