/**
 * @file mainCPU1.c
 * @author José Augusto Arbugeri (josearbugeri@gmail.com)
 * @brief Main CPU1 code
 * @version 1.0
 * @date 3 de dez de 2022
 *
 * @copyright Copyright (c) 2022
 *
 */

/**************
 *  Includes  *
 **************/

#include "mainCPU1_cla_shared.h"

/*************
 *  Defines  *
 *************/

long delay_time = 500000;
long counter = 0;

#pragma DATA_SECTION(dataCPU1,"CPU1TOCPU2DATA")
data_cpu1_t dataCPU1 = {0.0};

#pragma DATA_SECTION(dataCPU2,"CPU2TOCPU1DATA")
data_cpu2_t dataCPU2 = {0.0};

#pragma DATA_SECTION(counter1,"Cla1Data")
float counter1 = 0.0;

#define SCOPE_SIZE          2500

#pragma DATA_SECTION(scopeA,"CPU1_SCOPE")
float scopeA[SCOPE_SIZE] = {0};
float * ptr_scopeA  = 0;
Uint16 index_scopeA = 0;

#pragma DATA_SECTION(scopeB,"CPU1_SCOPE")
float scopeB[10] = {0};
float * ptr_scopeB  = 0;
Uint16 index_scopeB = 0;

#pragma DATA_SECTION(scopeC,"CPU1_SCOPE")
float scopeC[10] = {0};
float * ptr_scopeC  = 0;
Uint16 index_scopeC = 0;

#pragma DATA_SECTION(ResultadoA,"CPU1_SCOPE")
float ResultadoA[265];
#pragma DATA_SECTION(ResultadoB,"CPU1_SCOPE")
float ResultadoB[265];
#pragma DATA_SECTION(ResultadoC,"CPU1_SCOPE")
float ResultadoC[265];

#define Vin             220.00000       //!< 220    Vrms
#define L               0.000050        //!< 50     uH
#define C               0.001320        //!< 1.32   mF
#define R               147.0           //!< 147    R

/*************
 *  Structs  *
 *************/

#define SOGI_PLL_FS     2 * PWM_04_FREQUENCY / ET_5RD
#define SOGI_PLL_K      0.707
#define SOGI_PLL_FR     60.0
#define SOGI_PLL_KP     33.0
#define SOGI_PLL_KI     90.0

#pragma DATA_SECTION(sogi_pll,"Cla1Data")
sogi_pll_t sogi_pll = {0.0};

/* Filtros */

#define SO_FILTER_FS                2 * PWM_04_FREQUENCY / ET_5RD
#define SO_FILTER_FC                30000.0
#define SO_FILTER_FR                60.0
#define SO_LPF_IIN_FILTER_Q         0.707
#define SO_BPF_IIN_FILTER_Q         0.05
#define SO_LPF_VIN_FILTER_Q         0.707
#define SO_BPF_VIN_FILTER_Q         1
#define SO_LPF_VOUT_FILTER_Q        0.707
#define SO_BSF_VOUT_FILTER_Q        1.0

#pragma DATA_SECTION(so_filter_lowpass_current_input, "Cla1Data")
so_filter_t so_filter_lowpass_current_input  = {0.0};
#pragma DATA_SECTION(so_filter_bandpass_current_input,"Cla1Data")
so_filter_t so_filter_bandpass_current_input  = {0.0};

#pragma DATA_SECTION(so_filter_lowpass_voltage_input,"Cla1Data")
so_filter_t so_filter_lowpass_voltage_input  = {0.0};
#pragma DATA_SECTION(so_filter_bandpass_voltage_input,"Cla1Data")
so_filter_t so_filter_bandpass_voltage_input  = {0.0};

#pragma DATA_SECTION(so_filter_lowpass_voltage_output,"Cla1Data")
so_filter_t so_filter_lowpass_voltage_output = {0.0};
#pragma DATA_SECTION(so_filter_notch_voltage_output,"Cla1Data")
so_filter_t so_filter_notch_voltage_output   = {0.0};


/* COUNTER */

#pragma DATA_SECTION(counter_cla,"Cla1Data")
counter_t counter_cla = { 1000000.0 / 200000000.0, 0, 0, 0};
#pragma DATA_SECTION(counter_cpu,"Cla1Data")
counter_t counter_cpu = { 1000000.0 / 200000000.0, 0, 0, 0};

/* PFC */

#pragma DATA_SECTION(pfc,"Cla1Data")
pfc_t pfc = {0};

/* PFC */

#define PI_FS       2 * PWM_04_FREQUENCY / ET_5RD
#define PI_V_KP     0.02531
#define PI_V_KI     3.834
#define PI_I_KP     0.03488
#define PI_I_KI     1371.0

pi_t pi_voltage = {0};
pi_t pi_current = {0};


#pragma DATA_SECTION(ema_vin,"Cla1Data")
ema_t ema_vin = INIT_EMA(20000);
#pragma DATA_SECTION(ema_vout,"Cla1Data")
ema_t ema_vout = INIT_EMA(20000);
#pragma DATA_SECTION(ema_iin,"Cla1Data")
ema_t ema_iin = INIT_EMA(20000);

/************************
 * Functions prototypes *
 ************************/

__interrupt void isr_epwm1(void);
__interrupt void isr_epwm4(void);
__interrupt void isr_Adca(void);
__interrupt void Cla1Task1_callback();

void waitCPU2(void);

void pfc_control_run(void);
void pfc_self_control_run(void);
void pfc_open_loop_run(void);
void pfc_test_pll_run(void);
void pfc_initialize_configuration(pfc_t * pfc);
void pfc_gain_response(pfc_t * pfc);

/*************
 * Main CPU1 *
 *************/

int mainCPU1(void){

    Peripheral_Setup_ADC();

    EALLOW;
    PieVectTable.ADCA1_INT = &isr_Adca; //Inicia na calibração pois passa para : &isr_Adca
    PieVectTable.EPWM1_INT = &isr_epwm1;
    PieVectTable.EPWM4_INT = &isr_epwm4;
    EDIS;

    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx4 = 1;

    Peripheral_Setup_ConfigEPwm1();
    Peripheral_Setup_ConfigEPwm4();

    Peripheral_Setup_DAC();

    Peripheral_Setup_initCpu1Cla1(&Cla1Task1, &Cla1Task1_callback);

    IER |= M_INT1;      //!< Enable group 1 interrupts (ADC)
    IER |= M_INT3;      //!< Enable group 3 interrupts (PWM)

    /* Initialize HIL parameters */

    dataCPU1.voltage_input      = Vin;
    dataCPU1.Indutor            = L;
    dataCPU1.Capacitor          = C;
    dataCPU1.Carga              = R;
    dataCPU1.Freq_hil           = (float) PWM_12_FREQUENCY;

    waitCPU2();

    /* Scopes Variables */
    ptr_scopeA = &pfc.voltage_in.filtered;
    ptr_scopeB = &pfc.current_in.filtered;
    ptr_scopeC = &pfc.voltage_out.filtered;

    /* Initizalize sogi pll */
    sogi_pll_initialize(&sogi_pll, SOGI_PLL_FS, SOGI_PLL_K, SOGI_PLL_FR, SOGI_PLL_KP, SOGI_PLL_KI);

    /* Initizalize so filters */
    /* Current in filters */
    so_filter_initialize(&so_filter_lowpass_current_input,  SO_FILTER_FS, SO_FILTER_FC,       SO_LPF_IIN_FILTER_Q, SO_FILTER_LOWPASS,   PRE_WARP_OFF);
    so_filter_initialize(&so_filter_bandpass_current_input, SO_FILTER_FS, SO_FILTER_FR,       SO_BPF_IIN_FILTER_Q, SO_FILTER_BANDPASS,  PRE_WARP_OFF);
    /* Voltage in filters */
    so_filter_initialize(&so_filter_lowpass_voltage_input,  SO_FILTER_FS, SO_FILTER_FC,        SO_LPF_VIN_FILTER_Q, SO_FILTER_LOWPASS,  PRE_WARP_OFF);
    so_filter_initialize(&so_filter_bandpass_voltage_input, SO_FILTER_FS, SO_FILTER_FR,        SO_BPF_VIN_FILTER_Q, SO_FILTER_BANDPASS, PRE_WARP_OFF);
    /* Voltage out filters */
    so_filter_initialize(&so_filter_lowpass_voltage_output, SO_FILTER_FS,  100.0,            SO_LPF_VOUT_FILTER_Q, SO_FILTER_LOWPASS, PRE_WARP_OFF);
    so_filter_initialize(&so_filter_notch_voltage_output,   SO_FILTER_FS,  2.0 * SO_FILTER_FR, SO_LPF_VOUT_FILTER_Q, SO_FILTER_NOTCH  , PRE_WARP_OFF );

    /* Initizalize pfc struct values */
    pfc_initialize_configuration(&pfc);

    /* Initizalize pi struct values */
    pi_initialize(&pi_voltage, PI_FS, PI_V_KP, PI_V_KI);
    pi_initialize(&pi_current, PI_FS, PI_I_KP, PI_I_KI);

    EINT;
    ERTM;

    while(1){

        CALCULATE_TIME_COUNTER(counter_cla);
        CALCULATE_TIME_COUNTER(counter_cpu);

        //so_filter_prewarp_frequency(&so_filter_lowpass_current_input,  so_filter_lowpass_current_input.fc);
        so_filter_prewarp_frequency(&so_filter_bandpass_current_input,  so_filter_bandpass_current_input.fc);

        //so_filter_prewarp_frequency(&so_filter_lowpass_voltage_input,  so_filter_lowpass_voltage_input.fc);
        so_filter_prewarp_frequency(&so_filter_bandpass_voltage_input,  so_filter_bandpass_voltage_input.fc);

        so_filter_prewarp_frequency(&so_filter_lowpass_voltage_output, so_filter_lowpass_voltage_output.fc);
        so_filter_prewarp_frequency(&so_filter_notch_voltage_output, so_filter_notch_voltage_output.fc);

        //so_filter_update_coeff_lowpass(&so_filter_lowpass_current_input);
        so_filter_update_coeff_bandpass(&so_filter_bandpass_current_input);

        //so_filter_update_coeff_lowpass(&so_filter_lowpass_voltage_input);
        so_filter_update_coeff_bandpass(&so_filter_bandpass_voltage_input);

        so_filter_update_coeff_lowpass(&so_filter_lowpass_voltage_output);
        so_filter_update_coeff_notch(&so_filter_notch_voltage_output);


        //sogi_pll_update_coeff_sogi(&sogi_pll);

        GPIO_WritePin(GPIO_LED_BLUE, !pfc.status.voltage_flag);
        GPIO_WritePin(GPIO_LED_RED,  !pfc.status.current_flag);
    }

}

#pragma CODE_SECTION(isr_epwm4,".TI.ramfunc");
__interrupt void isr_epwm4(void){

    EPwm1Regs.TBCTR = 0x0000;

    EPwm4Regs.ETCLR.bit.INT = 1;        //!< Reset interrupt flag

    INIT_COUNTER(counter_cla);


    //!< Get filtered measurements from CLA filters
    if (so_filter_bandpass_current_input.states.y0 >= 0.0 ){
        pfc.current_in.filtered  = so_filter_bandpass_current_input.states.y0 * pfc.current_in.adc.gain_positive;
    }else{
        pfc.current_in.filtered  = so_filter_bandpass_current_input.states.y0 * pfc.current_in.adc.gain_negative;
    }

    pfc.voltage_in.filtered  = so_filter_bandpass_voltage_input.states.y0;
    pfc.voltage_out.filtered = so_filter_notch_voltage_output.states.y0;
    pfc.status.zero_crossing = sogi_pll.zero_crossing;


    //GPIO_WritePin(GPIO_DEBUG, pfc.status.zero_crossing);

    Cla1ForceTask1();

    AdccRegs.ADCSOCFRC1.bit.SOC0 = 1;   //!<Init new conversion ADC C
    AdcbRegs.ADCSOCFRC1.bit.SOC0 = 1;   //!<Init new conversion ADC B
    AdcaRegs.ADCSOCFRC1.bit.SOC0 = 1;   //!<Init new conversion ADC A

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}


#pragma CODE_SECTION(isr_Adca,".TI.ramfunc");
__interrupt void isr_Adca(void){

    INIT_COUNTER(counter_cpu);

    #if HIL_ON
        pfc.current_in.raw  = dataCPU2.current_input;  //!< Get result from ADC
        pfc.voltage_in.raw  = dataCPU2.voltage_input;  //!< Get result from ADC
        pfc.voltage_out.raw = dataCPU2.voltage_output; //!< Get result from ADC

        limiter_saturation(pfc.current_in.raw, -50.00, 50.00);
        limiter_saturation(pfc.voltage_in.raw, -500.0, 500.0);
        limiter_saturation(pfc.voltage_out.raw,-500.0, 500.0);
    #else

        pfc.voltage_out.adc.value = (float) AdcaResultRegs.ADCRESULT0;  //!< Get result from ADC
        pfc.voltage_in.adc.value  = (float) AdcbResultRegs.ADCRESULT0;  //!< Get result from ADC
        pfc.current_in.adc.value  = (float) AdccResultRegs.ADCRESULT0;  //!< Get result from ADC

        CONVERT_ADCVALUE_TO_RAW_NO_GAIN(pfc.current_in);                //!< Convert ADC value to real measurement value
        //CONVERT_ADCVALUE_TO_RAW(pfc.current_in);                //!< Convert ADC value to real measurement value
        CONVERT_ADCVALUE_TO_RAW(pfc.voltage_in);                        //!< Convert ADC value to real measurement value
        CONVERT_ADCVALUE_TO_RAW(pfc.voltage_out);                       //!< Convert ADC value to real measurement value
    #endif

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    /* PFC protection */
#if ENABLE_PROTECTION
    if (fabsf(pfc.current_in.filtered) >= pfc.status.current_max_value){
        pfc.control.bit.enable_pfc = 0;
        pfc.status.current_flag = 1;
        pfc.status.current_value = pfc.current_in.filtered;
    }
    if (pfc.voltage_out.filtered >= pfc.status.voltage_max_value){
        pfc.control.bit.enable_pfc = 0;
        pfc.status.voltage_flag = 1;
        pfc.status.voltage_value = pfc.voltage_out.filtered;
    }
#endif

    if(pfc.control.bit.enable_pfc){
        GPIO_WritePin(GPIO_PWM_EN_P, 1);
        GPIO_WritePin(GPIO_PWM_EN_N, 1);
        pfc_self_control_run();
    }else{
        GPIO_WritePin(GPIO_PWM_EN_P, 0);
        GPIO_WritePin(GPIO_PWM_EN_N, 0);
        pi_reset(&pi_current);
        pi_reset(&pi_voltage);
        pfc.control.bit.enable_current_loop = 0;
        pfc.control.bit.enable_voltage_loop = 0;
        EPwm4Regs.CMPA.bit.CMPA = 0;
        EPwm4Regs.CMPB.bit.CMPB = 0;

        dataCPU1.duty_p_pu = 0;
    }



    /* Scope update */
    if(++index_scopeA > (Uint16) SCOPE_SIZE) index_scopeA = 0;
    scopeA[index_scopeA] = (float) * ptr_scopeA;
    //if(++index_scopeB > (Uint16) SCOPE_SIZE) index_scopeB = 0;
    //scopeB[index_scopeB] = (float) * ptr_scopeB;
    //if(++index_scopeC > (Uint16) SCOPE_SIZE) index_scopeC = 0;
    //scopeC[index_scopeC] = (float) * ptr_scopeC;


    /* Update DAC output */
    #if HIL_ON && DAC_CPU1
        DacaRegs.DACVALS.bit.DACVALS = (Uint16) ((( (float) * ptr_DACA) * DACA_gain) + DACA_offset);
        DacbRegs.DACVALS.bit.DACVALS = (Uint16) ((( (float) * ptr_DACB) * DACB_gain) + DACB_offset);
        DaccRegs.DACVALS.bit.DACVALS = (Uint16) ((( (float) * ptr_DACC) * DACC_gain) + DACC_offset);
    #endif

    END_COUNTER(counter_cpu);

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}


void pfc_control_run(void)
{
    //!< Habilita o controle de corrente quando ocorrer cruzamento do zero
    if(sogi_pll.zero_crossing) pfc.control.bit.enable_current_loop = 1;

    // Habilita o controle de tensão quando a tensão de saída estiver acima da tensão de referência e ocorrer cruzamento do zero
    //if((pfc.voltage_out.filtered > pfc.voltage_out.reference) && sogi_pll.zero_crossing) pfc.control.bit.enable_voltage_loop = 1;

    // Controla a tensão de saída
    if (pfc.control.bit.enable_voltage_loop){
        pfc.voltage_out.error = pfc.voltage_out.filtered - pfc.voltage_out.reference;   // Calcula o erro de tensão
        pi_run(&pi_voltage, pfc.voltage_out.error);                                     // Executa o controlador PI de tensão
        //limiter_saturation(pi_voltage.output, -10.00, 10.00);                           // Limita a saída do controlador de te
    }else{
        pi_voltage.output = 0.0;                                                        // Caso desabilitado força saida para zero
    }

    // Define a referência de corrente de entrada
    pfc.current_in.reference = pfc.gains.k_current_reference * sogi_pll.d * sogi_pll.sine * (1.0 + pi_voltage.output);


    // Controla a corrente de entrada
    if (pfc.control.bit.enable_current_loop){
        pfc.current_in.error = fabsf(pfc.current_in.reference) - fabsf(pfc.current_in.filtered);
        pi_run(&pi_current, pfc.current_in.error);
        limiter_saturation(pi_current.output, 0.00, 1.00);
    }

    // Define os valores da modulação PWM para as chaves do conversor
    pfc.duty_p.value_pu = pi_current.output;
    pfc.duty_n.value_pu = pi_current.output;

    // Define os valores da modulação PWM em unidades PWM
    pfc.duty_p.value = pfc.duty_p.value_pu * pfc.duty_p.gain;
    pfc.duty_n.value = pfc.duty_n.value_pu * pfc.duty_n.gain;

    // Realiza a modulação PWM para acionamento das chaves do conversor
    if(pfc.voltage_in.filtered >= 0.0){   //!< Aciona chave Sp e desabilita chave Sn
        EPwm4Regs.CMPA.bit.CMPA = (Uint16) pfc.duty_p.value;
        EPwm4Regs.CMPB.bit.CMPB = 0;
    }else{                      //!< Aciona chave Sn e desabilita chave Sp
        EPwm4Regs.CMPA.bit.CMPA = 0;
        EPwm4Regs.CMPB.bit.CMPB = (Uint16) pfc.duty_n.value;
    }
}

/**

@brief Função que executa o controle PFC na placa DSP Texas 28379D.

@details A função é responsável por realizar o controle de um conversor PFC, através do uso de

controladores PI para o controle de tensão e corrente. Além disso, a função também realiza a modulação

PWM para acionamento das chaves do conversor.

@note A função assume que foram previamente inicializados os parâmetros dos controladores PI e dos ganhos do conversor.

@return void.
*/
void pfc_self_control_run(void)
{
    //!< Habilita o controle de corrente quando ocorrer cruzamento do zero
    if(sogi_pll.zero_crossing) pfc.control.bit.enable_current_loop = 1;

    // Habilita o controle de tensão quando a tensão de saída estiver acima da tensão de referência e ocorrer cruzamento do zero
    if((pfc.voltage_out.filtered > pfc.voltage_out.reference) && sogi_pll.zero_crossing) pfc.control.bit.enable_voltage_loop = 1;

    // Controla a tensão de saída
    if (pfc.control.bit.enable_voltage_loop){
        pfc.voltage_out.error = pfc.voltage_out.filtered - pfc.voltage_out.reference;   // Calcula o erro de tensão
        pi_run(&pi_voltage, pfc.voltage_out.error);                                     // Executa o controlador PI de tensão
        //limiter_saturation(pi_voltage.output, -10.00, 10.00);                           // Limita a saída do controlador de te
    }else{
        pi_voltage.output = 0.0;                                                        // Caso desabilitado força saida para zero
    }

    // Define a referência de corrente de entrada
    pfc.current_in.reference = 1.0 + pi_voltage.output;

    float moduladora = 0.0;
    // Controla a corrente de entrada
    if (pfc.control.bit.enable_current_loop){
        moduladora = fabsf(pfc.current_in.filtered) * pfc.gains.ki_self_control * pfc.current_in.reference; // Calcula o valor da modulação PWM
        limiter_saturation(moduladora, 0.00, 1.00);                                                         // Limita a modulação PWM
    }

    // Define os valores da modulação PWM para as chaves do conversor
    pfc.duty_p.value_pu = 1.0 - moduladora;
    pfc.duty_n.value_pu = 1.0 - moduladora;

    // Define os valores da modulação PWM em unidades PWM
    pfc.duty_p.value = pfc.duty_p.value_pu * pfc.duty_p.gain;
    pfc.duty_n.value = pfc.duty_n.value_pu * pfc.duty_n.gain;

    // Realiza a modulação PWM para acionamento das chaves do conversor
    if(pfc.voltage_in.filtered >= 0.0){   //!< Aciona chave Sp e desabilita chave Sn
        EPwm4Regs.CMPA.bit.CMPA = (Uint16) pfc.duty_p.value;
        EPwm4Regs.CMPB.bit.CMPB = 0;
    }else{                      //!< Aciona chave Sn e desabilita chave Sp]
        EPwm4Regs.CMPA.bit.CMPA = 0;
        EPwm4Regs.CMPB.bit.CMPB = (Uint16) pfc.duty_n.value;
    }
}


void pfc_open_loop_run(void)
{

    //if(pfc.voltage_in.filtered >= 0.0){
        //!< Aciona chave Sp e desabilita chave Sn
        pfc.duty_p.value = pfc.duty_p.value_pu * pfc.duty_p.gain;
        EPwm4Regs.CMPA.bit.CMPA = (Uint16) pfc.duty_p.value;
    //}else{
        //!< Aciona chave Sn e desabilita chave Sp
        pfc.duty_n.value = pfc.duty_n.value_pu * pfc.duty_n.gain;
        EPwm4Regs.CMPB.bit.CMPB = (Uint16) pfc.duty_n.value;
    //}

    dataCPU1.duty_p_pu = pfc.duty_p.value_pu;
    dataCPU1.duty_n_pu = pfc.duty_n.value_pu;


}
void pfc_test_pll_run(void)
{

    if (!pfc.control.bit.enable_current_loop){
        pfc.current_in.reference = 310.0 / 380.0;
    }

    float moduladora = 0.0;

    moduladora = fabsf(pfc.current_in.reference * sogi_pll.sine); // Calcula o valor da modulação PWM


    // Define os valores da modulação PWM para as chaves do conversor
    pfc.duty_p.value_pu = 1.0 - moduladora;
    pfc.duty_n.value_pu = 1.0 - moduladora;

    // Define os valores da modulação PWM em unidades PWM
    pfc.duty_p.value = pfc.duty_p.value_pu * pfc.duty_p.gain;
    pfc.duty_n.value = pfc.duty_n.value_pu * pfc.duty_n.gain;

    // Realiza a modulação PWM para acionamento das chaves do conversor
    if(pfc.voltage_in.filtered >= 0.0){   //!< Aciona chave Sp e desabilita chave Sn
        EPwm4Regs.CMPA.bit.CMPA = (Uint16) pfc.duty_p.value;
        EPwm4Regs.CMPB.bit.CMPB = 0;
        GPIO_WritePin(GPIO_DEBUG, 1);
    }else{                      //!< Aciona chave Sn e desabilita chave Sp
        EPwm4Regs.CMPA.bit.CMPA = 0;
        EPwm4Regs.CMPB.bit.CMPB = (Uint16) pfc.duty_n.value;
        GPIO_WritePin(GPIO_DEBUG, 0);
    }


}


#pragma CODE_SECTION(isr_epwm1,".TI.ramfunc");
__interrupt void isr_epwm1(void)
{
    EPwm1Regs.ETCLR.bit.INT = 1;    //!< Reset interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

void waitCPU2(void){
    IpcRegs.IPCSET.bit.IPC4 = 1;            // Inform to CPU2 that the CPU1 is OK
    while (IpcRegs.IPCSTS.bit.IPC5 == 0);   // Waiting the CPU2 inform that is OK
    IpcRegs.IPCACK.bit.IPC5 = 1;            // Clears the bit flag that CPU2 used to inform
}

__interrupt void Cla1Task1_callback()
{
    // Acknowledge the end-of-task interrupt for task 1
    PieCtrlRegs.PIEACK.all = M_INT11;
    END_COUNTER(counter_cla);
    counter++;
}

void pfc_initialize_configuration(pfc_t * pfc)
{

    pfc->current_in.adc.gain = -0.00081 ;
    pfc->current_in.adc.number_bits = 12 + ADC_RESOLUTION * 4;
    pfc->current_in.adc.offset = 30600;
    pfc->current_in.adc.gain_positive = -0.0007;
    pfc->current_in.adc.gain_negative = -0.00075;

    pfc->voltage_in.adc.gain = 0.0125000002  ;
    pfc->voltage_in.adc.number_bits = 12 + ADC_RESOLUTION * 4;
    pfc->voltage_in.adc.offset = 33700;

    pfc->voltage_out.adc.gain = 0.00789999962 ;
    pfc->voltage_out.adc.number_bits = 12 + ADC_RESOLUTION * 4;
    pfc->voltage_out.adc.offset = 89;

    pfc->duty_p.gain = EPwm4Regs.TBPRD;
    pfc->duty_p.value_pu = 0.0;

    pfc->duty_n.gain = EPwm4Regs.TBPRD;
    pfc->duty_n.value_pu = 0.0;

    pi_voltage.y0 = 0.0;
    pi_voltage.y1 = 0.0;

    pfc->voltage_out.reference = 200.0;

    pfc->status.current_max_value = 15.0;
    pfc->status.voltage_max_value = 250.0;

    pfc->gains.ki_self_control = 0.039;
    pfc->gains.k_current_reference = 21.0 / 311.0;

    //sogi_pll.wt_offset = -0.0680000037;

}

void pfc_gain_response(pfc_t * pfc)
{
    if (pfc->debug.enable){
        GPIO_WritePin(GPIO_DEBUG, 1);
        EPwm4Regs.CMPA.bit.CMPA = pfc->duty_p.value;
        if(++pfc->debug.counter > 150000L){

            pfc->debug.counter = 0;

            ResultadoA[(Uint16) pfc->duty_p.value] = pfc->voltage_in.filtered;
            ResultadoB[(Uint16) pfc->duty_p.value] = pfc->current_in.filtered ;
            ResultadoC[(Uint16) pfc->duty_p.value] = pfc->voltage_out.filtered;

            pfc->duty_p.value ++;

            if(pfc->duty_p.value == 265){
                pfc->debug.enable = 0;
                GPIO_WritePin(GPIO_DEBUG, 0);
            }
        }
    }
}
