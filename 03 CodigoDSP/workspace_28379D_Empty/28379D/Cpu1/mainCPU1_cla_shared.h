#ifndef _MAINCPU1_CLA_SHARED_H_
#define _MAINCPU1_CLA_SHARED_H_

/**************
 *  Includes  *
 **************/

#include "F28x_Project.h"
#include "F2837xD_Peripheral_Setup.h"
#include "cpu_data_shared.h"
#include "libs.h"

#ifdef __cplusplus
extern "C" {
#endif

/*************
 *  Defines  *
 *************/

#define ENABLE_PROTECTION           1

#define INIT_COUNTER(counter)   counter.period_initial = EPwm1Regs.TBCTR;
#define END_COUNTER(counter)    counter.period_final = EPwm1Regs.TBCTR;
#define CALCULATE_TIME_COUNTER(counter)                                                         \
        counter.time_us  = (counter.period_final - counter.period_initial) * counter.Ts_us;     \
        counter.time_MHz = 1.0 / counter.time_us;                                               \

#define CONVERT_ADCVALUE_TO_RAW(measurement)                                                     \
        measurement.raw = (measurement.adc.value - measurement.adc.offset) * measurement.adc.gain;        \

#define CONVERT_ADCVALUE_TO_RAW_NO_GAIN(measurement)                                                     \
        measurement.raw = measurement.adc.value - measurement.adc.offset;                                \




/*************
 *  Structs  *
 *************/


typedef struct _gain_response_t_
{
    Uint16 enable;
    long counter;
    float sumA;
    float sumB;
    float sumC;

}gain_response_t;

typedef struct _counter_t_
{
    float Ts_us;
    float period_initial;     //!<
    float period_final;       //!<
    float time_us;
    float time_MHz;
}counter_t;

typedef struct _adc_t_
{
    float value;        //!< Raw value of ADC (16 bits or 12 bits)
    float gain;         //!< Gain of measurement
    float offset;       //!< Offset gain of measurement
    float number_bits;  //!< 16 bits or 12 bits
    float gain_positive;         //!< Gain of measurement
    float gain_negative;         //!< Gain of measurement
}adc_t;

typedef struct _measurement_t_
{
    adc_t adc;          //!< ADC struct
    float raw;          //!< Faw value of adc measurement
    float filtered;     //!< Filtered value of adc measurement
    float reference;    //!< Reference value
    float error;        //!< Erro referente a malha
}measurement_t;

typedef struct _duty_cycle_t_
{
    float value_pu;     //!< Duty cycle of pwm in pu
    float value;        //!< Duty cycle of pwm with pwm gain
    float gain;         //!< Pwm gain
}duty_cycle_t;

typedef struct _pfc_control_bits_t_
{
    Uint16 enable_pfc           :1;             //!< Duty cycle of pwm in pu
    Uint16 enable_current_loop  :1;        //!< Duty cycle of pwm with pwm gain
    Uint16 enable_voltage_loop  :1;         //!< Pwm gain
    Uint16 rsv                  :13;
}pfc_control_bits_t;

typedef union _pfc_control_t_
{
    pfc_control_bits_t  bit;
    Uint16              all;
}pfc_control_t;

typedef struct _pfc_status_t_
{
    Uint16 current_flag;
    Uint16 voltage_flag;
    Uint16 zero_crossing;
    float  current_value;
    float  voltage_value;
    float  current_max_value;
    float  voltage_max_value;

}pfc_status_t;

typedef struct _pfc_gains_t_
{
    float   ki_self_control;
    float   k_current_reference;

}pfc_gains_t;

typedef struct _pfc_t_
{
    pfc_control_t control;
    pfc_status_t  status;
    measurement_t voltage_in;   //!< ADC struct for voltage in
    measurement_t current_in;   //!< ADC struct for voltage in
    measurement_t voltage_out;  //!< ADC struct for voltage in
    duty_cycle_t  duty_p;       //!< Duty cycle struct for duty cycle Mosfet Positive
    duty_cycle_t  duty_n;       //!< Duty cycle struct for duty cycle Mosfet Negative

    gain_response_t debug;
    pfc_gains_t     gains;
}pfc_t;

/*************
 *  Extern   *
 *************/

extern counter_t counter_cla;
extern counter_t counter_cpu;
extern float counter1;
extern sogi_pll_t sogi_pll;
extern WaveGenerator_t WaveGenerator;

extern pfc_t pfc;

extern so_filter_t so_filter_lowpass_current_input;
extern so_filter_t so_filter_bandpass_current_input;

extern so_filter_t so_filter_lowpass_voltage_input;
extern so_filter_t so_filter_bandpass_voltage_input;

extern so_filter_t so_filter_lowpass_voltage_output;
extern so_filter_t so_filter_notch_voltage_output;

/************************
 * Functions prototypes *
 ************************/
// The following are symbols defined in the CLA assembly code
// Including them in the shared header file makes them
// .global and the main CPU can make use of them.

__interrupt void Cla1Task1();
__interrupt void Cla1Task2();
__interrupt void Cla1Task3();
__interrupt void Cla1Task4();
__interrupt void Cla1Task5();
__interrupt void Cla1Task6();
__interrupt void Cla1Task7();
__interrupt void Cla1Task8();

#ifdef __cplusplus
}
#endif //!< extern "C"

#endif /* _MAINCPU1_CLA_SHARED_H_ */
