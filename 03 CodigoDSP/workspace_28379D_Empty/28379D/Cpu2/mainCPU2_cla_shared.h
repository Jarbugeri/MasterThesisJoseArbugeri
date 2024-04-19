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

#define INIT_COUNTER(counter)   counter.period_initial = EPwm12Regs.TBCTR;
#define END_COUNTER(counter)    counter.period_final   = EPwm12Regs.TBCTR;
#define CALCULATE_TIME_COUNTER(counter)                                                         \
        counter.time_us  = (counter.period_final - counter.period_initial) * counter.Ts_us;     \
        counter.time_MHz = 1.0 / counter.time_us;                                               \

/*************
 *  Structs  *
 *************/

typedef struct _counter_t_
{
    float Ts_us;
    float period_initial;     //!<
    float period_final;       //!<
    float time_us;
    float time_MHz;
}counter_t;

/*************
 *  Extern   *
 *************/

extern WaveGenerator_t  WaveGenerator;
extern counter_t        counter_cla;
extern counter_t        counter_cpu;

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
