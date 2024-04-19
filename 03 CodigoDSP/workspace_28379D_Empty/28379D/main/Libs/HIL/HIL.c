/*
 *  hil.c
 *
 *  Created on: 07 de dez de 2022
 *      Author: Jose
 */

/**
 * @defgroup hil hil
 * @{
 */

/************
 * Includes *
 ************/

#include "hil.h"

/*************
 * Functions *
 *************/

/**
 * @brief
 *
 * @param hil Ponteiro para estrutura hil
 *
 * @warning
 */
void hil_update(hil_t * hil ,float Fs, float L, float C, float R)
{
    hil->Fs = Fs;
    hil->dt = 1.0 / Fs;

    hil->L = L;
    hil->C = C;
    hil->R = R;

    hil->dt_L  = hil->dt / (L);
    hil->dt_C  = hil->dt / (C);
    hil->dt_RC = hil->dt / (R*C);
}


/**
 * @brief Roda o modelo comutado do conversor boost
 *
 * @param hil Ponteiro para estrutura hil
 *
 *  il_1(i+1) = il_1(i) + dt/L_1*(-(1 - s1(i))*vo(i)   + vin(i) );
 *  vo(i+1)   = vo(i)   + dt/C_o*( (1 - s2(i))*il_2(i) - vo(i)/R_o );
 *
 * @warning
 */
void hil_switched_model_run(hil_t * hil)
{
    if (hil->s == 1){
        hil->dx1 = hil->dx1 + hil->dt_L  * hil->u ;
        hil->dx2 = hil->dx2 - hil->dt_RC * hil->dx2 ;
    }else{
        hil->dx1 = hil->dx1 + hil->dt_L * (hil->u - hil->dx2);
        hil->dx2 = hil->dx2 - hil->dt_RC * hil->dx2 + hil->dt_C * hil->dx1;
    }

    if (hil->dx1 <= 0.0){
        hil->dx1  = 0.0;
    }
}

/**
 * @brief Roda o modelo medio do conversor boost
 *
 * @param hil Ponteiro para estrutura hil
 *
 *  il_1(i+1) = il_1(i) + dt/L_1*(-(1 - s1(i))*vo(i)   + vin(i) );
 *  vo(i+1)   = vo(i)   + dt/C_o*( (1 - s2(i))*il_2(i) - vo(i)/R_o );
 *
 * @warning
 */
void hil_average_model_run(hil_t * hil)
{
    hil->d_complement = 1.0 - hil->d;

    hil->dx1 = hil->dx1 + hil->dt_L * (hil->u - hil->d_complement * hil->dx2);
    hil->dx2 = hil->dx2 - hil->dt_RC * hil->dx2 + hil->d_complement * hil->dt_C * hil->dx1;

    if (hil->dx1 <= 0.0){
        hil->dx1 = 0.0;
    }

}

/**
 * @}
 */
