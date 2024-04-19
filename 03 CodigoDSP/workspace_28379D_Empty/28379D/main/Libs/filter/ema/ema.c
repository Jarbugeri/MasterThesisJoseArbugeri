/*
 * sogi_pll.c
 *
 *  Created on: 18 de jan de 2023
 *      Author: Jose
 */

/**************
 *  Includes  *
 **************/

#include "ema.h"

/*************
 * Functions *
 *************/


/**
 * @brief Roda a estrutura do filtro SO
 *
 * @param so_filter Ponteiro para a estrutura do filtro
 * @param input     Entrada do filtro
 */
void ema_update(ema_t * ema, Uint16 N){
    ema->N = N;
    ema->alpha = 2.0 / (N + 1.0);
}

/**
 * @brief Roda a estrutura do filtro SO
 *
 * @param so_filter Ponteiro para a estrutura do filtro
 * @param input     Entrada do filtro
 */
void ema_run(ema_t * ema, float input){
    //!< Input
    ema->u0 = input;
    //!< Output
    ema->y1 =  ema->y0;
    ema->y0 =  ema->alpha * ema->u0 + (1 -  ema->alpha) *  ema->y1;
}
