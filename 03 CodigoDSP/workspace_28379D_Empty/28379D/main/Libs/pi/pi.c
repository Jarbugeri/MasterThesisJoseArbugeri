/*
 * pi.h
 *
 *  Created on: 30 de jan de 2023
 *      Author: Jose
 */

/**************
 *  Includes  *
 **************/

#include "pi.h"

/*************
 * Functions *
 *************/

/**
 * @brief Recalcula os coeficientes dos filtros do pi
 *
 * @param pi  Ponteiro para a estrutura do pi
 * @param fs    Frequencia de atualizacao do pi
 * @param Kp    Ganho proporcional do pi
 * @param Ki    Ganho integral do pi
 */
void pi_initialize(pi_t * pi, float fs, float Kp, float Ki){

    //!< Atualiza os parametros usado no pi
    pi->fs = fs;
    pi->ts = 1.0 / fs;

    //!< Update pi filter
    pi->Kp = Kp;
    pi->Ki = Ki;

    //!< Auxiliares
    pi->Ki_ts = pi->Ki * pi->ts;
}

/**
 * @brief Subtraia o estado de saida do integrador pelo valor sub
 *
 * @param pi  Ponteiro para a estrutura do pi_pll
 * @param sub valor de decremento dos estados de saida
 */
void pi_reset_output(pi_t * pi, float sub){
    pi->y0 -= sub;
    pi->y1 -= sub;
}

/**
 * @brief Subtraia o estado de saida do integrador pelo valor sub
 *
 * @param pi  Ponteiro para a estrutura do pi_pll
 * @param sub valor de decremento dos estados de saida
 */
void pi_reset(pi_t * pi){
    pi->y0 = 0;
    pi->y1 = 0;
    pi->u0 = 0;
    pi->u1 = 0;
    pi->output = 0;
}

/**
 * @brief Roda o PI
 *
 * For a given step n > 0 with simulation time t(n), updates output y(n) as follows:
 *
 * Forward Euler method:
 * y(n) = y(n-1) + Ki*[t(n) - t(n-1)]*u(n-1)
 *
 * Backward Euler method:
 * y(n) = y(n-1) + Ki*[t(n) - t(n-1)]*u(n)
 *
 * Trapezoidal method:
 * y(n) = y(n-1) + Ki*[t(n)-t(n-1)]*[u(n)+u(n-1)]/2
 *
 * @param   input Entrada do integrador
 * @return  output of pi controller
 */
float pi_run(pi_t * pi, float input)
{
    //!< Update memory
    pi->u1 = pi->u0;
    pi->u0 = input;
    pi->y1 = pi->y0;

    //!< Run integrator method
//  pi->y0 = pi->y1 + pi->Ki_ts * pi->u1;                      //!< Forward Euler method
    pi->y0 = pi->y1 + pi->Ki_ts * pi->u0;                      //!< Backward Euler method
//  pi->y0 = pi->y1 + pi->Ki_ts * (pi->u1 + pi->u0) * 0.5;     //!< Trapezoidal method

    //!< Output integrator + proportional
    pi->output = pi->y0 + (pi->Kp * pi->u0);

    return pi->output;
}
