/*
 * sogi_pll.c
 *
 *  Created on: 3 de mai de 2022
 *      Author: Jose
 */

/**************
 *  Includes  *
 **************/

#include "sogi_pll.h"
#include "math.h"

/*************
 * Functions *
 *************/

/**
 * @brief Recalcula os coeficientes dos filtros do sogi
 *
 * @param sogi  Ponteiro para a estrutura do sogi_pll
 * @param fs    Frequencia de atualizacao do sogi_pll em Hz
 * @param k     Ganho k das estruturas dos filtros pll
 * @param fr    Frequencia da rede do sogi_pll em Hz
 */
void sogi_pll_update_coeff_sogi(sogi_pll_t * sogi)
{
    //!< Variaveis auxiliares para minimizar processamento
    float a = sogi->ts * sogi->ts * sogi->wc *  sogi->wc;       //!< (ts*wc)^2
    float b = 2.0 * sogi->k * sogi->ts * sogi->wc;              //!< 2*k*ts*wc
    float c = a + 4;                                            //!< (ts*wc)^2 + 4
    float d = a * sogi->k;                                      //!< k*(ts*wc)^2
    float aux_div = 1.0 / ( c + b );                            //!< 1 / (4 + 2*k*ts*wc + (ts*wc)^2)

    //!< Coeficientes do filtro de eixo alpha
    sogi->v_alpha.a1 = (2.0 * a - 8)    * aux_div;
    sogi->v_alpha.a2 = (c - b)          * aux_div;
    sogi->v_alpha.b0 = b                * aux_div;
    sogi->v_alpha.b1 = 0.0;
    sogi->v_alpha.b2 = -sogi->v_alpha.b0;

    //!< Coeficientes do filtro de eixo em beta
    sogi->v_beta.a1 = sogi->v_alpha.a1;
    sogi->v_beta.a2 = sogi->v_alpha.a2;
    sogi->v_beta.b0 = d * aux_div;
    sogi->v_beta.b1 = 2.0 * sogi->v_beta.b0;
    sogi->v_beta.b2 = sogi->v_beta.b0;
}

void sogi_pll_so_filter_run(sogi_so_filter_t * so_filter, float input)
{
    so_filter->u2 = so_filter->u1;
    so_filter->u1 = so_filter->u0;
    so_filter->u0 = input;
    so_filter->y2 = so_filter->y1;
    so_filter->y1 = so_filter->y0;
    so_filter->y0 = so_filter->u0 * so_filter->b0 +
                    so_filter->u1 * so_filter->b1 +
                    so_filter->u2 * so_filter->b2 -
                    so_filter->y2 * so_filter->a2 -
                    so_filter->y1 * so_filter->a1;
}

/**
 * @brief Roda o integrador da estrutura do sogi pll
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
 * @param input Entrada do integrador
 */
static inline sogi_pll_integrator_run(sogi_pll_t * sogi, float input)
{
    //!< Update memory
    sogi->integrador.u1 = sogi->integrador.u0;
    sogi->integrador.u0 = input;
    sogi->integrador.y1 = sogi->integrador.y0;

    //!< Run integrator method
//  sogi->integrador.y0 = sogi->integrador.y1 + sogi->integrador.Ki * sogi->ts * sogi->integrador.u1;                                   //!< Forward Euler method
    sogi->integrador.y0 = sogi->integrador.y1 + sogi->integrador.Ki * sogi->ts * sogi->integrador.u0;                                   //!< Backward Euler method
//  sogi->integrador.y0 = sogi->integrador.y1 + sogi->integrador.Ki * sogi->ts * (sogi->integrador.u1 + sogi->integrador.u0) * 0.5; //!< Trapezoidal method
}

/**
 * @brief Roda o PI da estrutura do sogi pll
 * 
 * @param input Entrada do integrador
 */
static inline sogi_pll_pi_run(sogi_pll_t * sogi, float input)
{
    //!< Update memory
    sogi->pi.u1 = sogi->pi.u0;
    sogi->pi.u0 = input;
    sogi->pi.y1 = sogi->pi.y0;

    //!< Run integrator method
//  sogi->pi.y0 = sogi->pi.y1 + sogi->pi.Ki * sogi->ts * sogi->pi.u1;                           //!< Forward Euler method
    sogi->pi.y0 = sogi->pi.y1 + sogi->pi.Ki * sogi->ts * sogi->pi.u0;                           //!< Backward Euler method
//  sogi->pi.y0 = sogi->pi.y1 + sogi->pi.Ki * sogi->ts * (sogi->pi.u1 + sogi->pi.u0) * 0.5; //!< Trapezoidal method

    //!< Output integrator + proportional 
    sogi->pi.output = sogi->pi.y0 + (sogi->pi.Kp * sogi->pi.u0);
}

/**
 * @brief Recalcula os coeficientes dos filtros do sogi
 *
 * @param sogi  Ponteiro para a estrutura do sogi_pll
 * @param fs    Frequencia de atualizacao do sogi_pll
 * @param k     Ganho k das estruturas dos filtros pll
 * @param fr    Frequencia da rede do sogi_pll
 * @param Kp    Ganho proporcional do pi sogi
 * @param Ki    Ganho integral do pi sogi
 */
void sogi_pll_initialize(sogi_pll_t * sogi, float fs, float k, float fr, float Kp, float Ki){

    //!< Atualiza os parametros usado no filtro PLL
    sogi->k  = k;
    sogi->fs = fs;
    sogi->ts = 1.0 / fs;
    sogi->fr = fr;
    sogi->wc = TWO_PI * fr;
    sogi->wt_offset = 0.0;
    //!< Update sogi filter
    sogi_pll_update_coeff_sogi(sogi);

    //!< Update pi filter
    sogi->pi.Kp = Kp;
    sogi->pi.Ki = Ki;

    //!< Update integrator filter
    sogi->integrador.Ki = 1.0; //!< Always 1 integrator gain.

    //!< Referencia para integrador
    sogi->w_ref = TWO_PI * fr;

}

/**
 * @brief Subtraia o estado de saida do integrador pelo valor sub
 *
 * @param sogi  Ponteiro para a estrutura do sogi_pll
 * @param sub valor de decremento dos estados de saida
 */
static inline sogi_pll_reset_integrador(sogi_pll_t * sogi, float sub){
    sogi->integrador.y0 -= sub;
    sogi->integrador.y1 -= sub;
}

/**
 * @brief Roda a estrutura do pll sogi
 *
 * @param sogi  Ponteiro para a estrutura do sogi_pll
 * @param input Tensao de entrada da rede
 */
void sogi_pll_run(sogi_pll_t * sogi, float input){

    sogi->input = input;

    /* SOGI Filter*/
    sogi_pll_so_filter_run(&sogi->v_alpha, sogi->input);
    sogi_pll_so_filter_run(&sogi->v_beta, sogi->input);

    // alpha beta to DQ0
    //!< Auxiliar sine and cosine
    sogi->cosine = cos(sogi->wt);
    sogi->sine   = sin(sogi->wt);

    sogi->d = sogi->v_alpha.y0 * sogi->sine   - sogi->v_beta.y0 * sogi->cosine;
    sogi->q = sogi->v_alpha.y0 * sogi->cosine + sogi->v_beta.y0 * sogi->sine;

    // PI(z)
    sogi_pll_pi_run(sogi, sogi->q);

    sogi->omega = sogi->w_ref + sogi->pi.output;

    // I(z)
    sogi_pll_integrator_run(sogi, sogi->omega);
    if (sogi->integrador.y0 >= TWO_PI){
        sogi_pll_reset_integrador(sogi, TWO_PI);
        sogi->zero_crossing = 1;
    }else{
        sogi->zero_crossing = 0;
    }

    sogi->wt = sogi->integrador.y0;

    // rad to Hz
    sogi->freq = sogi->omega * RAD2HZ;

}
