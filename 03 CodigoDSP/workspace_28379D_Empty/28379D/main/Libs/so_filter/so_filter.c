/*
 * sogi_pll.c
 *
 *  Created on: 18 de jan de 2023
 *      Author: Jose
 */

/**************
 *  Includes  *
 **************/

#include "so_filter.h"

/*************
 * Functions *
 *************/

/**
 * @brief Pre distorcao para a frequencia de corte do filtro
 *
 *      wp = 2/T * tan(wc*T/2)
 *
 * @param   fc              Frequencia de atualizacao do so_filter em Hz
 * @return  wp              Retorna a frequencia de corte pre distorcida em radianos
 */
void so_filter_prewarp_frequency(so_filter_t * so_filter, float fc)
{
    so_filter->fc = fc;
    so_filter->wc = 2.0 * so_filter->fs * tan( ONE_PI * fc * so_filter->ts );
}

/**
 * @brief Recalcula os coeficientes dos filtros lowpass
 *
 * @param so_filter     Ponteiro para a estrutura do so_filter
 * @param fs            Frequencia de atualizacao do so_filter em Hz
 * @param Q             Fator de qualidade do filtro
 * @param fc            Frequencia de corte em Hertz
 */
void so_filter_update_coeff_lowpass(so_filter_t * so_filter)
{
    //!< Variaveis auxiliares para minimizar processamento
    float a = so_filter->ts * so_filter->ts * so_filter->wc *  so_filter->wc;   //!< (ts*wc)^2
    float b = 2.0 * so_filter->ts * so_filter->wc / so_filter->Q;               //!< 2*ts*wc/Q
    float aux_div = 1.0 / ( 4 + b + a );                                        //!< 1 / (4 + 2*ts*wc/Q + (ts*wc)^2)

    //!< Coeficientes do filtro de eixo direto
    so_filter->coeffs.a1 = (2.0 * a - 8)    * aux_div;
    so_filter->coeffs.a2 = (a - b + 4)      * aux_div;
    so_filter->coeffs.b0 = a                * aux_div;
    so_filter->coeffs.b1 = 2.0 * so_filter->coeffs.b0;
    so_filter->coeffs.b2 = so_filter->coeffs.b0;
}

/**
 * @brief Recalcula os coeficientes dos filtros lowpass
 *
 * @param so_filter     Ponteiro para a estrutura do so_filter
 * @param fs            Frequencia de atualizacao do so_filter em Hz
 * @param Q             Fator de qualidade do filtro
 * @param fc            Frequencia de corte em Hertz
 */
void so_filter_update_coeff_bandpass(so_filter_t * so_filter)
{
    //!< Variaveis auxiliares para minimizar processamento
    float a = so_filter->ts * so_filter->ts * so_filter->wc *  so_filter->wc;   //!< (ts*wc)^2
    float b = 2.0 * so_filter->ts * so_filter->wc / so_filter->Q;               //!< 2*ts*wc/Q
    float aux_div = 1.0 / ( 4 + b + a );                                        //!< 1 / (4 + 2*ts*wc/Q + (ts*wc)^2)

    //!< Coeficientes do filtro de eixo direto
    so_filter->coeffs.a1 = (2.0 * a - 8)    * aux_div;
    so_filter->coeffs.a2 = (a - b + 4)      * aux_div;
    so_filter->coeffs.b0 = b                * aux_div;
    so_filter->coeffs.b1 = 0.0;
    so_filter->coeffs.b2 = - so_filter->coeffs.b0;
}

/**
 * @brief Recalcula os coeficientes dos filtros lowpass
 *
 * @param so_filter     Ponteiro para a estrutura do so_filter
 * @param fs            Frequencia de atualizacao do so_filter em Hz
 * @param Q             Fator de qualidade do filtro
 * @param fc            Frequencia de corte em Hertz
 */
void so_filter_update_coeff_notch(so_filter_t * so_filter)
{
    //!< Variaveis auxiliares para minimizar processamento
    float a = so_filter->ts * so_filter->ts * so_filter->wc *  so_filter->wc;   //!< (ts*wc)^2
    float b = 2.0 * so_filter->ts * so_filter->wc / so_filter->Q;               //!< 2*ts*wc/Q
    float c = a + 4;                                                            //!< (ts*wc)^2 + 4
    float aux_div = 1.0 / ( b + c );                                            //!< 1 / (4 + 2*ts*wc/Q + (ts*wc)^2)

    //!< Coeficientes do filtro de eixo direto
    so_filter->coeffs.a1 = (2.0 * a - 8)    * aux_div;
    so_filter->coeffs.a2 = (c - b)          * aux_div;
    so_filter->coeffs.b0 = c                * aux_div;
    so_filter->coeffs.b1 = so_filter->coeffs.a1;
    so_filter->coeffs.b2 = so_filter->coeffs.b0;
}

/**
 * @brief Recalcula os coeficientes dos filtros lowpass
 *
 * @param so_filter     Ponteiro para a estrutura do so_filter
 * @param fs            Frequencia de atualizacao do so_filter em Hz
 * @param Q             Fator de qualidade do filtro
 * @param fc            Frequencia de corte em Hertz
 */
void so_filter_update_coeff_highpass(so_filter_t * so_filter)
{
    float a = so_filter->ts * so_filter->ts * so_filter->wc *  so_filter->wc;   //!< (ts*wc)^2
    float b = 2.0 * so_filter->ts * so_filter->wc / so_filter->Q;               //!< 2*ts*wc/Q
    float c = a + 4;                                                            //!< (ts*wc)^2 + 4
    float aux_div = 1.0 / ( b + c );                                            //!< 1 / (4 + 2*ts*wc/Q + (ts*wc)^2)
    //!< Coeficientes do filtro de eixo direto
    so_filter->coeffs.a1 = (2.0 * a - 8)    * aux_div;
    so_filter->coeffs.a2 = (c - b)          * aux_div;
    so_filter->coeffs.b0 = 4.0              * aux_div;
    so_filter->coeffs.b1 = - 2.0 * so_filter->coeffs.b0;
    so_filter->coeffs.b2 = so_filter->coeffs.b0;
}

/**
 * @brief Reseta e inicializa filtro com valor desejado
 *
 * @param so_filter Ponteiro para a estrutura do filtro
 * @param value     Valore inicial de reset do filtro
 */
void so_filter_reset(so_filter_t * so_filter, float value){
    so_filter->states.u2 = value;
    so_filter->states.u1 = value;
    so_filter->states.u0 = value;
    so_filter->states.y2 = value;
    so_filter->states.y1 = value;
    so_filter->states.y0 = value;
}

/**
 * @brief Roda a estrutura do filtro SO
 *
 * @param so_filter Ponteiro para a estrutura do filtro
 * @param input     Entrada do filtro
 */
void so_filter_run(so_filter_t * so_filter, float input){
    so_filter->states.u2 = so_filter->states.u1;
    so_filter->states.u1 = so_filter->states.u0;
    so_filter->states.u0 = input;
    so_filter->states.y2 = so_filter->states.y1;
    so_filter->states.y1 = so_filter->states.y0;
    so_filter->states.y0 =  so_filter->states.u0 * so_filter->coeffs.b0 +
                            so_filter->states.u1 * so_filter->coeffs.b1 +
                            so_filter->states.u2 * so_filter->coeffs.b2 -
                            so_filter->states.y2 * so_filter->coeffs.a2 -
                            so_filter->states.y1 * so_filter->coeffs.a1;
}

/**
 * @brief Recalcula os coeficientes dos filtros lowpass
 *
 * @param so_filter     Ponteiro para a estrutura do so_filter
 * @param fs            Frequencia de atualizacao do so_filter em Hz
 * @param Q             Fator de qualidade do filtro
 * @param fc            Frequencia de corte em Hertz
 * @param Type          Tipo do filtro: 0 lowpass, 1 bandpass, 2 notch e 3 highpass
 */
void so_filter_initialize(so_filter_t * so_filter, float fs, float fc, float Q, Uint32 type, Uint32 is_prewarped)
{
    //!< Atualiza os parametros usado no filtro
    so_filter->type = type;
    so_filter->Q  = Q;
    so_filter->fs = fs;
    so_filter->ts = 1.0 / fs;
    so_filter->fc = fc;
    so_filter->is_prewarped = is_prewarped;
    if (is_prewarped){
        so_filter_prewarp_frequency(so_filter, fc);
    }else{
        so_filter->wc = TWO_PI * fs;
    }


    //!< Atualiza Filtro baseado no tipo
    switch (type)
    {
       case SO_FILTER_LOWPASS:  so_filter_update_coeff_lowpass(so_filter);
       break;
       case SO_FILTER_BANDPASS: so_filter_update_coeff_bandpass(so_filter);
       break;
       case SO_FILTER_NOTCH:    so_filter_update_coeff_notch(so_filter);
       break;
       case SO_FILTER_HIGHPASS: so_filter_update_coeff_highpass(so_filter);
       break;
       default:                  so_filter_update_coeff_lowpass(so_filter);
    }
}
