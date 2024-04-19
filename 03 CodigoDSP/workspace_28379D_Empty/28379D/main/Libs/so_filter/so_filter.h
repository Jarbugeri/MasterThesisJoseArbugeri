/*
 * sogi_pll.h
 *
 *  Created on: 18 de jan de 2023
 *      Author: Jose
 */

#ifndef SO_FILTER_H_
#define SO_FILTER_H_

/**************
 *  Includes  *
 **************/

#include "F28x_Project.h"
#if !defined(__TMS320C28XX_CLA__)   //!< CPU Compiler
#include "math.h"
#elif defined(__TMS320C28XX_CLA__)  //!< CLA Compiler
#include "CLAmath.h"
#endif

/*************
 *  Defines  *
 *************/

#define TWO_PI                      6.28318530717958647692528676655900
#define ONE_PI                      3.1415926535897932384626433832795
#define TWO_PI_INV                  0.15915494309189533576888376337251

#define SO_FILTER_LOWPASS           0
#define SO_FILTER_BANDPASS          1
#define SO_FILTER_NOTCH             2
#define SO_FILTER_HIGHPASS          3

#define PRE_WARP_ON                 1
#define PRE_WARP_OFF                0


/*************
 *  Structs  *
 *************/

typedef struct _coeffs_t_
{
    //!< Coeficientes do filtro
    //!<  a0 deve ser 1 para esse metodo 
    float a1;
    float a2;
    float b0;
    float b1;
    float b2;
}coeffs_t;

typedef struct _states_t_
{
    //!< Estados de entrada e saida 
    float u0;   //!< Estado entrada u[n]
    float u1;   //!< Estado entrada u[n-1]
    float u2;   //!< Estado entrada u[n-2]
    float y0;   //!< Estado entrada y[n]
    float y1;   //!< Estado entrada y[n-1]
    float y2;   //!< Estado entrada y[n-2]
}states_t;

typedef struct _so_filter_t_
{
    //!< Parametros de configuracao do SOGI
    Uint32   type;          //!< Tipo do filtro: 0 lowpass, 1 bandpass, 2 notch e 3 highpass
    Uint32   is_prewarped;  //!< 1 Frequencia de corte pre distorcida, 0 não distorcida
    float Q;                //!< Fator de qualidade do filtro
    float fs;               //!< Frequencia de amostragem do filtro
    float fc;               //!< Frequencia de corte em Hertz
    //!< Variaveis auxiliares
    float ts;               //!< Periodo de amostragem do do filtro, calculado na inicializacao
    float wc;               //!< Frequencia de corte em radianos/s, calculado na inicializacao a partir de fr

    //!< Coeficientes e estados dos filtro sogi
    coeffs_t coeffs;        //!< Coeficientes do filtro de segunda ordem
    states_t states;        //!< Estados de entrada e saida do filtro
}so_filter_t;

/************************
 * Functions prototypes *
 ************************/

#if !defined(__TMS320C28XX_CLA__)   //!< CPU Compiler
    /* Funcoes especificas de cada tipo de filtro */
    void so_filter_update_coeff_lowpass(so_filter_t * so_filter);
    void so_filter_update_coeff_bandpass(so_filter_t * so_filter);
    void so_filter_update_coeff_notch(so_filter_t * so_filter);
    void so_filter_update_coeff_highpass(so_filter_t * so_filter);
    /* Funcoes generica para qualquer filtro */
    void so_filter_initialize(so_filter_t * so_filter, float fs, float fc, float Q, Uint32 type, Uint32 is_prewarped);
    void so_filter_prewarp_frequency(so_filter_t * so_filter, float fc);
    void so_filter_reset(so_filter_t * so_filter, float value);
    void so_filter_run(so_filter_t * so_filter, float input);
#elif defined(__TMS320C28XX_CLA__)  //!< CLA Compiler
    /* Funcoes especificas de cada tipo de filtro */
    void so_filter_update_coeff_lowpass_cla(so_filter_t * so_filter);
    void so_filter_update_coeff_bandpass_cla(so_filter_t * so_filter);
    void so_filter_update_coeff_notch_cla(so_filter_t * so_filter);
    void so_filter_update_coeff_highpass_cla(so_filter_t * so_filter);
    /* Funcoes generica para qualquer filtro */
    void so_filter_reset_cla(so_filter_t * so_filter, float value);
    void so_filter_run_cla(so_filter_t * so_filter, float input);

#endif


#endif /* SO_FILTER_H_ */
