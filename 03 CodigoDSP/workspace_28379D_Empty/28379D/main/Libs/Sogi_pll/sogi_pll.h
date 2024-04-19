/*
 * sogi_pll.h
 *
 *  Created on: 23 de abr de 2022
 *      Author: Jose
 */

#ifndef SOGI_PLL_H_
#define SOGI_PLL_H_

/**************
 *  Includes  *
 **************/

#include "F28x_Project.h"

/*************
 *  Defines  *
 *************/

#define TWO_PI          6.28318530717958647692528676655900
#define RAD2HZ          0.15915494309189533576888376337251

/*************
 *  Structs  *
 *************/

typedef struct _sogi_so_filter_t_
{
    //!< Estados de entrada e saida 
    float u0;   //!< Estado entrada u[n]
    float u1;   //!< Estado entrada u[n-1]
    float u2;   //!< Estado entrada u[n-2]
    float y0;   //!< Estado saida y[n]
    float y1;   //!< Estado saida y[n-1]
    float y2;   //!< Estado saida y[n-2]
    //!< Coeficientes do filtro
//  float a0;   //!< Coeff a0: deve ser 1 para esse metodo
    float a1;
    float a2;
    float b0;
    float b1;
    float b2;
}sogi_so_filter_t;

typedef struct _sogi_integrador_t_
{
    //!< Estados de entrada e saida 
    float u0;   //!< Estado entrada u[n]
    float u1;   //!< Estado entrada u[n-1]
    float y0;   //!< Estado saida y[n]
    float y1;   //!< Estado saida y[n-1]
    //!< Integrator gain
    float Ki;   //!< Ganho Ki do integrador
}sogi_integrador_t;

typedef struct _sogi_pi_t_
{
    //!< Estados de entrada e saida do integrador
    float u0;   //!< Estado entrada u[n]
    float u1;   //!< Estado entrada u[n-1]
    float y0;   //!< Estado saida y[n]
    float y1;   //!< Estado saida y[n-1]
    //!< Ganho integrator e proporcional
    float Ki;   //!< Ganho integral Ki do PI
    float Kp;   //!< Ganho proporcional Kp do PI
    //!< Saida do pi
    float output;   //!< Saida da estrutura do PI
}sogi_pi_t;

typedef struct _sogi_pll_t_
{
    //!< Parametros de configuracao do SOGI
    float k;            //!< Ganho k dos filtros sogi
    float fs;           //!< Frequencia de amostragem do sogi pll
    float ts;           //!< Periodo de amostragem do sogi pll, calculado na inicializacao
    float fr;           //!< Frequencia da rede em Hz
    float wc;           //!< Frequencia da rede em radianos/s, calculado na inicializacao a partir de fr

    //!< Entrada do pll_sogi
    float input;        //!< Estado de entrado do sogi pll

    //!< Coeficientes e estados dos filtros sogi
    sogi_so_filter_t v_alpha;       //!< Tensao eixo alpha
    sogi_so_filter_t v_beta;        //!< Tensao eixo beta

    //!< Variaveis em dq0
    float d;                        //!< Tensao eixo direto
    float q;                        //!< Tensao eixo quadratura

    //!< PI
    sogi_pi_t pi;                   //!< Estrutura do PI do sogi pll

    float w_ref;                    //!< Referencia de frequencia em radianos

    //!< Integrator
    sogi_integrador_t integrador;   //!< Estrutura do I do sogi pll

    //!< Variaveis de saida do sogi
    Uint32 zero_crossing;
    float wt;                       //!< Angulo da tensao de entrada em radianos por segundo
    float wt_offset;
    float omega;                    //!< frequencia da tensao de entrada em radianos
    float freq;                     //!< frequencia da tensao de entrada em Hertz
    float sine;                     //!< Sinal seno de saida do pll
    float cosine;                   //!< Sinal coseno de saida do pll
}sogi_pll_t;

/************************
 * Functions prototypes *
 ************************/

#if !defined(__TMS320C28XX_CLA__)   //!< CPU Compiler
    void sogi_pll_initialize(sogi_pll_t * sogi, float fs, float k, float fr, float Kp, float Ki);
    void sogi_pll_update_coeff_sogi(sogi_pll_t * sogi);
    void sogi_pll_run(sogi_pll_t * sogi, float input);
#elif defined(__TMS320C28XX_CLA__)  //!< CLA Compiler
    void sogi_pll_update_coeff_sogi_cla(sogi_pll_t * sogi);
    void sogi_pll_run_cla(sogi_pll_t * sogi, float input);
#endif


#endif /* SOGI_PLL_H_ */
