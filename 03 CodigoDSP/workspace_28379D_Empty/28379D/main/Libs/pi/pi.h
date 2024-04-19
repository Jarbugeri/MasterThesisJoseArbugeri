/*
 * pi.h
 *
 *  Created on: 30 de jan de 2023
 *      Author: Jose
 */

#ifndef PI_H_
#define PI_H_

/**************
 *  Includes  *
 **************/

/*************
 *  Defines  *
 *************/

/*************
 *  Structs  *
 *************/

typedef struct _pi_t_
{
    //!< Parametros de configuracao do pi
    float fs;       //!< Frequencia de amostragem do sogi pll
    float ts;       //!< Periodo de amostragem do sogi pll, calculado na inicializacao

    //!< Ganho integrator e proporcional
    float Ki;       //!< Ganho integral Ki do PI
    float Kp;       //!< Ganho proporcional Kp do PI

    //!< Auxiliares
    float Ki_ts;     //!< Ganho integral Ki vezes o periodo de integração

    //!< Estados de entrada e saida do integrador
    float u0;       //!< Estado entrada u[n]
    float u1;       //!< Estado entrada u[n-1]
    float y0;       //!< Estado saida y[n]
    float y1;       //!< Estado saida y[n-1]

    //!< Saida do pi
    float output;   //!< Saida da estrutura do PI

}pi_t;

/************************
 * Functions prototypes *
 ************************/

#if !defined(__TMS320C28XX_CLA__)   //!< CPU Compiler
    void pi_initialize(pi_t * pi, float fs, float Kp, float Ki);
    void pi_reset_output(pi_t * pi, float sub);
    void pi_reset(pi_t * pi);
    float pi_run(pi_t * pi, float input);

#elif defined(__TMS320C28XX_CLA__)  //!< CLA Compiler

#endif


#endif /* v */
