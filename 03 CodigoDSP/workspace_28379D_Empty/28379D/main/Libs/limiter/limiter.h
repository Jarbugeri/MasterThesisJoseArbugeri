/*
 * limiter.h
 *
 *  Created on: 31 de jan de 2023
 *      Author: Jose
 */

#ifndef LIMITER_H_
#define LIMITER_H_

/**************
 *  Includes  *
 **************/

/*************
 *  Defines  *
 *************/

#define limiter_saturation(in, LIMITE_MIN, LIMITE_MAX)  \
                if (in >= LIMITE_MAX)  in = LIMITE_MAX; \
                if (in <= LIMITE_MIN)  in = LIMITE_MIN; \


/*************
 *  Structs  *
 *************/

typedef struct _limiter_t_
{
    //!< Parametros de configuracao do pi
    float fs;       //!< Frequencia de amostragem do sogi pll
    float ts;       //!< Periodo de amostragem do sogi pll, calculado na inicializacao

    //!< Saida do pi
    float output;   //!< Saida da estrutura do PI

}limiter_t;

/************************
 * Functions prototypes *
 ************************/

#if !defined(__TMS320C28XX_CLA__)   //!< CPU Compiler

#elif defined(__TMS320C28XX_CLA__)  //!< CLA Compiler

#endif


#endif /* LIMITER_H_ */
