/*
 * sogi_pll.h
 *
 *  Created on: 18 de jan de 2023
 *      Author: Jose
 */

#ifndef EMA_H_
#define EMA_H_

/**************
 *  Includes  *
 **************/

#include "F28x_Project.h"

/*************
 *  Defines  *
 *************/

#define INIT_EMA(N)        {N, 2.0/(N+1.0), 0, 0, 0}

/*************
 *  Structs  *
 *************/

typedef struct _ema_t_
{
    //!< Parametros de configuracao do SOGI
    Uint16 N;
    //!< Coeficientes e estados dos filtro sogi
    float alpha;
    float u0;
    float y0;
    float y1;

}ema_t;

/************************
 * Functions prototypes *
 ************************/

#if !defined(__TMS320C28XX_CLA__)   //!< CPU Compiler
    void ema_update(ema_t * ema, Uint16 N);
    void ema_run(ema_t * ema, float input);
#elif defined(__TMS320C28XX_CLA__)  //!< CLA Compiler
    void ema_run_cla(ema_t * ema, float input);
#endif


#endif /* EMA_H_ */
