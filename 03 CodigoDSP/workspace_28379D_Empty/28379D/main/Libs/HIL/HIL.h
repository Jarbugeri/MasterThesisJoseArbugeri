/*
 * HIL.h
 *
 *  Created on: 07 de dez de 2022
 *      Author: Jose
 */

#ifndef LIBS_HIL_HIL_H_
#define LIBS_HIL_HIL_H_

/**
 * @defgroup HIL HIL
 * @ingroup Libs
 *
 * @brief Description hear
 *
 *
 * @note Note hear
 *
 * @{
 */

/************
 * Includes *
 ************/

#include "math.h"

/************
 * Defines  *
 ************/



/**
 * @brief Função de inicialização
 * @warning Use para inicializar os paramentros da bibliteca
 */
#define HIL_INIT(Fs ,L ,C ,R)                                   \
        {Fs,                                                    \
         1.0/Fs, 1.0/(L * Fs), 1.0/(C * Fs), 1.0/(R * C * Fs),\
         0, 0,                                                  \
         L, C, R,                                               \
         0, 0}

/*************
 *  Struct   *
 *************/

/**
 * @brief Estrutura da biblioteca
 */
typedef struct _HIL_t_
{
    float Fs;       //!< Frequencia da integração

    float dt;       //!< Base de tempo
    float dt_L;     //!< Base de tempo normalizado por L
    float dt_C;     //!< Base de tempo normalizado por C
    float dt_RC;    //!< Base de tempo normalizado por RC

    float u;            //!< Estado de entrada do modelo
    int   s;            //!< Estado de entrada para modelo comutado
    float d;            //!< Estado de entrada para modelo medio
    float d_complement; //!< Estado complementar de entrada para modelo medio (1-d)

    float L;        //!< Valor de indutância do modelo
    float C;        //!< Valor de capacitância do modelo
    float R;        //!< Valor de resistencia do modelo

    float dx1;      //!< Estado 1 do modelo (Corrente)
    float dx2;      //!< Estado 2 do modelo (Tensão)
}hil_t;

/*************
 * Functions *
 *************/

void hil_update(hil_t * hil ,float Fs, float L, float C, float R);

void hil_average_model_run(hil_t * hil);
void hil_switched_model_run(hil_t * hil);

/**
 * @}
 */

#endif /* LIBS_HIL_HIL_H_ */
