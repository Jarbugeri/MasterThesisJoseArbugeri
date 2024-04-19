/*
 * cpu_data_shared.h
 *
 *  Created on: 26 de jan de 2023
 *      Author: Jose
 */

#ifndef CPU_DATA_SHARED_H_
#define CPU_DATA_SHARED_H_

/*************
 *  Defines  *
 *************/

#include "F28x_Project.h"

/*************
 *  Structs  *
 *************/

/**
 * @brief Estrutura de dados do CPU1
 *
 */
typedef struct _data_cpu1_t_
{
    float   voltage_input;      //!< Tensao de entrada
    float   current_input;      //!< Corrente de entrada
    float   voltage_output;     //!< Tensao de saida
    float   Indutor;            //!< Valor de indutância do modelo
    float   Capacitor;          //!< Valor de capacitância do modelo
    float   Carga;              //!< Valor de resistencia do modelo
    float   Freq_hil;           //!< Frequencia de atualizacao do modelo
    int     sp;                 //!< Sinal pwm P
    int     sn;                 //!< Sinal pwm N
    float   duty_p_pu;          //!< Sinal duty cycle em pu chave P
    float   duty_n_pu;          //!< Sinal duty cycle em pu chave N
}data_cpu1_t;

/**
 * @brief Estrutura de dados do CPU2
 *
 */
typedef struct _data_cpu2_t_
{
    float   voltage_input;      //!< Tensao de entrada
    float   current_input;      //!< Corrente de entrada
    float   voltage_output;     //!< Tensao de saida
}data_cpu2_t;

#endif /* CPU_DATA_SHARED_H_ */
