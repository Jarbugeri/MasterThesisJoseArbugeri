/*
 * WaveGenerator.h
 *
 *  Created on: 26 de nov de 2022
 *      Author: Jose
 */

#ifndef LIBS_WAVEGENERATOR_WAVEGENERATOR_H_
#define LIBS_WAVEGENERATOR_WAVEGENERATOR_H_

/**
 * @defgroup WaveGenerator WaveGenerator
 * @ingroup Libs
 *
 * @brief Description hear
 *
 *
 * @note Note hear
 *
 * @{
 */

/**************
 *  Includes  *
 **************/

#include "F28x_Project.h"
#if !defined(__TMS320C28XX_CLA__)   //!< CPU Compiler
#include "math.h"
#elif defined(__TMS320C28XX_CLA__)  //!< CLA Compiler
#include "CLAmath.h"
#endif

/************
 * Defines  *
 ************/

#define TWOPI           6.28318530717958647692528676655900
#define ONEPI           3.14159265358979323846264338327950
#define HALFPI          1.57079632679489661923132169163980
#define INV_TWOPI       0.15915494309189533576888376337251
#define INV_ONEPI       0.31830988618379067153776752674503
#define INV_HALFPI      0.63661977236758134307553505349006

#define THETA_A         -0.0 * TWOPI / 3.0
#define THETA_B         -2.0 * TWOPI / 3.0
#define THETA_C         -4.0 * TWOPI / 3.0

/**
 * @brief Tipo de onda escolhido
 * @warning Usado apenas na função generica wave
 */
#define SINE_1F         0
#define SINE_3F         1
#define SAWTOOTH        2
#define TRIANGLE        3
#define SQUARE          4

/**
 * @brief Função de inicialização
 * @warning Use para inicializar os paramentros da bibliteca
 */
#define WAVE_GENERATOR_INIT(Fs, Fg, A, TYPE) {Fs, 1.0 / Fs, TWOPI * Fg / Fs, 0.0, Fg, A, 0, TYPE, 0.0, 0.0, 0.0}

/*************
 *  Struct   *
 *************/

/**
 * @brief Estrutura da biblioteca
 */
typedef struct _WaveGenerator_t_
{
    float Fs;       //!< Frequencia da integração do gerador
    float Ts;       //!< Periodo da integração do gerador
    float Delta;    //!< Passo de integração
    float t ;       //!< Tempo
    float Fg;       //!< Frequencia fundamental da onda gerada
    float A;        //!< Amplitude da geração
    Uint16 updown;  //!< Sentido da cotagem
    Uint16 type;    //!< Seleciona o tipo de onda;
    float a;        //!< Saida do gerador A
    float b;        //!< Saida do gerador B
    float c;        //!< Saida do gerador C
}WaveGenerator_t;

/************************
 * Functions prototypes *
 ************************/

#if !defined(__TMS320C28XX_CLA__)   //!< CPU Compiler
    void WaveGenerator_update(WaveGenerator_t * SineWave, float Fs, float Fg, float A);

    void WaveGenerator_sine_single_run(WaveGenerator_t * SineWave);
    void WaveGenerator_sine_run(WaveGenerator_t * SineWave);
    void WaveGenerator_sawtooth_run(WaveGenerator_t * Sawtooth);
    void WaveGenerator_triangle_run(WaveGenerator_t * Triangle);
    void WaveGenerator_square_run(WaveGenerator_t * Square);

    void WaveGenerator_generic_run(WaveGenerator_t * wave);
#elif defined(__TMS320C28XX_CLA__)  //!< CLA Compiler
    void WaveGenerator_sine_single_run_cla(WaveGenerator_t * SineWave);
#endif


/**
 * @}
 */

#endif /* LIBS_WAVEGENERATOR_WAVEGENERATOR_H_ */
