/*
 * WaveGenerator.c
 *
 *  Created on: 26 de nov de 2022
 *      Author: Jose
 */

/**
 * @defgroup WaveGenerator WaveGenerator
 * @{
 */

/************
 * Includes *
 ************/

#include "WaveGenerator.h"

/*************
 * Functions *
 *************/

/**
 * @brief Gerador senoidal trifasico
 *
 * @param SineWave Ponteiro para estrutura WaveGenerator
 *
 * @warning
 */
void WaveGenerator_update(WaveGenerator_t * SineWave, float Fs, float Fg, float A)
{
    //!< Atualiza frequencia da integração do gerador
    SineWave->Fs = Fs;
    //!< Atualiza periodo da integração do gerador
    SineWave->Ts = 1.0 / Fs;
    //!< Atualiza Passo de integração
    SineWave->Delta = TWOPI * Fg * SineWave->Ts;
    //!< Atualiza frequencia fundamental da geração
    SineWave->Fg = Fg;
    //!< Atualiza amplitude do sinal de geração
    SineWave->A = A;

}

/**
 * @brief Gerador senoidal monofasico
 *
 * @param SineWave Ponteiro para estrutura WaveGenerator
 *
 * @warning
 */
void WaveGenerator_sine_single_run(WaveGenerator_t * SineWave)
{
    //!< Atualiza tempo
    SineWave->t = SineWave->t + SineWave->Delta;

    //!< Reseta quando complar um ciclo
    if (SineWave->t >= TWOPI){
        SineWave->t -= TWOPI;
    }

    //!< Atualiza as saidas
    SineWave->a = SineWave->A * sin(SineWave->t + THETA_A);

}


/**
 * @brief Gerador senoidal trifasico
 *
 * @param SineWave Ponteiro para estrutura WaveGenerator
 *
 * @warning
 */
void WaveGenerator_sine_run(WaveGenerator_t * SineWave)
{
    //!< Atualiza tempo
    SineWave->t = SineWave->t + SineWave->Delta;

    //!< Reseta quando complar um ciclo
    if (SineWave->t >= TWOPI){
        SineWave->t -= TWOPI;
    }

    //!< Atualiza as saidas

    SineWave->a = SineWave->A * sin(SineWave->t + THETA_A);
    SineWave->b = SineWave->A * sin(SineWave->t + THETA_B);
    SineWave->c = SineWave->A * sin(SineWave->t + THETA_C);

}

/**
 * @brief Gerador dente de serra trifasico
 *
 * @param Sawtooth Ponteiro para estrutura WaveGenerator
 *
 * @warning
 */
void WaveGenerator_sawtooth_run(WaveGenerator_t * Sawtooth)
{
    //!< Atualiza tempo
    Sawtooth->t = Sawtooth->t + Sawtooth->Delta;

    //!< Reseta quando complar um ciclo
    if (Sawtooth->t >= ONEPI){
        Sawtooth->t = -ONEPI;
    }

    //!< Atualiza as saidas

    Sawtooth->a = Sawtooth->A * INV_ONEPI * (Sawtooth->t + THETA_A);
    Sawtooth->b = Sawtooth->A * INV_ONEPI * (Sawtooth->t + THETA_B);
    Sawtooth->c = Sawtooth->A * INV_ONEPI * (Sawtooth->t + THETA_C);

}

/**
 * @brief Gerador dente de serra trifasico
 *
 * @param Sawtooth Ponteiro para estrutura WaveGenerator
 *
 * @warning
 */
void WaveGenerator_triangle_run(WaveGenerator_t * Triangle)
{

    //!< Atualiza tempo
    if (Triangle->updown == 1){
        Triangle->t = Triangle->t - Triangle->Delta;
    }else{
        Triangle->t = Triangle->t + Triangle->Delta;
    }

    //!< Reseta quando complar um ciclo
    if (Triangle->t >=  HALFPI){
        Triangle->updown = 1;
    }
    if (Triangle->t <= -HALFPI){
        Triangle->updown = 0;
    }

    //!< Atualiza as saidas

    Triangle->a = Triangle->A * INV_HALFPI * (Triangle->t + THETA_A);
    Triangle->b = Triangle->A * INV_HALFPI * (Triangle->t + THETA_B);
    Triangle->c = Triangle->A * INV_HALFPI * (Triangle->t + THETA_C);

}

/**
 * @brief Gerador onda quadrada trifasico
 *
 * @param Square Ponteiro para estrutura WaveGenerator
 *
 * @warning
 */
void WaveGenerator_square_run(WaveGenerator_t * Square)
{

    //!< Atualiza tempo
    Square->t = Square->t + Square->Delta;

    //!< Reseta quando complar um ciclo
    if (Square->t >= TWOPI){
        Square->t -= TWOPI;
    }

    //!< Atualiza as saidas
    //TODO terminar implementacao, adicionar dutycycle?
    Square->a = copysignf(1.0, sin(Square->t + THETA_A));
    Square->b = copysignf(1.0, sin(Square->t + THETA_B));
    Square->c = copysignf(1.0, sin(Square->t + THETA_C));

}

/**
 * @brief Gerador generico trifasico
 *
 * Pode ser trocado em tempo de execução qual tipo de onda é gerada.
 * O tipo de onda é selecionado pela variavel type da estrutura.
 *
 * @param Square Ponteiro para estrutura WaveGenerator
 *
 * @warning
 */
void WaveGenerator_generic_run(WaveGenerator_t * wave)
{
    switch(wave->type){
    default:
    case SINE_1F:
        WaveGenerator_sine_single_run(wave);
        break;
    case SINE_3F:
        WaveGenerator_sine_run(wave);
        break;
    case SAWTOOTH:
        WaveGenerator_sawtooth_run(wave);
        break;
    case TRIANGLE:
        WaveGenerator_triangle_run(wave);
        break;
    case SQUARE:
        WaveGenerator_square_run(wave);
        break;
    }
}



/**
 * @}
 */
