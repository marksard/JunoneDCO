/*!
 * JUNONE DCO
 * Copyright 2023 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */ 

#include <Arduino.h>
#include <hardware/pwm.h>
#include "SmoothAnalogRead.hpp"
#include "note.h"

#define DCO_GAIN 6
#define DCO_CLOCK 7
#define FREQ_TONE 0
#define TUNE_FINE A3
#define COARSE A2
#define VOCT A1
#define FREQMOD A0

#define PWM_RESO 1024
#define TIMER_INTR_TM 10      // us == 100kHz
#define DAC_MAX_MILLVOLT 5000 // mV
#define ADC_RESO 4096
#define MAX_COARSE_FREQ 550
#define MAX_FREQ 5000
#define UINT32_MAX_P1 4294967296

static SmoothAnalogRead vOct(VOCT);
static SmoothAnalogRead potCoarse(COARSE);
static SmoothAnalogRead potAux(TUNE_FINE);
static SmoothAnalogRead potFM(FREQMOD);

static repeating_timer timer;
const static float rateRatio = (float)ADC_RESO / (float)MAX_COARSE_FREQ;
const static float fmRatio = (float)ADC_RESO / (float)(MAX_COARSE_FREQ);

const static float intrruptClock = 1000000.0 / (float)TIMER_INTR_TM; // == 1sec / 10us == 1000000us / 10us == 100kHz
const static uint16_t pulseWidth = ADC_RESO / 2;

static uint32_t tuningWordM = 0;
static uint16_t biasLevel = 0;

extern const float noteFreq[];

void initPWM()
{
    gpio_set_function(DCO_GAIN, GPIO_FUNC_PWM);
    uint potSlice = pwm_gpio_to_slice_num(DCO_GAIN);
    // 最速設定（可能な限り高い周波数にしてRCの値をあげることなく平滑な電圧を得たい）
    // clockdiv = 125MHz / (PWM_RESO * 欲しいfreq)
    // 欲しいfreq = 125MHz / (PWM_RESO * clockdiv)
    pwm_set_clkdiv(potSlice, 1);
    pwm_set_wrap(potSlice, PWM_RESO - 1);
    pwm_set_enabled(potSlice, true);
}

bool intrTimer(struct repeating_timer *t)
{
    static uint32_t phaseAccum = 0;
    phaseAccum = phaseAccum + tuningWordM;
    int index = phaseAccum >> 20;
    byte reset = index < pulseWidth ? HIGH : LOW;
    gpio_put(DCO_CLOCK, reset);
    pwm_set_gpio_level(DCO_GAIN, biasLevel);
    return true;
}

void setup()
{
    analogReadResolution(12);
    pinMode(DCO_CLOCK, OUTPUT);
    pinMode(FREQ_TONE, INPUT_PULLUP);
    // 第一引数は負数でコールバック開始-開始間
    add_repeating_timer_us(-1 * TIMER_INTR_TM, intrTimer, NULL, &timer);
    initPWM();
}

void loop()
{
    static uint8_t dispCount = 0;
    static int16_t voctTune = 0;
    // static int16_t resetTune = 0;
    static int16_t vFine = 0;
    uint16_t voct = vOct.analogReadDirect();
    uint16_t coarse = (float)potCoarse.analogRead(false) / rateRatio;
    uint16_t fm = (float)potFM.analogRead(false) / fmRatio;
    if (digitalRead(FREQ_TONE) == LOW)
    {
        voctTune = (potAux.analogRead() / ((float)ADC_RESO / 400.0)) - (400 >> 1);
    }
    else
    {
        float fineWidth = 0;
        for (int i = 127; i >= 0; --i)
        {
            if (noteFreq[i] <= (float)coarse)
            {
                // coarse = noteFreq[i];
                fineWidth = noteFreq[i + 2] - noteFreq[i];
                break;
            }
        }
        
        vFine = (potAux.analogRead() / ((float)ADC_RESO / fineWidth)) - (fineWidth / 2);
        coarse += vFine;
        // resetTune = (potAux.analogRead() / ((float)ADC_RESO / 50.0));
    }


    // 0to5VのV/OCTの想定でmap変換。RP2040では抵抗分圧で5V->3.3Vにしておく
    uint16_t freqency = coarse *
                            (float)pow(2, map(voct, 0, ADC_RESO - voctTune, 0, DAC_MAX_MILLVOLT) * 0.001) +
                        fm;

    tuningWordM = UINT32_MAX_P1 * freqency / intrruptClock;
    biasLevel = map((uint16_t)freqency, 0, MAX_FREQ, 5, PWM_RESO); // 下限は波形みながら調整

    dispCount++;
    if (dispCount == 0)
    {
        Serial.print(voct);
        Serial.print(", ");
        Serial.print(coarse);
        Serial.print(", ");
        Serial.print(fm);
        Serial.print(", ");
        Serial.print(voctTune);
        Serial.print(", ");
        // Serial.print(resetTune);
        // Serial.print(", ");
        Serial.print(vFine);
        Serial.print(", ");
        Serial.print(freqency);
        Serial.print(", ");
        Serial.print(biasLevel);
        Serial.println();
    }

    sleep_us(50); // 20kHz
}
