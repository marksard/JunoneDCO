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
#include "EepromData.h"

#define DCO_GAIN 6
#define DCO_CLOCK 7
#define FREQ_TONE 0
#define TUNE_FINE A3
#define COARSE A2
#define VOCT A1
#define FREQMOD A0
#define PWM_INTR_PIN D25 // D0/D1ピンとPWMチャンネルがかぶらないように

#define SAMPLE_FREQ 120000 //一応この辺までいける
#define PWM_RESO 4096
#define PWM_RESO_BIT 12

#define DAC_MAX_MILLVOLT 5000 // mV
#define ADC_RESO 4096
#define MAX_COARSE_FREQ 550
#define MAX_FREQ 5000
#define UINT32_MAX_P1 4294967296

#define OSC_WAVE_BIT 32

static SmoothAnalogRead vOct(VOCT);
static SmoothAnalogRead potCoarse(COARSE);
static SmoothAnalogRead potAux(TUNE_FINE);
static SmoothAnalogRead potFM(FREQMOD);

static repeating_timer timer;
const static float rateRatio = (float)ADC_RESO / (float)MAX_COARSE_FREQ;
const static float fmRatio = (float)ADC_RESO / (float)(MAX_COARSE_FREQ);

const static float intrruptClock = SAMPLE_FREQ;
const static uint16_t pulseWidth = ADC_RESO / 2;
const uint32_t indexBit = OSC_WAVE_BIT - PWM_RESO_BIT;

static uint32_t tuningWordM = 0;
static uint16_t biasLevel = 0;

extern const float noteFreq[];
static uint interruptSliceNum;

void interruptPWM()
{
    pwm_clear_irq(interruptSliceNum);
    // digitalWrite(GATE_A, HIGH);

    static uint32_t phaseAccum = 0;
    phaseAccum = phaseAccum + tuningWordM;
    int index = phaseAccum >> indexBit;
    byte reset = index < pulseWidth ? HIGH : LOW;
    gpio_put(DCO_CLOCK, reset);
    pwm_set_gpio_level(DCO_GAIN, biasLevel);

    // digitalWrite(GATE_A, LOW);
}

// OUT_A/Bとは違うPWMチャンネルのPWM割り込みにすること
void initPWMIntr(uint gpio)
{
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);

    interruptSliceNum = slice;
    pwm_clear_irq(slice);
    pwm_set_irq_enabled(slice, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, interruptPWM);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // 割り込み回数をRESOの1/4にして、サンプルレートを上げられるように
    uint32_t reso = (PWM_RESO >> 2);
    pwm_set_wrap(slice, reso - 1);
    pwm_set_enabled(slice, true);
    // clockdiv = 125MHz / (PWM_RESO * 欲しいfreq)
    pwm_set_clkdiv(slice, 125000000.0 / (reso * SAMPLE_FREQ));
}

void initPWM(uint gpio)
{
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);
    pwm_set_wrap(slice, PWM_RESO - 1);
    pwm_set_enabled(slice, true);
    // 最速にして滑らかなPWMを得る
    pwm_set_clkdiv(slice, 1);
}

static uint8_t configMode = 0;
static UserConfig userConfig;
void setup()
{
    analogReadResolution(12);
    pinMode(DCO_CLOCK, OUTPUT);
    pinMode(FREQ_TONE, INPUT_PULLUP);
    initPWMIntr(PWM_INTR_PIN);
    initPWM(DCO_GAIN);

    initEEPROM();
    loadUserConfig(&userConfig);
    uint16_t coarse = (float)potCoarse.analogRead(false) / rateRatio;
    if (coarse == 0 && digitalRead(FREQ_TONE) == LOW)
    {
        configMode = 1;
    }
}

void loop()
{
    static uint8_t dispCount = 0;
    // static int16_t resetTune = 0;
    static int16_t vFine = 0;
    uint16_t voct = vOct.analogReadDirect();
    uint16_t coarse = (float)potCoarse.analogRead(false) / rateRatio;
    uint16_t fm = (float)potFM.analogRead(false) / fmRatio;
    // V/OCTのHIGH側微調整
    if (digitalRead(FREQ_TONE) == LOW)
    {
        if (configMode == 1)
        {
            userConfig.voctTune = (potAux.analogRead() / ((float)ADC_RESO / 400.0)) - (400 >> 1);
        }
    }
    else
    {
        if (configMode == 1)
        {
            saveUserConfig(&userConfig);
            configMode = 0;
        }
        
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
                            (float)pow(2, map(voct, 0, ADC_RESO - userConfig.voctTune, 0, DAC_MAX_MILLVOLT) * 0.001) +
                        fm;

    tuningWordM = UINT32_MAX_P1 * freqency / intrruptClock;
    biasLevel = map((uint16_t)freqency, 0, MAX_FREQ, 20, PWM_RESO); // 下限は波形みながら調整

    dispCount++;
    if (dispCount == 0)
    {
        Serial.print(voct);
        Serial.print(", ");
        Serial.print(coarse);
        Serial.print(", ");
        Serial.print(fm);
        Serial.print(", ");
        Serial.print(userConfig.voctTune);
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
