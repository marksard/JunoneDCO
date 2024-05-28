/*!
 * JUNONE DCO
 * Copyright 2023 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */ 

#include <Arduino.h>
#include <hardware/pwm.h>
#include "SmoothAnalogRead.hpp"
#include "Oscillator.hpp"
#include "note.h"
#include "EepromData.h"

#define LED_UPPER D4
#define LED_LOWER D2
#define DCO_BIAS D6
#define DCO_CLOCK D7
#define FREQ_TONE D0
#define TUNE_FINE A3
#define COARSE A2
#define VOCT A1
#define FREQMOD A0
#define PWM_INTR_PIN D25 // D0/D1ピンとPWMチャンネルがかぶらないように

// #define CPU_CLOCK 125000000.0
#define CPU_CLOCK 133000000.0 // 標準めいっぱい

#define INTR_PWM_RESO 1024
#define PWM_RESO 4096
#define PWM_RESO_BIT 12
#define DAC_MAX_MILLVOLT 5000 // mV
#define ADC_RESO 4096
#define MAX_COARSE_FREQ 550
#define MAX_FREQ 5000

#define UINT32_MAX_P1 4294967296
#define OSC_WAVE_BIT 32

// pwm_set_clkdivの演算で結果的に1
// #define SAMPLE_FREQ (CPU_CLOCK / INTR_PWM_RESO)
#define SAMPLE_FREQ 120000 //一応この辺までいける

static SmoothAnalogRead vOct(VOCT);
static SmoothAnalogRead potCoarse(COARSE);
static SmoothAnalogRead potAux(TUNE_FINE);
static SmoothAnalogRead potFM(FREQMOD);
static Oscillator osc;

// const static float rateRatio = (float)ADC_RESO / (float)MAX_COARSE_FREQ;
const static float fmRatio = (float)ADC_RESO / (float)MAX_FREQ;
#define EXP_CURVE(value, ratio) (exp((value * (ratio / (ADC_RESO-1)))) - 1) / (exp(ratio) - 1)

static uint16_t biasLevel = 0;

extern const float noteFreq[];
static uint interruptSliceNum;

static uint8_t configMode = 0;
static UserConfig userConfig;

void interruptPWM()
{
    pwm_clear_irq(interruptSliceNum);
    // digitalWrite(GATE_A, HIGH);

    bool pulse = osc.getWaveValue() > 0 ? HIGH : LOW;
    gpio_put(DCO_CLOCK, pulse);
    pwm_set_gpio_level(DCO_BIAS, biasLevel);

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

    // 割り込み頻度
    pwm_set_wrap(slice, INTR_PWM_RESO - 1);
    pwm_set_enabled(slice, true);
    // clockdiv = 125MHz / (INTR_PWM_RESO * 欲しいfreq)
    pwm_set_clkdiv(slice, CPU_CLOCK / (INTR_PWM_RESO * SAMPLE_FREQ));
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

void setup()
{
    analogReadResolution(12);

    pinMode(DCO_CLOCK, OUTPUT);
    pinMode(FREQ_TONE, INPUT_PULLUP);

    osc.init(SAMPLE_FREQ);

    initEEPROM();
    loadUserConfig(&userConfig);

    // coarse0でスイッチOFFでvoctチューニングモード
    // float coarse = (float)potCoarse.analogReadDropLow4bit() / rateRatio;
    // float coarse = EXP_CURVE((float)potCoarse.analogReadDropLow4bit(), 2.0) * MAX_COARSE_FREQ;
    // if (coarse == 0 && digitalRead(FREQ_TONE) == LOW)
    // {
    //     configMode = 1;
    // }

    initPWM(DCO_BIAS);
    initPWM(LED_UPPER);
    initPWM(LED_LOWER);
    initPWMIntr(PWM_INTR_PIN);
}

void loop()
{
    uint16_t voct = vOct.analogReadDirect();
    // float coarse = (float)potCoarse.analogReadDropLow4bit() / rateRatio;
    // Aカーブポットに模擬
    float coarse = EXP_CURVE((float)potCoarse.analogRead(false), 2.0) * MAX_COARSE_FREQ;
    coarse = max(coarse, 10);
    uint16_t fineIn = potAux.analogRead(false);
    uint16_t syncFmIn = potFM.analogReadDirect();

    if (coarse <= 10 && fineIn <= 5 && digitalRead(FREQ_TONE) == LOW)
    {
        configMode = 1;
    }

    float vFine = 0;
    float fm = 0;

    if (configMode == 1)
    {
        // V/OCTのHIGH側微調整
        if (digitalRead(FREQ_TONE) == LOW)
        {
            userConfig.voctTune = (fineIn / ((float)ADC_RESO / 400.0)) - (400 >> 1);
        }
        else
        {
            saveUserConfig(&userConfig);
            configMode = 0;
        }

        pwm_set_gpio_level(LED_UPPER, fineIn);
        pwm_set_gpio_level(LED_LOWER, 4095);
    }
    else
    {
        pwm_set_gpio_level(LED_UPPER, osc.getValue());
        pwm_set_gpio_level(LED_LOWER, biasLevel);

        float fineWidth = 0;
        for (int i = 127; i >= 0; --i)
        {
            // 半音間の周波数からfineの最大値を設定
            if (noteFreq[i] <= coarse)
            {
                int p1 = min(i + 1, 127);
                int m1 = max(i - 1, 0);
                fineWidth = noteFreq[p1] - noteFreq[m1];
                // fineWidth = noteFreq[i + 2] - noteFreq[i];
                break;
            }
        }
        
        vFine = (fineIn / ((float)ADC_RESO / fineWidth)) - (fineWidth / 2);

        if (digitalRead(FREQ_TONE) == LOW)
        {
            // Hard Sync
            static uint8_t lastTrig = 0;
            uint8_t trig = syncFmIn > 2047 ? 1 : 0;
            if (trig == 0 && lastTrig == 1)
            {
                osc.reset();
            }
            lastTrig = trig;
        }
        else
        {
            // -32はノイズの影響を抑えるため
            fm = max((float)(syncFmIn - 32) / fmRatio, 0);
        }
    }


    // 0to5VのV/OCTの想定でmap変換。RP2040では抵抗分圧で5V->3.3Vにしておく
    float freqency = (coarse + vFine) *
                            (float)pow(2, map(voct, 0, ADC_RESO - userConfig.voctTune, 0, DAC_MAX_MILLVOLT) * 0.001) + fm;
    osc.setFrequency(freqency);
    biasLevel = map((uint16_t)freqency, 0, MAX_FREQ, 30, PWM_RESO); // 下限は波形みながら調整

    static uint8_t dispCount = 0;
    dispCount++;
    if (dispCount == 0)
    {
        Serial.print(configMode);
        Serial.print(", ");
        Serial.print(coarse);
        Serial.print(", ");
        Serial.print(vFine);
        Serial.print(", ");
        Serial.print(userConfig.voctTune);
        Serial.print(", ");
        Serial.print(voct);
        Serial.print(", ");
        Serial.print(freqency);
        Serial.print(", ");
        Serial.print(biasLevel);
        Serial.print(", ");
        Serial.print(fm);
        Serial.println();
    }

    sleep_us(50); // 20kHz
}
