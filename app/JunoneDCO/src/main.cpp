/*!
 * JUNONE DCO
 * Copyright 2023,2026 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */ 

#include <Arduino.h>
#include <hardware/pwm.h>
#include "SmoothAnalogRead.hpp"
#include "ADCErrorCorrection.hpp"
#include "EepRomConfigIO.hpp"
#include "pwm_wrapper.h"
#include "MiniOsc.hpp"

// gpio_mapping
#define PWM_INTR_PIN D25 // PMW4 B
#define LED_UPPER D4
#define LED_LOWER D2
#define DCO_BIAS D6
#define DCO_CLOCK D7
#define SYNC_FM_SW D0
#define TUNE_FINE A3
#define COARSE A2
#define VOCT A1
#define FREQMOD A0

// basic_difinition
#define INTR_PWM_RESO 512
#define PWM_BIT 11
#define PWM_RESO 2048
#define ADC_BIT 12
#define ADC_RESO 4096
// #define DAC_MAX_MILLVOLT 5000 // mV
#define DAC_BIT 12
#define DAC_RESO 4096
// #define SAMPLE_FREQ (((float)(CPU_CLOCK) / (float)INTR_PWM_RESO) / (float)SAMPLE_FREQ_DIVIDER)
#define SAMPLE_FREQ 192000 //一応この辺までいける

// DCO
#define MAX_COARSE_FREQ 550
#define MAX_FREQ 5000

// 基本設定
struct SystemConfig
{
    char ver[15] = "JunoneDCO_001\0";
    float vRef;
    float noiseFloor;

    SystemConfig()
    {
        vRef = 3.30f;
        noiseFloor = 32.0f;
    }
};
static EEPROMConfigIO<SystemConfig> systemConfig(0);

static uint interruptSliceNum;
const static float rateRatio = (float)ADC_RESO / (float)MAX_COARSE_FREQ;
const static float fmRatio = (float)ADC_RESO / (float)MAX_FREQ;
// #define EXP_CURVE(value, ratio) (exp((value * (ratio / (ADC_RESO-1)))) - 1) / (exp(ratio) - 1)

static SmoothAnalogRead vOct;
static SmoothAnalogRead potCoarse;
static SmoothAnalogRead potFine;
static SmoothAnalogRead potFM;
static MiniOsc clockGen;
static ADCErrorCorrection adcErrorCorrection;
static float voctFine = 0;
static float voctCoarse = 0;
static uint16_t biasLevel = 0;

//////////////////////////////////////////

void interruptPWM()
{
    pwm_clear_irq(interruptSliceNum);

    gpio_put(DCO_CLOCK, clockGen.getWaveValue() > 0 ? true : false);
}

void setup()
{
    // Serial.begin(9600);
    // while (!Serial)
    // {
    // }
    // delay(500);

    set_sys_clock_hz(CPU_CLOCK, true);
    pinMode(23, OUTPUT);
    gpio_put(23, HIGH);

    pinMode(DCO_CLOCK, OUTPUT);
    pinMode(SYNC_FM_SW, INPUT_PULLUP);

    vOct.init(VOCT);
    potCoarse.init(COARSE);
    potFine.init(TUNE_FINE);
    potFM.init(FREQMOD);

    clockGen.init(SAMPLE_FREQ, PWM_BIT, false);
    clockGen.setWave(MiniOsc::Wave::SQU);

    systemConfig.initEEPROM();
    systemConfig.loadUserConfig();
    adcErrorCorrection.init(systemConfig.Config.vRef, systemConfig.Config.noiseFloor);
    adcErrorCorrection.getADCAvg16(VOCT);

    initPWM(DCO_BIAS, PWM_RESO);
    initPWM(LED_UPPER, PWM_RESO);
    initPWM(LED_LOWER, PWM_RESO);
    initPWMIntr(PWM_INTR_PIN, interruptPWM, &interruptSliceNum, SAMPLE_FREQ, INTR_PWM_RESO, CPU_CLOCK);
}

void loop()
{
    uint16_t voctValue = vOct.analogReadDirectFast();
    uint16_t coarseValue = potCoarse.analogRead(false);
    uint16_t fineIn = potFine.analogRead(false);
    uint16_t syncFmIn = potFM.analogReadDirectFast();
      
    float fm = 0;
    if (digitalRead(SYNC_FM_SW) == LOW)
    {
        // Hard Sync
        static uint8_t lastTrig = 0;
        uint8_t trig = syncFmIn > 2047 ? 1 : 0;
        if (trig == 0 && lastTrig == 1)
        {
            clockGen.reset();
        }
        lastTrig = trig;
    }
    else
    {
        // -32はノイズの影響を抑えるため
        fm = max((float)(syncFmIn - 32) / fmRatio, 0);
    }

    float voctPowV = adcErrorCorrection.voctPow(voctValue);
    float vOctFreq = (voctCoarse + voctFine) * voctPowV + fm;
    clockGen.setFrequency(vOctFreq);
    biasLevel = map((uint16_t)vOctFreq, 0, MAX_FREQ, 30, PWM_RESO); // 下限は波形みながら調整
    
    pwm_set_gpio_level(LED_UPPER, clockGen.getValue());
    pwm_set_gpio_level(LED_LOWER, biasLevel);
    pwm_set_gpio_level(DCO_BIAS, biasLevel);

    // static uint8_t dispCount = 0;
    // dispCount++;
    // if (dispCount == 0)
    // {
    //     Serial.print(voctValue);
    //     Serial.print(", ");
    //     Serial.print(voctCoarse);
    //     Serial.print(", ");
    //     Serial.print(voctFine);
    //     Serial.print(", ");
    //     Serial.print(vOctFreq);
    //     Serial.print(", ");
    //     Serial.print(biasLevel);
    //     Serial.print(", ");
    //     Serial.print(fm);
    //     Serial.println();
    // }

    tight_loop_contents();
}

void setup1()
{
}

void loop1()
{
    uint16_t voctValue = vOct.getValue();
    uint16_t coarseValue = potCoarse.getValue();
    uint16_t fineIn = potFine.getValue();
    uint16_t syncFmIn = potFM.getValue();

    voctCoarse = (float)coarseValue / rateRatio;
    // Aカーブポットに模擬
    // voctCoarse = EXP_CURVE((float)coarseValue, 2.0) * MAX_COARSE_FREQ;
    voctCoarse = max(voctCoarse, 10);

    float fineWidth = 0;
    for (int i = 127; i >= 0; --i)
    {
        // 半音間の周波数からfineの最大値を設定
        if (clockGen.getFreqFromNoteIndex(i) <= voctCoarse)
        {
            int p1 = min(i + 1, 127);
            int m1 = max(i - 1, 0);
            fineWidth = clockGen.getFreqFromNoteIndex(p1) - clockGen.getFreqFromNoteIndex(m1);
            break;
        }
    }
    
    voctFine = (fineIn / ((float)ADC_RESO / fineWidth)) - (fineWidth / 2);

    tight_loop_contents();
    sleep_ms(10);
}
