# JUNONE DCO

Digitally controlled oscillator (DCO) type monophonic synthesizer for Eurorack.  

This DCO uses the RP2040 MCU to control the oscillation and amplitude of the opamp.  

I found the references in the following link very helpful.  
[The Design of the Roland Juno oscillators](https://blog.thea.codes/the-design-of-the-juno-dco/)

## Specification

### Power

|Use Voltage|Current consumption|
|:--|:--|
|+5V|24mA|
|+12V|13mA|
|-12V|13mA|

### Input

|Name|Description|
|:--|:--|
|V/OCT|MAX 5V|
|CV (with POT)| MAX 5V<br> Vibrato, FM, etc.|
|PWM (with POT)| MAX 10V|

### Output

|Name|Description|
|:--|:--|
|SQR|SQUARE<br> BIPOLER 10.5Vpp|
|SAW|SAW<br> BIPOLER 10Vpp|
|MIX|SQUARE AND SAW MIX<br>BIPOLER 10Vpp|

Assumed operating frequency from 0 Hz to around 5 kHz.


### Controller

|Name|Description|
|:--|:--|
|Freq Pot|Freq 0-550Hz<br>(Depends on the application)|
|Fine Pot|Freq Fine tune<br>(Depends on the application)|
|Key Step/Freq Switch|Frequency pot increases/decreases by key<br>(Depends on the application)|
|PWM|Depth of effect|
|CV|Depth of effect|

## Image

## Schematic

![img](_data/JUNONE%20DCO%20Schematic%20rev1.0.0.png)


## Waveform

## Waveform

Square
![img](_data/junondco_wave_squ.png)  

Saw
![img](_data/junondco_wave_saw.png)  

Saw (low frequency. The rectangular waveform also becomes pulse wave-like at this time.)
![img](_data/junondco_wave_saw_low.png)  

Square + Saw Mix out
![img](_data/junondco_wave_mix.png)  

Mix out PWM
![img](_data/junondco_wave_mix_pwm.png)  