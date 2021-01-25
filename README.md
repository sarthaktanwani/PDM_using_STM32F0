# Pulse Density Modulation using STM32F0 #
===============================================================

This project was aimed at producing a PDM (Pulse Density Modulated) waveform for a 50 Hz Sine wave using the STM32F030 microcontroller.

At first I used the STM32F051 (F0 discovery board) to generate a Sine wave through samples on a pin (using DAC) as this discovery board had an inbuilt DAC peripheral. We do not actually need to produce an analog Sine wave. I only used it to debug.

For this, In STM32CubeMX I enabled DAC and 2 GPIO pins to see the output of the PDM signal. The PDM produces a 3-level output (-1, 0 and +1). To see it in the oscilloscope, I made the program such that if the ouput is -1, one GPIO is set and if the output is +1 the other GPIO is set. 
 
I used an array of 1000 samples of the standard sine wave to generate the PDM output. Hence, to produce a Sine wave of 50 Hz, the sampling frequency must be 50 kHz. For a sampling frequency of 50 kHz I needed an an accurate time interval of 20 us(microsecond) between 2 samples. To do this, I used a timer interrupt. I enabled the TIM2 timer interrupt, so that I can display each sample at accurate time intervals.
Reference for timer Interrupt: https://www.youtube.com/watch?v=nTOtXiJw1_Y&t=378s

