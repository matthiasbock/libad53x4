
/**
 * Example program demonstrating
 * how to control an Analog Devices AD5324 ADC
 * from a nRF51822 microcontroller
 *
 * Author: Matthias Bock <mail@matthiasbock.net>
 * License: GNU GPLv3
 */

#include "ad53x4.h"
#include "spi_master.h"
#include "delay.h"

int main()
{
    adc_struct ADC;
    adc_struct *HwMon = &ADC;

    adc_setup(
                HwMon,
                AD5324,
                SPI0,
                7, // nCS
                5, // SCK
                6  // MOSI
             );
    
    // Sawtooth function
    while (1)
    {
        uint16_t i;

        for (i=0; i<4095; i++)
        {
            adc_write(HwMon, ADC_OUT_A, i);
            adc_write(HwMon, ADC_OUT_B, i);
            adc_write(HwMon, ADC_OUT_C, i);
            adc_write(HwMon, ADC_OUT_D, i);
        }

        for (i=4095; i>0; i--)
        {
            adc_write(HwMon, ADC_OUT_A, i);
            adc_write(HwMon, ADC_OUT_B, i);
            adc_write(HwMon, ADC_OUT_C, i);
            adc_write(HwMon, ADC_OUT_D, i);
        }
    }

    return 0;
}
