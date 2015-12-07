
/**
 * Example program demonstrating
 * how to control an Analog Devices AD5324 ADC
 * from a nRF51822 microcontroller
 *
 * Author: Matthias Bock <mail@matthiasbock.net>
 * License: GNU GPLv3
 */

#include "ad53x4.h"
#include "../spi_master.h"
#include "../delay.h"

#define INTER_WRITE_DELAY 20

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

        for (i=0; i<20000; i++)
        {
            adc_write(HwMon, ADC_OUT_A, i);
            delay_us(INTER_WRITE_DELAY);
            adc_write(HwMon, ADC_OUT_B, i);
            delay_us(INTER_WRITE_DELAY);
            adc_write(HwMon, ADC_OUT_C, i);
            delay_us(INTER_WRITE_DELAY);
            adc_write(HwMon, ADC_OUT_D, i);
            delay_us(INTER_WRITE_DELAY);
        }

        for (i=0; i<20000; i++)
        {
            adc_write(HwMon, ADC_OUT_A, 20000-i);
            delay_us(INTER_WRITE_DELAY);
            adc_write(HwMon, ADC_OUT_B, 20000-i);
            delay_us(INTER_WRITE_DELAY);
            adc_write(HwMon, ADC_OUT_C, 20000-i);
            delay_us(INTER_WRITE_DELAY);
            adc_write(HwMon, ADC_OUT_D, 20000-i);
            delay_us(INTER_WRITE_DELAY);
        }
    }

    return 0;
}
