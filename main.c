
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

void main()
{
    ad53x4_t ADC;
    ad53x4_t *HwMon = &ADC;

    adc_setup(HwMon, AD5324, SPI0, 16, 17, 18); 
    
    // Sawtooth function
    while (1)
    {
        uint16_t i;
        
        // up
        for (i=0; i<2**12; i++)
        {
            adc_write(HwMon, ADC_OUT_A, i);
            delay_ms(1);
        }
        
        // down
        for (i=(2**12)-1; i>=0; i--)
        {
            adc_write(HwMon, ADC_OUT_A, i);
            delay_ms(1);
        }
    }
}
