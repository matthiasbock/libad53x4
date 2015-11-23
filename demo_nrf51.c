
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

int main()
{
    adc_struct ADC;
    adc_struct *HwMon = &ADC;

    adc_setup(
                HwMon,
                AD5324, // type
                SPI0,   // port
                21,     // nCS pin
                22,     // SCLK pin
                23      // MOSI pin
             );
    
    // Sawtooth function
    while (1)
    {
        uint16_t i;
        
        // up
        for (i=0; i<(2^12); i++)
        {
            adc_write(HwMon, ADC_OUT_A, i);
            delay_ms(1);
        }
        
        // down
        for (i=(2^12)-1; i>=0; i--)
        {
            adc_write(HwMon, ADC_OUT_A, i);
            delay_ms(1);
        }
    }

    return 0;
}
