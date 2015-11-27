
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
                AD5324,
                SPI0,
                21, // nCS
                22, // SCK
                23  // MOSI
             );
    
    // Sawtooth function
    while (1)
    {
        uint16_t i;
        
        // up
        for (i=0; i<(2^12); i++)
        {
            adc_write(HwMon, ADC_OUT_A, i);
            //uart_send(".", 1);
            delay_us(5);
        }
        
        // down
        for (i=(2^12)-1; i>=0; i--)
        {
            adc_write(HwMon, ADC_OUT_A, i);
            //uart_send(".", 1);
            delay_us(5);
        }
        //uart_send("\n", 1);
    }

    return 0;
}
