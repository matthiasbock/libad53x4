
/**
 * Library to control an Analog Devices AD5304/AD5314/AD5324
 * analog-to-digital converter IC (ADC)
 *
 * Author: Matthias Bock <mail@matthiasbock.net>
 * License: GNU GPLv3
 */
 
#include "ad53x4.h"

void adc_setup(
                adc_struct *adc,
                adc_t       type,
                uint32_t    spi_device,
                uint8_t     pin_SCK,
                uint8_t     pin_nSYNC,
                uint8_t     pin_DIN
                )
{
    adc->type       = type;
    adc->spi_device = spi_device;
    adc->pin_SCK    = pin_SCK;
    adc->pin_nSYNC  = pin_nSYNC;
    adc->pin_DIN    = pin_DIN;
    
    // initialize and configure SPI device

    spi_disable(spi_device);

    SPI_CONFIG = SPI_BITORDER_MSBFIRST | SPI_CLOCKPOLARITY_ACTIVELOW | SPI_CLOCKPHASE_TRAILING; 
    SPI_FREQUENCY(spi_device) = SPI_FREQUENCY_8M;

    spi_interrupt_upon_READY_enable(spi_device);
    interrupt_enable(INT_SPI);

    spi_pin_select_SCK (spi_device, pin_SCK);
    spi_pin_select_MOSI(spi_device, pin_DIN);
    spi_pin_select_MISO(spi_device, SPI_PIN_DISABLED);

    // unselect ADC

    gpio_setup(pin_nSYNC, GPIO_DIRECTION_OUTPUT);
    gpio_write(pin_nSYNC, GPIO_HIGH);
}

void adc_write(
                adc_struct *adc,
                adc_channel_t channel,
                uint16_t value
                )
{
    // trim value according to chip resolution
    value &= AD53X4_RESOLUTION[adc->type];
    value << AD53X4_LEFTSHIFT [adc->type];
    
    // append channel
    value |= (channel << 14);
    
    // SPI: select slave
    gpio_write(adc->pin_nSYNC, GPIO_LOW);

    // SPI_TX is double buffered, so two bytes can be pushed in one go    
    spi_write(adc->spi_device, (uint8_t) (value >> 8));    
    spi_write(adc->spi_device, (uint8_t) (value & 0xFF));

    // clear event
    SPI_EVENT_READY(adc->spi_device) = 0;

    // start transmission
    spi_enable(adc->spi_device);

    // wait for output to complete
    while (!SPI_EVENT_READY(adc->spi_device))
        asm("wfi");
    SPI_EVENT_READY(adc->spi_device) = 0;
    while (!SPI_EVENT_READY(adc->spi_device))
        asm("wfi");
    SPI_EVENT_READY(adc->spi_device) = 0;
    
    spi_disable(spi_device);
    
    // SPI: unselect slave
    gpio_write(adc->pin_nSYNC, GPIO_HIGH);
}
