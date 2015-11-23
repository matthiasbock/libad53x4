
/**
 * Library to control an Analog Devices AD5304/AD5314/AD5324
 * analog-to-digital converter IC (ADC)
 *
 * Author: Matthias Bock <mail@matthiasbock.net>
 * License: GNU GPLv3
 */
 
#include "ad53x4.h"

// Bitmasks for the corresponding chip variant's resolution
static uint16_t AD53X4_RESOLUTION[3] =
{
    0x00FF,
    0x03FF,
    0x0FFF
};

// How much the value needs to be shifted left to fit the 8-bit SPI frame
static uint8_t AD53X4_LEFTSHIFT[3] =
{
    0,
    2,
    4
};

void adc_setup(
                adc_struct *adc,
                adc_t       type,
                uint32_t    spi_device,
                uint8_t     pin_SCK,
                uint8_t     pin_nSYNC,
                uint8_t     pin_DIN
                )
{
    // Memorize parameters in ADC struct
    adc->type       = type;
    adc->spi_device = spi_device;
    adc->pin_SCK    = pin_SCK;
    adc->pin_nSYNC  = pin_nSYNC;
    adc->pin_DIN    = pin_DIN;
    
    // Initialize and configure SPI device
    spi_disable(spi_device);

    SPI_CONFIG(spi_device) = SPI_BITORDER_MSBFIRST | SPI_CLOCKPOLARITY_ACTIVELOW | SPI_CLOCKPHASE_TRAILING;
    SPI_FREQUENCY(spi_device) = SPI_FREQUENCY_8M;

    spi_interrupt_upon_READY_enable(spi_device);
    NVIC_EnableIRQ(INTERRUPT_SPI);

    spi_pin_select(
                    spi_device,
                    pin_SCK,            // Clock
                    pin_DIN,            // MOSI
                    SPI_PIN_DISABLED    // MISO
                   );

    // unselect ADC

    nrf_gpio_pin_dir_set(pin_nSYNC, NRF_GPIO_PIN_DIR_OUTPUT);
    nrf_gpio_pin_set(pin_nSYNC);
    /*
    gpio_setup(pin_nSYNC, GPIO_DIRECTION_OUTPUT);
    gpio_write(pin_nSYNC, GPIO_HIGH);
    */
}

void adc_write(
                adc_struct *adc,
                adc_channel_t channel,
                uint16_t value
                )
{
    // trim value according to chip resolution
    value  &= AD53X4_RESOLUTION[adc->type];
    value <<= AD53X4_LEFTSHIFT [adc->type];
    
    // append channel
    value |= (channel << 14);
    
    // SPI: select slave
    nrf_gpio_pin_clear(adc->pin_nSYNC);
    //gpio_write(adc->pin_nSYNC, GPIO_LOW);

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
    
    spi_disable(adc->spi_device);
    
    // SPI: unselect slave
    nrf_gpio_pin_set(adc->pin_nSYNC);
    //gpio_write(adc->pin_nSYNC, GPIO_HIGH);
}
