
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

static adc_struct *adc_at_spi0 = 0;
static adc_struct *adc_at_spi1 = 0;

void SPI0_TWI0_Handler()
{
    if (adc_at_spi0 != 0 && SPI_EVENT_READY(SPI0))
            adc_at_spi0->spi_state = IDLE;
}

void SPI1_TWI1_Handler()
{
    if (adc_at_spi1 != 0 && SPI_EVENT_READY(SPI1))
            adc_at_spi1->spi_state = IDLE;
}

void adc_setup(
                adc_struct *adc,
                adc_t       type,
                uint32_t    spi_device,
                uint8_t     pin_nSYNC,
                uint8_t     pin_SCK,
                uint8_t     pin_DIN
                )
{
    // apply parameters
    adc->adc_type   = type;
    adc->spi_device = spi_device;
    adc->pin_SCK    = pin_SCK;
    adc->pin_nSYNC  = pin_nSYNC;
    adc->pin_DIN    = pin_DIN;
    
    /*
     * Initialize SPI device
     */

    // make sure, the selected SPI device is disabled
    spi_disable(spi_device);

    // then configure parameters
    SPI_CONFIG(spi_device) = SPI_BITORDER_MSBFIRST
                           | SPI_CLOCKPOLARITY_ACTIVELOW
                           | SPI_CLOCKPHASE_TRAILING;
    SPI_FREQUENCY(spi_device) = SPI_FREQUENCY_125K;

    // select the pins to use
    spi_pin_select(
                    spi_device,
                    pin_SCK,            // SCK
                    pin_DIN,            // MOSI
                    SPI_PIN_DISABLED    // MISO
                  );

    // setup the selected pins
    nrf_gpio_pin_dir_set(pin_nSYNC, NRF_GPIO_PIN_DIR_OUTPUT);
    nrf_gpio_pin_dir_set(pin_DIN,   NRF_GPIO_PIN_DIR_OUTPUT);
    nrf_gpio_pin_dir_set(pin_SCK,   NRF_GPIO_PIN_DIR_OUTPUT);
    // Chip Select = HIGH (ADC not selected)
    nrf_gpio_pin_set(pin_nSYNC);
    /*
    TODO:
    gpio_setup(pin_nSYNC, OUTPUT);
    gpio_set(pin_nSYNC, HIGH);
    */

    // enable interrupt, so that we know, when a transmission is complete
    spi_interrupt_upon_READY_enable(spi_device);
    //TODO:
    //interrupt_enable(INTERRUPT_SPI);
    NVIC_EnableIRQ(INTERRUPT_SPI);

    // configure interrupt handler to clear TRANSMITTING flag
    adc->spi_state = IDLE;
    if (spi_device == SPI0)
        adc_at_spi0 = adc;
    else if (spi_device == SPI1)
        adc_at_spi1 = adc;
}

void adc_write(
                adc_struct *adc,
                adc_channel_t channel,
                uint16_t value
                )
{
    // trim value according to chip resolution
    value  &= AD53X4_RESOLUTION[adc->adc_type];
    value <<= AD53X4_LEFTSHIFT [adc->adc_type];
    
    // append channel
    value  |= (channel << 14);
    
    // SPI: select slave
    nrf_gpio_pin_clear(adc->pin_nSYNC);
    //TODO:
    //gpio_set(adc->pin_nSYNC, LOW);

    // SPI_TX is double buffered, so two bytes can be pushed in one go    
    spi_write(adc->spi_device, (uint8_t) (value >> 8));    
    spi_write(adc->spi_device, (uint8_t) (value & 0xFF));

    // clear event flag
    SPI_EVENT_READY(adc->spi_device) = 0;
    adc->spi_state = TRANSMITTING;

    // start transmission
    spi_enable(adc->spi_device);

    // wait for output to complete
    while (adc->spi_state == TRANSMITTING)
        asm("wfi");
    //delay_us(150);

    // SPI: unselect slave
    // Unselect slave first, so that it's faster free to begin processing the values.
    nrf_gpio_pin_set(adc->pin_nSYNC);
    //TODO:
    //gpio_set(adc->pin_nSYNC, HIGH);

    spi_disable(adc->spi_device);
}
