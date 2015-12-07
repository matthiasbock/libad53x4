
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
    0x00FF, // AD5304
    0x03FF, // AD5314
    0x0FFF  // AD5324
};

// How much the value needs to be shifted left to fit the 8-bit SPI frame
static uint8_t AD53X4_LEFTSHIFT[3] =
{
    4,      // AD5304
    2,      // AD5314
    0       // AD5324
};

/*
 * SPI interrupt handlers
 * Notify a sleeping process, that an SPI device has finished transmitting
 */

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
                uint8_t     pin_nCS,
                uint8_t     pin_SCK,
                uint8_t     pin_MOSI
                )
{
    // apply parameters
    adc->adc_type   = type;
    adc->spi_device = spi_device;
    adc->spi_state  = IDLE;
    adc->pin_nCS    = pin_nCS;
    adc->pin_SCK    = pin_SCK;
    adc->pin_MOSI   = pin_MOSI;
    
    /*
     * Initialize SPI device
     */

    // make sure, the selected SPI device is disabled
    spi_disable(spi_device);

    // setup GPIO pins
    nrf_gpio_pin_dir_set(pin_nCS,   NRF_GPIO_PIN_DIR_OUTPUT);
    nrf_gpio_pin_dir_set(pin_SCK,   NRF_GPIO_PIN_DIR_OUTPUT);
    nrf_gpio_pin_dir_set(pin_MOSI,  NRF_GPIO_PIN_DIR_OUTPUT);
/*
    GPIO_PIN_CNF[pin_nCS]   = GPIO_OUTPUT
                            | GPIO_INPUTBUFFER_CONNECT
                            | GPIO_NOPULL
                            | GPIO_DRIVE_S0S1
                            | GPIO_SENSE_DISABLED;
    GPIO_PIN_CNF[pin_SCK]   = 1;
    GPIO_PIN_CNF[pin_MOSI]  = 1;
*/
    // Chip Select = HIGH: ADC not selected
    nrf_gpio_pin_set(pin_nCS);

    // GPIO pull settings on the SCK pin have no effect

    // For clarity on the oscilloscope, the level between transmissions should be LOW:
    NRF_GPIO->PIN_CNF[pin_MOSI] |= (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos);

    /*
    TODO: use gpio.h instead of nrf_gpio.h
    gpio_setup(pin_nCS, OUTPUT);
    gpio_set(pin_nCS, HIGH);
    */

    // select GPIO pins for SPI device
    spi_pin_select(
                    spi_device,
                    pin_SCK,            // SCK
                    pin_MOSI,           // MOSI
                    SPI_PIN_DISABLED    // MISO
                  );

    // configure SPI parameters
    SPI_CONFIG(spi_device)      = SPI_BITORDER_MSBFIRST
                                | SPI_CLOCKPOLARITY_ACTIVELOW
                                | SPI_CLOCKPHASE_LEADING;
    SPI_FREQUENCY(spi_device)   = SPI_FREQUENCY_250K;

    // enable interrupt, so that we know, when a transmission is complete
    //spi_interrupt_upon_READY_enable(spi_device);
    //TODO: use own NVIC routines from cortex_m0.h
    //interrupt_enable(INTERRUPT_SPI);
    //NVIC_EnableIRQ(INTERRUPT_SPI);

    // configure interrupt handler to clear TRANSMITTING flag
    if (spi_device == SPI0)
        adc_at_spi0 = adc;
    else if (spi_device == SPI1)
        adc_at_spi1 = adc;

    // we are ready to go
    spi_enable(spi_device);
}

void adc_write(
                adc_struct     *adc,
                adc_channel_t   channel,
                uint16_t        value
                )
{
    // trim value according to chip resolution
    value  &= AD53X4_RESOLUTION[adc->adc_type];

    //TODO: The following line causes the SPI MOSI to stay low
    //value <<= AD53X4_LEFTSHIFT [adc->adc_type];

    // select channel
    value  |= (channel << 14);

    // nLDAC
    if (channel == ADC_OUT_D)
        // clear nLDAC: update all outputs after transmission completion
        value &= ~(1 << 12);
    else
        // set nLDAC: only update input register, don't change output
        value |= (1 << 12);

    // no power-down (nPD)
    value |= (1 << 13);

    // clear event flag
    SPI_EVENT_READY(adc->spi_device) = 0;
    adc->spi_state = TRANSMITTING;

    // SPI: select slave
    nrf_gpio_pin_clear(adc->pin_nCS);
    //TODO:
    //gpio_set(adc->pin_nCS, LOW);

    delay_us(5);

    // SPI_TX is double buffered, so two bytes can be pushed in one go
    spi_write(adc->spi_device, (value >> 8) & 0xFF);
    // transmission starts about 0-5us after write
    spi_write(adc->spi_device, (value & 0xFF));

    // wait for output to complete
    /* TODO: somehow it doesn't work with interrupts and flags
    while (adc->spi_state == TRANSMITTING)
        asm("wfi");
    */
    // for 1 MHz: 14us
    delay_us(64);

    // this has no effect:
    //nrf_gpio_pin_clear(adc->pin_MOSI);

    // SPI: unselect slave
    nrf_gpio_pin_set(adc->pin_nCS);
    //TODO:
    //gpio_set(adc->pin_nCS, HIGH);
}
