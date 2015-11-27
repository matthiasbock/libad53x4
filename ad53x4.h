
/**
 * Library to control an Analog Devices AD5304/AD5314/AD5324
 * analog-to-digital converter IC (ADC)
 *
 * Author: Matthias Bock <mail@matthiasbock.net>
 * License: GNU GPLv3
 */
 
#ifndef AD53X4_H
#define AD53X4_H

#include <stdint.h>

//TODO: use my own libraries for NVIC stuff and GPIO control
//#include "../cortex_m0.h"
#include "../nrf_gpio.h"
//#include "../gpio.h"
#include "../spi_master.h"
#include "../delay.h"
#include "../clock.h"

// Chip variants
typedef enum
{
    AD5304 = 0,
    AD5314 = 1,
    AD5324 = 2
} adc_t;

// The ADC chip has four output channels
typedef enum
{
    ADC_OUT_A = 0,
    ADC_OUT_B = 1,
    ADC_OUT_C = 2,
    ADC_OUT_D = 3
} adc_channel_t;

// remember the state of the selected SPI device
typedef enum
{
    IDLE,
    TRANSMITTING
} spi_activity_t;

// a struct holding the ADC session parameters
typedef struct
{
             adc_t          adc_type;
             uint32_t       spi_device;
    volatile spi_activity_t spi_state;
             uint8_t        pin_nCS;
             uint8_t        pin_SCK;
             uint8_t        pin_MOSI;
} adc_struct;

// Configure
void adc_setup(
                adc_struct *adc,
                adc_t       type,
                uint32_t    spi_device,
                uint8_t     pin_nCS,
                uint8_t     pin_SCK,
                uint8_t     pin_MOSI
                );

// Use
void adc_write(
                adc_struct     *adc,
                adc_channel_t   channel,
                uint16_t        value
                );

#endif
