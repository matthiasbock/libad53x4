
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

#include "../spi_master.h"
#include "../nrf_gpio.h"

// Chip variants
typedef enum
{
    AD5304 = 0,
    AD5314 = 1,
    AD5324 = 2
} adc_t;


// Bitmasks for the corresponding chip variant's resolution
uint16_t AD53X4_RESOLUTION[3] =
{
    0x00FF,
    0x03FF,
    0x0FFF
};

// How much the value needs to be shifted left to fit the 8-bit SPI frame
uint8_t AD53X4_LEFTSHIFT[3] =
{
    0,
    2,
    4
};

// The ADC chip has four output channels
typedef enum
{
    ADC_OUT_A = 0,
    ADC_OUT_B = 1,
    ADC_OUT_C = 2,
    ADC_OUT_D = 3
} adc_channel_t;

typedef struct
{
    adc_t       type;
    uint32_t    spi_device;
    uint8_t     pin_SCK;
    uint8_t     pin_nSYNC;
    uint8_t     pin_DIN;
} adc_struct;

// Configure
void adc_setup(
                adc_struct *adc,
                adc_t       type,
                uint32_t    spi_device,
                uint8_t     pin_SCK,
                uint8_t     pin_nSYNC,
                uint8_t     pin_DIN
                );

// Use
void adc_write(
                adc_struct     *adc,
                adc_channel_t   channel,
                uint16_t        value
                );

#endif
