#include "TLC5957.h"
#include <SPI.h>
#include <Arduino.h>
#include <inttypes.h>

#define GS_BITS 16
#define BC_BITS 3
#define CC_BITS 9
#define FC_BITS 48

#define WRTFC 5
#define FCWRTEN 15
#define WRTGS 1
#define LATGS 3

void TLC5957::init(uint8_t lat, uint8_t sin, uint8_t sclk, uint8_t gsclk)
{
    _lat = lat;
    _sin = sin;
    _sclk = sclk;
    _gsclk = gsclk;

    pinMode(_lat, OUTPUT);
    pinMode(_sin, OUTPUT);
    pinMode(_sclk, OUTPUT);
    pinMode(_gsclk, OUTPUT);

    mSettings = SPISettings(_spi_baud_rate, MSBFIRST, SPI_MODE0);
    SPI.setMOSI(_sin);
    SPI.begin();

    digitalWrite(_lat, LOW);
    setGsclkFreq(_gsclk_frequency);
}

void TLC5957::setSpiBaudRate(uint32_t baud_rate)
{
    _spi_baud_rate = baud_rate;
    mSettings = SPISettings(_spi_baud_rate, MSBFIRST, SPI_MODE0);
}

uint32_t TLC5957:: getSpiBaudRate()
{
    return _spi_baud_rate;
}

void TLC5957::setGsclkFreq(uint32_t gsclk_frequency)
{
    _gsclk_frequency = gsclk_frequency;
    analogWriteFrequency(_gsclk, _gsclk_frequency);
    analogWriteResolution(1);
    analogWrite(_gsclk, 1);
}

uint32_t TLC5957::getGsclkFreq()
{
    return _gsclk_frequency;
}

void TLC5957::latch(int num_edges)
{
    // TODO: do I  need to enforce integer division??
    int lat_delay_microseconds = _spi_baud_rate / 1'000'000;
    delayMicroseconds(lat_delay_microseconds);
    digitalWrite(_lat, HIGH);
    delayMicroseconds(lat_delay_microseconds * num_edges);
    digitalWrite(_lat, LOW);
    delayMicroseconds(lat_delay_microseconds);
}

// https://www.ti.com/lit/ds/symlink/tlc5957.pdf?HQS=dis-dk-null-digikeymode-dsf-pf-null-wwe&ts=1648586629571
// Page 10-12
void TLC5957::latch(uint16_t data, uint8_t data_len, uint8_t num_edges)
{
    Serial.printf("manual latch\n");
    uint64_t buffer_delay_us = 1000;
    SPI.end();
    digitalWrite(_sclk, LOW);
    for (uint8_t i = data_len - 1; i >= 0; i--)
    {
        digitalWrite(_sin, data >> i & 1);
        digitalWrite(_sclk, HIGH);
        delayMicroseconds(buffer_delay_us);
        digitalWrite(_sclk, LOW);
        if (i == num_edges - 1)
            digitalWrite(_lat, HIGH);
        else if (i == 0)
            digitalWrite(_lat, LOW);
    }
    SPI.begin();
}

void TLC5957::setAllLed(uint16_t gsvalue)
{
    for (int8_t chip = tlc_count - 1; chip >= 0; chip--)
    {
        for (int8_t led = 0; led < LEDS_PER_CHIP; led++)
        {
            for (int8_t channel = 0; channel < COLOR_CHANNEL_COUNT; channel++)
                grayscale_data[chip][led][channel] = gsvalue;
        }
    }
}

void TLC5957::setAllLedRgb(uint16_t red, uint16_t green, uint16_t blue)
{
    if (COLOR_CHANNEL_COUNT == 3)
    {
        for (int8_t chip = tlc_count - 1; chip >= 0; chip--)
        {
            for (int8_t channel = 0; channel < LEDS_PER_CHIP; channel++)
            {
                grayscale_data[chip][channel][2] = blue;
                grayscale_data[chip][channel][1] = green;
                grayscale_data[chip][channel][0] = red;
            }
        }
    }
}

void TLC5957::setLed(int led_number, uint16_t red, uint16_t green, uint16_t blue)
{
    int chip = led_number / LEDS_PER_CHIP;
    int channel = led_number % LEDS_PER_CHIP;
    grayscale_data[chip][channel][2] = blue;
    grayscale_data[chip][channel][1] = green;
    grayscale_data[chip][channel][0] = red;
}

void TLC5957::setLed(int led_number, uint16_t rgb)
{
    int chip = led_number / LEDS_PER_CHIP;
    int channel = led_number % LEDS_PER_CHIP;
    grayscale_data[chip][channel][2] = rgb;
    grayscale_data[chip][channel][1] = rgb;
    grayscale_data[chip][channel][0] = rgb;
}

int TLC5957::updateLeds(double* output_current)
{
    double power_output_amps = getTotalCurrent();
    if (output_current != nullptr)
        *output_current = power_output_amps;
    if (enforce_max_current && power_output_amps > max_current_amps)
        return 1;
    // TODO: timing for latch changes if poker mode is activated
    latch(WRTGS);
    for (int8_t chip = (int8_t)tlc_count - 1; chip >= 0; chip--)
    {
        for (int8_t led_channel_index = (int8_t)LEDS_PER_CHIP - 1; led_channel_index >= 0; led_channel_index--)
        {
            Serial.printf("%d\n", led_channel_index);
            for (int8_t color_channel_index = (int8_t)COLOR_CHANNEL_COUNT - 1; color_channel_index >= 0; color_channel_index--)
            {
                SPI.transfer16(grayscale_data[chip][led_channel_index][color_channel_index]);




                // TODO; assuming color channel index order is (r: 0, g: 1, b:2)
                // if (chip > 0 && led_channel_index > 0 && color_channel_index > 0)
                // {
                    // SPI.transfer16(grayscale_data[chip][led_channel_index][color_channel_index]);
                // }
                // else
                // {
                    // Serial.printf("second latch attempt\n");
                    // manually send last 16 bits
                    // latch(grayscale_data[0][0][0], 16, LATGS);
                    // Serial.printf("second latch\n");
                // }
            }
            latch(LATGS);
        }
    }
    Serial.printf("update_led\n");
    return 0;
}

void TLC5957::clearLeds()
{
    latch(WRTGS);
    for (uint8_t chip = (uint8_t)tlc_count - 1; chip >= 0; chip--)
    {
        for (uint8_t led_channel_index = (uint8_t)LEDS_PER_CHIP - 1; led_channel_index >= 0; led_channel_index--)
        {
            for (uint8_t color_channel_index = (uint8_t)COLOR_CHANNEL_COUNT - 1; color_channel_index >= 0; color_channel_index--)
            {
                if (chip > 0 && led_channel_index > 0 && color_channel_index > 0)
                {
                    SPI.transfer16(0);
                }
                else
                {
                    // manually send last 16 bits
                    latch((uint16_t)0, 16, LATGS);
                }
            }
        }
    }
}

void TLC5957::getLedCurrents(double* currents, uint16_t* gs)
{
    for (int color_channel_index = 0; color_channel_index < COLOR_CHANNEL_COUNT; color_channel_index++)
    {
        // https://www.ti.com/lit/ds/symlink/tlc5957.pdf?HQS=dis-dk-null-digikeymode-dsf-pf-null-wwe&ts=1648586629571
        // Page 8 (Equation 1 & Equation 2)
        currents[color_channel_index] = (_maxOutputCurrent * _CC[color_channel_index] / 511 * _BC)
                                            * (gs[color_channel_index] / 0xFFFF);
    }
}

double TLC5957::getTotalCurrent()
{
    uint32_t totalCurrent_channel[COLOR_CHANNEL_COUNT];
    for (uint8_t color_channel_index = 0; color_channel_index < COLOR_CHANNEL_COUNT; color_channel_index++)
        totalCurrent_channel[color_channel_index] = 0;

    for (uint8_t color_channel_index = 0; color_channel_index < COLOR_CHANNEL_COUNT; color_channel_index++)
        for (uint8_t chip = 0; chip < tlc_count; chip++)
            for (uint8_t led_channel_index = 0; led_channel_index < LEDS_PER_CHIP; led_channel_index++)
                totalCurrent_channel[color_channel_index] += grayscale_data[chip][led_channel_index][color_channel_index];

    double totalCurrent = 0;
    for (uint8_t color_channel_index = 0; color_channel_index < COLOR_CHANNEL_COUNT; color_channel_index++)
        totalCurrent += (_maxOutputCurrent * _CC[color_channel_index] / 511 * _BC)
                        * (totalCurrent_channel[color_channel_index] / 0xFFFF);

    return totalCurrent;
}

void TLC5957::setLodDetection(uint8_t lod)
{
    uint64_t new_data = ((uint64_t)lod) << 1;
    _function_data |= LOD_DETECTION_MASK & new_data;
}

void TLC5957::setTdSelection(uint8_t td)
{
    uint64_t new_data = ((uint64_t)td) << 2;
    _function_data |= TD_SELECTION_MASK & new_data;
}

void TLC5957::setGroupDelaySelect(bool group_delay_bit)
{
    uint64_t new_data = group_delay_bit << 4;
    _function_data |= GROUP_DELAY_SELECT_MASK & new_data;
}

void TLC5957::setRefreshMode(bool refresh_mode_bit)
{
    uint64_t new_data = refresh_mode_bit << 5;
    _function_data |= REFRESH_MODE_MASK & new_data;
}

void TLC5957::setGclkEdgeSelect(bool gsclk_edge_bit)
{
    uint64_t new_data = gsclk_edge_bit << 6;
    _function_data |= GSCLK_EDGE_SELECT_MASK & new_data;
}

void TLC5957::setPrechargeMode(bool precharge_bit)
{
    uint64_t new_data = precharge_bit << 7;
    _function_data |= PRECHARGE_MODE_MASK & new_data;
}

void TLC5957::setEspwm(bool espwm_bit)
{
    uint64_t new_data = espwm_bit << 8;
    _function_data |= ESPWM_MASK & new_data;
}

void TLC5957::setBlueCompensation(bool blue_compensation_bit)
{
    uint64_t new_data = blue_compensation_bit << 9;
    _function_data |= BLUE_COMPENSATION_MASK & new_data;
}

void TLC5957::setSclkEdgeSelect(bool sclk_edge_bit)
{
    uint64_t new_data = sclk_edge_bit << 10;
    _function_data |= SCLK_EDGE_SELECT & new_data;
}

void TLC5957::setLowGsEnhancement(uint8_t enhancement)
{
    uint64_t new_data = ((uint64_t)enhancement) << 11;
    _function_data |= LOW_GS_ENHANCEMENT_MASK & new_data;
}

void TLC5957::setColorControl(uint16_t cc)
{
    setColorControl(cc, cc, cc);
}

void TLC5957::setColorControl(uint16_t ccr, uint16_t ccg, uint16_t ccb)
{
    uint64_t new_data = 0;

    if (ccr > 511)
        ccr = 511;
    new_data |= ccr << 14;

    if (ccg > 511)
        ccg = 511;
    new_data |= ((uint32_t)ccg) << 23;

    if (ccb > 511)
        ccb = 511;
    new_data |= ((uint64_t)ccb) << 32;

    _function_data |= COLOR_CONTROL_MASK & new_data;
    _CC[0] = ccr;
    _CC[1] = ccg;
    _CC[2] = ccb;
}

void TLC5957::getColorControl(uint16_t* colorControl)
{
    colorControl[0] = (_function_data >> 32) & 0x01FF;
    colorControl[1] = (_function_data >> 23) & 0x01FF;
    colorControl[2] = (_function_data >> 14) & 0x01FF;
}

void TLC5957::setBrightnessControl(uint8_t bc)
{
    if (bc > 7)
        bc = 7;
    uint64_t new_data = ((uint64_t)bc) << 41;
    _function_data |= BRIGHTNESS_CONTROL_MASK & new_data;
    _BC = bc;
}

uint8_t TLC5957::getBrightnessControl()
{
    return _function_data >> 41 & 7;
}

void TLC5957::setPokerMode(bool poker_mode_bit)
{
    uint64_t new_data = (uint64_t)poker_mode_bit << 44;
    _function_data |= POKER_MODE_MASK & new_data;
}

void TLC5957::setFirstLineImprovement(uint8_t first_line_improvement)
{
    uint64_t new_data = ((uint64_t)first_line_improvement) << 45;
    _function_data |= FIRST_LINE_IMPROVEMENT_MASK & new_data;
}

void TLC5957::updateControl()
{

    // Serial.printf("tlc_update\n");
    uint8_t word_size = 8; // bits
    uint8_t num_words = FC_BITS / word_size;
    uint8_t buffer;

    latch(FCWRTEN);
    // Serial.printf("first_latch\n");
    // send first 5 bytes
    // Serial.printf("%" PRIu64 "\n", _function_data);
    for (uint8_t i = num_words; i > 0; i--)
    {
        buffer = _function_data >> (8 * i) & 255;
        // Serial.printf("%d.", buffer);
        SPI.transfer(buffer);
    }
    latch(WRTFC);
    // Serial.printf("\n");
    // manually send last 8 bits
    // latch((uint16_t)_function_data, 8, FCWRTEN);
}








