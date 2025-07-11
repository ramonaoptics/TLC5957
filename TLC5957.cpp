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

#define DEFAULT_SPI_BAUD_RATE 4'000'000
#define DEFAULT_GSCLK_FREQUENCY 3'500'000

#define LOD_DETECTION_MASK ((uint64_t)0b11)
#define TD_SELECTION_MASK ((uint64_t)0b11 << 2)
#define GROUP_DELAY_SELECT_MASK ((uint64_t)0b1 << 4)
#define REFRESH_MODE_MASK ((uint64_t)0b1 << 5)
#define GSCLK_EDGE_SELECT_MASK ((uint64_t)0b1 << 6)
#define PRECHARGE_MODE_MASK ((uint64_t)0b1 << 7)
#define ESPWM_MASK ((uint64_t)0b1 << 8)
#define BLUE_COMPENSATION_MASK ((uint64_t)0b1 << 9)
#define SCLK_EDGE_SELECT ((uint64_t)0b1 << 10)
#define LOW_GS_ENHANCEMENT_MASK ((uint64_t)0b111 << 11)
#define COLOR_CONTROL_MASK ((uint64_t)0x7FFFFFF << 14)
#define BRIGHTNESS_CONTROL_MASK ((uint64_t)0b111 << 41)
#define POKER_MODE_MASK ((uint64_t)0b1 << 44)
#define FIRST_LINE_IMPROVEMENT_MASK ((uint64_t)0b111 << 45)

void TLC5957::init(uint8_t lat, uint8_t spi_mosi, uint8_t spi_clk, uint8_t gsclk)
{
    this->_lat = lat;
    this->_spi_mosi = spi_mosi;
    this->_spi_clk = spi_clk;
    this->_gsclk = gsclk;

    this->_function_data = 0;
    this->_BC = 0;
    this->_CC[0] = 0;
    this->_CC[1] = 0;
    this->_CC[2] = 0;

    this->setSpiBaudRate(DEFAULT_SPI_BAUD_RATE);

    SPI.setMOSI(this->_spi_mosi);
    SPI.begin();

    pinMode(this->_lat, OUTPUT);
    digitalWrite(this->_lat, LOW);
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

// https://www.ti.com/lit/ds/symlink/tlc5957.pdf?HQS=dis-dk-null-digikeymode-dsf-pf-null-wwe&ts=1648586629571
// Page 10-12
void TLC5957::latch(uint16_t data, uint8_t data_len, uint8_t num_edges)
{
    int spi_delay_us = 1 / this->_spi_baud_rate / 2 * 1'000'000;

    SPI.end();
    pinMode(_spi_clk, OUTPUT);
    pinMode(_spi_mosi, OUTPUT);

    uint8_t bit_to_send;
    uint8_t remaining_bits;

    digitalWrite(_spi_clk, LOW);
    delayMicroseconds(spi_delay_us);
    for (uint8_t i = 0; i < data_len; i++)
    {
        remaining_bits = data_len - i;
        bit_to_send = (data >> (remaining_bits - 1)) & 0x01;
        digitalWrite(_spi_mosi, bit_to_send);
        if (remaining_bits == num_edges) {
            digitalWrite(_lat, HIGH);
        }
        delayMicroseconds(spi_delay_us);
        digitalWrite(_spi_clk, HIGH);
        delayMicroseconds(spi_delay_us);
        digitalWrite(_spi_clk, LOW);
    }
    delayMicroseconds(spi_delay_us);
    digitalWrite(_spi_mosi, LOW);
    digitalWrite(_lat, LOW);
    SPI.begin();
}

void TLC5957::setAllLed(uint16_t gsvalue)
{
    for (int32_t chip = tlc_count - 1; chip >= 0; chip--)
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
        for (int32_t chip = tlc_count - 1; chip >= 0; chip--)
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
    int32_t chip = led_number / LEDS_PER_CHIP;
    int channel = led_number % LEDS_PER_CHIP;
    grayscale_data[chip][channel][2] = blue;
    grayscale_data[chip][channel][1] = green;
    grayscale_data[chip][channel][0] = red;
}

void TLC5957::setLed(int led_number, uint16_t rgb)
{
    int32_t chip = led_number / LEDS_PER_CHIP;
    int channel = led_number % LEDS_PER_CHIP;
    grayscale_data[chip][channel][2] = rgb;
    grayscale_data[chip][channel][1] = rgb;
    grayscale_data[chip][channel][0] = rgb;
}

int TLC5957::updateLeds(double* output_current, int clear)
{
    int32_t chip;
    double power_output_amps = getTotalCurrent();
    if (output_current != nullptr)
        *output_current = power_output_amps;
    if (enforce_max_current && power_output_amps > max_current_amps)
    {
        Serial.printf("%f.overcurrent", power_output_amps);
        return 1;
    }

    analogWrite(_gsclk, 0);

    // TODO: timing for latch changes if poker mode is activated
    // https://www.ti.com/lit/ds/symlink/tlc5957.pdf?HQS=dis-dk-null-digikeymode-dsf-pf-null-wwe&ts=1648586629571
    // Page 17, Figure 11
    uint16_t data_to_send;
    uint8_t num_edges;
    for (int8_t led_channel_index = LEDS_PER_CHIP - 1; led_channel_index >= 0; led_channel_index--)
    {
        if (led_channel_index == 0) {
            num_edges = LATGS;
        } else {
            num_edges = WRTGS;
        }
        SPI.beginTransaction(mSettings);
        for (chip = tlc_count - 1; chip > 0; chip--)
        {
            data_to_send = grayscale_data[chip][led_channel_index][2];
            SPI.transfer16(data_to_send);
            data_to_send = grayscale_data[chip][led_channel_index][1];
            SPI.transfer16(data_to_send);
            data_to_send = grayscale_data[chip][led_channel_index][0];
            SPI.transfer16(data_to_send);
        }

        data_to_send = grayscale_data[chip][led_channel_index][2];
        SPI.transfer16(data_to_send);
        data_to_send = grayscale_data[chip][led_channel_index][1];
        SPI.transfer16(data_to_send);
        SPI.endTransaction();

        data_to_send = grayscale_data[chip][led_channel_index][0];
        latch(data_to_send, 16, num_edges);
    }
    analogWrite(_gsclk, 1);
    return 0;
}

void TLC5957::clearLeds()
{
    updateLeds(nullptr, 1);
}

void TLC5957::getLedCurrents(double* currents, uint16_t* gs)
{

    for (int color_channel_index = 0; color_channel_index < COLOR_CHANNEL_COUNT; color_channel_index++)
    {
        // https://www.ti.com/lit/ds/symlink/tlc5957.pdf?HQS=dis-dk-null-digikeymode-dsf-pf-null-wwe&ts=1648586629571
        // Page 8 (Equation 1 & Equation 2)
        currents[color_channel_index] = (
            _maxOutputCurrent *
            (((double)(_CC[color_channel_index])) / 511) *
            _gainRatioValues[_BC] *
            (((double)(gs[color_channel_index])) / 0xFFFF)
        );
    }
}

double TLC5957::getTotalCurrent()
{
    uint32_t totalCurrent_channel[COLOR_CHANNEL_COUNT];
    for (uint8_t color_channel_index = 0; color_channel_index < COLOR_CHANNEL_COUNT; color_channel_index++)
        totalCurrent_channel[color_channel_index] = 0;

    for (uint8_t color_channel_index = 0; color_channel_index < COLOR_CHANNEL_COUNT; color_channel_index++)
        for (int32_t chip = 0; chip < tlc_count; chip++)
            for (uint8_t led_channel_index = 0; led_channel_index < LEDS_PER_CHIP; led_channel_index++)
                totalCurrent_channel[color_channel_index] += grayscale_data[chip][led_channel_index][color_channel_index];

    double totalCurrent = 0;
    for (uint8_t color_channel_index = 0; color_channel_index < COLOR_CHANNEL_COUNT; color_channel_index++)
    {
        double current = (_maxOutputCurrent * ((double)_CC[color_channel_index]) / 511 * _gainRatioValues[_BC]);
        totalCurrent += current * ((double)(totalCurrent_channel[color_channel_index]) / 0xFFFF);
    }

    return totalCurrent;
}

void TLC5957::setLodDetection(uint8_t lod)
{
    uint64_t new_data = ((uint64_t)lod) << 1;
    _function_data = (
        (_function_data & (~LOD_DETECTION_MASK)) |
        (LOD_DETECTION_MASK & new_data)
    );
}

void TLC5957::setTdSelection(uint8_t td)
{
    uint64_t new_data = ((uint64_t)td) << 2;
    _function_data = (
        (_function_data & (~TD_SELECTION_MASK)) |
        (TD_SELECTION_MASK & new_data)
    );
}

void TLC5957::setGroupDelaySelect(bool group_delay_bit)
{
    uint64_t new_data = group_delay_bit << 4;
    _function_data = (
        (_function_data & (~GROUP_DELAY_SELECT_MASK)) |
        (GROUP_DELAY_SELECT_MASK & new_data)
    );
}

void TLC5957::setRefreshMode(bool refresh_mode_bit)
{
    uint64_t new_data = refresh_mode_bit << 5;
    _function_data = (
        (_function_data & (~REFRESH_MODE_MASK)) |
        (REFRESH_MODE_MASK & new_data)
    );
}

void TLC5957::setGclkEdgeSelect(bool gsclk_edge_bit)
{
    uint64_t new_data = gsclk_edge_bit << 6;
    _function_data = (
        (_function_data & (~GSCLK_EDGE_SELECT_MASK)) |
        (GSCLK_EDGE_SELECT_MASK & new_data)
    );
}

void TLC5957::setPrechargeMode(bool precharge_bit)
{
    uint64_t new_data = precharge_bit << 7;
    _function_data = (
        (_function_data & (~PRECHARGE_MODE_MASK)) |
        (PRECHARGE_MODE_MASK & new_data)
    );
}

void TLC5957::setEspwm(bool espwm_bit)
{
    uint64_t new_data = espwm_bit << 8;
    _function_data = (
        (_function_data & (~ESPWM_MASK)) |
        (ESPWM_MASK & new_data)
    );
}

void TLC5957::setBlueCompensation(bool blue_compensation_bit)
{
    uint64_t new_data = blue_compensation_bit << 9;
    _function_data = (
        (_function_data & (~BLUE_COMPENSATION_MASK)) |
        (BLUE_COMPENSATION_MASK & new_data)
    );
}

void TLC5957::setSclkEdgeSelect(bool sclk_edge_bit)
{
    uint64_t new_data = sclk_edge_bit << 10;
    _function_data = (
        (_function_data & (~SCLK_EDGE_SELECT)) |
        (SCLK_EDGE_SELECT & new_data)
    );
}

void TLC5957::setLowGsEnhancement(uint8_t enhancement)
{
    uint64_t new_data = ((uint64_t)enhancement) << 11;
    _function_data = (
        (_function_data & (~LOW_GS_ENHANCEMENT_MASK)) |
        (LOW_GS_ENHANCEMENT_MASK & new_data)
    );
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
    new_data |= (((uint64_t)ccr) << 32);

    if (ccg > 511)
        ccg = 511;
    new_data |= (((uint64_t)ccg) << 23);

    if (ccb > 511)
        ccb = 511;
    new_data |= (((uint64_t)ccb) << 14);

    _function_data = (
        (_function_data & (~COLOR_CONTROL_MASK)) |
        (COLOR_CONTROL_MASK & new_data)
    );
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
    _function_data = (
        (_function_data & (~BRIGHTNESS_CONTROL_MASK)) |
        (BRIGHTNESS_CONTROL_MASK & new_data)
    );
    _BC = bc;
}

uint8_t TLC5957::getBrightnessControl()
{
    return _function_data >> 41 & 7;
}

void TLC5957::setPokerMode(bool poker_mode_bit)
{
    uint64_t new_data = (uint64_t)poker_mode_bit << 44;
    _function_data = (
        (_function_data & (~POKER_MODE_MASK)) |
        (POKER_MODE_MASK & new_data)
    );
}

void TLC5957::setFirstLineImprovement(uint8_t first_line_improvement)
{
    uint64_t new_data = ((uint64_t)first_line_improvement) << 45;
    _function_data = (
        (_function_data & (~FIRST_LINE_IMPROVEMENT_MASK)) |
        (FIRST_LINE_IMPROVEMENT_MASK & new_data)
    );
}

void TLC5957::updateControl()
{
    uint16_t word_to_send = 0;

    analogWrite(_gsclk, 0);

    SPI.beginTransaction(mSettings);
    for (int32_t chip = tlc_count - 1; chip > 0; chip--)
    {
        word_to_send = (_function_data >> 32) & 0xFFFF;
        SPI.transfer16(word_to_send);
        word_to_send = (_function_data >> 16) & 0xFFFF;
        SPI.transfer16(word_to_send);
        word_to_send = (_function_data >> 0) & 0xFFFF;
        if (chip != 1) {
            SPI.transfer16(word_to_send);
        }
    }
    SPI.endTransaction();

    latch(word_to_send, 16, FCWRTEN);
    SPI.beginTransaction(mSettings);
    word_to_send = (_function_data >> 32) & 0xFFFF;
    SPI.transfer16(word_to_send);
    word_to_send = (_function_data >> 16) & 0xFFFF;
    SPI.transfer16(word_to_send);
    SPI.endTransaction();
    // manually send last 8 bits
    latch((uint16_t)(_function_data & 0xFFFF), 16, WRTFC);
    analogWrite(_gsclk, 1);

}
