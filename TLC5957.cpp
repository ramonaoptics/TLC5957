#include "TLC5957.h"
#include <SPI.h>
#include <Arduino.h>

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
    setGsclkFrequency(_gsclk_frequency);

}

void TLC5957::setSpiBaudRate(uint32_t baud_rate)
{
    _spi_baud_rate = baud_rate;
    mSettings = SPISettings(_spi_baud_rate, MSBFIRST, SPI_MODE0);
}

void TLC5957::setGsclkFrequency(uint32_t gsclk_frequency)
{
    _gsclk_frequency = gsclk_frequency;
    analogWriteFrequency(_gsclk, _gsclk_frequency);
    analogWriteResolution(1);
    analogWrite(_gsclk, 1);
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

void TLC5957::setBuffer(uint8_t bit)
{
    bitWrite(_buffer, _buffer_count--, bit);
    SPI.beginTransaction(mSettings);
    if (_buffer_count == -1)
    {
        SPI.transfer(_buffer);
        _buffer_count = 7;
        _buffer = 0;
    }
    SPI.endTranscation();
}

void TLC5957::setLodDetection(bool bit_0, bool bit_1)
{
    _function_data |= bit_0;
    _function_data |= bit_1 << 1;
}

void TLC5957::setTdSelection(bool bit_2, bool bit_3)
{
    _function_data |= bit_2 << 2;
    _function_data |= bit_3 << 3;
}

void TLC5957::setGroupDelaySelect(bool bit_4)
{
    _function_data |= bit_4 << 4;
}

void TLC5957::setRefreshMode(bool bit_5)
{
    _function_data |= bit_5 << 5;
}

void TLC5957::setGclkEdgeSelect(bool bit_6)
{
    _function_data |= bit_6 << 6;
}

void TLC5957::setPrechargeMode(bool bit_7)
{
    _function_data |= bit_7 << 7;
}

void TLC5957::setEspwm(bool bit_8)
{
    _function_data |= bit_8 << 8;
}

void TLC5957::setBlueCompensation(bool bit_9)
{
    _function_data |= bit_9 << 9;
}

void TLC5957::setSclkEdgeSelect(bool bit_10)
{
    _function_data |= bit_10 << 10;
}

void TLC5957::setLowGsEnhancement(bool bit_11, bool bit_12, bool bit_13)
{
    _function_data |= bit_11 << 11;
    _function_data |= bit_12 << 12;
    _function_data |= bit_13 << 13;
}

void TLC5957::setColorControl(uint16_t ccr, uint16_t ccg, uint16_t ccb)
{
    if (ccr > 511)
        ccr = 511;
    _function_data |= ccr << 14;

    if (ccg > 511)
        ccg = 511;
    _function_data |= ccg << 23;

    if (ccb > 511)
        ccb = 511;
    _function_data |= ccb << 32;
}

void TLC5957::getColorControl(uint16_t* colorControl)
{
    colorControl[0] = _function_data >> 32 & 511;
    colorControl[1] = _function_data >> 23 & 511;
    colorControl[1] = _function_data >> 14 & 511;
}

void TLC5957::setBrightnessControl(uint8_t bc)
{
    if (bc > 7)
        bc = 7;
    _function_data |= bc << 41;
}

uint8_t TLC5957::getBrightnessControl()
{
    return _function_data >> 41 & 7;
}

void TLC5957::setPokerMode(bool bit_44)
{
    _function_data |= bit_44 << 44;
}

void TLC5957::setFirstLineImprovement(bool bit_45, bool bit_46, bool bit_47)
{
    _function_data |= bit_45 << 45;
    _function_data |= bit_46 << 46;
    _function_data |= bit_47 << 47;
}

void TLC5957::updateControl()
{

    uint8_t word_size = 8; // bits
    uint8_t num_words = FC_BITS / word_size;
    uint8_t buffer;

    // send first 5 bytes
    latch(FCWRTEN);
    for (uint8_t i = num_words - 1; i > 0; i--)
    {
        buffer = _function_data >> (8 * i) & 256;
        SPI.transfer(buffer);
    }

    // manually send last 8 bits
    SPI.end();
    digitalWrite(_sclk, LOW);
    for (uint8_t i = 7; i >= 0; i--)
    {
        digitalWrite(_sin, _function_data >> i & 1);
        digitalWrite(_sclk, HIGH);
        digitalWrite(_sclk, LOW);
        if (i == 4)
            digitalWrite(_lat, HIGH);
        else if (i == 0)
        digitalWrite(_lat, LOW);
    }
    SPI.begin();
}








