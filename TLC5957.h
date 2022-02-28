#ifndef TLC5957_H
#define TLC5957_H

#include <stdint.h>
#include <SPI.h>

#define GS_BITS 16
#define BC_BITS 3
#define CC_BITS 9
#define FC_BITS 48

#define WRTFC 5
#define FCWRTEN 15
#define WRTGS 1
#define LATGS 3

class TLC5957
{
    public:
        // SPI
        void init(uint8_t lat, uint8_t sin, uint8_t sclk, uint8_t gsclk);
        void setSpiBaudRate(uint32_t baud_rate);
        void setGsclkFrequency(uint32_t gsclk_frequency);
        void latch(int num_edges);
        void setBuffer(uint8_t bit);

        // Function Control Register Data
        void setLodDetection(bool bit_0, bool bit_1);
        void setTdSelection(bool bit_2, bool bit_3);
        void setGroupDelaySelect(bool bit_4);
        void setRefreshMode(bool bit_5);
        void setGclkEdgeSelect(bool bit_6);
        void setPrechargeMode(bool bit_7);
        void setEspwm(bool bit_8);
        void setBlueCompensation(bool bit_9);
        void setSclkEdgeSelect(bool bit_10);
        void setLowGsEnhancement(bool bit_11, bool bit_12, bool bit_13);
        void setColorControl(uint16_t ccr, uint16_t ccg, uint16_t ccb);
        void getColorControl(uint16_t* colorControl);
        void setBrightnessControl(uint8_t bc);
        uint8_t getBrightnessControl();
        void setPokerMode(bool bit_44);
        void setFirstLineImprovement(bool bit_45, bool bit_46, bool bit_47);
        void updateControl();

        static const uint8_t COLOR_CHANNEL_COUNT = 3;
        static const uint8_t LEDS_PER_CHIP = 16;
        static uint16_t grayscale_data[][LEDS_PER_CHIP][COLOR_CHANNEL_COUNT];

    private:
        uint8_t _lat;
        uint8_t _sin;
        uint8_t _sclk;
        uint8_t _gsclk;

        uint8_t _buffer = 0;
        int8_t _buffer_count = 7;

        uint64_t _function_data = 0;

        // SPI
        SPISettings mSettings;
        uint32_t _spi_baud_rate = 1'000'000;
        uint32_t _gsclk_frequency = 2'500'000;
};

#endif
