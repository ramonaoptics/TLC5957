#ifndef TLC5957_H
#define TLC5957_H

#include <stdint.h>
#include <SPI.h>

#define LOD_DETECTION_MASK ((uint64_t)0b11)
#define TD_SELECTION_MASK ((uint64_t)0b11 << 2
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

class TLC5957
{
    public:
        // SPI
        void init(uint8_t lat, uint8_t sin, uint8_t sclk, uint8_t gsclk);
        void setSpiBaudRate(uint32_t baud_rate);
        uint32_t getSpiBaudRate();
        void setGsclkFrequency(uint32_t gsclk_frequency);
        uint32_t getGsclkFrequency();
        void latch(int num_edges);
        void latch(uint16_t data, uint8_t data_len, uint8_t num_edges);

        // control led
        void setAllLed(uint16_t gsvalue);
        void setAllLedRgb(uint16_t red, uint16_t green, uint16_t blue);
        void setLed(int led_number, uint16_t red, uint16_t green, uint16_t blue);
        void setLed(int led_number, uint16_t rgb);
        int updateLeds(double* output_current);
        void clearLeds();

        // led intensities
        void getLedCurrents(double* currents, uint16_t* gs);
        double getTotalCurrent();

        // Function Control register data
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
        void setColorControl(uint16_t cc);
        void setColorControl(uint16_t ccr, uint16_t ccg, uint16_t ccb);
        void getColorControl(uint16_t* colorControl);
        void setBrightnessControl(uint8_t bc);
        uint8_t getBrightnessControl();
        void setPokerMode(bool bit_44);
        void setFirstLineImprovement(bool bit_45, bool bit_46, bool bit_47);
        void updateControl();

        static const uint8_t tlc_count;
        static const uint8_t COLOR_CHANNEL_COUNT = 3;
        static const uint8_t LEDS_PER_CHIP = 16;
        static bool enforce_max_current;
        static double max_current_amps;

        static uint16_t grayscale_data[][LEDS_PER_CHIP][COLOR_CHANNEL_COUNT];

    private:
        uint8_t _lat;
        uint8_t _sin;
        uint8_t _sclk;
        uint8_t _gsclk;

        uint64_t _function_data = 0;

        // Analog Control Values)
        uint8_t _BC;
        uint16_t _CC[3];
        // https://www.ti.com/lit/ds/symlink/tlc5957.pdf?HQS=dis-dk-null-digikeymode-dsf-pf-null-wwe&ts=1648586629571
        // Page 9 (Table 2)
        const double _gainRatioValues[8] = {0.129, 0.256, 0.379, 0.524, 0.647, 0.733, 0.917, 1.0};
        // TODO: I_OLCMAX set by R_IREF resistor
        // TODO: think of more descriptive var name there are so many different "maxCurrents" ..
        const double _maxOutputCurrent = 25; // placeholder

        // SPI
        SPISettings mSettings;
        uint32_t _spi_baud_rate = 1'000'000;
        uint32_t _gsclk_frequency = 2'500'000;
};

#endif
