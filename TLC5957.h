#ifndef TLC5957_H
#define TLC5957_H

#include <stdint.h>
#include <SPI.h>

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

class TLC5957
{
    public:
        // SPI
        void init(uint8_t lat, uint8_t sin, uint8_t sclk, uint8_t gsclk);
        void setSpiBaudRate(uint32_t baud_rate);
        uint32_t getSpiBaudRate();
        void setGsclkFreq(uint32_t gsclk_frequency);
        uint32_t getGsclkFreq();
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
        void setLodDetection(bool lod_bit_0, bool lod_bit_1);
        void setTdSelection(bool td_bit_0, bool td_bit_1);
        void setGroupDelaySelect(bool group_delay_bit);
        void setRefreshMode(bool refresh_mode_bit);
        void setGclkEdgeSelect(bool gsclk_edge_bit);
        void setPrechargeMode(bool precharge_bit);
        void setEspwm(bool espwm_bit);
        void setBlueCompensation(bool blue_compensation_bit);
        void setSclkEdgeSelect(bool sclk_edge_bit);
        void setLowGsEnhancement(bool low_gs_bit_0, bool low_gs_bit_1, bool low_gs_bit_2);
        void setColorControl(uint16_t cc);
        void setColorControl(uint16_t ccr, uint16_t ccg, uint16_t ccb);
        void getColorControl(uint16_t* colorControl);
        void setBrightnessControl(uint8_t bc);
        uint8_t getBrightnessControl();
        void setPokerMode(bool poker_mode_bit);
        void setFirstLineImprovement(bool first_line_bit_0,
                                        bool first_line_bit_1,
                                        bool first_line_bit_2);
        void updateControl();

        static const uint8_t tlc_count;
        static const uint8_t COLOR_CHANNEL_COUNT = 3;
        static const uint8_t LEDS_PER_CHIP = 16;
        static bool enforce_max_current;
        static double max_current_amps;

        static uint16_t grayscale_data[][LEDS_PER_CHIP][COLOR_CHANNEL_COUNT];
        // TODO: do we need to track BC and CC for each LED group?
        // TODO: do we need rgb pin order?

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
