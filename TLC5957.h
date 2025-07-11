#ifndef TLC5957_H
#define TLC5957_H

#include <stdint.h>
#include <SPI.h>

class TLC5957
{
    public:
        // SPI
        void init(uint8_t lat, uint8_t spi_mosi, uint8_t spi_clk, uint8_t gsclk);
        void setSpiBaudRate(uint32_t baud_rate);
        uint32_t getSpiBaudRate();
        void setGsclkFreq(uint32_t gsclk_frequency);
        uint32_t getGsclkFreq();
        void latch(uint16_t data, uint8_t data_len, uint8_t num_edges);

        // control led
        void setAllLed(uint16_t gsvalue);
        void setAllLedRgb(uint16_t red, uint16_t green, uint16_t blue);
        void setLed(int led_number, uint16_t red, uint16_t green, uint16_t blue);
        void setLed(int led_number, uint16_t rgb);
        int updateLeds(double* output_current, int clear);
        void clearLeds();

        // led intensities
        void getLedCurrents(double* currents, uint16_t* gs);
        double getTotalCurrent();

        // Function Control register data
        void setLodDetection(uint8_t lod);
        void setTdSelection(uint8_t td);
        void setGroupDelaySelect(bool group_delay_bit);
        void setRefreshMode(bool refresh_mode_bit);
        void setGclkEdgeSelect(bool gsclk_edge_bit);
        void setPrechargeMode(bool precharge_bit);
        void setEspwm(bool espwm_bit);
        void setBlueCompensation(bool blue_compensation_bit);
        void setSclkEdgeSelect(bool sclk_edge_bit);
        void setLowGsEnhancement(uint8_t enhancement);
        void setColorControl(uint16_t cc);
        void setColorControl(uint16_t ccr, uint16_t ccg, uint16_t ccb);
        void getColorControl(uint16_t* colorControl);
        void setBrightnessControl(uint8_t bc);
        uint8_t getBrightnessControl();
        void setPokerMode(bool poker_mode_bit);
        void setFirstLineImprovement(uint8_t first_line_improvement);
        void updateControl();

        static const int32_t tlc_count;
        static const uint8_t COLOR_CHANNEL_COUNT = 3;
        static const uint8_t LEDS_PER_CHIP = 16;
        static bool enforce_max_current;
        static double max_current_amps;

        static uint16_t grayscale_data[][LEDS_PER_CHIP][COLOR_CHANNEL_COUNT];
        // TODO: do we need to track BC and CC for each LED group?
        // TODO: do we need rgb pin order?

    private:
        uint8_t _lat;
        uint8_t _spi_mosi;
        uint8_t _spi_clk;
        uint8_t _gsclk;

        uint64_t _function_data;

        // Analog Control Values)
        uint8_t _BC;
        uint16_t _CC[3];
        // https://www.ti.com/lit/ds/symlink/tlc5957.pdf?HQS=dis-dk-null-digikeymode-dsf-pf-null-wwe&ts=1648586629571
        // Page 9 (Table 2)
        const double _gainRatioValues[8] = {0.129, 0.256, 0.379, 0.524, 0.647, 0.733, 0.917, 1.0};
        // TODO: I_OLCMAX set by R_IREF resistor
        // TODO: think of more descriptive var name there are so many different "maxCurrents" ..
        const double _maxOutputCurrent = 25E-3; // placeholder

        // SPI
        SPISettings mSettings;
        uint32_t _spi_baud_rate;
        uint32_t _gsclk_frequency;
};

#endif
