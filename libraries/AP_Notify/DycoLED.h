/*
  Copyright (C) 2017 Siddharth Bharat Purohit. All rights reserved.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <AP_HAL/AP_HAL.h>
#include "NotifyDevice.h"

#define CLK_PERIOD 200          //200ns

#define DYCOCLK 50
#define DYCODOUT 51

#define MAX_NUM_LEDS    35
#define MAX_PATTERN_STEPS 16


//DycoLEDDriver: this class is used for setting up of parameters
//related to a single DycoLEDDriver.
class DycoLEDDriver
{
private:
    uint32_t clk_prev_time;
    uint32_t clk_pulse_count;
    uint32_t data_hold;
    uint32_t data_buf;
    //pattern parameters
    uint16_t _iter;
    uint8_t _step;
    uint8_t _res;
    uint8_t _step_cnt;
    uint16_t* _pattern_color;
    uint16_t* _pattern_time;
    uint32_t _prev_time;
    float* _brightness;
    void set_rgb(uint16_t red, uint16_t green, uint16_t blue);
    //RGB values corresponding to|  OFF  |   RED  | ORANGE  |  AMBER  |  YELLOW | GREEN  |  BLUE  | PURPLE  |  WHITE   |
    uint8_t preset_color[9][3] = {{0,0,0},{255,0,0},{255,128,0},{255,192,0},{255,255,0},{0,255,0},{0,0,255},{128,0,128},{255,255,255}};

public:
    DycoLEDDriver();
    void set_solid_color(uint8_t color);
    bool pop_data();
    void reset();
    //pattern functions
    void pattern_step();
    void set_pattern(uint16_t color_series[],float bright_series[],uint16_t time_series[],uint8_t res, uint8_t step_cnt);
};

//DycoLEDStrip: this class is used for controling each and every
//LED which are a part of LED Strip to create a pattern.
class DycoLEDStripDriver
{
private:
    uint32_t clk_pulse_count;
    uint8_t _length;
    bool _init;
    bool _commcomp;
    bool clk_pin;
    uint16_t _strip_cnt;
    uint16_t cntr;
    DycoLEDDriver* _led;

public:
    DycoLEDStripDriver();
    void generate_beat_pattern();
    bool update();
    void set_solid_color(uint8_t led_num, uint8_t color);
    void set_pattern(uint16_t led_num,uint16_t color_series[],float bright_series[],uint16_t time_series[],uint8_t res, uint8_t step_cnt);
    void init(uint16_t length);
};

struct led_pattern
{
    uint16_t color[MAX_PATTERN_STEPS];
    uint16_t time[MAX_PATTERN_STEPS];
    float brightness[MAX_PATTERN_STEPS];
    uint8_t res;
    uint8_t len;
};

class DycoLED: public NotifyDevice
{
public:
    // init - initialised the LED
    bool init();
    enum led_type{
        STATUS_LED,
        STROBE_LED
    };
    enum pattern {
        NOTIFY_INITIALISING,
        NOTIFY_SAV_TRIM_ESC_CAL,
        NOTIFY_FS_RAD_BATT,
        NOTIFY_FS_GPS,
        NOTIFY_BARO_GLITCH,
        NOTIFY_EKF_BAD,
        NOTIFY_ARMED_GPS,
        NOTIFY_ARMED_NOGPS,
        NOTIFY_PREARM_CHECK,
        NOTIFY_DISARMED_GPS,
        NOTIFY_DISARMED_NOGPS,
        NOTIFY_SAFE_STROBE,
        NOTIFY_FAIL_STROBE,
        NOTIFY_NEUTRAL_STROBE
    };
    // update - updates led according to timed_updated.  Should be
    // called at 50Hz
    void update();
    enum colors {
        BLACK,
        RED,
        ORANGE,
        AMBER,
        YELLOW,
        GREEN,
        BLUE,
        PURPLE,
        WHITE
    };
protected:
    DycoLEDStripDriver _ledstrip;
    void set_preset_pattern(uint16_t led,uint8_t patt);
    static led_pattern preset_pattern[14];
};

