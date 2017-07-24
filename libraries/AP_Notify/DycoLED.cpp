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
/*
DycoLED Driver Usage Guide
Five Parameters to set pattern:
* Color Series: color values after completion or at start of each step of pattern
* Time Series: time between color transitions.
* Brightness series: Brightness of the color at the same index
* Resolution: No. of steps in which color transitions will occur.
* Steps: Total no. of color transitions
Example:
with color series as {BLUE,RED,GREEN}
Time Series as {1000,1000,1000} --> first time is transition time in ms for BLUE -> RED
                                --> second time is for RED -> GREEN
                                --> Third time is for GREEN -> BLUE
Brightness series as {0.5,0.5,1.0}
Resolution as 10
Steps as 3
The loop of pattern will be:
first the LED will be set to HALF BRIGHT BLUE color. After
every 1000/10 milliseconds BLUE color will reduce at such
a rate that it diminishes at 1000 ms after the start of 
pattern and RED component will increase such that it increases
to HALF_BRIGHT at same time as BLUE goes to zero and similar
process will continue with RED diminishing and GREEN increases
to FULL brightness (1.0) thus completing 3 basic transitions
of loop i.e. from BLUE to RED to GREEN. The loop will until
another pattern or solid color is set.
*/


#include <AP_HAL/AP_HAL.h>

#include "DycoLED.h"
#include "AP_Notify.h"
#include <AP_GPS/AP_GPS.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <errno.h>

extern const AP_HAL::HAL& hal;
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"

#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 

led_pattern DycoLED::preset_pattern[16]={{{BLUE,RED},{100,100},{1.0,1.0},10,2},                             //INITIALISING
                                       {{RED,BLUE,GREEN},{250,250,250},{1.0,1.0,1.0},15,3},                 //SAV_TRIM_ESC_CAL
                                       {{BLACK,YELLOW},{250,250},{1.0,1.0},15,2},                           //FS_RAD_BATT
                                       {{BLUE,YELLOW},{250,250},{1.0,1.0},15,2},                            //FS_GPS
                                       {{PURPLE,YELLOW},{250,250},{1.0,1.0},15,2},                          //BARO_GLITCH
                                       {{RED,YELLOW},{250,250},{1.0,1.0},15,2},                             //EKF_BAD
                                       {{GREEN,GREEN},{1000,1000},{1.0,1.0},1,2},                           //ARMED_GPS
                                       {{BLUE,BLUE},{1000,1000},{1.0,1.0},1,2},                             //ARMED_NOGPS
                                       {{BLACK,YELLOW},{500,500},{1.0,1.0},15,2},                           //PREARM_CHECK
                                       {{BLACK,GREEN},{500,500},{1.0,1.0},15,2},                            //DISARMED_GPS
                                       {{BLACK,BLUE},{100,100},{1.0,1.0},31,2},                             //DISARMED_NOGPS
                                       {{GREEN,BLACK,GREEN,BLACK},{50,25,50,500},{1.0,1.0,1.0,1.0},1,4}, //SAFE_STROBE
                                       {{RED,BLACK,RED,BLACK},{50,25,50,500},{1.0,1.0,1.0,1.0},1,4},     //FAIL_STROBE
                                       {{BLUE,BLACK,BLUE,BLACK},{50,25,50,500},{1.0,1.0,1.0,1.0},1,4},  //NEUTRAL_STROBE
                                       {{WHITE,BLACK,WHITE,BLACK},{50,25,50,1125},{1.0,1.0,1.0,1.0},1,4},  //NEUTRAL_STROBE
                                       {{BLACK,WHITE,BLACK,WHITE,BLACK},{625,50,25,50,500},{1.0,1.0,1.0,1.0,1.0},1,5}};  //NEUTRAL_STROBE


bool DycoLED::init()
{
    _ledstrip.init(MAX_NUM_LEDS);
    return true;
}

void DycoLED::set_preset_pattern(uint16_t led,uint8_t patt)
{
      _ledstrip.set_pattern(led,preset_pattern[patt].color,preset_pattern[patt].brightness,preset_pattern[patt].time,
                                 preset_pattern[patt].res,preset_pattern[patt].len);
}
 
// update - updates led according to timed_updated.  Should be called
// at 50Hz
void DycoLED::update()
{
    // Do Strobe on LED 3 and 4
    set_preset_pattern(2, 14);
    set_preset_pattern(3, 15);
    
    if (AP_Notify::flags.initialising) {
        set_preset_pattern(STATUS_LED,NOTIFY_INITIALISING);
        set_preset_pattern(STROBE_LED,NOTIFY_NEUTRAL_STROBE);
        return;                  // exit so no other status modify this pattern
    }
    if (AP_Notify::flags.save_trim || AP_Notify::flags.esc_calibration){
        set_preset_pattern(STATUS_LED,NOTIFY_SAV_TRIM_ESC_CAL);
        set_preset_pattern(STROBE_LED,NOTIFY_NEUTRAL_STROBE);
        return;
    }
    if(AP_Notify::flags.failsafe_radio || AP_Notify::flags.failsafe_battery){
        set_preset_pattern(STATUS_LED,NOTIFY_FS_RAD_BATT);
        set_preset_pattern(STROBE_LED,NOTIFY_FAIL_STROBE);
        return;
    }

    if(AP_Notify::flags.ekf_bad){
        set_preset_pattern(STATUS_LED,NOTIFY_EKF_BAD);
        set_preset_pattern(STROBE_LED,NOTIFY_FAIL_STROBE);
        return;
    }
    
    if (AP_Notify::flags.armed) {
        if (AP_Notify::flags.gps_status >= AP_GPS::GPS_OK_FIX_3D_DGPS && AP_Notify::flags.gps_fusion) {
            _ledstrip.set_solid_color(STATUS_LED,GREEN);
            set_preset_pattern(STROBE_LED,NOTIFY_SAFE_STROBE);
        } else{
            _ledstrip.set_solid_color(STATUS_LED,BLUE);
            set_preset_pattern(STROBE_LED,NOTIFY_NEUTRAL_STROBE);
        }
    } else{
        if (!AP_Notify::flags.pre_arm_check) {
            set_preset_pattern(STATUS_LED,NOTIFY_PREARM_CHECK);
            set_preset_pattern(STROBE_LED,NOTIFY_NEUTRAL_STROBE);
        } else{
            if (AP_Notify::flags.gps_status >= AP_GPS::GPS_OK_FIX_3D_DGPS && AP_Notify::flags.gps_fusion) {
                set_preset_pattern(STATUS_LED,NOTIFY_DISARMED_GPS);
                set_preset_pattern(STROBE_LED,NOTIFY_SAFE_STROBE);
            } else{
                set_preset_pattern(STATUS_LED,NOTIFY_DISARMED_NOGPS);
                set_preset_pattern(STROBE_LED,NOTIFY_SAFE_STROBE);
            }
        }
    }
}

//LED single module functions
DycoLEDDriver::DycoLEDDriver()
{    
    data_buf =0x1000000UL;
    clk_prev_time = 0;
    clk_pulse_count = 0;
    data_hold = 0x1000000UL;                      //set the initial color of LED as BLACK/OFF 
    _pattern_color = NULL;
    _brightness = NULL;
    _pattern_time = NULL;
    _prev_beat = 0;
}

void DycoLEDDriver::reset()
{
    data_buf = data_hold;                   //load the latest rgb data sequence for next pop cycle
}

void DycoLEDDriver::set_rgb(uint16_t red, uint16_t green, uint16_t blue)
{    
    clk_prev_time = 0;
    clk_pulse_count = 0;
    data_hold = red << 8 | green | blue << 16 | 1U<<24;     //1 set_bit(MSB),8 bits Red,8 bits Blue,8 bits Green intensities
                                                            //in the same sequence
}
void DycoLEDDriver::set_solid_color(uint8_t color)
{
    set_rgb(preset_color[color][0],preset_color[color][1],preset_color[color][2]);
    _res = 0;
}
bool DycoLEDDriver::pop_data(){

    data_buf = data_buf << 1;
    return (bool)(data_buf & 0x1000000);
}
/*
Five Parameters to set pattern:
Color Series: color values after completion or at start of each step of pattern
Time Series: time between color transitions.
Brightness series: Brightness of the color at the same index
Resolution: No. of steps in which color transitions will occur.
Steps: Total no. of color transitions
*/
void DycoLEDDriver::set_pattern(uint16_t color_series[],float bright_series[],
                          uint16_t time_series[],uint8_t res, uint8_t step_cnt)
{
    _pattern_color = color_series;
    _brightness = bright_series;
    _pattern_time = time_series;
    _res = res;
    _step_cnt = step_cnt;
}

void DycoLEDDriver::pattern_step(uint32_t beat)
{
    int16_t red_diff, blue_diff, green_diff;
    uint16_t red, blue, green;
    uint16_t step_size;
    uint8_t next_color,color;
    if(_res == 0){
        return;                 //pattern not set
    }
    if(_step >= _step_cnt){
        _step = 0;
        _iter = 0;
    }
    
    next_color = _pattern_color[(_step+1)%_step_cnt];
    color = _pattern_color[_step];
    step_size = _pattern_time[_step]/_res;

    if(((beat - _prev_beat) > step_size) && (_iter < _res)){
        _iter++;
        _prev_beat = beat;
    }
    //calculate difference between two consecutive colors b/w which transition is taking place
    red_diff = preset_color[next_color][0]*_brightness[(_step+1)%_step_cnt] - preset_color[color][0]*_brightness[_step];
    green_diff = preset_color[next_color][1]*_brightness[(_step+1)%_step_cnt] - preset_color[color][1]*_brightness[_step];
    blue_diff = preset_color[next_color][2]*_brightness[(_step+1)%_step_cnt] - preset_color[color][2]*_brightness[_step];

    //calculate rgb values at current stage/time
    red = preset_color[color][0]*_brightness[_step] + (red_diff*_iter)/_res;
    green = preset_color[color][1]*_brightness[_step] + (green_diff*_iter)/_res;
    blue = preset_color[color][2]*_brightness[_step] + (blue_diff*_iter)/_res;

    if(_iter >= _res){
        _step++;                //get ready for next step/transition
        _iter = 0;
    }
    set_rgb(red,green,blue);  
}

//LED Strip Driver Functions
DycoLEDStripDriver::DycoLEDStripDriver()
{
    _init = false;
    _strip_cnt = 0;
    tx_complete = true;
    cntr = 0;
    _beat = 0;
}

void DycoLEDStripDriver::init(uint16_t length)
{
    _led = new DycoLEDDriver[length];
    hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&DycoLEDStripDriver::generate_beat_pattern, void));
    // Setup GPIO to do dma supported bitbanging
    hal.gpio->pinMode(DYCODOUT,HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(DYCOCLK,HAL_GPIO_OUTPUT);
    hal.gpio->write(DYCOCLK,0);                     // both Clock pin and data pin at low state
    hal.gpio->write(DYCODOUT,0);
    hal.gpio->setup_dma_bitbang(1, 0, 2*(75+25*(length-1)) + 1); //Run @ ~2.5 MHz clock
    _length = length;
    _init = true;
}

void DycoLEDStripDriver::set_solid_color(uint8_t led_num, uint8_t color)
{
    _led[led_num].set_solid_color(color);
}

//function for data transfer to dycoled and update with latest rgb values
bool DycoLEDStripDriver::update()
{
    if(!_init){
        return false;
    }
    bool data;
    uint16_t len = _length;
    uint32_t total_clk_pulses = (75+25*(len-1));
    if(clk_pulse_count < total_clk_pulses){
        if(clk_pulse_count < 50) {
            //send 50 LOW bits to initiate data transfer
            hal.gpio->push_bitbang_state(DYCOCLK, 1);
            hal.gpio->push_bitbang_state(DYCODOUT, 0);
            hal.gpio->step_bitbang_state();
            hal.gpio->push_bitbang_state(DYCOCLK, 0);
            hal.gpio->push_bitbang_state(DYCODOUT, 0);
            hal.gpio->step_bitbang_state();
        } else if(clk_pulse_count >= 50 && clk_pulse_count % 25 == 0){
            //set bit HIGH to enable led
            hal.gpio->push_bitbang_state(DYCOCLK, 1);
            hal.gpio->push_bitbang_state(DYCODOUT, 1);
            hal.gpio->step_bitbang_state();
            hal.gpio->push_bitbang_state(DYCOCLK, 0);
            hal.gpio->push_bitbang_state(DYCODOUT, 0);
            hal.gpio->step_bitbang_state();
            _led[_strip_cnt].reset();
            _strip_cnt++;
        } else if(clk_pulse_count > (uint16_t)(50+25*(_strip_cnt-1)) && clk_pulse_count < (uint16_t)(75+25*(_strip_cnt-1))){
            cntr = _strip_cnt - 1;

            data = _led[_strip_cnt-1].pop_data();       //pop rbg data bits out of the led data buffer

            hal.gpio->push_bitbang_state(DYCOCLK, 1);
            hal.gpio->push_bitbang_state(DYCODOUT, data);
            hal.gpio->step_bitbang_state();
            hal.gpio->push_bitbang_state(DYCOCLK, 0);
            hal.gpio->push_bitbang_state(DYCODOUT, 0);
            hal.gpio->step_bitbang_state();
        }
        clk_pulse_count++;
    } else{
        //transfer complete, reset CLK and DATA pins and
        //initialise flags and counters for next data transfer cycle
        hal.gpio->push_bitbang_state(DYCODOUT, 0);
        hal.gpio->push_bitbang_state(DYCOCLK, 0);
        _strip_cnt = 0;
        hal.gpio->step_bitbang_state();
        clk_pulse_count = 0;
        for(int i=0; i<len; i++){
            _led[cntr].reset();             //prepare data buffer for next data transfer cycle
        }
        return true;
    }
    return false;
}

void DycoLEDStripDriver::set_pattern(uint16_t led_num,uint16_t color_series[],
                                          float bright_series[],uint16_t time_series[],
                                          uint8_t res, uint8_t step_cnt)
{
    _led[led_num].set_pattern(color_series,bright_series,time_series,res,step_cnt);
}

void DycoLEDStripDriver::generate_beat_pattern()
{
    if(tx_complete) {
        //increment beat value
        _beat++;
        // step only if tx_completed
        // we retry after timeout
        for(int i=0;i<_length;i++){
            _led[i].pattern_step(_beat);
        }
    } else if(_tx_timeout-- > 0) {
        return;
    }
    while(!update());
    hal.gpio->flush_bitbang_states(FUNCTOR_BIND_MEMBER(&DycoLEDStripDriver::tx_complete_callback, void));
    tx_complete = false;
    _tx_timeout = 5;    //timeout after 5ms
}

void DycoLEDStripDriver::tx_complete_callback()
{
    // set tx complete flag so we can begin next transmission
    tx_complete = true;
}
