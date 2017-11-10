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


#include <AP_HAL/AP_HAL.h>

#include "DycoLED.h"
#include "AP_Notify.h"
#include <AP_GPS/AP_GPS.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <errno.h>
#include <unistd.h>
#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

extern const AP_HAL::HAL& hal;

bool DycoLED::init()
{
    if(_num_leds < MAX_NUM_LEDS) {
        _ledstrip.init(_num_leds);
        return true;
    }
    return false;
}

// update - updates led according to timed_updated.  Should be called
// at 50Hz
void DycoLED::update()
{
    if(_override_set) {
        return;
    }
    // Do Strobe on LED 3 and 4 
    if (AP_Notify::flags.initialising) {
        _ledstrip.set_led_file("Initialising.ledbin");
        return;                  // exit so no other status modify this pattern
    }
    if (AP_Notify::flags.save_trim || AP_Notify::flags.esc_calibration){
        _ledstrip.set_led_file("Breathing_orange.ledbin");
        return;
    }
    if(AP_Notify::flags.failsafe_radio || AP_Notify::flags.failsafe_battery){
        _ledstrip.set_led_file("Breathing_orange.ledbin");
        return;
    }

    if(AP_Notify::flags.ekf_bad){
        _ledstrip.set_led_file("Breathing_orange.ledbin");
        return;
    }
    
    if (AP_Notify::flags.armed) {
        if (AP_Notify::flags.gps_status >= AP_GPS::GPS_OK_FIX_3D_DGPS && AP_Notify::flags.gps_fusion) {
            _ledstrip.set_led_file("Solid_green.ledbin");
        } else{
            _ledstrip.set_led_file("Solid_blue.ledbin");
        }
    } else{
        if (!AP_Notify::flags.pre_arm_check) {
            _ledstrip.set_led_file("Breathing_orange.ledbin");
        } else{
            if (AP_Notify::flags.gps_status >= AP_GPS::GPS_OK_FIX_3D_DGPS && AP_Notify::flags.gps_fusion) {
                _ledstrip.set_led_file("Breathing_green.ledbin");
            } else{
                _ledstrip.set_led_file("Breathing_blue.ledbin");
            }
        }
    }
}

void DycoLED::handle_set_led_pattern_cmd(float led_num, float pattern_id)
{
    char name[22];
    sprintf(name, "LEDPattern%d.ledbin", (uint32_t)pattern_id);
    _ledstrip.set_led_file((const char*)name);
    _override_set = true;
}

DycoLEDDriver::DycoLEDDriver()
{    
    data_buf =0x1000000UL;
    data_hold = 0x1000000UL;                      //set the initial color of LED as BLACK/OFF 
    _prev_beat = 0;
}

void DycoLEDDriver::reset()
{
    data_buf = data_hold;                   //load the latest rgb data sequence for next pop cycle
}

void DycoLEDDriver::set_rgb(uint16_t red, uint16_t green, uint16_t blue)
{
    data_hold = red << 8 | green | blue << 16 | 1U<<24;     //1 set_bit(MSB),8 bits Red,8 bits Blue,8 bits Green intensities
                                                            //in the same sequence
}

bool DycoLEDDriver::pop_data(){

    data_buf = data_buf << 1;
    return (bool)(data_buf & 0x1000000);
}

//LED Strip Driver Functions
DycoLEDStripDriver::DycoLEDStripDriver()
{
    _init = false;
    _strip_cnt = 0;
    tx_complete = true;
    cntr = 0;
    _beat = 0;
    _restart_beat_counter = false;
    _next_time = 0;
    _fd = -1;
    strcpy(_filename, "Breathing_blue.ledbin");
}

void DycoLEDStripDriver::init(uint16_t length)
{
    _length = length + 1;
    _led = new DycoLEDDriver[_length];
    hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&DycoLEDStripDriver::generate_beat_pattern, void));
    // Setup GPIO to do dma supported bitbanging
    hal.gpio->pinMode(DYCODOUT,HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(DYCOCLK,HAL_GPIO_OUTPUT);
    hal.gpio->write(DYCOCLK,0);                     // both Clock pin and data pin at low state
    hal.gpio->write(DYCODOUT,0);
    hal.gpio->setup_dma_bitbang(1, 0, 2*(75+25*(_length-1)) + 1); //Run @ ~2.5 MHz clock
    _init = true;
}


//function for data transfer to dycoled and update with latest rgb values
bool DycoLEDStripDriver::update()
{
    if(!_init){
        return true;
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

bool DycoLEDStripDriver::time_to_send()
{
    if(_next_time*10 == _beat) {
        _beat++;
        return true;
    }

    _beat++;
    return false;
}

void DycoLEDStripDriver::set_led_file(const char filename[])
{
    if(strcmp(filename,_filename) == 0) {
        return;
    }
    strcpy(_filename, filename);
    if(_fd != -1) {
        ::close(_fd);
    }
    _fd = -1;
}

bool DycoLEDStripDriver::populate_next_state()
{
    if(_fd == -1) {
        char filepath[50];

        sprintf(filepath, "%s/%s", HAL_BOARD_SDCARD_PROFILED_DIR, _filename);
        printf("Trying profiled file %s\n", filepath);
        _fd = ::open(filepath, O_RDONLY);
        if(_fd == -1) {
            //fallback to default
            sprintf(filepath, "%s/%s", HAL_BOARD_DEFAULT_PROFILED_DIR, _filename);
            printf("Trying profiled file %s\n", filepath);
            _fd = ::open(filepath, O_RDONLY);
        }
        if (_fd == -1) {
            printf("Open %s/%s failed - %s\n",
                                HAL_BOARD_DEFAULT_PROFILED_DIR, _filename, strerror(errno));
            return false;
        }
        ::lseek(_fd, 0, SEEK_SET);
        _next_time = 0;
        _beat = 0;
    }
    uint8_t data[6] = {0};
    uint16_t prev_time = _next_time;
    uint16_t num_runs = 0;
    while(num_runs < MAX_NUM_LEDS) {
        num_runs++;
        //read in 6 bytes packet
        uint8_t size = ::read(_fd, data, 6);
        if(size < 6) {
            //we have reached the end
            ::lseek(_fd, 0, SEEK_SET);
            size = ::read(_fd, data, 6);
            if(size < 6) { //possible invalid file
                return false;
            }
        }
        _next_time = (uint16_t(data[0]) << 8) + data[1];
        if(_next_time != prev_time) {
            //move back for next update and return
            lseek(_fd, -6, SEEK_CUR);
            if(_next_time < prev_time) {
                _beat = 0;
                prev_time = _next_time;
                continue;   // we have looped, so start with first step immediately
            }
            return true;
        }
        if(data[2] < _length) {
            _led[data[2]].set_rgb(data[3], data[4], data[5]);
        } else {
            return false;   //Bad LED file do nothing
        }
    }
    return false;
}

void DycoLEDStripDriver::generate_beat_pattern()
{
    if(!_init){
        return;
    }
    if(!time_to_send()) {
        return;
    }
    //read next states of led
    if(!populate_next_state()) {
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
