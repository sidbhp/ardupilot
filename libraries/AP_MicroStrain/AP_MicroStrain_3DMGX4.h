// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
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

#include "AP_MicroStrain.h"
#include "AP_MicroStrain_Backend.h"

class AP_MicroStrain_3DMGX4 : public AP_MicroStrain_Backend {
public:
    // constructor
    AP_MicroStrain_3DMGX4(AP_MicroStrain &_ms, uint8_t instance, AP_MicroStrain::MicroStrain_State &_state,
                                   AP_SerialManager &serial_manager);

    void init(void);

    void update(void);
    bool get_accel_health() { return true; }

    bool healthy() { return true; }
    bool new_data() { return ((processed_data.fields & 0x3U) == 0x3U); }

    float get_delta_time() { return processed_data.delta_gyro_time; }
    float get_delta_velocity_dt() { return processed_data.delta_accel_time; }
    
    bool  get_delta_angle(Vector3f &dangle) 
    { 
        dangle = Vector3f(processed_data.delta_angle[0], processed_data.delta_angle[1], processed_data.delta_angle[2]); 
        return true;
    }

    bool get_delta_velocity(Vector3f &dvel)
    {
        dvel = Vector3f(processed_data.delta_velocity[0], processed_data.delta_velocity[1], processed_data.delta_velocity[2])*GRAVITY_MSS;
        return true;
    }

    Vector3f get_accel() { return Vector3f(processed_data.accel[0], processed_data.accel[1], processed_data.accel[2])*GRAVITY_MSS;}
    Vector3f get_gyro() { return Vector3f(processed_data.gyro[0], processed_data.gyro[1], processed_data.gyro[2]);}
    static bool detect(AP_MicroStrain &_ms, uint8_t instance, AP_MicroStrain::MicroStrain_State &_state ,AP_SerialManager &serial_manager);
    void read_data();
private:
    enum sync{
        SYNC1 = 0x75,
        SYNC2 = 0x65
    };

    enum command_class {
        COMMAND_CLASS_BASE   = 0x01,
        COMMAND_CLASS_3DM    = 0x0C,
        COMMAND_CLASS_FILTER = 0x0D
    };

    enum data_class {
        DATA_CLASS_IMU      = 0x80,
        DATA_CLASS_FILTER   = 0x82
    };

    enum function_apply {
        FUNCTION_APPLY   =    0x01
    };

    enum selector {
        SELECTOR_IMU     =    0x01,
        SELECTOR_FILTER  =    0x03
    };
    enum device_cmd{
        //  base commands
        DEVICE_PING      =    0x01,
        DEVICE_IDLE      =    0x02,
        DEVICE_RESUME    =    0x06 
    };
    enum filter_3dm_cmd {
        //  3DM and FILTER commands
        COMMAND_GET_DEVICE_INFO       = 0x03,
        COMMAND_GET_IMU_BASE_RATE     = 0x06,
        COMMAND_GET_FILTER_BASE_RATE  = 0x0B,
        COMMAND_IMU_MESSAGE_FORMAT    = 0x08,
        COMAMND_FILTER_MESSAGE_FORMAT = 0x0A,
        COMMAND_ENABLE_DATA_STREAM    = 0x11,
        COMMAND_FILTER_CONTROL_FLAGS  = 0x14,
        COMMAND_UART_BAUD_RATE        = 0x40,
        COMMAND_SET_HARD_IRON         = 0x3A,
        COMMAND_SET_SOFT_IRON         = 0x3B,
        COMMAND_ENABLE_MEASUREMENTS   = 0x41,
        COMMAND_DEVICE_STATUS         = 0x64
    };
    enum supported_fields {
        //  supported fields
        FIELD_QUATERNION        = 0x03,
        FIELD_ACCELEROMETER     = 0x04,
        FIELD_GYROSCOPE         = 0x05,
        FIELD_GYRO_BIAS         = 0x06,
        FIELD_MAGNETOMETER      = 0x06,
        FIELD_ANGLE_UNCERTAINTY = 0x0A,
        FIELD_BIAS_UNCERTAINTY  = 0x0B,
        FIELD_BAROMETER         = 0x17,
        FIELD_DEVICE_INFO       = 0x81,
        FIELD_IMU_BASERATE      = 0x83,
        FIELD_FILTER_BASERATE   = 0x8A,
        FIELD_STATUS_REPORT     = 0x90,
        FIELD_ACK_OR_NACK       = 0xF1
    };

    struct ms_imu_data {
        uint8_t fields;
        float accel[3];
        float delta_velocity[3];
        float delta_accel_time;
        float gyro[3];
        float delta_angle[3];
        float delta_gyro_time;
        float mag[3];
        float press;
        bool valid;
    }processed_data;

    uint64_t last_gyro_time;
    uint64_t last_accel_time;

    struct parse_state {
        uint8_t step;
        uint8_t desc;
        uint8_t length;
        uint8_t *dat;
        uint8_t dat_ptr;
        uint16_t rcv_checksum;
        uint16_t checksum;
    }_global_parse_state;

    void calc_checksum();
    void _parse_char(uint8_t c);
    void handle_packet();
    void handle_imu_packet(parse_state &imu_parse_state);
    void extract_floats(float fout[], uint8_t cin[], uint8_t num);

    AP_HAL::UARTDriver *uart = nullptr;
};