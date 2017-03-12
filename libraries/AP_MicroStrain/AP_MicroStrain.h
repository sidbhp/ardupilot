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
#include <AP_SerialManager/AP_SerialManager.h>

#define MICROSTRAIN_MAX_INSTANCES 1

class AP_MicroStrain_Backend;

class AP_MicroStrain {

public:
    friend class AP_MicroStrain_Backend;
    AP_MicroStrain(AP_SerialManager &_serial_manager);

    struct MicroStrain_State {
        uint8_t instance;
        uint16_t id;
    };

    void init(void);

    void update(void);

    bool get_accel_health();

    bool healthy();

    float get_delta_time();
    float get_delta_velocity_dt();
    
    bool  get_delta_angle(Vector3f &dangle);

    bool get_delta_velocity(Vector3f &dangle);

    Vector3f get_accel();
    Vector3f get_gyro();
    void wait_for_sample(void);

private:
    AP_MicroStrain_Backend *drivers[MICROSTRAIN_MAX_INSTANCES];
    MicroStrain_State state[MICROSTRAIN_MAX_INSTANCES];
    AP_SerialManager &serial_manager;

    bool _have_sample;
    uint64_t _last_sample_usec, _next_sample_usec, _delta_time = 0;
    uint8_t num_instances;

    void detect_instance(uint8_t instance);
    void update_instance(uint8_t instance);

    void _add_backend(AP_MicroStrain_Backend *driver);
};