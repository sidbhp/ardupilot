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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_MicroStrain.h"

class AP_MicroStrain_Backend
{
public:
    // constructor. This incorporates initialisation as well.
    AP_MicroStrain_Backend(AP_MicroStrain &_ms, uint8_t instance, AP_MicroStrain::MicroStrain_State &_state) :
    ms(_ms),
    state(_state) {}
    // we declare a virtual destructor so that MicroStrain drivers can
    // override with a custom destructor if need be
    virtual ~AP_MicroStrain_Backend(void) {}

    // update the state structure
    virtual void update() = 0;

    virtual bool get_accel_health() = 0;

    virtual bool healthy() = 0;

    virtual float get_delta_time() = 0;
    virtual float get_delta_velocity_dt() = 0;
    
    virtual bool  get_delta_angle(Vector3f &dangle) = 0;

    virtual bool get_delta_velocity(Vector3f &dangle) = 0;

    virtual Vector3f get_accel() = 0;
    virtual Vector3f get_gyro() = 0;

    virtual void read_data() = 0;
    virtual bool new_data() = 0;
    
private:
    AP_MicroStrain &ms;
    AP_MicroStrain::MicroStrain_State &state;
};