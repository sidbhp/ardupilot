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

#include "AP_MicroStrain.h"
#include "AP_MicroStrain_3DMGX4.h"
#include <stdio.h>
extern const AP_HAL::HAL& hal;
#define MS_TIMING_DEBUG
#ifdef MS_TIMING_DEBUG
#include <stdio.h>
#define timing_printf(fmt, args...)      do { printf("[timing] " fmt, ##args); } while(0)
#else
#define timing_printf(fmt, args...)
#endif
AP_MicroStrain::AP_MicroStrain(AP_SerialManager &_serial_manager) :
    serial_manager(_serial_manager),
    num_instances(0)
{
    // init state and drivers
    memset(state,0,sizeof(state));
    for (uint8_t i=0; i<MICROSTRAIN_MAX_INSTANCES; i++) {
        drivers[i] = nullptr;
    }
}


void AP_MicroStrain::init(void)
{
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }

    for (uint8_t i=0; i < MICROSTRAIN_MAX_INSTANCES; i++) {
        detect_instance(i);
        if (drivers[i] != nullptr) {
            hal.console->printf("MicroStrain initialised\n");
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = i+1;
        } else {
            hal.console->printf("AP_MicroStrain: Failed to Initialize MicroStrain...");
        }
    }

    _delta_time = 0;
    _next_sample_usec = 0;
    _last_sample_usec = 0;
    _have_sample = false;
}


void AP_MicroStrain::wait_for_sample(void)
{
    if (drivers[0] == nullptr) {
        return;
    }

    if (_have_sample) {
        // the user has called wait_for_sample() again without
        // consuming the sample with update()
        return;
    }

    uint64_t now = AP_HAL::micros();
    const uint64_t _sample_period_usec = 20000;
    if (_next_sample_usec == 0 && _delta_time <= 0) {
        // this is the first call to wait_for_sample()
        _last_sample_usec = now - _sample_period_usec;
        _next_sample_usec = now + _sample_period_usec;
        goto check_sample;
    }

    // see how long it is till the next sample is due
    if (_next_sample_usec - now <=_sample_period_usec) {
        // we're ahead on time, schedule next sample at expected period
        uint32_t wait_usec = _next_sample_usec - now;
        hal.scheduler->delay_microseconds_boost(wait_usec);
        uint32_t now2 = AP_HAL::micros();
        if (now2+100 < _next_sample_usec) {
            timing_printf("shortsleep %u\n", (unsigned)(_next_sample_usec-now2));
        }
        if (now2 > _next_sample_usec+400) {
            timing_printf("longsleep %u wait_usec=%u\n",
                          (unsigned)(now2-_next_sample_usec),
                          (unsigned)wait_usec);
        }
        _next_sample_usec += _sample_period_usec;
    } else if (now - _next_sample_usec < _sample_period_usec/8) {
        // we've overshot, but only by a small amount, keep on
        // schedule with no delay
        timing_printf("overshoot1 %u\n", (unsigned)(now-_next_sample_usec));
        _next_sample_usec += _sample_period_usec;
    } else {
        // we've overshot by a larger amount, re-zero scheduling with
        // no delay
        timing_printf("overshoot2 %u\n", (unsigned)(now-_next_sample_usec));
        _next_sample_usec = now + _sample_period_usec;
    }

check_sample:
    // we also wait for at least one backend to have a sample of both
    // accel and gyro. This normally completes immediately.
    bool gyro_available = false;
    bool accel_available = false;
    while (true) {
        drivers[0]->read_data();
        if (drivers[0]->new_data()) {
            break;
        }

        hal.scheduler->delay_microseconds(500);
    }

    now = AP_HAL::micros();

    _delta_time = (now - _last_sample_usec) * 1.0e-6f;
    _last_sample_usec = now;

#if 1
    {
        static uint64_t delta_time_sum;
        static uint16_t counter;
        if (delta_time_sum == 0) {
            delta_time_sum = _sample_period_usec;
        }
        delta_time_sum += _delta_time * 1.0e6f;
        if (counter++ == 400) {
            counter = 0;
            hal.console->printf("now=%lu _delta_time_sum=%lu diff=%ld\n",
                                (unsigned long)now,
                                (unsigned long)delta_time_sum,
                                (long)(now - delta_time_sum));
        }
    }
#endif

    _have_sample = true;
}


/*
  update MicroStrain state for all instances.
 */
void AP_MicroStrain::update(void)
{
    if(drivers[0] == nullptr) {
        AP_HAL::panic("No MicroStrain instance!");
    }
    wait_for_sample();
    for (uint8_t i=0; i < num_instances; i++) {
        if (drivers[i] != nullptr) {
            drivers[i]->update();
            _have_sample = false;
        }
    }
}

/*
  detect if an instance of a microstrain is connected. 
 */
void AP_MicroStrain::detect_instance(uint8_t instance)
{
    if(AP_MicroStrain_3DMGX4::detect(*this, instance, state[instance], serial_manager)){
        state[instance].instance = instance;
        drivers[instance] = new AP_MicroStrain_3DMGX4(*this, instance, state[instance], serial_manager);
        if(drivers[instance] == nullptr) {
            AP_HAL::panic("Failed to allocate space to MicroStrain instance!");
        } else {
            printf("AP_MicroStrain: allocated space for instance %d\n", instance);
        }
        return;
    }
}

bool AP_MicroStrain::get_accel_health() 
{ 
    if (drivers[0] != nullptr) {
        return drivers[0]->get_accel_health();
    } else {
        return false;
    }
}

bool AP_MicroStrain::healthy()
{ 
    if (drivers[0] != nullptr) {
        return drivers[0]->healthy();
    } else {
        return false;
    }
}

float AP_MicroStrain::get_delta_time()
{ 
    if (drivers[0] != nullptr) {
        return drivers[0]->get_delta_time();
    } else {
        return 0.0f;
    }
}

float AP_MicroStrain::get_delta_velocity_dt()
{ 
    if (drivers[0] != nullptr) {
        return drivers[0]->get_delta_velocity_dt();
    } else {
        return 0.0f;
    }
}

bool  AP_MicroStrain::get_delta_angle(Vector3f &dangle)
{ 
    if (drivers[0] != nullptr) {
        return drivers[0]->get_delta_angle(dangle);
    } else {
        return false;
    }
}

bool AP_MicroStrain::get_delta_velocity(Vector3f &dvel)
{ 
    if (drivers[0] != nullptr) {
        return drivers[0]->get_delta_velocity(dvel);
    } else {
        return false;
    }
}

Vector3f AP_MicroStrain::get_accel()
{ 
    if (drivers[0] != nullptr) {
        return drivers[0]->get_accel();
    } else {
        return Vector3f(0.0f,0.0f,0.0f);
    }
}

Vector3f AP_MicroStrain::get_gyro()
{ 
    if (drivers[0] != nullptr) {
        return drivers[0]->get_gyro();
    } else {
        return Vector3f(0.0f,0.0f,0.0f);
    }
}