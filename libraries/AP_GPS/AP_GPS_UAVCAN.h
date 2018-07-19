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

//
//  UAVCAN GPS driver
//
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_GPS.h"
#include "GPS_Backend.h"
#include <uavcan/equipment/gnss/Fix.hpp>
#include <uavcan/equipment/gnss/Auxiliary.hpp>

class AP_GPS_UAVCAN : public AP_GPS_Backend {
public:
    AP_GPS_UAVCAN(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    bool read() override;
    void set_uavcan_manager(AP_UAVCAN* uavcan);
    AP_UAVCAN* get_uavcan_manager() { return _uavcan; }
    const char *name() const override { return "UAVCAN"; }
    static void subscribe_gps_uavcan_messages();

private:
    bool _new_data;
    uint8_t _manager;

    void handle_fix_msg(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Fix> &msg);
    void handle_aux_msg(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Auxiliary> &msg);
    AP_UAVCAN* _uavcan;

    //global callbacks
    typedef AP_UAVCAN::Callback<uavcan::equipment::gnss::Fix, AP_GPS_UAVCAN, AP_GPS> FixCb;
    typedef AP_UAVCAN::Callback<uavcan::equipment::gnss::Auxiliary, AP_GPS_UAVCAN, AP_GPS> AuxCb;
};
