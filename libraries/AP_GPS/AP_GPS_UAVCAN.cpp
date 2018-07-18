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
#include "AP_GPS_UAVCAN.h"
#include <stdint.h>

#if HAL_WITH_UAVCAN
#include <AP_UAVCAN/AP_UAVCAN.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

extern const AP_HAL::HAL& hal;

#define debug_gps_uavcan(level, fmt, args...) do { if ((level) <= AP_BoardConfig_CAN::get_can_debug()) { printf(fmt, ##args); }} while (0)

//Static Methods for UAVCAN GPS module detection, msg handling and registration
void AP_GPS_UAVCAN::uavcan_gps_fix_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Fix> &msg, uint8_t manager)
{
    uint8_t driver_id = AP::gps().get_uavcan_backend(msg.getSrcNodeID(), manager);

    if (driver_id != UINT8_MAX) {
        ((AP_GPS_UAVCAN*)(&AP::gps().drivers[driver_id]))->handle_fix_msg(msg);
    }
}

void AP_GPS_UAVCAN::uavcan_gps_aux_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Auxiliary> &msg, uint8_t manager)
{
    uint8_t driver_id = AP::gps().get_uavcan_backend(msg.getSrcNodeID(), manager);

    if (driver_id != UINT8_MAX) {
        ((AP_GPS_UAVCAN*)(&AP::gps().drivers[driver_id]))->handle_aux_msg(msg);
    }
}

void AP_GPS_UAVCAN::subscribe_gps_uavcan_messages()
{
    for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
        AP_UAVCAN *ap_uavcan = AP_UAVCAN::get_uavcan(i);
        if (ap_uavcan == nullptr) {
            continue;
        }
        if (ap_uavcan->get_uavcan_id() > 2) {
            continue;
        }
        auto* node = ap_uavcan->get_node();
        uavcan::Subscriber<uavcan::equipment::gnss::Fix> *gnss_fix;
        gnss_fix = new uavcan::Subscriber<uavcan::equipment::gnss::Fix>(*node);

        const int gnss_fix_start_res = gnss_fix->start(uavcan_gps_fix_cb(ap_uavcan->get_uavcan_id()));
        if (gnss_fix_start_res < 0) {
            AP_HAL::panic("UAVCAN GNSS subscriber start problem\n\r");
            return;
        }

        uavcan::Subscriber<uavcan::equipment::gnss::Auxiliary> *gnss_aux;
        gnss_aux = new uavcan::Subscriber<uavcan::equipment::gnss::Auxiliary>(*node);
        const int gnss_aux_start_res = gnss_aux->start(uavcan_gps_aux_cb(ap_uavcan->get_uavcan_id()));
        if (gnss_aux_start_res < 0) {
            AP_HAL::panic("UAVCAN GNSS Aux subscriber start problem\n\r");
            return;
        }
    }
}

//Member Methods
AP_GPS_UAVCAN::AP_GPS_UAVCAN(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port)
{}

void AP_GPS_UAVCAN::set_uavcan_manager(uint8_t mgr)
{
    _manager = mgr;
}

// Consume new data and mark it received
bool AP_GPS_UAVCAN::read(void)
{
    return _new_data;
}

void AP_GPS_UAVCAN::handle_fix_msg(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Fix> &msg)
{
    bool process = false;

    if (msg.status == uavcan::equipment::gnss::Fix::STATUS_NO_FIX) {
        state.status = AP_GPS::GPS_Status::NO_FIX;
    } else {
        if (msg.status == uavcan::equipment::gnss::Fix::STATUS_TIME_ONLY) {
            state.status = AP_GPS::GPS_Status::NO_FIX;
        } else if (msg.status == uavcan::equipment::gnss::Fix::STATUS_2D_FIX) {
            state.status = AP_GPS::GPS_Status::GPS_OK_FIX_2D;
            process = true;
        } else if (msg.status == uavcan::equipment::gnss::Fix::STATUS_3D_FIX) {
            state.status = AP_GPS::GPS_Status::GPS_OK_FIX_3D;
            process = true;
        }

        if (msg.gnss_time_standard == uavcan::equipment::gnss::Fix::GNSS_TIME_STANDARD_UTC) {
            uint64_t epoch_ms = uavcan::UtcTime(msg.gnss_timestamp).toUSec();
            epoch_ms /= 1000;
            uint64_t gps_ms = epoch_ms - UNIX_OFFSET_MSEC;
            state.time_week = (uint16_t)(gps_ms / AP_MSEC_PER_WEEK);
            state.time_week_ms = (uint32_t)(gps_ms - (state.time_week) * AP_MSEC_PER_WEEK);
        }
    }

    if (process) {
        Location loc = { };
        loc.lat = msg.latitude_deg_1e8 / 10;
        loc.lng = msg.longitude_deg_1e8 / 10;
        loc.alt = msg.height_msl_mm / 10;
        state.location = loc;
        state.location.options = 0;

        if (!uavcan::isNaN(msg.ned_velocity[0])) {
            Vector3f vel(msg.ned_velocity[0], msg.ned_velocity[1], msg.ned_velocity[2]);
            state.velocity = vel;
            state.ground_speed = norm(vel.x, vel.y);
            state.ground_course = wrap_360(degrees(atan2f(vel.y, vel.x)));
            state.have_vertical_velocity = true;
        } else {
            state.have_vertical_velocity = false;
        }

        float pos_cov[9];
        msg.position_covariance.unpackSquareMatrix(pos_cov);
        if (!uavcan::isNaN(pos_cov[8])) {
            if (pos_cov[8] > 0) {
                state.vertical_accuracy = sqrtf(pos_cov[8]);
                state.have_vertical_accuracy = true;
            } else {
                state.have_vertical_accuracy = false;
            }
        } else {
            state.have_vertical_accuracy = false;
        }

        const float horizontal_pos_variance = MAX(pos_cov[0], pos_cov[4]);
        if (!uavcan::isNaN(horizontal_pos_variance)) {
            if (horizontal_pos_variance > 0) {
                state.horizontal_accuracy = sqrtf(horizontal_pos_variance);
                state.have_horizontal_accuracy = true;
            } else {
                state.have_horizontal_accuracy = false;
            }
        } else {
            state.have_horizontal_accuracy = false;
        }

        float vel_cov[9];
        msg.velocity_covariance.unpackSquareMatrix(vel_cov);
        if (!uavcan::isNaN(vel_cov[0])) {
            state.speed_accuracy = sqrtf((vel_cov[0] + vel_cov[4] + vel_cov[8]) / 3.0);
            state.have_speed_accuracy = true;
        } else {
            state.have_speed_accuracy = false;
        }

        state.num_sats = msg.sats_used;
    } else {
        state.have_vertical_velocity = false;
        state.have_vertical_accuracy = false;
        state.have_horizontal_accuracy = false;
        state.have_speed_accuracy = false;
        state.num_sats = 0;
    }

    state.last_gps_time_ms = AP_HAL::millis();

    _new_data = true;
}

void AP_GPS_UAVCAN::handle_aux_msg(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Auxiliary> &msg)
{
    if (!uavcan::isNaN(msg.hdop)) {
        state.hdop = msg.hdop * 100.0;
    }

    if (!uavcan::isNaN(msg.vdop)) {
        state.vdop = msg.vdop * 100.0;
    }
}

#endif // HAL_WITH_UAVCAN
