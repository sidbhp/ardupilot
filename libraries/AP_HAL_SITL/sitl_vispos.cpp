/*
  SITL handling

  This simulates a vispos sensor

  Siddharth Bharat Purohit
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_HAL_SITL.h"

using namespace HALSITL;

extern const AP_HAL::HAL& hal;

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <cmath>
#include <AP_VisPos/AP_VisPos.h>
void SITL_State::_update_vispos(void)
{
	Vector3f gyro;
    static uint32_t last_flow_ms;

    if (!_vispos ||
            !_sitl->vispos_enable) {
        return;
    }

    // update at the requested rate
    uint32_t now = AP_HAL::millis();
    if (now - last_flow_ms < 1000*(1.0f/_sitl->vispos_rate)) {
        return;
    }
    last_flow_ms = now;
    Vector3f lpos;
    Quaternion Qef2bf;
    struct Location location;
    location.lat = _sitl->state.latitude*1.0e7;
    location.lng = _sitl->state.longitude*1.0e7;
    Vector2f posne(location_diff(sitl_model->get_home(), location));
    lpos(posne.x, posne.y, _sitl->state.altitude);
    Qef2bf.from_euler(radians(_sitl->state.rollDeg),
                      radians(_sitl->state.pitchDeg),
                      radians(_sitl->state.yawDeg));
    Qef2bf.earth_to_body(lpos);
    _vispos->setHIL(lpos, now);
}

#endif