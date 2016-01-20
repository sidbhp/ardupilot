// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_RANGEFINDER_HIL_H__
#define __AP_RANGEFINDER_HIL_H__

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

class AP_RangeFinder_HIL : public AP_RangeFinder_Backend
{

public:
	AP_RangeFinder_HIL(RangeFinder &_ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state) :
	AP_RangeFinder_Backend(_ranger, instance, _state) {}
    // update state
    void update(void) {}

};
#endif  // __AP_RANGEFINDER_HIL_H__
