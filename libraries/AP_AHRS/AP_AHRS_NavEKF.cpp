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

/*
 *  NavEKF based AHRS (Attitude Heading Reference System) interface for
 *  ArduPilot
 *
 */
#include <AP_HAL.h>
#include <AP_AHRS.h>
#include <AP_Vehicle.h>

#if AP_AHRS_NAVEKF_AVAILABLE

extern const AP_HAL::HAL& hal;

// return the smoothed gyro vector corrected for drift
const Vector3f &AP_AHRS_NavEKF::get_gyro(void) const
{
    if (!using_EKF()) {
        return AP_AHRS_DCM::get_gyro();
    }
    return _gyro_estimate;
}

const Matrix3f &AP_AHRS_NavEKF::get_dcm_matrix(void) const
{
    if (!using_EKF()) {
        return AP_AHRS_DCM::get_dcm_matrix();
    }
    return _dcm_matrix;
}

const Vector3f &AP_AHRS_NavEKF::get_gyro_drift(void) const
{
    if (!using_EKF()) {
        return AP_AHRS_DCM::get_gyro_drift();
    }
    return _gyro_bias;
}

// reset the current gyro drift estimate
//  should be called if gyro offsets are recalculated
void AP_AHRS_NavEKF::reset_gyro_drift(void)
{
    // update DCM
    AP_AHRS_DCM::reset_gyro_drift();

    // reset the EKF gyro bias states
    EKF.resetGyroBias();

    // zero gyro3 bias
    _gyro3_bias.zero();
}

void AP_AHRS_NavEKF::update(void)
{
    // update gyro 3 bias
    update_gyro3_bias();

    // we need to restore the old DCM attitude values as these are
    // used internally in DCM to calculate error values for gyro drift
    // correction
    roll = _dcm_attitude.x;
    pitch = _dcm_attitude.y;
    yaw = _dcm_attitude.z;
    update_cd_values();

    AP_AHRS_DCM::update();

    // keep DCM attitude available for get_secondary_attitude()
    _dcm_attitude(roll, pitch, yaw);

    if (!ekf_started) {
        // wait 1 second for DCM to output a valid tilt error estimate
        if (start_time_ms == 0) {
            start_time_ms = hal.scheduler->millis();
        }
        if (hal.scheduler->millis() - start_time_ms > startup_delay_ms) {
            ekf_started = EKF.InitialiseFilter();

            if (ekf_started) {
                lastEkfHealthyTime_ms = hal.scheduler->millis();
                lastEkfResetTime_ms = lastEkfHealthyTime_ms;
            }
        }
    }
    if (ekf_started) {
        EKF.UpdateFilter();
        // If EKF is started we switch away if it reports unhealthy. This could be due to bad
        // sensor data. If EKF reversion is inhibited, we only switch across if the EKF encounters
        // an internal processing error, but not for bad sensor data.
        uint8_t ekf_faults;
        EKF.getFilterFaults(ekf_faults);
        bool ekfHealthy = ekf_started && ((_ekf_use == EKF_USE_WITH_FALLBACK && EKF.healthy()) || (_ekf_use == EKF_USE_WITHOUT_FALLBACK && ekf_faults == 0));
        // Check if the EKF is healthy and reinitialise if unhealthy for 200 msec
        // Don't repeat until 1500 msec has lapsed to allow EKF time to restart
        // Don't do on the ground because the health criteria are tightened before the vehicle arms and we could end up with repeating resets
        // Use the vehicle arm status as a proxy for determining if we are on-ground or in-air
        if (ekfHealthy) {
            lastEkfHealthyTime_ms = hal.scheduler->millis();
        }
        if ((hal.scheduler->millis() - lastEkfHealthyTime_ms > 200) && (hal.scheduler->millis() - lastEkfResetTime_ms > 1500) && hal.util->get_soft_armed()) {
            bool restartSuccessful = EKF.InitialiseFilter();
            if (restartSuccessful) {
                hal.console->printf("EKF restarted\n");
                lastEkfHealthyTime_ms = hal.scheduler->millis();
                lastEkfResetTime_ms = lastEkfHealthyTime_ms;
             } else {
                // If the restart is unsuccessful, then indicate that the ekf has not started and bypass the update steps
                ekf_started = false;
                return;
             }
        }

        EKF.getRotationBodyToNED(_dcm_matrix);
        if (using_EKF()) {
            Vector3f eulers;
            EKF.getEulerAngles(eulers);
            roll  = eulers.x;
            pitch = eulers.y;
            yaw   = eulers.z;

            update_cd_values();
            update_trig();

            // keep _gyro_bias for get_gyro_drift()
            // filter with 5s time constant
            Vector3f ekf_gyro_bias;
            EKF.getGyroBias(ekf_gyro_bias);
            ekf_gyro_bias = -ekf_gyro_bias;
            _gyro_bias += (ekf_gyro_bias-_gyro_bias)*(0.0025f / (0.0025f + 5.0f));

            // calculate corrected gryo estimate for get_gyro()
            _gyro_estimate.zero();
            uint8_t healthy_count = 0;    
            for (uint8_t i=0; i<_ins.get_gyro_count(); i++) {
                if (_ins.get_gyro_health(i)) {
                    _gyro_estimate += _ins.get_gyro(i);
                    healthy_count++;
                }
            }
            if (healthy_count > 1) {
                _gyro_estimate /= healthy_count;
            }
            _gyro_estimate += _gyro_bias;

            float abias;
            EKF.getAccelZBias(abias);

            // update _accel_ef_ekf
            for (uint8_t i=0; i<_ins.get_accel_count(); i++) {
                Vector3f accel = _ins.get_accel(i);
                if (i==_ins.get_primary_accel()) {
                    accel.z -= abias;
                }
                if (_ins.get_accel_health(i)) {
                    _accel_ef_ekf[i] = _dcm_matrix * accel;
                }
            }
                _accel_ef_ekf_blended = _accel_ef_ekf[0];
        }
    }
}

Vector3f AP_AHRS_NavEKF::get_gyro_for_control() const
{
    if (_ins.get_gyro_health(2)) {
        return _ins.get_gyro(2)-_gyro3_bias;
    } else {
        return get_gyro();
    }
}

// update _gyro3_bias by comparing ins.get_gyro(2) with get_gyro
void AP_AHRS_NavEKF::update_gyro3_bias()
{
    float dt = 1.0f/_ins.get_sample_rate();

    static const float gyro3BiasFiltTC = 20.0f;

    if (!_ins.get_gyro_health(2)) {
        return;
    }

    // compute alpha
    float alpha = constrain_float(dt / (dt+gyro3BiasFiltTC),0.0f,1.0f);

    Vector3f gyro3_bias_unfiltered = _ins.get_gyro(2) - get_gyro();
    _gyro3_bias += (gyro3_bias_unfiltered-_gyro3_bias)*alpha;
}

// accelerometer values in the earth frame in m/s/s
const Vector3f &AP_AHRS_NavEKF::get_accel_ef(uint8_t i) const
{
    if(!using_EKF()) {
        return AP_AHRS_DCM::get_accel_ef(i);
    }
    return _accel_ef_ekf[i];
}

// blended accelerometer values in the earth frame in m/s/s
const Vector3f &AP_AHRS_NavEKF::get_accel_ef_blended(void) const
{
    if(!using_EKF()) {
        return AP_AHRS_DCM::get_accel_ef_blended();
    }
    return _accel_ef_ekf_blended;
}

void AP_AHRS_NavEKF::reset(bool recover_eulers)
{
    AP_AHRS_DCM::reset(recover_eulers);
    if (ekf_started) {
        ekf_started = EKF.InitialiseFilter();        
    }
}

// reset the current attitude, used on new IMU calibration
void AP_AHRS_NavEKF::reset_attitude(const float &_roll, const float &_pitch, const float &_yaw)
{
    AP_AHRS_DCM::reset_attitude(_roll, _pitch, _yaw);
    if (ekf_started) {
        ekf_started = EKF.InitialiseFilter();        
    }
}

// dead-reckoning support
bool AP_AHRS_NavEKF::get_position(struct Location &loc) const
{
    Vector3f ned_pos;
    if (using_EKF() && EKF.getLLH(loc) && EKF.getPosNED(ned_pos)) {
        // fixup altitude using relative position from AHRS home, not
        // EKF origin
        loc.alt = get_home().alt - ned_pos.z*100;
        return true;
    }
    return AP_AHRS_DCM::get_position(loc);
}

// status reporting of estimated errors
float AP_AHRS_NavEKF::get_error_rp(void) const
{
    return AP_AHRS_DCM::get_error_rp();
}

float AP_AHRS_NavEKF::get_error_yaw(void) const
{
    return AP_AHRS_DCM::get_error_yaw();
}

// return a wind estimation vector, in m/s
Vector3f AP_AHRS_NavEKF::wind_estimate(void)
{
    if (!using_EKF()) {
        // EKF does not estimate wind speed when there is no airspeed
        // sensor active
        return AP_AHRS_DCM::wind_estimate();
    }
    Vector3f wind;
    EKF.getWind(wind);
    return wind;
}

// return an airspeed estimate if available. return true
// if we have an estimate
bool AP_AHRS_NavEKF::airspeed_estimate(float *airspeed_ret) const
{
    return AP_AHRS_DCM::airspeed_estimate(airspeed_ret);
}

// true if compass is being used
bool AP_AHRS_NavEKF::use_compass(void)
{
    if (using_EKF()) {
        return EKF.use_compass();
    }
    return AP_AHRS_DCM::use_compass();
}


// return secondary attitude solution if available, as eulers in radians
bool AP_AHRS_NavEKF::get_secondary_attitude(Vector3f &eulers)
{
    if (using_EKF()) {
        // return DCM attitude
        eulers = _dcm_attitude;
        return true;
    }
    if (ekf_started) {
        // EKF is secondary
        EKF.getEulerAngles(eulers);
        return true;
    }
    // no secondary available
    return false;
}

// return secondary position solution if available
bool AP_AHRS_NavEKF::get_secondary_position(struct Location &loc)
{
    if (using_EKF()) {
        // return DCM position
        AP_AHRS_DCM::get_position(loc);
        return true;
    }    
    if (ekf_started) {
        // EKF is secondary
        EKF.getLLH(loc);
        return true;
    }
    // no secondary available
    return false;
}

// EKF has a better ground speed vector estimate
Vector2f AP_AHRS_NavEKF::groundspeed_vector(void)
{
    if (!using_EKF()) {
        return AP_AHRS_DCM::groundspeed_vector();
    }
    Vector3f vec;
    EKF.getVelNED(vec);
    return Vector2f(vec.x, vec.y);
}

void AP_AHRS_NavEKF::set_home(const Location &loc)
{
    AP_AHRS_DCM::set_home(loc);
}

// return true if inertial navigation is active
bool AP_AHRS_NavEKF::have_inertial_nav(void) const 
{
    return using_EKF();
}

// return a ground velocity in meters/second, North/East/Down
// order. Must only be called if have_inertial_nav() is true
bool AP_AHRS_NavEKF::get_velocity_NED(Vector3f &vec) const
{
    if (using_EKF()) {
        EKF.getVelNED(vec);
        return true;
    }
    return false;
}

// Get a derivative of the vertical position which is kinematically consistent with the vertical position is required by some control loops.
// This is different to the vertical velocity from the EKF which is not always consistent with the verical position due to the various errors that are being corrected for.
bool AP_AHRS_NavEKF::get_vert_pos_rate(float &velocity)
{
    EKF.getPosDownDerivative(velocity);
    return true;
}

// get latest height above ground level estimate in metres and a validity flag
bool AP_AHRS_NavEKF::get_hagl(float &height) const
{
    return EKF.getHAGL(height);
}

// return a relative ground position in meters/second, North/East/Down
// order. Must only be called if have_inertial_nav() is true
bool AP_AHRS_NavEKF::get_relative_position_NED(Vector3f &vec) const
{
    if (using_EKF()) {
        return EKF.getPosNED(vec);
    }
    return false;
}

bool AP_AHRS_NavEKF::using_EKF(void) const
{
    uint8_t ekf_faults;
    EKF.getFilterFaults(ekf_faults);
    // If EKF is started we switch away if it reports unhealthy. This could be due to bad
    // sensor data. If EKF reversion is inhibited, we only switch across if the EKF encounters
    // an internal processing error, but not for bad sensor data.
    // if EKF is unhealthy for longer than 200msec, we re-initiliase the filter
    bool ret = ekf_started && ((_ekf_use == EKF_USE_WITH_FALLBACK && EKF.healthy()) || (_ekf_use == EKF_USE_WITHOUT_FALLBACK && ekf_faults == 0));
    if (!ret) {
        return false;
    }
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane) || APM_BUILD_TYPE(APM_BUILD_APMrover2)
    nav_filter_status filt_state;
    EKF.getFilterStatus(filt_state);
    if (hal.util->get_soft_armed() && filt_state.flags.const_pos_mode) {
        return false;
    }
    if (!filt_state.flags.attitude ||
        !filt_state.flags.horiz_vel ||
        !filt_state.flags.vert_vel ||
        !filt_state.flags.horiz_pos_abs ||
        !filt_state.flags.vert_pos) {
        return false;
    }
#endif
    return ret;
}

/*
  check if the AHRS subsystem is healthy
*/
bool AP_AHRS_NavEKF::healthy(void) const
{
    // If EKF is started we switch away if it reports unhealthy. This could be due to bad
    // sensor data. If EKF reversion is inhibited, we only switch across if the EKF encounters
    // an internal processing error, but not for bad sensor data.
    if (_ekf_use != EKF_DO_NOT_USE) {
        return ekf_started && EKF.healthy();
    }
    return AP_AHRS_DCM::healthy();    
}

void AP_AHRS_NavEKF::set_ekf_use(bool setting)
{
#if !AHRS_EKF_USE_ALWAYS
    _ekf_use.set(setting);
#endif
}

// true if the AHRS has completed initialisation
bool AP_AHRS_NavEKF::initialised(void) const
{
    // initialisation complete 10sec after ekf has started
    return (ekf_started && (hal.scheduler->millis() - start_time_ms > AP_AHRS_NAVEKF_SETTLE_TIME_MS));
};

// get_filter_status : returns filter status as a series of flags
bool AP_AHRS_NavEKF::get_filter_status(nav_filter_status &status) const
{
    EKF.getFilterStatus(status);
    return true;
}

// write optical flow data to EKF
void  AP_AHRS_NavEKF::writeOptFlowMeas(uint8_t &rawFlowQuality, Vector2f &rawFlowRates, Vector2f &rawGyroRates, uint32_t &msecFlowMeas)
{
    EKF.writeOptFlowMeas(rawFlowQuality, rawFlowRates, rawGyroRates, msecFlowMeas);
}

// inhibit GPS useage
uint8_t AP_AHRS_NavEKF::setInhibitGPS(void)
{
    return EKF.setInhibitGPS();
}

// get speed limit
void AP_AHRS_NavEKF::getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler)
{
    EKF.getEkfControlLimits(ekfGndSpdLimit,ekfNavVelGainScaler);
}

// get compass offset estimates
// true if offsets are valid
bool AP_AHRS_NavEKF::getMagOffsets(Vector3f &magOffsets)
{
    bool status = EKF.getMagOffsets(magOffsets);
    return status;
}

void AP_AHRS_NavEKF::setTakeoffExpected(bool val)
{
    EKF.setTakeoffExpected(val);
}

void AP_AHRS_NavEKF::setTouchdownExpected(bool val)
{
    EKF.setTouchdownExpected(val);
}

// returns the inertial navigation origin in lat/lon/alt
struct Location AP_AHRS_NavEKF::get_origin() const
{
    struct Location ret;
    if (!EKF.getOriginLLH(ret)) {
        // initialise location to all zeros if EKF2 origin not yet set
        memset(&ret, 0, sizeof(ret));
    }
    return ret;
}

// get_hgt_ctrl_limit - get maximum height to be observed by the control loops in metres and a validity flag
// this is used to limit height during optical flow navigation
// it will return invalid when no limiting is required
bool AP_AHRS_NavEKF::get_hgt_ctrl_limit(float& limit) const
{
    return EKF.getHeightControlLimit(limit);

    return false;
}

// get_location - updates the provided location with the latest calculated locatoin
//  returns true on success (i.e. the EKF knows it's latest position), false on failure
bool AP_AHRS_NavEKF::get_location(struct Location &loc) const
{
    return EKF.getLLH(loc);
}

#endif // AP_AHRS_NAVEKF_AVAILABLE

