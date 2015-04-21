#include <AP_Common.h>
#include <AP_HAL.h>
#include <AP_Notify.h>
#include <AP_Vehicle.h>
#include "GyroCalibrator.h"

extern const AP_HAL::HAL& hal;

GyroCalibrator::GyroCalibrator() :
    _status(GYRO_CAL_WAITING)
{
}

void GyroCalibrator::start()
{
    // remove existing gyro offsets
    _new_gyro_offset.zero();
    _best_diff = 0;
    _last_average.zero();
    _converged = false;

    set_status(GYRO_CAL_COLLECTION);
    return;
}

void GyroCalibrator::collect_samples(Vector3f sample, Vector3f accel_value)
{
    if(_status == GYRO_CAL_COLLECTION){
        if(_sample_cnt == 0){
            _gyro_sum.zero();
            _accel_start = accel_value;
        }
        _gyro_sum += sample;
        if(_sample_cnt == MAX_GYRO_CALIB_SAMPLES){
            set_status(GYRO_CAL_CALIBRATE);
        }
        _sample_cnt++;
    } else {
        return;
    }
}

void GyroCalibrator::calibrate(Vector3f accel_value)
{
    Vector3f accel_diff = accel_value - _accel_start;
    Vector3f gyro_avg, gyro_diff;
    float diff_length;

    if (accel_diff.length() > 0.2f) {
        // the accelerometers changed during the gyro sum. Skip
        // this sample. This copes with doing gyro cal on a
        // steadily moving platform. The value 0.2 corresponds
        // with around 5 degrees/second of rotation.
        return;
    }

    gyro_avg = _gyro_sum / _sample_cnt;
    gyro_diff = _last_average - gyro_avg;
    diff_length = gyro_diff.length();

    if (_num_steps == 0) {
        _best_diff = diff_length;
        _best_avg = gyro_avg;
    } else if (gyro_diff.length() < ToRad(0.1f)) {
        // we want the average to be within 0.1 bit, which is 0.04 degrees/s
        _last_average = (gyro_avg * 0.5f) + (_last_average * 0.5f);
        if (!_converged || _last_average.length() < _new_gyro_offset.length()) {
            _new_gyro_offset = _last_average;
        }
        if (!_converged) {
            _converged = true;
        }
    } else if (diff_length < _best_diff) {
        _best_diff = diff_length;
        _best_avg = (gyro_avg * 0.5f) + (_last_average * 0.5f);
    }
    _last_average = gyro_avg;
    _num_steps++;
}

bool GyroCalibrator::get_new_offsets(AP_Vector3f &offsets){
    // we've kept the user waiting long enough - use the best pair we
    // found so far
    bool ret;
    if (!_converged) {
        offsets = _best_avg;
        // flag calibration as failed for this gyro
        ret = false;
    } else {
        offsets = _new_gyro_offset;
        ret = true;
    }

    return ret;
}

void GyroCalibrator::set_status(enum gyro_calib_status_t status)
{
    switch(status){
        case GYRO_CAL_WAITING:
            _status = GYRO_CAL_WAITING;
            break;
        case GYRO_CAL_INITIAL:
            _status = GYRO_CAL_INITIAL;
            break;
        case GYRO_CAL_COLLECTION:
            _status = GYRO_CAL_COLLECTION;
            _sample_cnt = 0;
            break;
        case GYRO_CAL_CALIBRATE:
            _status = GYRO_CAL_CALIBRATE;
            break;
        case GYRO_CAL_FAILED:
            _status = GYRO_CAL_FAILED;
            break;
        case GYRO_CAL_SUCCESS:
            _status = GYRO_CAL_SUCCESS;
            break;
    }
}

bool GyroCalibrator::update(Vector3f gyro, Vector3f accel_value)
{
    
    if(_status == GYRO_CAL_CALIBRATE){
        //check if calibration is completed
        if(_num_steps >= MAX_GYRO_CALIB_STEPS || _converged){
            if(_converged){
                set_status(GYRO_CAL_SUCCESS);
            }else{
                set_status(GYRO_CAL_FAILED);
            }
            return true;
        }
        calibrate(accel_value);
        set_status(GYRO_CAL_COLLECTION);
    } else if(_status == GYRO_CAL_COLLECTION){
        collect_samples(gyro, accel_value);
    }
    return false;
}