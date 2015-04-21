/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __GYRO_CALIBRATOR_H__
#define __GYRO_CALIBRATOR_H__

#define MAX_GYRO_CALIB_SAMPLING_TIME  500       //average samples over 500ms
#define MAX_GYRO_CALIB_STEPS    120

enum gyro_calib_status_t {
    GYRO_CAL_WAITING=0,
    GYRO_CAL_INITIAL=1,
    GYRO_CAL_COLLECTION=2,
    GYRO_CAL_CALIBRATE=3,
    GYRO_CAL_SUCCESS=4,
    GYRO_CAL_FAILED=5
};

class GyroCalibrator {
public:
    GyroCalibrator();
    void start();
    bool get_new_offsets(AP_Vector3f &offsets);
    void collect_samples(Vector3f sample, Vector3f accel_value);
    gyro_calib_status_t get_status() const{ return _status;}
    bool update(Vector3f gyro, Vector3f accel_value);

private:
    void calibrate(Vector3f accel_value);
    void set_status(gyro_calib_status_t status);
    gyro_calib_status_t _status;

    //gyro init variables
    uint16_t _sample_cnt, _sampling_start_time;
    uint16_t _num_steps;
    Vector3f _last_average, _best_avg;
    Vector3f _new_gyro_offset;
    float _best_diff;
    Vector3f _gyro_sum;
    bool _converged;
    Vector3f _accel_start;
};

#endif