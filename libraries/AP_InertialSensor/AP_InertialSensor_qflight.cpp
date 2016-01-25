/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define __STDC_FORMAT_MACROS 1

#include <AP_HAL/AP_HAL.h>
#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_QFLIGHT

#include "AP_InertialSensor_qflight.h"
#include <AP_HAL_Linux/qflight/qflight_util.h>
#include <AP_HAL_Linux/qflight/qflight_dsp.h>

const extern AP_HAL::HAL& hal;
static bool offset_set;
static int64_t time_offset;
void calc_time_offset();
uint64_t read_dsp_cnt();

AP_InertialSensor_QFLIGHT::AP_InertialSensor_QFLIGHT(AP_InertialSensor &imu) :
    AP_InertialSensor_Backend(imu)
{
}

/*
  detect the sensor
 */
AP_InertialSensor_Backend *AP_InertialSensor_QFLIGHT::detect(AP_InertialSensor &_imu)
{
    AP_InertialSensor_QFLIGHT *sensor = new AP_InertialSensor_QFLIGHT(_imu);
    if (sensor == NULL) {
        return NULL;
    }
    if (!sensor->init_sensor()) {
        delete sensor;
        return NULL;
    }
    return sensor;
}

bool AP_InertialSensor_QFLIGHT::init_sensor(void) 
{
    gyro_instance = _imu.register_gyro(1000);
    accel_instance = _imu.register_accel(1000);

    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_QFLIGHT::timer_update, void));
    _product_id = AP_PRODUCT_ID_MPU9250;
    return true;
}

void AP_InertialSensor_QFLIGHT::timer_update(void)
{
    if (imubuf == nullptr) {
        imubuf = QFLIGHT_RPC_ALLOCATE(DSPBuffer::IMU);
        if (imubuf == nullptr) {
            AP_HAL::panic("unable to allocate IMU buffer");
        }
    }
    int ret = qflight_get_imu_data((uint8_t *)imubuf, sizeof(*imubuf));
    uint64_t linux_sample_time;
    static uint64_t prev_dsp_cnt, prev_linux_time;
    
    if (ret != 0) {
        return;
    }

    uint64_t timestamp_da, dspcnt;
    struct timespec td;
    clock_gettime(CLOCK_MONOTONIC, &td);
    timestamp_da = 1.0e6*((td.tv_sec + (td.tv_nsec*1.0e-9)));

    for (uint16_t i=0; i<imubuf->num_samples; i++) {
        DSPBuffer::IMU::BUF &b = imubuf->buf[i];
        Vector3f accel(b.accel[0], b.accel[1], b.accel[2]);
        Vector3f gyro(b.gyro[0], b.gyro[1], b.gyro[2]);
        _rotate_and_correct_accel(accel_instance, accel);
        _rotate_and_correct_gyro(gyro_instance, gyro);

	// Despite being a uint64_t, the time from the Qualcomm driver wraps at 2^31 us.
	uint32_t time_ago = (timestamp_da - (b.timestamp - time_offset)) & 0x7fffffff;
        linux_sample_time = timestamp_da - time_ago;

//	printf("dsp:%" PRIu64 ", linux:%" PRIu64 ", (%" PRIu64 " %uus ago)\n", b.timestamp, linux_sample_time, timestamp_da - linux_sample_time, time_ago);
        _notify_new_accel_raw_sample(accel_instance, accel, linux_sample_time);
        _notify_new_gyro_raw_sample(gyro_instance, gyro, linux_sample_time);
    }
}

bool AP_InertialSensor_QFLIGHT::update(void) 
{
    calc_time_offset();
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}

void calc_time_offset()
{
    uint64_t dsptime;
    char time_str[17];
    int8_t fd = open("/sys/kernel/boot_adsp/qdsp_qtimer", O_RDONLY);
    if(fd == -1) {
        AP_HAL::panic("\nDSP time file open Failed with error: %s !!\n", strerror(errno));
    }
    int8_t len = read(fd,time_str,17);
    if(len == -1) {
        AP_HAL::panic("\nDSP time Read Failed!!\n");
    } else {
        time_str[len-1] = '\0';
    }

    uint64_t timestamp_da, rpcdsptime;
    struct timespec td;
    clock_gettime(CLOCK_MONOTONIC, &td);
    timestamp_da = 1.0e6*((td.tv_sec + (td.tv_nsec*1.0e-9)));

    int ret  = sscanf(time_str,"%llx",&dsptime);
    close(fd);
    dsptime /= 19.2;
    qflight_get_time(&rpcdsptime);
    time_offset = dsptime - timestamp_da;
//    printf("rpcdsptime:%llu dsptime:%llu timestamp_da:%llu\n", rpcdsptime, dsptime, timestamp_da);
//    printf("rpc - dsp:%lld timestamp_da - dsp:%lld rpc - timestamp_da:%lld\n", rpcdsptime - dsptime, timestamp_da - dsptime, rpcdsptime - timestamp_da);
    offset_set = true;
}
uint64_t read_dsp_cnt()
{
    uint64_t dsptime;
    char time_str[17];
    int8_t fd = open("/sys/kernel/boot_adsp/qdsp_qtimer", O_RDONLY);
    if(fd == -1) {
        AP_HAL::panic("\nDSP time file open Failed with error: %s !!\n", strerror(errno));
    }
    int8_t len = read(fd,time_str,17);
    if(len == -1) {
        AP_HAL::panic("\nDSP time Read Failed!!\n");
    } else {
        time_str[len-1] = '\0';
    }
    int ret  = sscanf(time_str,"%llx",&dsptime);
    close(fd);
    return dsptime;
}

uint64_t read_arch_time()
{
    uint64_t arch_time;
    char time_str[17];
    int8_t fd = open("/sys/kernel/boot_adsp/arch_qtimer", O_RDONLY);
    if(fd == -1) {
        AP_HAL::panic("\nDSP time file open Failed with error: %s !!\n", strerror(errno));
    }
    int8_t len = read(fd,time_str,17);
    if(len == -1) {
        AP_HAL::panic("\nDSP time Read Failed!!\n");
    } else {
        time_str[len-1] = '\0';
    }
    int ret  = sscanf(time_str,"%llx",&arch_time);
    close(fd);
    return arch_time;
}
#endif // HAL_BOARD_QFLIGHT
