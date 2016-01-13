#include <stdarg.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/system.h>
#include <AP_HAL_Linux/Scheduler.h>

#include <AP_HAL_Linux/qflight/qflight_util.h>
#include <AP_HAL_Linux/qflight/qflight_dsp.h>
#include <time.h>
#include <errno.h>

extern const AP_HAL::HAL& hal;

namespace AP_HAL {
static int64_t time_offset;

static struct{
    struct timespec start_time;
}state;
void microdelay(uint32_t usec)
{
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = usec*1000UL;
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR) ;
}
void init()
{
    struct timespec t0, t1;
    int64_t T0,T1;
    uint64_t dsptime;
    int64_t offset;
    for(uint8_t i=0;i<200;i++) {
        clock_gettime(CLOCK_MONOTONIC, &t0);
        T0 = 1.0e6*(t0.tv_sec + (t0.tv_nsec*1.0e-9));
        qflight_get_time(&dsptime);
        clock_gettime(CLOCK_MONOTONIC, &t1);
        T1 = 1.0e6*(t1.tv_sec + (t1.tv_nsec*1.0e-9));
        offset = (dsptime - (T1-T0)/2) - T0;
        time_offset = time_offset + (offset - time_offset)/(i+1);
    }
    state.start_time = t1;
}

void panic(const char *errormsg, ...)
{
    va_list ap;

    va_start(ap, errormsg);
    vdprintf(1, errormsg, ap);
    va_end(ap);
    write(1, "\n", 1);

    hal.rcin->deinit();
    hal.scheduler->delay_microseconds(10000);
    exit(1);
}

uint32_t micros()
{
    return micros64() & 0xFFFFFFFF;
}

uint32_t millis()
{
    return millis64() & 0xFFFFFFFF;
}

uint64_t micros64()
{
    const Linux::Scheduler* scheduler = Linux::Scheduler::from(hal.scheduler);
    uint64_t stopped_usec = scheduler->stopped_clock_usec();
    if (stopped_usec) {
        return stopped_usec;
    }

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e6*((ts.tv_sec + (ts.tv_nsec*1.0e-9)) -
                  (state.start_time.tv_sec +
                   (state.start_time.tv_nsec*1.0e-9)));
}

uint64_t millis64()
{
    const Linux::Scheduler* scheduler = Linux::Scheduler::from(hal.scheduler);
    uint64_t stopped_usec = scheduler->stopped_clock_usec();
    if (stopped_usec) {
        return stopped_usec / 1000;
    }

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e3*((ts.tv_sec + (ts.tv_nsec*1.0e-9)) -
                  (state.start_time.tv_sec +
                   (state.start_time.tv_nsec*1.0e-9)));
}

int64_t dsp2linuxtimeoff()
{
    return time_offset;
}
} // namespace AP_HAL
