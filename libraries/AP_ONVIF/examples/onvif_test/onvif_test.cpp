
#include <AP_HAL/AP_HAL.h>
#include <AP_ONVIF/AP_ONVIF.h> 
// #include <DeviceBinding.nsmap>
// #include <MediaBinding.nsmap>
// #include <PTZBinding.nsmap>
#include "lwipthread.h"

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// AP_ONVIF onvif;

void setup()
{
    lwipInit(NULL);
    hal.console->printf("AP_ONVIF library test\n");
    while (!AP::onvif().start("user","123456","http://192.168.1.19:10000")) {
        ETHARP_STATS_DISPLAY();
        LINK_STATS_DISPLAY();
        hal.scheduler->delay(1000);
    }
}

void loop()
{
    static float pan = 0.0, tilt = 0.0;
    static bool move_up;
    printf("Sending: %f %f\n", pan, tilt);
    AP::onvif().set_absolutemove(pan, tilt, 0);
    if (pan < 1.0 && move_up) {
        pan += 0.1;
        tilt += 0.1;
    } else if(pan > -1.0 && !move_up) {
        pan -= 0.1;
        tilt -= 0.1;
    }
    if (pan >= 1.0 && move_up) {
        move_up = false;
    }
    if (pan <= -1.0 && !move_up) {
        move_up = true;
    }
    hal.scheduler->delay(10000);
}

AP_HAL_MAIN();
