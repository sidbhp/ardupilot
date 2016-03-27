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

/* Globally Shared Topics - Inspired by uORB on PX4 */

#include <AP_GST/AP_GST_Helper.h>
#include <vector>
#include "topics/sensor_mag.h"
#include "topics/sensor_accel.h"
#include "topics/sensor_gyro.h"
#include "topics/sensor_baro.h"
#include "topics/output_pwm.h"
#include "topics/input_rc.h"
#include "topics/pwm_input.h"
#include "topics/vehicle_attitude.h"

struct gst_metadata {
	const char *o_name;		/**< unique object name */
	const size_t o_size;		/**< object size */
};

GST_DEFINE(sensor_mag, struct sensor_mag_s);
GST_DEFINE(sensor_accel0, struct sensor_accel_s);
GST_DEFINE(sensor_accel1, struct sensor_accel_s);
GST_DEFINE(sensor_accel2, struct sensor_accel_s);
GST_DEFINE(sensor_gyro0, struct sensor_gyro_s);
GST_DEFINE(sensor_gyro1, struct sensor_gyro_s);
GST_DEFINE(sensor_gyro2, struct sensor_gyro_s);
GST_DEFINE(sensor_baro, struct sensor_baro_s);
GST_DEFINE(output_pwm, struct output_pwm_s);
GST_DEFINE(input_rc, struct input_rc_s);
GST_DEFINE(pwm_input, struct pwm_input_s);
GST_DEFINE(vehicle_attitude, struct vehicle_attitude_s);


enum open_type {
    O_READ,
    O_WRITE,
};

class AP_GST {

    AP_GST() : last_rdwr_fd(0),
                last_write_fd(0),
                last_read_fd(0)
    {}
    int8_t open(struct gst_metadata *meta, enum open_type fdt);
    int8_t write(int8_t fd, const void* data);
    int8_t read(int8_t fd, void *buffer);
    int8_t available(int8_t fd);
private:

    struct write_node {
        struct gst_metadata* meta;
        void* data;
    };

    struct read_node {
        struct gst_metadata* meta;
        int8_t write_fd;
        int8_t updated;
        void* context;          //TBD
        void (*callback)();     //TBD
    };

    struct rdwr_info {
        int8_t fd;
        open_type fd_type;
    };
    
    std::vector<write_node*> write_list;
    std::vector<read_node*> read_list;
    std::vector<rdwr_info> rdwr_info_list;
 
    int8_t last_write_fd;
    int8_t last_read_fd;
    int8_t last_rdwr_fd;
};