/*
    This class has been implemented based on
    yavta -- Yet Another V4L2 Test Application written by:
    Laurent Pinchart <laurent.pinchart@ideasonboard.com>

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

#include "OpticalFlow_Qflight.h"
#include "CameraSensor_Qflight.h"

OpticalFlow_Qflight::OpticalFlow_Qflight()
{
  	    TestConfig config = setConfig();

    /* setup syslog level */
    if (config.logLevel == CAM_LOG_SILENT) {
        setlogmask(LOG_UPTO(LOG_EMERG));
    } else if (config.logLevel == CAM_LOG_DEBUG) {
        setlogmask(LOG_UPTO(LOG_DEBUG));
    } else if (config.logLevel == CAM_LOG_INFO) {
        setlogmask(LOG_UPTO(LOG_INFO));
    } else if (config.logLevel == CAM_LOG_ERROR) {
        setlogmask(LOG_UPTO(LOG_ERR));
    }
    openlog(NULL, LOG_NDELAY, LOG_DAEMON);

    CameraSensor_Qflight test(config);
    test.run();
}
void OpticalFlow_Qflight::init(AP_HAL::OpticalFlow::Gyro_Cb get_gyro)
{
    TestConfig config = setConfig();

    /* setup syslog level */
    if (config.logLevel == CAM_LOG_SILENT) {
        setlogmask(LOG_UPTO(LOG_EMERG));
    } else if (config.logLevel == CAM_LOG_DEBUG) {
        setlogmask(LOG_UPTO(LOG_DEBUG));
    } else if (config.logLevel == CAM_LOG_INFO) {
        setlogmask(LOG_UPTO(LOG_INFO));
    } else if (config.logLevel == CAM_LOG_ERROR) {
        setlogmask(LOG_UPTO(LOG_ERR));
    }
    openlog(NULL, LOG_NDELAY, LOG_DAEMON);

    CameraSensor_Qflight test(config);
    test.run();
}

bool OpticalFlow_Qflight::read(AP_HAL::OpticalFlow::Data_Frame& frame)
{
	return false;
}