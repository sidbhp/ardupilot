/* Copyright (c) 2015, The Linux Foundataion. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*************************************************************************
*
*Application Notes:
*
*Camera selection:
*  Each camera is given a unique function id in the sensor driver.
*   HIRES = 0, OPTIC_FLOW = 1, LEFT SENSOR = 2, STEREO = 3
*  getNumberOfCameras gives information on number of camera connected on target.
*  getCameraInfo provides information on each camera loop.
*  camid is obtained by looping through available cameras and matching info.func
*  with the requested camera.
*
*Camera configuration:
*
* Optic flow:
*   Do not set the following parameters :
*    PictureSize
*    focus mode
*    white balance
*    ISO
*    preview format
*
*  The following parameters can be set only after starting preview :
*    Manual exposure
*    Manual gain
*
*  Notes:
*    Snapshot is not supported for optic flow.
*
*  How to enable RAW mode for optic flow sensor ?
*    RAW stream is only available for OV7251
*    RAW stream currently returns images in the preview callback.
*    When configuring RAW stream, video stream on the same sensor must not be enabled. Else you will not see preview callbacks.
*    When configuration RAW, these parameters must be set  in addition to other parameters for optic flow
*                params_.set("preview-format", "bayer-rggb");
*                params_.set("picture-format", "bayer-mipi-10gbrg");
*                params_.set("raw-size", "640x480");
*
*
*  Stereo:
*    Do not set the following parameters :
*     PictureSize
*     focus mode
*     white balance
*     ISO
*     preview format
*
*   The following parameters can be set only after starting preview :
*     Manual exposure
*     Manual gain
*     setVerticalFlip
*     setHorizontalMirror
*  left/right:
*    code is written with reference to schematic but for end users the left and right sensor appears swapped.
*    so for user experience in the current app left option is changed to right.
*    right sensor with reference to schematic always goes in stereo mode, so no left option for end users.
*
* How to perform vertical flip and horizontal mirror on individual images in stereo ?
*  In stereo since the image is merged,
*  it makes it harder to perform these operation on individual images which may be required based on  senor  orientation on target.
*  setVerticalFlip and setHorizontalMirror perform  perform these operation by changing the output configuration from the sensor.
*
*
* How to set FPS
*  Preview fps is set using the function : setPreviewFpsRange
*  Video fps is set using the function : setVideoFPS
*  setFPSindex scans through the supported fps values and returns index of requested fps in the array of supported fps.
*
* How to change the format to NV12
*  To change the format to NV12 use the "preview-format" key.
*  params.set(std::string("preview-format"), std::string("nv12"));
*
****************************************************************************/
#ifndef __CAMERASENSOR_QFLIGHT_H__
#define __CAMERASENSOR_QFLIGHT_H__

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <errno.h>

#include "camera.h"
#include "camera_log.h"
#include "camera_parameters.h"

#define DEFAULT_EXPOSURE_VALUE  250
#define MIN_EXPOSURE_VALUE 0
#define MAX_EXPOSURE_VALUE 65535
#define DEFAULT_GAIN_VALUE  50
#define MIN_GAIN_VALUE 0
#define MAX_GAIN_VALUE 255

#define DEFAULT_CAMERA_FPS 30
#define MS_PER_SEC 1000
#define NS_PER_MS 1000000
#define NS_PER_US 1000

const int SNAPSHOT_WIDTH_ALIGN = 64;
const int SNAPSHOT_HEIGHT_ALIGN = 64;
const int TAKEPICTURE_TIMEOUT_MS = 2000;

using namespace std;
using namespace camera;

struct CameraCaps
{
    vector<ImageSize> pSizes, vSizes, picSizes;
    vector<string> focusModes, wbModes, isoModes;
    Range brightness, sharpness, contrast;
    vector<Range> previewFpsRanges;
    vector<VideoFPS> videoFpsValues;
    vector<string> previewFormats;
    string rawSize;
};

enum OutputFormatType{
    YUV_FORMAT,
    RAW_FORMAT,
    JPEG_FORMAT
};

enum CamFunction {
    CAM_FUNC_HIRES = 0,
    CAM_FUNC_OPTIC_FLOW = 1,
    CAM_FUNC_RIGHT_SENSOR = 2,
    CAM_FUNC_STEREO = 3,
};

enum AppLoglevel {
    CAM_LOG_SILENT = 0,
    CAM_LOG_ERROR = 1,
    CAM_LOG_INFO = 2,
    CAM_LOG_DEBUG = 3,
    CAM_LOG_MAX,
};

/**
*  Helper class to store all parameter settings
*/
struct TestConfig
{
    bool dumpFrames;
    bool infoMode;
    bool testSnapshot;
    bool testVideo;
    int runTime;
    int exposureValue;
    int gainValue;
    CamFunction func;
    OutputFormatType outputFormat;
    OutputFormatType snapshotFormat;
    ImageSize pSize;
    ImageSize vSize;
    ImageSize picSize;
    int picSizeIdx;
    int fps;
    AppLoglevel logLevel;
};

/**
 * CLASS  CameraSensor_Qflight
 *
 * - inherits ICameraListers which provides core functionality
 * - User must define onPreviewFrame (virtual) function. It is
 *    the callback function for every preview frame.
 * - If user is using VideoStream then the user must define
 *    onVideoFrame (virtual) function. It is the callback
 *    function for every video frame.
 * - If any error occurs,  onError() callback function is
 *    called. User must define onError if error handling is
 *    required.
 */
TestConfig setConfig();

class CameraSensor_Qflight : ICameraListener
{
public:

    CameraSensor_Qflight();
    CameraSensor_Qflight(TestConfig config);
    ~CameraSensor_Qflight();
    int run();

    int initialize(int camId);

    /* listener methods */
    virtual void onError();
    virtual void onPreviewFrame(ICameraFrame* frame);
    virtual void onVideoFrame(ICameraFrame* frame);

private:
    ICameraDevice* camera_;
    CameraParams params_;
    ImageSize pSize_, vSize_, picSize_;
    CameraCaps caps_;
    TestConfig config_;

    uint32_t vFrameCount_, pFrameCount_;
    float vFpsAvg_, pFpsAvg_;

    uint64_t vTimeStampPrev_, pTimeStampPrev_;

    pthread_cond_t cvPicDone;
    pthread_mutex_t mutexPicDone;
    bool isPicDone;

    int printCapabilities();
    int setParameters();
    int setFPSindex(int fps, int &pFpsIdx, int &vFpsIdx);
};
#endif //CAMERASENSOR_QFLIGHT