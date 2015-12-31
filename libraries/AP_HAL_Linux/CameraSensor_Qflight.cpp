#include "CameraSensor_Qflight.h"

CameraSensor_Qflight::CameraSensor_Qflight() :
    vFrameCount_(0),
    pFrameCount_(0),
    vFpsAvg_(0.0f),
    pFpsAvg_(0.0f),
    vTimeStampPrev_(0),
    pTimeStampPrev_(0),
    camera_(NULL)
{
    pthread_cond_init(&cvPicDone, NULL);
    pthread_mutex_init(&mutexPicDone, NULL);
}

CameraSensor_Qflight::CameraSensor_Qflight(TestConfig config) :
    vFrameCount_(0),
    pFrameCount_(0),
    vFpsAvg_(0.0f),
    pFpsAvg_(0.0f),
    vTimeStampPrev_(0),
    pTimeStampPrev_(0)
{
    config_ = config;
    pthread_cond_init(&cvPicDone, NULL);
    pthread_mutex_init(&mutexPicDone, NULL);
}

int CameraSensor_Qflight::initialize(int camId)
{
    int rc;
    rc = ICameraDevice::createInstance(camId, &camera_);
    if (rc != 0) {
        printf("could not open camera %d\n", camId);
        return rc;
    }
    camera_->addListener(this);

    rc = params_.init(camera_);
    if (rc != 0) {
        printf("failed to init parameters\n");
        ICameraDevice::deleteInstance(&camera_);
        return rc;
    }
    //printf("params = %s\n", params_.toString().c_str());
    /* query capabilities */
    caps_.pSizes = params_.getSupportedPreviewSizes();
    caps_.vSizes = params_.getSupportedVideoSizes();
    caps_.picSizes = params_.getSupportedPictureSizes();
    caps_.focusModes = params_.getSupportedFocusModes();
    caps_.wbModes = params_.getSupportedWhiteBalance();
    caps_.isoModes = params_.getSupportedISO();
    caps_.brightness = params_.getSupportedBrightness();
    caps_.sharpness = params_.getSupportedSharpness();
    caps_.contrast = params_.getSupportedContrast();
    caps_.previewFpsRanges = params_.getSupportedPreviewFpsRanges();
    caps_.videoFpsValues = params_.getSupportedVideoFps();
    caps_.previewFormats = params_.getSupportedPreviewFormats();
    caps_.rawSize = params_.get("raw-size");
}

CameraSensor_Qflight::~CameraSensor_Qflight()
{
}

static int dumpToFile(uint8_t* data, uint32_t size, char* name, uint64_t timestamp)
{
    FILE* fp;
    fp = fopen(name, "wb");
    if (!fp) {
        printf("fopen failed for %s\n", name);
        return -1;
    }
    fwrite(data, size, 1, fp);
    printf("saved filename %s\n", name);
    fclose(fp);
}

static inline uint32_t align_size(uint32_t size, uint32_t align)
{
    return ((size + align - 1) & ~(align-1));
}

void CameraSensor_Qflight::onError()
{
    printf("camera error!, aborting\n");
    exit(EXIT_FAILURE);
}

/**
 *
 * FUNCTION: onPreviewFrame
 *
 *  - This is called every frame I
 *  - In the test app, we save files only after every 30 frames
 *  - In parameter frame (ICameraFrame) also has the timestamps
 *    field which is public
 *
 * @param frame
 *
 */
void CameraSensor_Qflight::onPreviewFrame(ICameraFrame* frame)
{
    if (pFrameCount_ > 0 && pFrameCount_ % 30 == 0) {
        char name[50];

        if ( config_.outputFormat == RAW_FORMAT )
        {
            snprintf(name, 50, "P_%dx%d_%04d_%llu.raw",
                 pSize_.width, pSize_.height, pFrameCount_,frame->timeStamp);
        }else{
             snprintf(name, 50, "P_%dx%d_%04d_%llu.yuv",
                 pSize_.width, pSize_.height, pFrameCount_,frame->timeStamp);
        }

        if (config_.dumpFrames == true) {
            dumpToFile(frame->data, frame->size, name, frame->timeStamp);
        }
        //printf("Preview FPS = %.2f\n", pFpsAvg_);
    }

    uint64_t diff = frame->timeStamp - pTimeStampPrev_;
    pFpsAvg_ = ((pFpsAvg_ * pFrameCount_) + (1e9 / diff)) / (pFrameCount_ + 1);
    pFrameCount_++;
    pTimeStampPrev_  = frame->timeStamp;
}

/**
 *
 * FUNCTION: onVideoFrame
 *
 *  - This is called every frame I
 *  - In the test app, we save files only after every 30 frames
 *  - In parameter frame (ICameraFrame) also has the timestamps
 *    field which is public
 *
 * @param frame
 *
 */
void CameraSensor_Qflight::onVideoFrame(ICameraFrame* frame)
{
    if (vFrameCount_ > 0 && vFrameCount_ % 30 == 0) {
        char name[50];
        snprintf(name, 50, "V_%dx%d_%04d_%llu.yuv",
                 vSize_.width, vSize_.height, vFrameCount_,frame->timeStamp);
        if (config_.dumpFrames == true) {
            dumpToFile(frame->data, frame->size, name, frame->timeStamp);
        }
        //printf("Video FPS = %.2f\n", vFpsAvg_);
    }

    uint64_t diff = frame->timeStamp - vTimeStampPrev_;
    vFpsAvg_ = ((vFpsAvg_ * vFrameCount_) + (1e9 / diff)) / (vFrameCount_ + 1);
    vFrameCount_++;
    vTimeStampPrev_  = frame->timeStamp;
}

int CameraSensor_Qflight::printCapabilities()
{
    printf("Camera capabilities\n");

    printf("available preview sizes:\n");
    for (int i = 0; i < caps_.pSizes.size(); i++) {
        printf("%d: %d x %d\n", i, caps_.pSizes[i].width, caps_.pSizes[i].height);
    }
    printf("available video sizes:\n");
    for (int i = 0; i < caps_.vSizes.size(); i++) {
        printf("%d: %d x %d\n", i, caps_.vSizes[i].width, caps_.vSizes[i].height);
    }
    printf("available picture sizes:\n");
    for (int i = 0; i < caps_.picSizes.size(); i++) {
        printf("%d: %d x %d\n", i, caps_.picSizes[i].width, caps_.picSizes[i].height);
    }
    printf("available preview formats:\n");
    for (int i = 0; i < caps_.previewFormats.size(); i++) {
        printf("%d: %s\n", i, caps_.previewFormats[i].c_str());
    }
    printf("available focus modes:\n");
    for (int i = 0; i < caps_.focusModes.size(); i++) {
        printf("%d: %s\n", i, caps_.focusModes[i].c_str());
    }
    printf("available whitebalance modes:\n");
    for (int i = 0; i < caps_.wbModes.size(); i++) {
        printf("%d: %s\n", i, caps_.wbModes[i].c_str());
    }
    printf("available ISO modes:\n");
    for (int i = 0; i < caps_.isoModes.size(); i++) {
        printf("%d: %s\n", i, caps_.isoModes[i].c_str());
    }
    printf("available brightness values:\n");
    printf("min=%d, max=%d, step=%d\n", caps_.brightness.min,
           caps_.brightness.max, caps_.brightness.step);
    printf("available sharpness values:\n");
    printf("min=%d, max=%d, step=%d\n", caps_.sharpness.min,
           caps_.sharpness.max, caps_.sharpness.step);
    printf("available contrast values:\n");
    printf("min=%d, max=%d, step=%d\n", caps_.contrast.min,
           caps_.contrast.max, caps_.contrast.step);

    printf("available preview fps ranges:\n");
    for (int i = 0; i < caps_.previewFpsRanges.size(); i++) {
        printf("%d: [%d, %d]\n", i, caps_.previewFpsRanges[i].min,
               caps_.previewFpsRanges[i].max);
    }
    printf("available video fps values:\n");
    for (int i = 0; i < caps_.videoFpsValues.size(); i++) {
        printf("%d: %d\n", i, caps_.videoFpsValues[i]);
    }
    return 0;
}
ImageSize UHDSize(3840,2160);
ImageSize FHDSize(1920,1080);
ImageSize HDSize(1280,720);
ImageSize VGASize(640,480);
ImageSize stereoVGASize(1280, 480);
ImageSize QVGASize(320,240);
ImageSize stereoQVGASize(640,240);

const char usageStr[] =
    "Camera API test application \n"
    "\n"
    "usage: camera-test [options]\n"
    "\n"
    "  -t <duration>   capture duration in seconds [10]\n"
    "  -d              dump frames\n"
    "  -i              info mode\n"
    "                    - print camera capabilities\n"
    "                    - streaming will not be started\n"
    "  -f <type>       camera type\n"
    "                    - hires\n"
    "                    - optic\n"
    "                    - right \n"
    "                    - stereo \n"
    "  -p <size>       Set resolution for preview frame\n"
    "                    - 4k             ( imx sensor only ) \n"
    "                    - 1080p          ( imx sensor only ) \n"
    "                    - 720p           ( imx sensor only ) \n"
    "                    - VGA            ( Max resolution of optic flow and right sensor )\n"
    "                    - QVGA           ( 320x240 ) \n"
    "                    - stereoVGA      ( 1280x480 : Stereo only - Max resolution )\n"
    "                    - stereoQVGA     ( 640x240  : Stereo only )\n"
    "  -v <size>       Set resolution for video frame\n"
    "                    - 4k             ( imx sensor only ) \n"
    "                    - 1080p          ( imx sensor only ) \n"
    "                    - 720p           ( imx sensor only ) \n"
    "                    - VGA            ( Max resolution of optic flow and right sensor )\n"
    "                    - QVGA           ( 320x240 ) \n"
    "                    - stereoVGA      ( 1280x480 : Stereo only - Max resolution )\n"
    "                    - stereoQVGA     ( 640x240  : Stereo only )\n"
    "                    - disable        ( do not start video stream )\n"
    "  -n              take a picture with  max resolution of camera ( disabled by default)\n"
    "                  $camera-test -f <type> -i to find max picture size\n"
    "  -s <size>       take pickture at set resolution ( disabled by default) \n"
    "                    - 4k             ( imx sensor only ) \n"
    "                    - 1080p          ( imx sensor only ) \n"
    "                    - 720p           ( imx sensor only ) \n"
    "                    - VGA            ( Max resolution of optic flow and right sensor )\n"
    "                    - QVGA           ( 320x240 ) \n"
    "                    - stereoVGA      ( 1280x480 : Stereo only - Max resolution )\n"
    "                    - stereoQVGA     ( 640x240  : Stereo only )\n"
    "  -e <value>      set exposure control (only for ov7251)\n"
    "                     min - 0\n"
    "                     max - 65535\n"
    "  -g <value>      set gain value (only for ov7251)\n"
    "                     min - 0\n"
    "                     max - 255\n"
    "  -r < value>     set fps value      (Enter supported fps for requested resolution) \n"
    "                    -  30 (default)\n"
    "                    -  60 \n"
    "                    -  90 \n"
    "  -o <value>      Output format\n"
    "                     0 :YUV format (default)\n"
    "                     1 : RAW format (default of optic)\n"
    "  -j <value>      Snapshot Format\n"
    "                     jpeg : JPEG format (default)\n"
    "                     raw  : Full-size MIPI RAW format\n"
    "  -V <level>      syslog level [0]\n"
    "                    0: silent\n"
    "                    1: error\n"
    "                    2: info\n"
    "                    3: debug\n"
    "  -h              print this message\n"
;

static inline void printUsageExit(int code)
{
    printf("%s", usageStr);
    exit(code);
}
/**
 * FUNCTION: setFPSindex
 *
 * scans through the supported fps values and returns index of
 * requested fps in the array of supported fps
 *
 * @param fps      : Required FPS  (Input)
 * @param pFpsIdx  : preview fps index (output)
 * @param vFpsIdx  : video fps index   (output)
 *
 *  */
int CameraSensor_Qflight::setFPSindex(int fps, int &pFpsIdx, int &vFpsIdx)
{
    int defaultPrevFPSIndex = -1;
    int defaultVideoFPSIndex = -1;
    int i,rc = 0;
    for (i = 0; i < caps_.previewFpsRanges.size(); i++) {
        if (  (caps_.previewFpsRanges[i].max)/1000 == fps )
        {
            pFpsIdx = i;
            break;
        }
        if ( (caps_.previewFpsRanges[i].max)/1000 == DEFAULT_CAMERA_FPS )
        {
            defaultPrevFPSIndex = i;
        }
    }
    if ( i >= caps_.previewFpsRanges.size() )
    {
        if (defaultPrevFPSIndex != -1 )
        {
            pFpsIdx = defaultPrevFPSIndex;
        } else
        {
            pFpsIdx = -1;
            rc = -1;
        }
    }

    for (i = 0; i < caps_.videoFpsValues.size(); i++) {
        if ( fps == caps_.videoFpsValues[i])
        {
            vFpsIdx = i;
            break;
        }
        if ( DEFAULT_CAMERA_FPS == caps_.videoFpsValues[i])
        {
            defaultVideoFPSIndex = i;
        }
    }
    if ( i >= caps_.videoFpsValues.size())
    {
        if (defaultVideoFPSIndex != -1)
        {
            vFpsIdx = defaultVideoFPSIndex;
        }else
        {
            vFpsIdx = -1;
            rc = -1;
        }
    }
    return rc;
}
/**
 *  FUNCTION : setParameters
 *
 *  - When camera is opened, it is initialized with default set
 *    of parameters.
 *  - This function sets required parameters based on camera and
 *    usecase
 *  - params_setXXX and params_set  only updates parameter
 *    values in a local object.
 *  - params_.commit() function will update the hardware
 *    settings with the current state of the parameter object
 *  - Some functionality will not be application for all for
 *    sensor modules. for eg. optic flow sensor does not support
 *    autofocus/focus mode.
 *  - Reference setting for different sensors and format are
 *    provided in this function.
 *
 *  */
int CameraSensor_Qflight::setParameters()
{
    int focusModeIdx = 3;
    int wbModeIdx = 2;
    int isoModeIdx = 0;
    int pFpsIdx = 3;
    int vFpsIdx = 3;
    int prevFmtIdx = 0;
    int rc = 0;

    pSize_ = config_.pSize;
    vSize_ = config_.vSize;
    picSize_ = config_.picSize;

    switch ( config_.func ){
        case CAM_FUNC_OPTIC_FLOW:
            if (config_.outputFormat == RAW_FORMAT) {
                /* Do not turn on videostream for optic flow in RAW format */
                config_.testVideo = false;
                printf("Setting output = RAW_FORMAT for optic flow sensor \n");
                params_.set("preview-format", "bayer-rggb");
                params_.set("picture-format", "bayer-mipi-10gbrg");
                params_.set("raw-size", "640x480");
            }
            break;
        case CAM_FUNC_RIGHT_SENSOR:
            break;
        case CAM_FUNC_STEREO:
            break;
        case CAM_FUNC_HIRES:
            if (config_.picSizeIdx != -1 ) {
                picSize_ = caps_.picSizes[config_.picSizeIdx];
                config_.picSize = picSize_;
            } else {
                picSize_ = config_.picSize;
            }
                    if (config_.snapshotFormat == RAW_FORMAT) {
                        printf("raw picture format: %s\n",
                               "bayer-mipi-10bggr");
                        params_.set("picture-format",
                                     "bayer-mipi-10bggr");
                        printf("raw picture size: %s\n", caps_.rawSize.c_str());
                    } else {
                        printf("setting picture size: %dx%d\n",
                                picSize_.width, picSize_.height);
                        params_.setPictureSize(picSize_);
                    }

            printf("setting focus mode: %s\n",
                 caps_.focusModes[focusModeIdx].c_str());
            params_.setFocusMode(caps_.focusModes[focusModeIdx]);
            printf("setting WB mode: %s\n", caps_.wbModes[wbModeIdx].c_str());
            params_.setWhiteBalance(caps_.wbModes[wbModeIdx]);
            printf("setting ISO mode: %s\n", caps_.isoModes[isoModeIdx].c_str());
            params_.setISO(caps_.isoModes[isoModeIdx]);

            printf("setting preview format: %s\n",
                 caps_.previewFormats[prevFmtIdx].c_str());
            params_.setPreviewFormat(caps_.previewFormats[prevFmtIdx]);
            break;
        default:
            printf("invalid sensor function \n");
            break;
    }
    printf("setting preview size: %dx%d\n", pSize_.width, pSize_.height);
    params_.setPreviewSize(pSize_);
    printf("setting video size: %dx%d\n", vSize_.width, vSize_.height);
    params_.setVideoSize(vSize_);

    /* Find index and set FPS  */
    rc = setFPSindex(config_.fps, pFpsIdx, vFpsIdx);
    if ( rc == -1)
    {
        return rc;
    }
    printf("setting preview fps range: %d, %d ( idx = %d ) \n",
    caps_.previewFpsRanges[pFpsIdx].min,
    caps_.previewFpsRanges[pFpsIdx].max, pFpsIdx);
    params_.setPreviewFpsRange(caps_.previewFpsRanges[pFpsIdx]);
    printf("setting video fps: %d ( idx = %d )\n", caps_.videoFpsValues[vFpsIdx], vFpsIdx );
    params_.setVideoFPS(caps_.videoFpsValues[vFpsIdx]);


    return params_.commit();
}

int CameraSensor_Qflight::run()
{
    int rc = EXIT_SUCCESS;

    /* returns the number of camera-modules connected on the board */
    int n = getNumberOfCameras();

    if (n < 0) {
        printf("getNumberOfCameras() failed, rc=%d\n", n);
        return EXIT_FAILURE;
    }

    printf("num_cameras = %d\n", n);

    if (n < 1) {
        printf("No cameras found.\n");
        return EXIT_FAILURE;
    }

    /* The camID for sensor is not fixed. It depends on which drivers comes up first.
       Hence loop throuch all modules to find camID based on the functionality. */
    int camId=-1;

    /* find camera based on function */
    for (int i=0; i<n; i++) {
        CameraInfo info;
        getCameraInfo(i, info);
        printf(" i = %d , info.func = %d \n",i, info.func);
        if (info.func == config_.func) {
            camId = i;
        }
    }

    if (camId == -1 )
    {
        printf("Camera not found \n");
        exit(1);
    }

    printf("Testing camera id=%d\n", camId);

    initialize(camId);

    if (config_.infoMode) {
        printCapabilities();
        return rc;
    }

    rc = setParameters();
    if (rc) {
        printf("setParameters failed\n");
        printUsageExit(0);
        goto del_camera;
    }

    /* initialize perf counters */
    vFrameCount_ = 0;
    pFrameCount_ = 0;
    vFpsAvg_ = 0.0f;
    pFpsAvg_ = 0.0f;

    /* starts the preview stream. At every preview frame onPreviewFrame( ) callback is invoked */
    printf("start preview\n");
    camera_->startPreview();

    /* Set parameters which are required after starting preview */
    switch(config_.func)
    {
        case CAM_FUNC_OPTIC_FLOW:
            {
                 params_.setManualExposure(config_.exposureValue);
                 params_.setManualGain(config_.gainValue);
                 printf("Setting exposure value =  %d , gain value = %d \n", config_.exposureValue, config_.gainValue );
            }
            break;
        case CAM_FUNC_RIGHT_SENSOR:
            {
                 params_.setManualExposure(config_.exposureValue);
                 params_.setManualGain(config_.gainValue);
                 printf("Setting exposure value =  %d , gain value = %d \n", config_.exposureValue, config_.gainValue );
            }
            break;
        case CAM_FUNC_STEREO:
            {
                params_.setManualExposure(config_.exposureValue);
                params_.setManualGain(config_.gainValue);
                printf("Setting exposure value =  %d , gain value = %d \n", config_.exposureValue, config_.gainValue );
                params_.setVerticalFlip(true);
                params_.setHorizontalMirror(true);
                printf("Setting Vertical Flip and Horizontal Mirror bit in sensor \n");
            }
            break;

    }
    rc = params_.commit();
    if (rc) {
        printf("commit failed\n");
        exit(EXIT_FAILURE);
    }

    if (config_.testVideo  == true ) {
        /* starts video stream. At every video frame onVideoFrame( )  callback is invoked */
        printf("start recording\n");
        camera_->startRecording();
    }

    /* Put the main/run thread to sleep and process the frames in the callbacks */
    printf("waiting for %d seconds ...\n", config_.runTime);
    sleep(config_.runTime);

    /* After the sleep interval stop preview stream, stop video stream and end application */
    if (config_.testVideo  == true) {
        printf("stop recording\n");
        camera_->stopRecording();
    }
    printf("stop preview\n");
    camera_->stopPreview();

    printf("Average preview FPS = %.2f\n", pFpsAvg_);
    if( config_.testVideo  == true )
        printf("Average video FPS = %.2f\n", vFpsAvg_);

del_camera:
    /* release camera device */
    ICameraDevice::deleteInstance(&camera_);
    return rc;
}

/**
 *  FUNCTION: setDefaultConfig
 *
 *  set default config based on camera module
 *
 * */
static int setDefaultConfig(TestConfig &cfg) {

    cfg.outputFormat = YUV_FORMAT;
    cfg.dumpFrames = false;
    cfg.runTime = 10;
    cfg.infoMode = false;
    cfg.testVideo = true;
    cfg.testSnapshot = false;
    cfg.exposureValue = DEFAULT_EXPOSURE_VALUE;  /* Default exposure value */
    cfg.gainValue = DEFAULT_GAIN_VALUE;  /* Default gain value */
    cfg.fps = DEFAULT_CAMERA_FPS;
    cfg.picSizeIdx = -1;
    cfg.logLevel = CAM_LOG_SILENT;
    cfg.snapshotFormat = JPEG_FORMAT;

    switch (cfg.func) {
    case CAM_FUNC_OPTIC_FLOW:
        cfg.pSize   = VGASize;
        cfg.vSize   = VGASize;
        cfg.picSize   = VGASize;
        cfg.outputFormat = RAW_FORMAT;
        break;
    case CAM_FUNC_RIGHT_SENSOR:
        cfg.pSize   = VGASize;
        cfg.vSize   = VGASize;
        cfg.picSize   = VGASize;
        break;
    case CAM_FUNC_STEREO:
        cfg.pSize = stereoVGASize;
        cfg.vSize  = stereoVGASize;
        cfg.picSize  = stereoVGASize;
        break;
    case CAM_FUNC_HIRES:
        cfg.pSize = FHDSize;
        cfg.vSize = HDSize;
        cfg.picSize = FHDSize;
        break;
    default:
        printf("invalid sensor function \n");
        break;
    }

}

/**
 *  FUNCTION: parseCommandline
 *
 *  parses commandline options and populates the config
 *  data structure
 *
 *  */
TestConfig setConfig()
{
    TestConfig cfg;
    cfg.func = CAM_FUNC_HIRES;

    int c;
    int outputFormat;
    int exposureValueInt = 0;
    int gainValueInt = 0;

    cfg.func = CAM_FUNC_OPTIC_FLOW;
    
    setDefaultConfig(cfg);
    cfg.outputFormat = YUV_FORMAT;
    cfg.dumpFrames = true;
    if (cfg.snapshotFormat == RAW_FORMAT) {
        cfg.testVideo = false;
    }

    return cfg;
}