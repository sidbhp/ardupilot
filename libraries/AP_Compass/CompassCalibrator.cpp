#include "CompassCalibrator.h"
#include <AP_HAL.h>
#include <AP_Notify.h>

extern const AP_HAL::HAL& hal;

////////////////////////////////////////////////////////////
///////////////////// PUBLIC INTERFACE /////////////////////
////////////////////////////////////////////////////////////

CompassCalibrator::CompassCalibrator():
_tolerance(COMPASS_CAL_DEFAULT_TOLERANCE)
{
    clear();
}

void CompassCalibrator::clear() {
    set_status(COMPASS_CAL_NOT_STARTED);
}

void CompassCalibrator::start(bool retry, bool autosave, float delay) {
    if(running()) {
        return;
    }
    _autosave = autosave;
    _attempt = 1;
    _retry = retry;
    _delay_start_sec = delay;
    _start_time_ms = hal.scheduler->millis();
    set_status(COMPASS_CAL_WAITING_TO_START);
}

void CompassCalibrator::get_calibration(Vector3f &offsets, Vector3f &diagonals, Vector3f &offdiagonals) {
    if (_status != COMPASS_CAL_SUCCESS) {
        return;
    }

    offsets = _params.offset;
    diagonals = _params.diag;
    offdiagonals = _params.offdiag;
}

float CompassCalibrator::get_completion_percent() const {
    // first sampling step is 1/3rd of the progress bar
    // never return more than 99% unless _status is COMPASS_CAL_SUCCESS
    switch(_status) {
        case COMPASS_CAL_NOT_STARTED:
        case COMPASS_CAL_WAITING_TO_START:
            return 0.0f;
        case COMPASS_CAL_RUNNING_STEP_ONE:
            return 33.3f * _samples_collected/COMPASS_CAL_NUM_SAMPLES;
        case COMPASS_CAL_RUNNING_STEP_TWO:
            return 33.3f + 65.7f*((float)(_samples_collected-_samples_thinned)/(COMPASS_CAL_NUM_SAMPLES-_samples_thinned));
        case COMPASS_CAL_SUCCESS:
            return 100.0f;
        case COMPASS_CAL_FAILED:
        default:
            return 0.0f;
    };
}

bool CompassCalibrator::check_for_timeout() {
    uint32_t tnow = hal.scheduler->millis();
    if(running() && tnow - _last_sample_ms > 1000) {
        _retry = false;
        set_status(COMPASS_CAL_FAILED);
        return true;
    }
    return false;
}

void CompassCalibrator::new_sample(const Vector3f& sample) {
    _last_sample_ms = hal.scheduler->millis();

    if(_status == COMPASS_CAL_WAITING_TO_START) {
        set_status(COMPASS_CAL_RUNNING_STEP_ONE);
    }

    if(running() && _samples_collected < COMPASS_CAL_NUM_SAMPLES && accept_sample(sample)) {
        set_sample(sample, _samples_collected);
        _samples_collected++;
    }
}

void CompassCalibrator::run_fit_chunk() {
    if(!fitting()) {
        return;
    }

    if (_fit_step == 0){                                             //initialise optimiser for sphere fit
        optimise.set_fitness_function(&calc_sphere_residual);
        optimise.set_jacobian_function(&calc_sphere_jacob);
        optimise.set_preprocess_sample_function(&get_sample);
        optimise.set_buffer(_sample_buffer, _samples_collected);
        optimise.set_init_params(_params.get_sphere_params(), NULL, COMPASS_CAL_NUM_SPHERE_PARAMS, _fitness, _sphere_lambda);
    }else if (_fit_step == 15){                                     //initialise optimiser for ellipsoid fit
        optimise.set_fitness_function(&calc_ellipsoid_residual);
        optimise.set_jacobian_function(&calc_ellipsoid_jacob);
        optimise.set_preprocess_sample_function(&get_sample);
        optimise.set_buffer(_sample_buffer, _samples_collected);
        optimise.set_init_params(_params.get_ellipsoid_params(), _params.get_sphere_params(),
                                COMPASS_CAL_NUM_ELLIPSOID_PARAMS, _fitness, _ellipsoid_lambda);
    }

    if(_status == COMPASS_CAL_RUNNING_STEP_ONE) {
        if(_fit_step < 10) {
            optimise.do_levenberg_marquardt_fit(10.0f);

            _fit_step++;
        } else {
            if(_fitness == _initial_fitness || isnan(_fitness)) {           //if true, means that fitness is diverging instead of converging
                set_status(COMPASS_CAL_FAILED);
            }

            set_status(COMPASS_CAL_RUNNING_STEP_TWO);
        }
    } else if(_status == COMPASS_CAL_RUNNING_STEP_TWO) {
        if(_fit_step < 15) {
            optimise.do_levenberg_marquardt_fit(10.0f);
        } else if(_fit_step < 35) {
            optimise.do_levenberg_marquardt_fit(10.0f);                     //ellipsoid fit
        } else {
            if(fit_acceptable()) {
                set_status(COMPASS_CAL_SUCCESS);
            } else {
                set_status(COMPASS_CAL_FAILED);
            }
        }
        _fit_step++;
    }
}

/////////////////////////////////////////////////////////////
////////////////////// PRIVATE METHODS //////////////////////
/////////////////////////////////////////////////////////////
bool CompassCalibrator::running() const {
    return _status == COMPASS_CAL_RUNNING_STEP_ONE || _status == COMPASS_CAL_RUNNING_STEP_TWO;
}

bool CompassCalibrator::fitting() const {
    return running() && _samples_collected == COMPASS_CAL_NUM_SAMPLES;
}

void CompassCalibrator::initialize_fit() {
    //initialize _fitness before starting a fit
    if (_samples_collected != 0) {
        _fitness = calc_mean_squared_residuals();
    } else {
        _fitness = 1.0e30f;
    }
    _ellipsoid_lambda = 1.0f;
    _sphere_lambda = 1.0f;
    _initial_fitness = _fitness;
    _fit_step = 0;
}

void CompassCalibrator::reset_state() {
    _samples_collected = 0;
    _samples_thinned = 0;
    _params.radius = 200;
    _params.offset.zero();
    _params.diag = Vector3f(1.0f,1.0f,1.0f);
    _params.offdiag.zero();

    initialize_fit();
}

bool CompassCalibrator::set_status(compass_cal_status_t status) {
    if (status != COMPASS_CAL_NOT_STARTED && _status == status) {
        return true;
    }

    switch(status) {
        case COMPASS_CAL_NOT_STARTED:
            reset_state();
            _status = COMPASS_CAL_NOT_STARTED;

            if(_sample_buffer != NULL) {
                free(_sample_buffer);
                _sample_buffer = NULL;
            }
            return true;

        case COMPASS_CAL_WAITING_TO_START:
            reset_state();
            _status = COMPASS_CAL_WAITING_TO_START;

            set_status(COMPASS_CAL_RUNNING_STEP_ONE);
            return true;

        case COMPASS_CAL_RUNNING_STEP_ONE:
            if(_status != COMPASS_CAL_WAITING_TO_START) {
                return false;
            }

            if(_attempt == 1 && (hal.scheduler->millis()-_start_time_ms)*1.0e-3f < _delay_start_sec) {
                return false;
            }

            if(_sample_buffer != NULL) {
                initialize_fit();
                _status = COMPASS_CAL_RUNNING_STEP_ONE;
                return true;
            }

            _sample_buffer = (Vector3i*)malloc(sizeof(Vector3i)*COMPASS_CAL_NUM_SAMPLES);

            if(_sample_buffer != NULL) {
                initialize_fit();
                _status = COMPASS_CAL_RUNNING_STEP_ONE;
                return true;
            }

            return false;

        case COMPASS_CAL_RUNNING_STEP_TWO:
            if(_status != COMPASS_CAL_RUNNING_STEP_ONE) {
                return false;
            }
            thin_samples();
            initialize_fit();
            _status = COMPASS_CAL_RUNNING_STEP_TWO;
            return true;

        case COMPASS_CAL_SUCCESS:
            if(_status != COMPASS_CAL_RUNNING_STEP_TWO) {
                return false;
            }

            if(_sample_buffer != NULL) {
                free(_sample_buffer);
                _sample_buffer = NULL;
            }

            _status = COMPASS_CAL_SUCCESS;
            return true;

        case COMPASS_CAL_FAILED:
            if(_status == COMPASS_CAL_NOT_STARTED) {
                return false;
            }

            AP_Notify::events.compass_cal_failed = 1;

            if(_retry && set_status(COMPASS_CAL_WAITING_TO_START)) {
                _attempt++;
                return true;
            }

            if(_sample_buffer != NULL) {
                free(_sample_buffer);
                _sample_buffer = NULL;
            }

            _status = COMPASS_CAL_FAILED;
            return true;

        default:
            return false;
    };
}

bool CompassCalibrator::fit_acceptable() {
    if( !isnan(_fitness) &&
        _params.radius   > 200  &&                  //Earth's magnetic field strength range: 250-650mG
        _params.radius   < 700  &&
        fabsf(_params.offset.x) < 1000 &&
        fabsf(_params.offset.y) < 1000 &&
        fabsf(_params.offset.z) < 1000 &&
        fabsf(_params.offdiag.x) <  1.0f &&      //absolute of sine/cosine output cannot be greater than 1
        fabsf(_params.offdiag.x) <  1.0f &&
        fabsf(_params.offdiag.x) <  1.0f ){

            return _fitness <= sq(_tolerance);
        }
    return false;
}

void CompassCalibrator::thin_samples() {
    if(_sample_buffer == NULL) {
        return;
    }

    _samples_thinned = 0;
    // shuffle the samples http://en.wikipedia.org/wiki/Fisher%E2%80%93Yates_shuffle
    // this is so that adjacent samples don't get sequentially eliminated
    for(uint16_t i=_samples_collected-1; i>=1; i--) {
        uint16_t j = get_random() % (i+1);
        Vector3i temp = _sample_buffer[i];
        _sample_buffer[i] = _sample_buffer[j];
        _sample_buffer[j] = temp;
    }

    for(uint16_t i=0; i < _samples_collected; i++) {
        if(!accept_sample(get_sample(_sample_buffer, i))) {
            _sample_buffer[i] = _sample_buffer[_samples_collected-1];
            _samples_collected --;
            _samples_thinned ++;
        }
    }
}

bool CompassCalibrator::accept_sample(const Vector3f& sample)
{
    if(_sample_buffer == NULL) {
        return false;
    }

    float max_distance = fabsf(5.38709f * _params.radius / sqrtf((float)COMPASS_CAL_NUM_SAMPLES)) / 3.0f;

    for (uint16_t i = 0; i<_samples_collected; i++){
        float distance = (sample - get_sample(_sample_buffer, i)).length();
        if(distance < max_distance) {
            return false;
        }
    }
    return true;
}


float CompassCalibrator::calc_mean_squared_residuals()
{
    return optimise.calc_mean_squared_residuals(_params.get_sphere_params());
}

float CompassCalibrator::calc_sphere_residual(const Vector3f& sample, const float params[], const float const_params[]) {
    float radius = params[0];
    Vector3f offset = Vector3f(params[1], params[2], params[3]);

    return radius - ((sample+offset)).length();
}

float CompassCalibrator::calc_ellipsoid_residual(const Vector3f& sample, const float params[], const float const_params[]) {

    const Vector3f &offset = Vector3f(params[0], params[1], params[2]);
    const Vector3f &diag = Vector3f(params[3], params[4], params[5]);
    const Vector3f &offdiag = Vector3f(params[6], params[7], params[8]);

    Matrix3f softiron(
        diag.x    , offdiag.x , offdiag.y,
        offdiag.x , diag.y    , offdiag.z,
        offdiag.y , offdiag.z , diag.z
    );
    return const_params[0] - (softiron*(sample+offset)).length();
}

void CompassCalibrator::calc_sphere_jacob(const Vector3f& sample, const float params[], float* ret, const float const_params[]) {

    const Vector3f &offset = Vector3f(params[1], params[2], params[3]);
    const Vector3f &diag = Vector3f(1.0f, 1.0f, 1.0f);
    const Vector3f &offdiag = Vector3f(0.0f, 0.0f, 0.0f);
    Matrix3f softiron(
        diag.x    , offdiag.x , offdiag.y,
        offdiag.x , diag.y    , offdiag.z,
        offdiag.y , offdiag.z , diag.z
    );

    float A =  (diag.x    * (sample.x + offset.x)) + (offdiag.x * (sample.y + offset.y)) + (offdiag.y * (sample.z + offset.z));
    float B =  (offdiag.x * (sample.x + offset.x)) + (diag.y    * (sample.y + offset.y)) + (offdiag.z * (sample.z + offset.z));
    float C =  (offdiag.y * (sample.x + offset.x)) + (offdiag.z * (sample.y + offset.y)) + (diag.z    * (sample.z + offset.z));
    float length = (softiron*(sample+offset)).length();

    // 0: radius
    ret[0] = 1;
    // 1-3: offsets
    ret[1] = -1.0f * (((diag.x    * A) + (offdiag.x * B) + (offdiag.y * C))/length);
    ret[2] = -1.0f * (((offdiag.x * A) + (diag.y    * B) + (offdiag.z * C))/length);
    ret[3] = -1.0f * (((offdiag.y * A) + (offdiag.z * B) + (diag.z    * C))/length);
}


void CompassCalibrator::calc_ellipsoid_jacob(const Vector3f& sample, const float params[], float* ret, const float const_params[]) {

    const Vector3f &offset = Vector3f(params[0], params[1], params[2]);
    const Vector3f &diag = Vector3f(params[3], params[4], params[5]);
    const Vector3f &offdiag = Vector3f(params[6], params[7], params[8]);
    Matrix3f softiron(
        diag.x    , offdiag.x , offdiag.y,
        offdiag.x , diag.y    , offdiag.z,
        offdiag.y , offdiag.z , diag.z
    );

    float A =  (diag.x    * (sample.x + offset.x)) + (offdiag.x * (sample.y + offset.y)) + (offdiag.y * (sample.z + offset.z));
    float B =  (offdiag.x * (sample.x + offset.x)) + (diag.y    * (sample.y + offset.y)) + (offdiag.z * (sample.z + offset.z));
    float C =  (offdiag.y * (sample.x + offset.x)) + (offdiag.z * (sample.y + offset.y)) + (diag.z    * (sample.z + offset.z));
    float length = (softiron*(sample+offset)).length();

    // 0-2: offsets
    ret[0] = -1.0f * (((diag.x    * A) + (offdiag.x * B) + (offdiag.y * C))/length);
    ret[1] = -1.0f * (((offdiag.x * A) + (diag.y    * B) + (offdiag.z * C))/length);
    ret[2] = -1.0f * (((offdiag.y * A) + (offdiag.z * B) + (diag.z    * C))/length);
    // 3-5: diagonals
    ret[3] = -1.0f * ((sample.x + offset.x) * A)/length;
    ret[4] = -1.0f * ((sample.y + offset.y) * B)/length;
    ret[5] = -1.0f * ((sample.z + offset.z) * C)/length;
    // 6-8: off-diagonals
    ret[6] = -1.0f * (((sample.y + offset.y) * A) + ((sample.z + offset.z) * B))/length;
    ret[7] = -1.0f * (((sample.z + offset.z) * A) + ((sample.x + offset.x) * C))/length;
    ret[8] = -1.0f * (((sample.z + offset.z) * B) + ((sample.y + offset.y) * C))/length;
}

//////////////////////////////////////////////////////////
////////////////////// MATH HELPERS //////////////////////
//////////////////////////////////////////////////////////

uint16_t CompassCalibrator::get_random(void)
{
    static uint32_t m_z = 1234;
    static uint32_t m_w = 76542;
    m_z = 36969 * (m_z & 65535) + (m_z >> 16);
    m_w = 18000 * (m_w & 65535) + (m_w >> 16);
    return ((m_z << 16) + m_w) & 0xFFFF;
}

//////////////////////////////////////////////////////////
//////////// CompassSample public interface //////////////
//////////////////////////////////////////////////////////

Vector3f CompassCalibrator::get_sample(const Vector3i in[], uint16_t num) {
    Vector3f out;
    out.x = (float)in[num].x*2048.0f/32700.0f;
    out.y = (float)in[num].y*2048.0f/32700.0f;
    out.z = (float)in[num].z*2048.0f/32700.0f;
    return out;
}

void CompassCalibrator::set_sample(const Vector3f& in, uint16_t num) {
    _sample_buffer[num].x = (int16_t)(in.x*32700.0f/2048.0f + 0.5f);
    _sample_buffer[num].y = (int16_t)(in.y*32700.0f/2048.0f + 0.5f);
    _sample_buffer[num].z = (int16_t)(in.z*32700.0f/2048.0f + 0.5f);
}
