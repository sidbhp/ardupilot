// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef OPTIMISER_H
#define OPTIMISER_H


class Optimiser{
public:
    //Functions to be passed for sample preprocessing, fitness tests and Jacobian calculations
    typedef float (*residual)(const Vector3f& sample, const float params[], const float const_params[]);
    typedef void (*jacob)(const Vector3f& sample, const float params[], float* ret, const float const_params[]);
    typedef Vector3f (*process_sample)(const Vector3i _sample_buffer[], uint16_t num);

    //Funtions to Initialise Optimser    
    void set_fitness_function(residual calc_residual) { _calc_residual = calc_residual; }
    void set_jacobian_function(jacob calc_jacob) { _calc_jacob = calc_jacob; }
    void set_preprocess_sample_function(process_sample preprocess_sample) { _process_sample = preprocess_sample; }
    void set_buffer( Vector3i sample_buffer[], uint16_t sample_buffer_size) { 
        _sample_buffer = sample_buffer;
        _sample_buffer_size = sample_buffer_size;
    }
    void set_init_params(float params[], const float const_params[], uint16_t num_params, float& fitness, float& lambda);
    
    //Optimiser Funtions 
    void do_levenberg_marquardt_fit(float lma_damping, uint16_t num_iters = 1);
    void do_gauss_newton_fit(uint16_t iters = 1);

    float calc_mean_squared_residuals(float *params);

protected:

    jacob _calc_jacob;
    residual _calc_residual;
    process_sample _process_sample;

    float *_fitness;
    float *_lambda;
    
    Vector3i *_sample_buffer;
    float *_params;
    const float *_const_params;

    uint16_t _num_params;
    uint16_t _sample_buffer_size;
};

#endif  //OPTIMISER_H