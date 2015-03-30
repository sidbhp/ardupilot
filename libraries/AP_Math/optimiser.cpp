// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Math.h>
#include "optimiser.h"

void Optimiser::set_init_params(float params[], const float const_params[], uint16_t num_params, float& fitness, float& lambda)
{
    _params = params;
    _const_params = const_params;
    _num_params = num_params;
    _fitness = &fitness;
    _lambda = &lambda;
}

float Optimiser::calc_mean_squared_residuals(float* params)
{
    if(_sample_buffer_size == 0) {
        return 1.0e30f;
    }
    float sum = 0.0f;
    for(uint16_t i=0; i < _sample_buffer_size; i++){
        Vector3f sample = _process_sample(_sample_buffer, i);
        float resid = _calc_residual(sample, params, _const_params);
        sum += sq(resid);
    }
    sum /= _sample_buffer_size;
    return sum;
}

void Optimiser::do_levenberg_marquardt_fit(float lma_damping, uint16_t num_iters)
{
    if(_sample_buffer_size == 0) {
        return;
    }
    
    float fitness = *_fitness;
    float fit1, fit2;
   

    float *JTJ = new float[_num_params*_num_params];
    float *JTJ2 = new float[_num_params*_num_params];
    float *JTFI = new float[_num_params];
    float *jacob = new float[_num_params];

    float *fit1_params = new float[_num_params];
    float *fit2_params = new float[_num_params]; 
    if (JTJ == NULL || JTJ2 == NULL || JTFI == NULL || jacob == NULL || fit1_params == NULL || fit2_params == NULL ){
        goto lm_ret;
    }

    memcpy(fit1_params,_params,sizeof(float)*_num_params);
    memcpy(fit2_params,_params,sizeof(float)*_num_params);

    for(uint16_t iters = 0; iters < num_iters; iters++){

        memset(JTJ,0,sizeof(float)*_num_params*_num_params);
        memset(JTJ2,0,sizeof(float)*_num_params*_num_params);
        memset(JTFI,0,sizeof(float)*_num_params);

        for(uint16_t k = 0; k<_sample_buffer_size; k++) {
            Vector3f sample = _process_sample(_sample_buffer, k);

            _calc_jacob(sample, fit1_params, jacob, _const_params);

            for(uint8_t i = 0;i < _num_params; i++) {
                // compute JTJ
                for(uint8_t j = 0; j < _num_params; j++) {
                    JTJ[i*_num_params+j] += jacob[i] * jacob[j];
                    JTJ2[i*_num_params+j] += jacob[i] * jacob[j];   //a backup JTJ for LM
                }
                // compute JTFI
                JTFI[i] += jacob[i] * _calc_residual(sample, fit1_params, _const_params);
            }
        }

        //------------------------Levenberg-part-starts-here---------------------------------//
        for(uint8_t i = 0; i < _num_params; i++) {
            JTJ[i*_num_params+i] += (*_lambda);
            JTJ2[i*_num_params+i] += (*_lambda)/lma_damping;
        }
        
        if(!inverse(JTJ, JTJ, _num_params)) {
            goto lm_ret;
        }

        if(!inverse(JTJ2, JTJ2, _num_params)) {
            goto lm_ret;
        }

        for(uint8_t row=0; row < _num_params; row++) {
            for(uint8_t col=0; col < _num_params; col++) {
                fit1_params[row] -= JTFI[col] * JTJ[row*_num_params+col];
                fit2_params[row] -= JTFI[col] * JTJ2[row*_num_params+col];
            }
        }

        fit1 = calc_mean_squared_residuals(fit1_params);
        fit2 = calc_mean_squared_residuals(fit2_params);

        if(fit1 > *_fitness && fit2 > *_fitness){
            *_lambda *= lma_damping;
        } else if(fit2 < *_fitness && fit2 < fit1) {
            *_lambda /= lma_damping;
            memcpy(fit1_params, fit2_params, sizeof(float)*_num_params);
            fitness = fit2;
        } else if(fit1 < *_fitness){
            fitness = fit1;
        }
        //--------------------Levenberg-part-ends-here------------------------------------//

        if(!isnan(fitness) && fitness < *_fitness) {
            *_fitness = fitness;
            memcpy(_params,fit1_params,sizeof(float)*_num_params); 
        }
    }

lm_ret:
    delete[] fit1_params;
    delete[] fit2_params;
    delete[] JTJ;
    delete[] JTJ2;
    delete[] JTFI;
    delete[] jacob;
}