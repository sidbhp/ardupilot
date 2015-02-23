/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

/*
 *       AP_Compass_Callib.cpp 
 *       Code by Siddharth Bharat Purohit. 3DRobotics Inc.
 *
 */

#include "Compass.h"

#define AIMED_FITNESS  1.0     //desired max value of fitness
#define MAX_ITERS      10    //no. of iterations after which if convergence doesn't happen calib process is said to be failed
#define SAMPLE_RATE    5     //no. of samples/sec
#define MAX_OFF_VAL    1000  //this value makes sure no insane value get through
#define MIN_OFF_VAL   -1000
#define SAMPLE_DIST    50
#define GRADIENT       5
#define MAX_RAD        500
/*GRADIENT value specify the speed with which the optimiser will try to converge
very high value means very low chance of convergence as the steps taken will
be too large, while very low value will ensure the convergence will occur but
may take huge amount of time. Striking balance with this factor is the key to
-wards successful result in viable time period.
*/
#define JACOB_DELTA 0.000000001

extern const AP_HAL::HAL& hal;

/// magnetometer Calibration Routine
void Compass::magcalib(void)
{
    int iters = 0;
    bool done;
    for(int i=0;i<COMPASS_MAX_INSTANCES;i++){
        callib_samples[i] = new Vector3i[100];
        calib_complete[i] = false;
        for(int j=0;j<4;j++){
            sphere_param[i][j]=20;
        }
    }
    while((iters < MAX_ITERS) || !done){             //if false means good covergence didn't occured in 10 iters: Callibration failed
        
        collect_samples();
        for(int i=0;i<get_count();i++){
            if(calib_complete[i]){
                hal.console->printf("Calibration Completed!!!! I[%d]: Best Match: \nOff1: %.2f Off2: %.2f Off3: %.2f \n\n\n",
                                    i,sphere_param[i][1],sphere_param[i][2],sphere_param[i][3]);
                continue;
            }
            if(process_samples(i)){
                calib_complete[i] = true;
            }
        }
        for(int i=0;i<get_count();i++){
            if(calib_complete[i]){
                done = true;
            }else{
                done = false;
            }
        }
    }
    if(!done){
        hal.console->printf("\nCalibration Failed!!!!");
        hal.scheduler->delay(10);
    }
    for(int j=0;j<COMPASS_MAX_INSTANCES;j++){
        delete[] callib_samples[j];
    }
    
}

bool Compass::process_samples(int id)
{
    double global_best_f, best_past_f = 0;
    bool ofb;

    global_best_f = evaluatelm(callib_samples[id],sphere_param[id]);
    while(global_best_f > AIMED_FITNESS/2){
        global_best_f = evaluatelm(callib_samples[id],sphere_param[id]);             //crunching data        
        if(best_past_f <= global_best_f){                
            break;
        }                                                      //goes downhill within the iteration
    }

    hal.console->printf("I[%d]: \nRad: %.2f Off1: %.2f Off2: %.2f Off3: %.2f fitness: %.5f \n\n",id,sphere_param[id][0],
    sphere_param[id][1], sphere_param[id][2],sphere_param[id][3],global_best_f);
    //check if we are getting close
    if(global_best_f <= AIMED_FITNESS){
        _passed[id]++;                      //total consecutive fitness test passed
        hal.console->printf("Good Fitness Test Passed:  %d\n",_passed[id]);
    }else{
        _passed[id] = 0;
    }
    //check if this is it
    if(_passed[id] == 2){
        hal.scheduler->delay(10);
        return true;
    }
    return false;
}

void Compass::collect_samples()
{
    //collect Samples
    int count[COMPASS_MAX_INSTANCES]={0},c=0,scomp = 0;
    while(1){
        for(int i=0;i<get_count();i++){
            c = count[i];
            if(c == 100){
                continue;
            }
            read();
            if (!healthy()) {
                hal.console->print("not healthy      \r");
                continue;
            }
            const Vector3f &mag = get_field(i);


            if( c >= 1){
                if((abs(callib_samples[i][c-1].x - (int)mag.x) > SAMPLE_DIST) ||
                    (abs(callib_samples[i][c-1].y - (int)mag.y) > SAMPLE_DIST) ||
                    (abs(callib_samples[i][c-1].z - (int)mag.z) > SAMPLE_DIST)){
                    callib_samples[i][c].x = mag.x;
                    callib_samples[i][c].y = mag.y;
                    callib_samples[i][c].z = mag.z;
                    if(validate_sample(callib_samples[i],c)){
                        c++;
                    }
                }
            }else{
                callib_samples[i][c].x = mag.x;
                callib_samples[i][c].y = mag.y;
                callib_samples[i][c].z = mag.z;
                c++;
            }
            if(c == 100){
                scomp++;
            }

            count[i] = c;
            hal.console->printf("%d/%d :",scomp,get_count());
            for(int j=0;j<COMPASS_MAX_INSTANCES;j++){
                hal.console->printf("[%d]       ",count[j]);
            }

            hal.console->printf("\r");
        }
        
        if(scomp == get_count()){
            break;
        }
        hal.scheduler->delay(1000/(SAMPLE_RATE)); 
    }
    hal.console->printf("Sampling Over \n");
}

bool Compass::validate_sample(Vector3i samples[],int count){
    for(int i=0;i<count;i++){
        if((samples[i].x == samples[count].x) && (samples[i].z == samples[count].z) && (samples[i].y == samples[count].y)){
            return false;
        }
    }
    return true;
}

/// Returns Squared Sum with provided set of fitness data(delta) as generated in sphere_fitness
double Compass::square_sum(void)
{
    double sqsum=0;
    for(int i=0;i<100;i++){
        sqsum = sqsum + delta[i]*delta[i];
    }
    return sqsum;
}

/// calculates fitness of points to sphere
void Compass::sphere_fitness(Vector3i data[], double param[], double fit[])
{
    double a;
    if(param[0] == 0){
        a = 1;
        param[0] = 1;
    }else{
        a = 1/(param[0]*param[0]);
    }
    for(int i=0;i<100;i++){
        fit[i] = 1 - a*((data[i].x+param[1])*(data[i].x+param[1]) +
                          (data[i].y+param[2])*(data[i].y+param[2]) +
                          (data[i].z+param[3])*(data[i].z+param[3]));
    }
}

/*matrix inverse code only for 4x4 square matrix
copied from gluInvertMatrix implementation in opengl for 4x4 matrices
*/
bool Compass::inverse4x4(double m[],double invOut[])
{
    double inv[16], det;
    int i;

    inv[0] = m[5]  * m[10] * m[15] - 
             m[5]  * m[11] * m[14] - 
             m[9]  * m[6]  * m[15] + 
             m[9]  * m[7]  * m[14] +
             m[13] * m[6]  * m[11] - 
             m[13] * m[7]  * m[10];

    inv[4] = -m[4]  * m[10] * m[15] + 
              m[4]  * m[11] * m[14] + 
              m[8]  * m[6]  * m[15] - 
              m[8]  * m[7]  * m[14] - 
              m[12] * m[6]  * m[11] + 
              m[12] * m[7]  * m[10];

    inv[8] = m[4]  * m[9] * m[15] - 
             m[4]  * m[11] * m[13] - 
             m[8]  * m[5] * m[15] + 
             m[8]  * m[7] * m[13] + 
             m[12] * m[5] * m[11] - 
             m[12] * m[7] * m[9];

    inv[12] = -m[4]  * m[9] * m[14] + 
               m[4]  * m[10] * m[13] +
               m[8]  * m[5] * m[14] - 
               m[8]  * m[6] * m[13] - 
               m[12] * m[5] * m[10] + 
               m[12] * m[6] * m[9];

    inv[1] = -m[1]  * m[10] * m[15] + 
              m[1]  * m[11] * m[14] + 
              m[9]  * m[2] * m[15] - 
              m[9]  * m[3] * m[14] - 
              m[13] * m[2] * m[11] + 
              m[13] * m[3] * m[10];

    inv[5] = m[0]  * m[10] * m[15] - 
             m[0]  * m[11] * m[14] - 
             m[8]  * m[2] * m[15] + 
             m[8]  * m[3] * m[14] + 
             m[12] * m[2] * m[11] - 
             m[12] * m[3] * m[10];

    inv[9] = -m[0]  * m[9] * m[15] + 
              m[0]  * m[11] * m[13] + 
              m[8]  * m[1] * m[15] - 
              m[8]  * m[3] * m[13] - 
              m[12] * m[1] * m[11] + 
              m[12] * m[3] * m[9];

    inv[13] = m[0]  * m[9] * m[14] - 
              m[0]  * m[10] * m[13] - 
              m[8]  * m[1] * m[14] + 
              m[8]  * m[2] * m[13] + 
              m[12] * m[1] * m[10] - 
              m[12] * m[2] * m[9];

    inv[2] = m[1]  * m[6] * m[15] - 
             m[1]  * m[7] * m[14] - 
             m[5]  * m[2] * m[15] + 
             m[5]  * m[3] * m[14] + 
             m[13] * m[2] * m[7] - 
             m[13] * m[3] * m[6];

    inv[6] = -m[0]  * m[6] * m[15] + 
              m[0]  * m[7] * m[14] + 
              m[4]  * m[2] * m[15] - 
              m[4]  * m[3] * m[14] - 
              m[12] * m[2] * m[7] + 
              m[12] * m[3] * m[6];

    inv[10] = m[0]  * m[5] * m[15] - 
              m[0]  * m[7] * m[13] - 
              m[4]  * m[1] * m[15] + 
              m[4]  * m[3] * m[13] + 
              m[12] * m[1] * m[7] - 
              m[12] * m[3] * m[5];

    inv[14] = -m[0]  * m[5] * m[14] + 
               m[0]  * m[6] * m[13] + 
               m[4]  * m[1] * m[14] - 
               m[4]  * m[2] * m[13] - 
               m[12] * m[1] * m[6] + 
               m[12] * m[2] * m[5];

    inv[3] = -m[1] * m[6] * m[11] + 
              m[1] * m[7] * m[10] + 
              m[5] * m[2] * m[11] - 
              m[5] * m[3] * m[10] - 
              m[9] * m[2] * m[7] + 
              m[9] * m[3] * m[6];

    inv[7] = m[0] * m[6] * m[11] - 
             m[0] * m[7] * m[10] - 
             m[4] * m[2] * m[11] + 
             m[4] * m[3] * m[10] + 
             m[8] * m[2] * m[7] - 
             m[8] * m[3] * m[6];

    inv[11] = -m[0] * m[5] * m[11] + 
               m[0] * m[7] * m[9] + 
               m[4] * m[1] * m[11] - 
               m[4] * m[3] * m[9] - 
               m[8] * m[1] * m[7] + 
               m[8] * m[3] * m[5];

    inv[15] = m[0] * m[5] * m[10] - 
              m[0] * m[6] * m[9] - 
              m[4] * m[1] * m[10] + 
              m[4] * m[2] * m[9] + 
              m[8] * m[1] * m[6] - 
              m[8] * m[2] * m[5];

    det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

    if (det == 0)
        return false;

    det = 1.0 / det;

    for (i = 0; i < 16; i++)
        invOut[i] = inv[i] * det;
    return true;
}

/// generate Jacobian Matrix
void Compass::calc_jacob(Vector3i data[], double param[])
{
    double *temp;
    temp = new double[100];
    sphere_fitness(data,param,delta);
    int l;
    for(int i=0;i<4;i++){
        param[i] = param[i] + JACOB_DELTA;
        sphere_fitness(data,param,temp);

        for(int j=0;j<100;j++){
            jacob[i*100+j] = delta[j] - temp[j];                //note the change with response to slight variation(JACOB_DELTA)
        }                                                       // in sphere_parameters
        param[i] = param[i] - JACOB_DELTA;
    }
    delete[] temp;
}

/// calculates Transpose(Jacobian_Matrix)*Jacobian_Matrix + lambda*Identity_Matrix
///@issue: handle inverse fault
void Compass::calc_JTJ_LI(double lambda)
{
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            for(int k=0;k<100;k++){
                JTJ_LI[i*4+j] = JTJ_LI[i*4+j] + jacob[i*100+k]*jacob[j*100+k];    //linear array used to minimise memory footprint
            }
        }
    }
    for(int i=0;i<4;i++){
            JTJ_LI[i*4+i] = JTJ_LI[i*4+i]+lambda;
    }    
    inverse4x4(JTJ_LI,JTJ_LI);                                                //calc and return Inverse of JTJ_LI = [(JT).J + L.I]
}

/// calculates Transpose(Jacobian_Matrix)*Fitness_Matrix
void Compass::calc_JTFI(Vector3i data[], double param[])
{
    sphere_fitness(data,param,delta);
    for(int i=0;i<4;i++){
        for(int j=0;j<100;j++){
            JTFI[i] = JTFI[i] + jacob[i*100+j]*delta[j];
        }
    }
}

/// do iterations of Levenberg_Marquadt on Samples
double Compass::evaluatelm(Vector3i data[], double param[])
{
    double l=1,last_fitness,global_best[4],global_best_f;  //initial state
    double cur_fitness, difference = 1;   //initialise values to be used again
    int g = 0;
    jacob = new double[400];
    delta = new double[100];
    JTJ_LI = new double[16];
    JTFI = new double[4];
    sphere_fitness(data,param,delta);
    last_fitness = square_sum();
    global_best_f = last_fitness;
    hal.console->printf("\n");
    while(g <= 8 || difference < 1e-8){            //close results are generated in 10-20 steps most of the time
        //Initialise everything
        for(int j=0;j<400;j++)
            jacob[j]=0;
        for(int j=0;j<16;j++)
            JTJ_LI[j]=0;
        for(int j=0;j<4;j++)
            JTFI[j]=0;

        calc_jacob(data,param);                  //step  1
        calc_JTJ_LI(l);                                //step  2
        calc_JTFI(data,param);              //step  3
        for(int j=0;j<4;j++){
            for(int p=0;p<4;p++){
                param[j]=param[j]+JTFI[p]*JTJ_LI[j*4+p];    //final step
            }
        }                                                   //LM iteration complete
        
        //pass generated result through conditions
        sphere_fitness(data,param,delta);
        cur_fitness = square_sum();
        
        //hal.console->printf("%lf %lf %lf %lf %lf\n", param[0], param[1], param[2], param[3], cur_fitness);

        if(cur_fitness >= last_fitness){
            l*=GRADIENT;
            g++;
        }else{
            l/=GRADIENT;
            difference = last_fitness - cur_fitness;
            last_fitness = cur_fitness;
            g--;
        }
        if(cur_fitness < global_best_f){
            global_best_f = cur_fitness;
            for(int j=0;j<4;j++)
                global_best[j] = param[j];
        }
        if(cur_fitness<AIMED_FITNESS/2){
            break;
        }
    }

    for(int i=0;i<4;i++){
        param[i] = global_best[i];
    }
    
    //free memory
    delete[] delta;
    delete[] jacob;
    delete[] JTJ_LI;
    delete[] JTFI;
    return global_best_f;
}