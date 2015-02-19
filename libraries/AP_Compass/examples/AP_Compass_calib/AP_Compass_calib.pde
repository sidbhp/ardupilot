/*
 *       Compass Calibration Prototype Code
 */

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <StorageManager.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_FLYMAPLE.h>
#include <AP_HAL_Empty.h>
#include <AP_HAL_VRBRAIN.h>

#include <AP_Math.h>    // ArduPilot Mega Vector/Matrix math Library
#include <AP_Declination.h>
#include <AP_Compass.h> // Compass Library
#include <GCS_MAVLink.h>
#include <AP_Scheduler.h>
#include <DataFlash.h>
#include <AP_GPS.h>
#include <AP_Vehicle.h>
#include <AP_InertialSensor.h>
#include <Filter.h>
#include <AP_Baro.h>
#include <AP_AHRS.h>
#include <AP_Airspeed.h>
#include <AP_NavEKF.h>
#include <AP_ADC.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_Notify.h>
#include <AP_Mission.h>
#include <AP_Terrain.h>
#include <AP_Rally.h>
#include <AP_BattMonitor.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

#define CONFIG_COMPASS HAL_COMPASS_DEFAULT
#define AIMED_FITNESS  1.0     //desired max value of fitness
#define MAX_ITERS      10    //no. of iterations after which if convergence doesn't happen calib process is said to be failed
#define SAMPLE_RATE    5     //no. of samples/sec
#define MAX_OFF_VAL    1000  //this value makes sure no insane value get through
#define MIN_OFF_VAL   -1000
#define SAMPLE_DIST    50
#define GRADIENT       10
/*GRADIENT value specify the speed with which the optimiser will try to converge
very high value means very low chance of convergence as the steps taken will
be too large, while very low value will ensure the convergence will occur but
may take huge amount of time. Striking balance with this factor is the key to
-wards successful result in viable time period.
*/
#define JACOB_DELTA 0.000000001

#if CONFIG_COMPASS == HAL_COMPASS_PX4
static AP_Compass_PX4 compass;
#elif CONFIG_COMPASS == HAL_COMPASS_VRBRAIN
static AP_Compass_VRBRAIN compass;
#elif CONFIG_COMPASS == HAL_COMPASS_HMC5843
static AP_Compass_HMC5843 compass;
#elif CONFIG_COMPASS == HAL_COMPASS_HIL
static AP_Compass_HIL compass;
#elif CONFIG_COMPASS == HAL_COMPASS_AK8963
static AP_Compass_AK8963_MPU9250 compass;
#else
 #error Unrecognized CONFIG_COMPASS setting
#endif

uint32_t timer;
int calib_complete;

using namespace std;

struct Vector{
    int x,y,z;
};

/* calculates fitness of points to sphere
args:
param[] --- radius=param[0], offset1=param[1], offset2=param[2], offset3=param[3]
data[]  --- samples collected from magnetometer
*/
void sphere_fitness(double param[],Vector data[],double delta[])
{
    double a = 1/(param[0]*param[0]);
    for(int i=0;i<100;i++){
        delta[i] = 1 - a*((data[i].x+param[1])*(data[i].x+param[1]) +
                          (data[i].y+param[2])*(data[i].y+param[2]) +
                          (data[i].z+param[3])*(data[i].z+param[3]));
    }
}
/*matrix inverse code 
copied from gluInvertMatrix implementation in opengl for 4x4 matrices
*/
int inverse(double m[],double invOut[])
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
        return -1;

    det = 1.0 / det;

    for (i = 0; i < 16; i++)
        invOut[i] = inv[i] * det;
    return 0;
}

/* does Squared Sum with provided set of fitness data(delta) as generated in sphere_fitness
args:
delta[] --- fitness of single point to sphere with said parameters[4]
*/
double square_sum(double delta[])
{
    double sqsum=0;
    for(int i=0;i<100;i++){
        sqsum = sqsum + delta[i]*delta[i];
    }
    return sqsum;
}
/* generate Jacobian Matrix
args:
param[] --- radius=param[0], offset1=param[1], offset2=param[2], offset3=param[3]
data[]  --- samples collected from magnetometer
*/
void calc_jacob(double param[],Vector data[],double jacob[],double delta[])
{
    double temp[100]={0};
    sphere_fitness(param,data,delta);
    int l;
    for(int i=0;i<4;i++){
        param[i] = param[i] + JACOB_DELTA;
        sphere_fitness(param,data,temp);

        for(int j=0;j<100;j++){
            jacob[i*100+j] = delta[j] - temp[j];                //note the change with response to slight variation(JACOB_DELTA)
        }                                                       // in parameters
        param[i] = param[i] - JACOB_DELTA;
    }

}
/*calculates Transpose(Jacobian_Matrix)*Jacobian_Matrix
args:
param[] --- radius=param[0], offset1=param[1], offset2=param[2], offset3=param[3]
data[]  --- samples collected from magnetometer
*/
void calc_JTJ_LI(double jacob[],double lambda,double JTJ_LI[])
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
    inverse(JTJ_LI,JTJ_LI);                                                //calc and return Inverse of JTJ_LI = [(JT).J + L.I]
}
/*calculates Transpose(Jacobian_Matrix)*Fitness_Matrix
args:
param[] --- radius=param[0], offset1=param[1], offset2=param[2], offset3=param[3]
data[]  --- samples collected from magnetometer
*/
void calc_JTFI(double param[],Vector data[],double jacob[],double JTFI[],double delta[])
{
    sphere_fitness(param,data,delta);
    for(int i=0;i<4;i++){
        for(int j=0;j<100;j++){
            JTFI[i] = JTFI[i] + jacob[i*100+j]*delta[j];
        }
    }
}


double evaluatelm(Vector data[], double param[])
{
    double l=1,last_fitness,delta[100]={0},global_best[4],global_best_f;  //initial state
    double jacob[400]={0},JTJ_LI[16]={0},JTFI[4]={0},cur_fitness;   //initialise values to be used again
    sphere_fitness(param,data,delta);
    last_fitness = square_sum(delta);
    global_best_f = last_fitness;
    
    for(int i=0;i<20;i++){                                 //close results are generated in 10-20 steps most of the time
        calc_jacob(param,data,jacob,delta);                    //step  1
        calc_JTJ_LI(jacob,l,JTJ_LI);                           //step  2
        calc_JTFI(param,data,jacob,JTFI,delta);                //step  3
        for(int j=0;j<4;j++){
            for(int p=0;p<4;p++){
                param[j]=param[j]+JTFI[p]*JTJ_LI[j*4+p];    //final step
            }
        }                                                   //LM iteration complete
        
        //pass generated result through conditions
        sphere_fitness(param,data,delta);
        cur_fitness = square_sum(delta);
        if(cur_fitness > last_fitness){
            l*=GRADIENT;
        }else if(cur_fitness < last_fitness){
            l/=GRADIENT;
        }
        if(cur_fitness < global_best_f){
            global_best_f = cur_fitness;
            for(int j=0;j<4;j++)
                global_best[j] = param[j];
        }
        //Initialise everything
        for(int j=0;j<400;j++)
            jacob[j]=0;
        for(int j=0;j<16;j++)
            JTJ_LI[j]=0;
        for(int j=0;j<4;j++)
            JTFI[j]=0;
        last_fitness = cur_fitness;
        if(cur_fitness<AIMED_FITNESS/2){
            break;
        }
    }

    for(int i=0;i<4;i++){
        param[i] = global_best[i];
    }
    return global_best_f;
}
bool validate_sample(Vector data[],int count){
    for(int i=0;i<count;i++){
        if((data[i].x == data[count].x) && (data[i].z == data[count].z) && (data[i].y == data[count].y)){
            return false;
        }
    }
    return true;
}
void setup()
{
    hal.console->println("Compass Calibration");

    if (!compass.init()) {
        hal.console->println("compass initialisation failed!");
        while (1) ;
    }
    hal.console->println("init done");

    compass.set_and_save_offsets(0,0,0,0); // set offsets to account for surrounding interference
    compass.set_declination(ToRad(0.0)); // set local difference between magnetic north and true north

    hal.console->print("Compass auto-detected as: ");
    switch( compass.product_id ) {
    case AP_COMPASS_TYPE_HIL:
        hal.console->println("HIL");
        break;
    case AP_COMPASS_TYPE_HMC5843:
        hal.console->println("HMC5843");
        break;
    case AP_COMPASS_TYPE_HMC5883L:
        hal.console->println("HMC5883L");
        break;
    case AP_COMPASS_TYPE_PX4:
        hal.console->println("PX4");
        break;
    case AP_COMPASS_TYPE_AK8963_MPU9250:
        hal.console->println("AK8963");
        break;
    default:
        hal.console->println("unknown");
        break;
    }

    hal.scheduler->delay(1000);
    calib_complete = 0;
    timer = hal.scheduler->micros();
}
void loop()
{   
    double param[4] = {20,0,0,0},global_best_f,best_past_param[4]={20,0,0,0}, best_past_f;  
    int iters = 0,sat = 0,passed = 0;
    bool ofb;
    Vector data[100];
    
    if((hal.scheduler->micros()- timer) > 100000L && !calib_complete){
        timer = hal.scheduler->micros();
        
        hal.console->print("\r");
        do{
            //collect Samples
            int c=0;
            while(c < 100){
                compass.read();
                if (!compass.healthy()) {
                    hal.console->print("not healthy      \r");
                    continue;
                }
                const Vector3f &mag = compass.get_field();
                if( c >= 1){
                    if((abs(data[c-1].x - (int)mag.x) > SAMPLE_DIST) ||
                        (abs(data[c-1].y - (int)mag.y) > SAMPLE_DIST) ||
                        (abs(data[c-1].z - (int)mag.z) > SAMPLE_DIST)){
                        data[c].x = mag.x;
                        data[c].y = mag.y;
                        data[c].z = mag.z;
                        if(validate_sample(data,c)){
                            c++;
                            hal.console->printf("Sample Collection Progress[%d] [%d][%d][%d]      \r",c,(int)mag.x,(int)mag.y,(int)mag.z);
                        }
                    }
                }else{
                    data[c].x = mag.x;
                    data[c].y = mag.y;
                    data[c].z = mag.z;
                    c++;
                }
                hal.scheduler->delay(1000/SAMPLE_RATE);                  //5 samples/second
            }
            //Samples collected, let's start crunching data
            global_best_f = evaluatelm(data,param);

            while(global_best_f > AIMED_FITNESS/2){
                global_best_f = evaluatelm(data,param);             //crunching data
                hal.console->printf("\nOff1: %.2f Off2: %.2f Off3: %.2f fitness: %.5f \n\n",param[1],param[2],param[3],global_best_f);
                
                if(best_past_f == global_best_f){                
                    break;                                          //result saturation, we need a fresh start
                }
                for(int i=0;i<4;i++){                
                    if(param[i] > MAX_OFF_VAL or param[i] < MIN_OFF_VAL){            //check if result is not out of bounds
                        ofb = true;
                        break;
                    }
                    else{
                        ofb = false;
                    }
                }
                if(ofb){                                                //break iteration if ofb, we need a fresh start
                    break;
                }
                for(int i=0;i<4;i++){
                    best_past_param[i] = param[i];                      //store best past parameters to be used if anything
                }                                                       //goes downhill within the iteration
                best_past_f = global_best_f;
            }
            for(int i=0;i<4;i++){
                param[i] = best_past_param[i];                          //get ready for next iteration
            }


            hal.console->printf("\nOff1: %.2f Off2: %.2f Off3: %.2f fitness: %.5f \n\n",param[1],param[2],param[3],global_best_f);
            iters++;
            //check if we are getting close
            if(global_best_f <= AIMED_FITNESS){
                passed += 1;                      //total consecutive fitness test passed
                hal.console->printf("Good Fitness Test Passed:  %d\n",passed);
            }else{
                passed=0;
            }
            //check if this is it
            if(passed == 3){
                calib_complete = 1;                             //we are done no more processing
                hal.console->printf("\nCalibration Completed!!!! Best Match: \nOff1: %.2f Off2: %.2f Off3: %.2f ",param[1],param[2],param[3]);
                hal.scheduler->delay(10);
                return;
            }
        }while(iters < MAX_ITERS);             //if false means good covergence didn't occured in 10 iters: Callibration failed
        hal.console->printf("\nCalibration Failed!!!!");
        calib_complete = 1;
        hal.scheduler->delay(10);
        return;
    }else{
	    hal.scheduler->delay(1);
    }
    
    //print result

    /*
            Result generated after this point will be fed again to the above loop but with new data_set
    */
}

AP_HAL_MAIN();
