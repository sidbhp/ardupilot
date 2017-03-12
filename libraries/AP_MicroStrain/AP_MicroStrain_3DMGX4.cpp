// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

#include <AP_HAL/AP_HAL.h>
#include "AP_MicroStrain_3DMGX4.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <stdio.h>


#define MICROSTRAIN_DEBUGGING 0

#if MICROSTRAIN_DEBUGGING
 # define Debug(fmt, args ...)  do {hal.console->printf(fmt "\n", ## args); } while(0)
#else
 # define Debug(fmt, args ...)
#endif

extern const AP_HAL::HAL& hal;

AP_MicroStrain_3DMGX4::AP_MicroStrain_3DMGX4(AP_MicroStrain &_ms, uint8_t instance,
 AP_MicroStrain::MicroStrain_State &_state, AP_SerialManager &serial_manager) : 
    AP_MicroStrain_Backend(_ms, instance, _state)
{
    memset(&_global_parse_state, 0, sizeof(_global_parse_state));
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_MicroStrain, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_MicroStrain, 0));
    }
}

/* 
   detect if a MicroStrain is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
bool AP_MicroStrain_3DMGX4::detect(AP_MicroStrain &_ms, uint8_t instance, AP_MicroStrain::MicroStrain_State &state, AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_MicroStrain, 0) != nullptr;
}

void AP_MicroStrain_3DMGX4::calc_checksum() {
  uint8_t byte1 = 0, byte2 = 0;

#define add_byte(x)  \
  byte1 += (x);      \
  byte2 += byte1;

  add_byte(SYNC1);
  add_byte(SYNC2);
  add_byte(_global_parse_state.desc);
  add_byte(_global_parse_state.length);

  for (int i = 0; i < _global_parse_state.length; i++) {
    add_byte(_global_parse_state.dat[i]);
  }
#undef add_byte

  _global_parse_state.checksum = (static_cast<uint16_t>(byte1) << 8) + static_cast<uint16_t>(byte2);
}


void AP_MicroStrain_3DMGX4::_parse_char(uint8_t c)
{
    switch(_global_parse_state.step) {
        case 0:
            if(c == SYNC1) {
                _global_parse_state.step++;
            }
            break;
        case 1:
            if(c == SYNC2) {
                _global_parse_state.step++;
            } else {
                _global_parse_state.step = 0;
            }
            break;
        case 2: //read descriptor
            Debug("DESCRIPTOR: ");
            Debug("0x%x ", c);
            _global_parse_state.desc = c;
            _global_parse_state.step++;
            break;
        case 3: //read length
            Debug("DATA LENGTH: ");
            Debug("0x%x ", c);
            _global_parse_state.length = c;
            _global_parse_state.step++;
            _global_parse_state.dat = new uint8_t[_global_parse_state.length];
            if(_global_parse_state.dat == nullptr) {
                hal.console->printf("Failed to allocate space for Data\n");
                memset(&_global_parse_state,0 ,sizeof(_global_parse_state));
            }
            break;
        case 4: //read payload and checksum
            if(_global_parse_state.dat_ptr < _global_parse_state.length - 1) {
                _global_parse_state.dat[_global_parse_state.dat_ptr] = c;
                _global_parse_state.dat_ptr++;
            } else {
                _global_parse_state.dat[_global_parse_state.dat_ptr] = c;
                calc_checksum();
                _global_parse_state.step++;
            }
            break;
        case 5: //Checksum MSB
            _global_parse_state.rcv_checksum = (static_cast<uint16_t>(c) << 8);
            _global_parse_state.step++;
            break;
        case 6: //Checksum LSB
            _global_parse_state.rcv_checksum += static_cast<uint16_t>(c);
            Debug("CHK_RCV: 0x%x CHK_CALC: 0x%x\n",_global_parse_state.rcv_checksum, _global_parse_state.checksum);
            if(_global_parse_state.rcv_checksum == _global_parse_state.checksum) {
                Debug("Checksum correct!\n");
                handle_packet();
            } else {
                hal.console->printf("MicroStrain:  Bad Checksum! CHK_RCV: 0x%x CHK_CALC: 0x%x\n", _global_parse_state.rcv_checksum, _global_parse_state.checksum);
            }
            delete[] _global_parse_state.dat;
            memset(&_global_parse_state,0 ,sizeof(_global_parse_state));
            break;
        default:
            if(_global_parse_state.dat != nullptr) {
                delete[] _global_parse_state.dat;
            }
            memset(&_global_parse_state,0 ,sizeof(_global_parse_state));
            break;
    };
}

void AP_MicroStrain_3DMGX4::handle_packet()
{

    switch(_global_parse_state.desc) {
        case DATA_CLASS_IMU:
            parse_state imu_parse_state;
            memset(&imu_parse_state,0,sizeof(imu_parse_state));
            for(uint8_t i = 0; i < _global_parse_state.length; i++) {
                switch(imu_parse_state.step) {
                    case 0: // Get length
                        Debug("IMU Packet Length: 0x%x\n", _global_parse_state.dat[i]);
                        imu_parse_state.length = _global_parse_state.dat[i];
                        imu_parse_state.step++;
                        break;
                    case 1:
                        Debug("IMU Packet Descriptor: 0x%x\n", _global_parse_state.dat[i]);
                        imu_parse_state.desc = _global_parse_state.dat[i];
                        imu_parse_state.dat = new uint8_t[imu_parse_state.length-2];
                        if(imu_parse_state.dat == nullptr) {
                            hal.console->printf("Failed to allocate space for Data\n");
                            memset(&imu_parse_state,0 ,sizeof(imu_parse_state));
                        }
                        imu_parse_state.step++;
                        Debug("Data Packet: ");
                        break;
                    case 2:
                        Debug("0x%x ", _global_parse_state.dat[i]);
                        if(imu_parse_state.dat_ptr < imu_parse_state.length - 3) {
                            imu_parse_state.dat[imu_parse_state.dat_ptr] = _global_parse_state.dat[i];
                            imu_parse_state.dat_ptr++;
                        } else {
                            imu_parse_state.dat[imu_parse_state.dat_ptr] = _global_parse_state.dat[i];
                            handle_imu_packet(imu_parse_state);
                            imu_parse_state.step++;
                            Debug("\n");
                            delete[] imu_parse_state.dat;
                            memset(&imu_parse_state, 0, sizeof(imu_parse_state));
                        }
                        break;
                    default:
                        break;
                };
            }
            break;

        default:
            break;
    };
}

void AP_MicroStrain_3DMGX4::extract_floats(float fout[], uint8_t cin[], uint8_t num)
{
    for(uint8_t i = 0; i < num; i++) {
        uint32_t temp = (static_cast<uint32_t>(cin[i*4]) << 24) + (static_cast<uint32_t>(cin[i*4+1]) << 16) + (static_cast<uint32_t>(cin[i*4+2]) << 8) + static_cast<uint32_t>(cin[i*4+3]);
        memcpy(&fout[i],&temp,sizeof(temp));
    }
}

void AP_MicroStrain_3DMGX4::handle_imu_packet(parse_state &imu_parse_state)
{

    switch(imu_parse_state.desc) {
        case FIELD_ACCELEROMETER:
            if(last_accel_time == 0) {
                last_accel_time = AP_HAL::micros();
                break; // ignore first sample
            }
            extract_floats(processed_data.accel, imu_parse_state.dat, 3);
            processed_data.delta_accel_time = 0.02f;//float(AP_HAL::micros() - last_accel_time)/1000000.0f;
            processed_data.delta_velocity[0] = processed_data.accel[0]*processed_data.delta_accel_time;
            processed_data.delta_velocity[1] = processed_data.accel[1]*processed_data.delta_accel_time;
            processed_data.delta_velocity[2] = processed_data.accel[2]*processed_data.delta_accel_time;
            //hal.console->printf("%x %x %x\n", *(uint32_t*)(&processed_data.accel[0]), *(uint32_t*)(&processed_data.accel[1]), *(uint32_t*)(&processed_data.accel[2]));
            last_accel_time = AP_HAL::micros();
            processed_data.fields |= 1;
            break;
        case FIELD_GYROSCOPE:
            if(last_gyro_time == 0) {
                last_gyro_time = AP_HAL::micros();
                break;  //ignore first sample
            }
            extract_floats(processed_data.gyro, imu_parse_state.dat, 3);
            processed_data.delta_gyro_time = 0.02f;//float(AP_HAL::micros() - last_gyro_time)/1000000.0f;
            processed_data.delta_angle[0] = processed_data.gyro[0]*processed_data.delta_gyro_time;
            processed_data.delta_angle[1] = processed_data.gyro[1]*processed_data.delta_gyro_time;
            processed_data.delta_angle[2] = processed_data.gyro[2]*processed_data.delta_gyro_time;
            //hal.console->printf("%x %x %x\n", *(uint32_t*)(&processed_data.gyro[0]), *(uint32_t*)(&processed_data.gyro[1]), *(uint32_t*)(&processed_data.gyro[2]));

            static uint32_t last_print_time;
            if(AP_HAL::millis() - last_print_time > 100) {
                //hal.console->printf("%f %f %f\n", processed_data.gyro[0], processed_data.gyro[1], processed_data.gyro[2]);
                //hal.console->printf("%f\n", processed_data.delta_gyro_time);
            }
            last_gyro_time = AP_HAL::micros();
            processed_data.fields |= 1<<1;
            break;
        case FIELD_MAGNETOMETER:
            extract_floats(processed_data.mag, imu_parse_state.dat, 3);
            processed_data.fields |= 1<<2;
            break;
        case FIELD_BAROMETER:
            extract_floats(&processed_data.press, imu_parse_state.dat, 1);
            processed_data.fields |= 1<<3;
            break;
        default:
            hal.console->printf("Unsupported Data Field %x", imu_parse_state.desc);
            break;
    };
    //if(processed_data.fields == 0xF) {
    //    static uint32_t last_print_time;
    //    if(AP_HAL::millis() - last_print_time > 1000) {
    //        hal.console->printf("MicroStrain Data: \n");
    //        hal.console->printf("Accel: %.6f %.6f %.6f\n", processed_data.accel[0], processed_data.accel[1], processed_data.accel[2]);
    //        hal.console->printf("delta_velocity: %.6f %.6f %.6f %.6f\n",processed_data.delta_accel_time ,  processed_data.delta_velocity[0], processed_data.delta_velocity[1], processed_data.delta_velocity[2]);
    //        hal.console->printf("Gyro: %.6f %.6f %.6f\n", processed_data.gyro[0], processed_data.gyro[1], processed_data.gyro[2]);
    //        hal.console->printf("delta_angle: %.6f %.6f %.6f %.6f\n", processed_data.delta_gyro_time , processed_data.delta_angle[0], processed_data.delta_angle[1], processed_data.delta_angle[2]);
    //        hal.console->printf("Mag: %.6f %.6f %.6f\n", processed_data.mag[0], processed_data.mag[1], processed_data.mag[2]);
    //        hal.console->printf("Baro: %.6f\n\n\n", processed_data.press);
    //        last_print_time = AP_HAL::millis();
    //    }
    //    processed_data.fields = 0x0;
    //}
}

void AP_MicroStrain_3DMGX4::update(void)
{
    processed_data.fields = 0x0;
}

void AP_MicroStrain_3DMGX4::read_data(void)
{
    int16_t nbytes = uart->available();
//    static uint32_t last_print_time, last_time;
//    if(AP_HAL::millis() - last_print_time  > 1000) {
//        last_print_time = AP_HAL::millis();
//        hal.console->printf("MicroStrain Rate: %f\n", 1000.0f/float(AP_HAL::millis() - last_time));
//    }
//    last_time = AP_HAL::millis();

    while (nbytes-- > 0) {
        char c = uart->read();
        _parse_char(c);
    }
    return;
}