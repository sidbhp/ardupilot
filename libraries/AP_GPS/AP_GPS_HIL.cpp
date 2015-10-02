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
  /*
    return values:
    |   -1: error message
    |    0: no message (tells caller to please read more data from the stream)
    |    1: input observation data
    |    2: input ephemeris
    |    3: input sbas message
    |    9: input ion/utc parameter
  */
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Notify/AP_Notify.h>
#include "AP_GPS.h"
#include <stdio.h>
#include <time.h>
#include <string.h>
#define DEBUG 0
extern const AP_HAL::HAL& hal;


void AP_GPS::setHIL_SFRB_hdr(uint32_t timems,uint8_t gnssId,uint8_t svId,uint8_t freqId,uint8_t numWords)
{
    _has_raw = true;
    memset(&_sfrbx,0,sizeof(_sfrbx));
    _sfrbx.gnssId = gnssId;
    _sfrbx.svId = svId;
    _sfrbx.freqId = freqId;
    _sfrbx.numWords = numWords;
    _raw_frames = 0;
}

void AP_GPS::setHIL_RAW_hdr(uint32_t timems,double rcvTime,uint16_t week,int8_t leapS,uint8_t numMeas,uint8_t recStat)
{
    _has_raw = true;
    memset(&_rawmeas,0,sizeof(_rawmeas));
    _rawmeas.rcvTow = rcvTime;
    _rawmeas.week = week;
    _rawmeas.leapS = leapS;
    _rawmeas.numMeas = numMeas;
    _rawmeas.recStat = recStat;
    _cur_meas = 0;
    //printf("Header!!\n");
}

void AP_GPS::setHIL_SFRB_data(uint32_t timems,uint8_t numWord,uint32_t dwrd)
{

    if(_raw_frames < _sfrbx.numWords) {
        _sfrbx.dwrd[_raw_frames] = dwrd;
        _raw_frames++;
    }
    if(_raw_frames == _sfrbx.numWords) {
        _raw_frames++;
        uint8_t *data;
        uint16_t len = sizeof(ubx_sfrbx)-((8-_sfrbx.numWords)*(sizeof(uint32_t)));
        data = new uint8_t[len];
        memcpy(data,&_sfrbx,len);
        int8_t ret = emulate_ubx(data,CLASS_RXM,MSG_RXM_SFRBX,len);
#if DEBUG
        switch(ret) {
            case -1: printf("SFRBX: Bad Data!!\n"); break;
            case  0: printf("SFRBX: Reading!!\n"); break;
            case  1: printf("SFRBX: OBSERVATION DATA\n"); break;
            case  2: printf("SFRBX: EPHIMERIS DATA\n"); break;
            case  3: printf("SFRBX: SBAS MSG\n"); break;
            case  9: printf("SFRBX: ION/UTC PARAMS\n"); break;
            default: printf("Unknown Return %d",ret);
        };
#endif
        if(ret == 9) {
            _processing_complete = true;
        }
        double *rs, *dts, *var;
        prcopt_t opt = prcopt_default;
        opt.mode = PMODE_KINEMA;
        int32_t *svh;
        sol_t sol;
        double *azel;
        char msg[20];
        double pos[3];
        prcopt_t prcopt=prcopt_default;
        rs = new double[_raw.obs.n*6];
        dts = new double[_raw.obs.n*2];
        var = new double[_raw.obs.n];
        svh = new int32_t[_raw.obs.n];
        azel = new double[_raw.obs.n*2];
        //satposs(_raw.time, _raw.obs.data, _raw.obs.n, &_raw.nav,
        //            EPHOPT_SBAS, rs, dts, var, svh);

        for(uint8_t i=0;i<_raw.obs.n;i++) {
            //printf("SAT:%d, Time: {x:%lf,y:%lf,z:%lf}, {vx:%lf,vy:%lf,vz:%lf}, {clock_bias:%lf,clock_drift:%lf}\n",i+1,
            //        rs[i*6],rs[i*6+1],rs[i*6+2],rs[i*6+3],rs[i*6+4],rs[i*6+5], dts[i*2],dts[i*2+1]);
        }
        fflush(stdout);
        delete[] rs;
        delete[] dts;
        delete[] var;
        delete[] svh;
        delete[] azel;
        delete[] data;
    }
}

void AP_GPS::setHIL_RAW_data(uint32_t timems,double prMes,double cpMes,float doMes,uint8_t gnss,uint8_t sv,uint8_t freq, uint16_t lock,uint8_t cno,uint8_t prD,uint8_t cpD,uint8_t doD,uint8_t trk)
{
    if(_cur_meas < _rawmeas.numMeas) {
        _rawmeas.svinfo[_cur_meas].prMes = prMes;
        _rawmeas.svinfo[_cur_meas].cpMes = cpMes;
        _rawmeas.svinfo[_cur_meas].doMes = doMes;
        _rawmeas.svinfo[_cur_meas].gnss = gnss;
        _rawmeas.svinfo[_cur_meas].sv = sv;
        _rawmeas.svinfo[_cur_meas].freq = freq;
        _rawmeas.svinfo[_cur_meas].lock = lock;
        _rawmeas.svinfo[_cur_meas].cno = cno;
        _rawmeas.svinfo[_cur_meas].prD = prD;
        _rawmeas.svinfo[_cur_meas].cpD = cpD;
        _rawmeas.svinfo[_cur_meas].doD = doD;
        _rawmeas.svinfo[_cur_meas].trk = trk;
        _cur_meas++;
        //::printf("%d/%d\n",_cur_meas,_rawmeas.numMeas);
    }
    if(_cur_meas == _rawmeas.numMeas) {        //ship it
        _cur_meas++;
        uint8_t *data;
        uint16_t len = sizeof(ubx_rawx)-((32-_rawmeas.numMeas)*(sizeof(AP_GPS::ubx_rawx::ubx_rxm_rawx_sv)));
        data = new uint8_t[len];
        memcpy(data,&_rawmeas,len);
        int8_t ret = emulate_ubx(data,CLASS_RXM,MSG_RXM_RAWX,len);
#if DEBUG
        switch(ret) {
            case -1: printf("RAW: Bad Data!!\n"); break;
            case  0: printf("RAW: Reading!!\n"); break;
            case  1: printf("RAW: OBSERVATION DATA Sats: %d\n",_raw.obs.n); break;
            default: printf("RAW: Invalid return %d\n", ret);
        };
#endif
        delete[] data;
    }
}


int8_t AP_GPS::emulate_ubx(uint8_t* data, uint8_t classid, uint8_t msgid, uint16_t len)
{
    uint8_t hdr[] = {PREAMBLE1,PREAMBLE2,classid,msgid,(len)&0x00FF,(len>>8 & 0x00FF)};
    if(!_init_raw) {
        if(init_raw(&_raw)) {
            printf("Raw Structure Intitialised!!\n");
        } else {
            printf("Raw Structure Intitialisation failed!!\n");
        }
        strcat(_raw.opt,"-EPHALL");
        _init_raw = true;
    }
    //printf("data: 0x%X%X\n",(len>>8 & 0x00FF),(len)&0x00FF);
    int8_t ret;
    //generate ck_a and ck_b
    uint8_t ck_a = 0,ck_b = 0;
    uint8_t *pkt;
    uint16_t pkt_len = sizeof(hdr)+len+2;
    pkt = new uint8_t[pkt_len];

    memcpy(pkt,hdr,sizeof(hdr));
    memcpy(pkt+sizeof(hdr),data,len);

    for(uint16_t i=2; i < pkt_len-2;i++) {
        ck_a = (ck_a + pkt[i]);
        ck_b = (ck_b + ck_a);
    }
    pkt[len+sizeof(hdr)] = ck_a;
    pkt[len+sizeof(hdr)+1] = ck_b;

    for(uint16_t i=0;i<pkt_len;i++) {
        ret = input_raw(&_raw,STRFMT_UBX,pkt[i]);
    }
    delete[] pkt;
    return ret;
}
