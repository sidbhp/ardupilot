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
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#pragma once
#include <AP_HAL/utility/RingBuffer.h>
#include "AP_Radio_backend.h"
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <AP_HAL/AP_HAL.h>

#define SYSLINK_MTU 32

#define CRTP_START_BYTE  0xAA
#define SYSLINK_START_BYTE1 0xBC
#define SYSLINK_START_BYTE2 0xCF

// Defined packet types
#define SYSLINK_GROUP_MASK    0xF0

#define SYSLINK_RADIO_GROUP         0x00
#define SYSLINK_RADIO_RAW           0x00
#define SYSLINK_RADIO_CHANNEL       0x01
#define SYSLINK_RADIO_DATARATE      0x02
#define SYSLINK_RADIO_CONTWAVE      0x03
#define SYSLINK_RADIO_RSSI          0x04
#define SYSLINK_RADIO_ADDRESS       0x05
#define SYSLINK_RADIO_RAW_BROADCAST 0x06
#define SYSLINK_RADIO_POWER         0x07

#define SYSLINK_PM_GROUP              0x10
#define SYSLINK_PM_SOURCE             0x10
#define SYSLINK_PM_ONOFF_SWITCHOFF    0x11
#define SYSLINK_PM_BATTERY_VOLTAGE    0x12
#define SYSLINK_PM_BATTERY_STATE      0x13
#define SYSLINK_PM_BATTERY_AUTOUPDATE 0x14

#define SYSLINK_OW_GROUP    0x20
#define SYSLINK_OW_SCAN     0x20
#define SYSLINK_OW_GETINFO  0x21
#define SYSLINK_OW_READ     0x22
#define SYSLINK_OW_WRITE    0x23

// Limited by the CRTP packet which is limited by the ESB protocol used by the NRF
#define SYSLINK_MAX_DATA_LEN	32

#define SYSLINK_RADIO_RATE_250K 0
#define SYSLINK_RADIO_RATE_1M 1
#define SYSLINK_RADIO_RATE_2M 2

#define CRTP_MAX_DATA_SIZE 30

#define CRTP_HEADER(port, channel) (((port & 0x0F) << 4) | (channel & 0x0F))

#define CRTP_IS_NULL_PACKET(P) ((P.header&0xF3)==0xF3)

class AP_Radio_CrazyRadio : public AP_Radio_backend {
public:
    AP_Radio_CrazyRadio(AP_Radio &_radio);

    // init - initialise radio
    bool init(void);

    // init - reset radio
    bool reset(void);

    // send a packet
    bool send(const uint8_t *pkt, uint16_t len);

    // start bind process as a receiver
    void start_recv_bind(void);

    // return time in microseconds of last received R/C packet
    uint32_t last_recv_us(void)
    {
        return _last_recv_us;
    }

    // return number of input channels
    uint8_t num_channels(void);

    // return current PWM of a channel
    uint16_t read(uint8_t chan);

    // update status
    void update(void)
    {
        mavlink_update();
    }

    // get TX fw version
    uint32_t get_tx_version(void);

    // get radio statistics structure
    const AP_Radio::stats &get_stats(void);

    // set the 2.4GHz wifi channel used by companion computer, so it can be avoided
    void set_wifi_channel(uint8_t channel);

    void mavlink_update(uint32_t max_time_us = 1000);

    void mavlink_write(const uint8_t *pkt, uint8_t len);

    uint8_t mavlink_read();

    int16_t mavlink_available();

private:
    AP_Radio::stats _stats;
    ByteBuffer _readbuf{0};
    ByteBuffer _writebuf{0};
    uint32_t _last_recv_us;
    uint16_t pwm_channels[5] = {0};

    struct __attribute__((packed)) Syslink_Packet {
        uint8_t type;
        uint8_t length;
        uint8_t data[SYSLINK_MAX_DATA_LEN];
        uint8_t cksum[2];
    };

    enum {
        SYSLINK_STATE_START = 0,
        SYSLINK_STATE_TYPE,
        SYSLINK_STATE_LENGTH,
        SYSLINK_STATE_DATA,
        SYSLINK_STATE_CKSUM
    } _state;

    uint16_t _index;

    enum Port {
        CRTP_PORT_CONSOLE          = 0x00,
        CRTP_PORT_PARAM            = 0x02,
        CRTP_PORT_SETPOINT         = 0x03,
        CRTP_PORT_MEM              = 0x04,
        CRTP_PORT_LOG              = 0x05,
        CRTP_PORT_LOCALIZATION     = 0x06,
        CRTP_PORT_SETPOINT_GENERIC = 0x07,
        CRTP_PORT_MAVLINK          = 0x08,
        CRTP_PORT_PLATFORM         = 0x0D,
        CRTP_PORT_LINK             = 0x0F,
    } _port;

    struct __attribute__((packed)) CRTP_Packet {
        uint8_t size;                         //< Size of data
        union {
            struct {
                union {
                    uint8_t header;                 //< Header selecting channel and port
                    struct {
                        uint8_t channel     : 2;      //< Selected channel within port
                        uint8_t reserved    : 2;
                        uint8_t port        : 4;      //< Selected port
                    };
                };
                uint8_t data[CRTP_MAX_DATA_SIZE]; //< Data
            };
            uint8_t raw[CRTP_MAX_DATA_SIZE+1];  //< The full packet "raw"
        };
    };

    struct CRTP_Commander {
        float roll; // -20 to 20
        float pitch;
        float yaw; // -150 to 150
        uint16_t thrust;
    };

    bool address_set;
    bool chan_set;
    bool data_rate_set;
    uint8_t _settings_count;
    AP_HAL::UARTDriver *_uart;
    void compute_cksum(Syslink_Packet *_syslink_pkt);
    void handlePacket(Syslink_Packet *_syslink_pkt);
    void mavlink_send();
    void send_message(Syslink_Packet *msg);
    void set_datarate(uint8_t datarate);
    void set_address(uint64_t address);
    void set_channel(uint8_t channel);
};
#endif