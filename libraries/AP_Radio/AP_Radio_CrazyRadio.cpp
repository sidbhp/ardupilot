/*
  driver for CrazyRadio
*/

#include "AP_Radio_CrazyRadio.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <stdio.h>
#include "hal.h"

#pragma GCC optimize("O0")

#define Debug(fmt, args...)   //do { hal.console->printf(fmt, ##args); } while (0)

extern const AP_HAL::HAL& hal;

AP_Radio_CrazyRadio::AP_Radio_CrazyRadio(AP_Radio &_radio) :
    AP_Radio_backend(_radio),
    address_set(false),
    chan_set(false),
    data_rate_set(false)
{}

const char *syslink_magic = "\xbc\xcf";

// init - initialise radio
bool AP_Radio_CrazyRadio::init(void)
{
    AP_SerialManager *serial_manager = AP_SerialManager::get_instance();
    if (!serial_manager) {
        return false;
    }
    _uart = serial_manager->find_serial(AP_SerialManager::SerialProtocol_CrazyRadio,0);
    if (!_uart) {
        return false;
    }
    _readbuf.set_size(256);
    _writebuf.set_size(512);
    set_datarate(radio.cr_rate);
    uint64_t address = ((uint64_t)(radio.cr_addr1) << 24) | (uint64_t)(radio.cr_addr2);
    set_address(address);
    set_channel(radio.cr_chan);
    return true;
}

// init - reset radio
bool AP_Radio_CrazyRadio::reset(void)
{
    return true;
}

// send a packet
bool AP_Radio_CrazyRadio::send(const uint8_t *pkt, uint16_t len)
{
    return true;
}

void AP_Radio_CrazyRadio::mavlink_write(const uint8_t *pkt, uint8_t len)
{
    _writebuf.write(pkt, len);
}

void AP_Radio_CrazyRadio::mavlink_send()
{
    CRTP_Packet crtp_msg;
    Syslink_Packet msg;
    // Queue and send next time we get a RAW radio packet
    uint8_t remaining = _writebuf.available();
    if (remaining == 0) {
        return;
    }

    int datasize = MIN(remaining, CRTP_MAX_DATA_SIZE);
    crtp_msg.size = datasize + sizeof(crtp_msg.header);
    crtp_msg.port = CRTP_PORT_MAVLINK;
    _writebuf.read(crtp_msg.data, datasize);
    msg.type = SYSLINK_RADIO_RAW;
    memcpy(&msg.length, &crtp_msg, sizeof(CRTP_Packet));
    //Debug("%d %d %d\n", remaining, msg.length, msg.type);
    send_message(&msg);
}


void AP_Radio_CrazyRadio::set_datarate(uint8_t datarate)
{
    Syslink_Packet slp;

    slp.type = SYSLINK_RADIO_DATARATE;
    slp.length = 1;
    slp.data[0] = datarate;
    send_message(&slp);
}

void AP_Radio_CrazyRadio::set_address(uint64_t address)
{
    Syslink_Packet slp;

    slp.type = SYSLINK_RADIO_ADDRESS;
    slp.length = 5;
    memcpy(&slp.data[0], &address, 5);
    send_message(&slp);
}

void AP_Radio_CrazyRadio::set_channel(uint8_t channel)
{
    Syslink_Packet slp;

    slp.type = SYSLINK_RADIO_CHANNEL;
    slp.length = 1;
    slp.data[0] = channel;
    send_message(&slp);
}

void AP_Radio_CrazyRadio::send_message(Syslink_Packet *msg)
{
    compute_cksum(msg);
    _uart->write((const uint8_t*)syslink_magic, 2);
    _uart->write((const uint8_t*)&msg->type, sizeof(msg->type));
    _uart->write((const uint8_t*)&msg->length, sizeof(msg->length));
    _uart->write((const uint8_t*)msg->data, msg->length);
    _uart->write((const uint8_t*)&msg->cksum, sizeof(msg->cksum));
}

uint8_t AP_Radio_CrazyRadio::mavlink_read()
{
    uint8_t c = 0;
    _readbuf.read_byte(&c);
    return c;
}


int16_t AP_Radio_CrazyRadio::mavlink_available()
{
    return _readbuf.available();
}

// start bind process as a receiver
void AP_Radio_CrazyRadio::start_recv_bind(void)
{

}

// return number of input channels
uint8_t AP_Radio_CrazyRadio::num_channels(void)
{
    return 5;
}

// return current PWM of a channel
uint16_t AP_Radio_CrazyRadio::read(uint8_t chan)
{
    if (chan >= 5) {
        return 0;
    }
    return pwm_channels[chan];
}

void AP_Radio_CrazyRadio::mavlink_update(uint32_t max_time_us)
{
    Syslink_Packet _syslink_pkt;
    memset(&_syslink_pkt, 0, sizeof(Syslink_Packet));
    uint16_t nbytes = _uart->available();
    uint8_t c;
    int16_t val;

    for (uint16_t i=0; i<nbytes; i++) {
        val = _uart->read();
        if (val < 0) {
            break;
        }
        c = val;
        switch (_state) {
        case SYSLINK_STATE_START:
            if (c == (uint8_t)syslink_magic[_index]) {
                _index++;

            }
            else {
                _index = 0;
            }

            if (_index == 2) {
                _state = SYSLINK_STATE_TYPE;
            }

            break;

        case SYSLINK_STATE_TYPE:
            _syslink_pkt.type = c;
            _state = SYSLINK_STATE_LENGTH;
            break;

        case SYSLINK_STATE_LENGTH:
            _syslink_pkt.length = c;

            if (c > SYSLINK_MAX_DATA_LEN) { // Too long
                _state = SYSLINK_STATE_START;
            }
            else {
                _state = c > 0 ? SYSLINK_STATE_DATA : SYSLINK_STATE_CKSUM;
            }

            _index = 0;
            break;

        case SYSLINK_STATE_DATA:
            _syslink_pkt.data[_index] = c;
            _index++;
            if (_index >= _syslink_pkt.length) {
                _state = SYSLINK_STATE_CKSUM;
                _index = 0;
                compute_cksum(&_syslink_pkt);
            }

            break;

        case SYSLINK_STATE_CKSUM:
            if (c != _syslink_pkt.cksum[_index]) {
                Debug("bad packet 0x%X 0x%X ", _syslink_pkt.type, _syslink_pkt.length);
                for (uint8_t j = 0; j < _syslink_pkt.length; j++) {
                    Debug("%X ", _syslink_pkt.data[i]);
                }
                Debug("0x%X 0x%X %d\n", _syslink_pkt.cksum[_index], c, _index);
                _state = SYSLINK_STATE_START;
                _index = 0;
                break;
            }
            _index++;

            if (_index >= 2) {
                _state = SYSLINK_STATE_START;
                _index = 0;
                handlePacket(&_syslink_pkt);
                if (_syslink_pkt.type == SYSLINK_RADIO_RAW) {
                    mavlink_send();
                }
                //we send mavlink packets here as per protocol
            }
            break;
        }
    }
}

void AP_Radio_CrazyRadio::handlePacket(Syslink_Packet *_syslink_pkt)
{
    if (_syslink_pkt->type == SYSLINK_RADIO_RAW) {
        CRTP_Packet *crtp_packet = (CRTP_Packet *) &_syslink_pkt->length;
        if (crtp_packet->port == CRTP_PORT_MAVLINK) {
            _readbuf.write(crtp_packet->data, sizeof(crtp_packet->data));
        }
        else if (crtp_packet->port == CRTP_PORT_SETPOINT) {
            CRTP_Commander *rcin = (CRTP_Commander *)crtp_packet->data;
            float roll, pitch, yaw;
            roll = rcin->roll;
            pitch = rcin->pitch;
            yaw = rcin->yaw;
            pwm_channels[0] = ((roll * 500)/20) + 1500;
            pwm_channels[1] = ((pitch * 500)/20) + 1500;
            pwm_channels[2] = ((rcin->thrust * 1000)/65535) + 1000;
            pwm_channels[3] = ((yaw * 500)/150) + 1500;
            pwm_channels[4] = 1000;
            _last_recv_us = AP_HAL::micros();
            //Debug("%d %d %d %d\n", pwm_channels[0], pwm_channels[1], pwm_channels[2], pwm_channels[3]);
        }
    }
}

void AP_Radio_CrazyRadio::compute_cksum(Syslink_Packet *_syslink_pkt)
{
    uint8_t cksum[2] = {0};
    uint8_t *Di = (uint8_t *)_syslink_pkt;
    int dataSize = _syslink_pkt->length + 2;
    // Calculate checksum delux
    for (uint8_t i = 0; i < dataSize; i++) {
        cksum[0] += Di[i];
        cksum[1] += cksum[0];
    }

    _syslink_pkt->cksum[0] = cksum[0];
    _syslink_pkt->cksum[1] = cksum[1];
}

// get TX fw version
uint32_t AP_Radio_CrazyRadio::get_tx_version(void)
{
    return 0;
}

// get radio statistics structure
const AP_Radio::stats &AP_Radio_CrazyRadio::get_stats(void)
{
    return _stats;
}

// set the 2.4GHz wifi channel used by companion computer, so it can be avoided
void AP_Radio_CrazyRadio::set_wifi_channel(uint8_t channel)
{

}