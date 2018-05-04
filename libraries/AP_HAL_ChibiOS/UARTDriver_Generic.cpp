
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && !(defined(STM32F4xx_MCUCONF) || defined(STM32F7xx_MCUCONF))
#include "UARTDriver_Generic.h"
#include "GPIO.h"
#include <usbcfg.h>
#if HAL_USE_SERIAL == TRUE

extern const AP_HAL::HAL& hal;

using namespace ChibiOS;

static UARTDriver_Generic::SerialDef _serial_tab[] = { HAL_UART_DEVICE_LIST };
#ifndef DEFAULT_TX_BUF_SIZE
#define DEFAULT_TX_BUF_SIZE 4096
#endif

#ifndef DEFAULT_RX_BUF_SIZE
#define DEFAULT_RX_BUF_SIZE 1024
#endif

UARTDriver_Generic::UARTDriver_Generic(uint8_t serial_num) :
_baudrate(57600),
_is_usb(false),
_in_timer(false),
_initialised(false)
{
    _serial = _serial_tab[serial_num].serial;
    _is_usb = _serial_tab[serial_num].is_usb;
    _serial_num = serial_num;
    chMtxObjectInit(&_write_mutex);
}

void UARTDriver_Generic::begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    if (_serial == nullptr) {
        return;
    }
    bool was_initialised = _initialised;
    uint16_t min_tx_buffer = DEFAULT_TX_BUF_SIZE;
    uint16_t min_rx_buffer = DEFAULT_RX_BUF_SIZE;
    if (_is_usb) {
        min_tx_buffer = 4096;
        min_rx_buffer = 1024;
    }
    // on PX4 we have enough memory to have a larger transmit and
    // receive buffer for all ports. This means we don't get delays
    // while waiting to write GPS config packets
    if (txS < min_tx_buffer) {
        txS = min_tx_buffer;
    }
    if (rxS < min_rx_buffer) {
        rxS = min_rx_buffer;
    }

    /*
      allocate the read buffer
      we allocate buffers before we successfully open the device as we
      want to allocate in the early stages of boot, and cause minimum
      thrashing of the heap once we are up. The ttyACM0 driver may not
      connect for some time after boot
     */
    if (rxS != _readbuf.get_size()) {
        _initialised = false;
        while (_in_timer) {
            hal.scheduler->delay(1);
        }

        _readbuf.set_size(rxS);
    }

    if (b != 0) {
        _baudrate = b;
    }

    /*
      allocate the write buffer
     */
    if (txS != _writebuf.get_size()) {
        _initialised = false;
        while (_in_timer) {
            hal.scheduler->delay(1);
        }
        _writebuf.set_size(txS);
    }

    if (_is_usb) {
        /*
         * Initializes a serial-over-USB CDC driver.
         */
        if (!was_initialised) {
#if HAL_USE_SERIAL_USB == TRUE
            sduObjectInit((SerialUSBDriver*)_serial);
            sduStart((SerialUSBDriver*)_serial, &serusbcfg);
            /*
             * Activates the USB driver and then the USB bus pull-up on D+.
             * Note, a delay is inserted in order to not have to disconnect the cable
             * after a reset.
             */
            usbDisconnectBus(serusbcfg.usbp);
            hal.scheduler->delay_microseconds(1500);
            usbStart(serusbcfg.usbp, &usbcfg);
            usbConnectBus(serusbcfg.usbp);
#endif
        }
    } else {
        if (_baudrate != 0) {
            sercfg.speed = _baudrate;
            sercfg.cr1 = 0;
            sercfg.cr2 = USART_CR2_STOP1_BITS;
            sercfg.cr3 = 0;
            sercfg.irq_cb = NULL;
            sercfg.ctx = NULL;
            sdStart((SerialDriver*)_serial, &sercfg);
        }
    }

    if (_writebuf.get_size() && _readbuf.get_size()) {
        _initialised = true;
    }
    _uart_owner_thd = chThdGetSelfX();
}


void UARTDriver_Generic::begin(uint32_t b)
{
    begin(b, 0, 0);
}

void UARTDriver_Generic::end()
{
    _initialised = false;
    while (_in_timer) hal.scheduler->delay(1);

    if (_is_usb) {
#if HAL_USE_SERIAL_USB == TRUE
        sduStop((SerialUSBDriver*)_serial);
#endif
    } else {
        sdStop((SerialDriver*)_serial);
    }
    _readbuf.set_size(0);
    _writebuf.set_size(0);
}

void UARTDriver_Generic::flush()
{
    if (_is_usb) {
#if HAL_USE_SERIAL_USB == TRUE
        sduSOFHookI((SerialUSBDriver*)_serial);
#endif
    } else {
        //TODO: Handle this for other serial ports
    }
}

bool UARTDriver_Generic::is_initialized()
{
    return _initialised;
}

void UARTDriver_Generic::set_blocking_writes(bool blocking)
{
    _nonblocking_writes = !blocking;
}

bool UARTDriver_Generic::tx_pending() { return false; }

/* Empty implementations of Stream virtual methods */
uint32_t UARTDriver_Generic::available() {
    if (!_initialised) {
        return 0;
    }
    if (_is_usb) {
#if HAL_USE_SERIAL_USB == TRUE
        if (((SerialUSBDriver*)_serial)->config->usbp->state != USB_ACTIVE) {
            return 0;
        }
#else
        return 0;
#endif
    }
    return _readbuf.available();
}

uint32_t UARTDriver_Generic::txspace()
{
    if (!_initialised) {
        return 0;
    }
    return _writebuf.space();
}

int16_t UARTDriver_Generic::read()
{
    if (_uart_owner_thd != chThdGetSelfX()){
        return -1;
    }
    if (!_initialised) {
        return -1;
    }

    uint8_t byte;
    if (!_readbuf.read_byte(&byte)) {
        return -1;
    }

    return byte;
}

/* Empty implementations of Print virtual methods */
size_t UARTDriver_Generic::write(uint8_t c)
{
    if (!chMtxTryLock(&_write_mutex)) {
        return -1;
    }
    
    if (!_initialised) {
        chMtxUnlock(&_write_mutex);
        return 0;
    }

    while (_writebuf.space() == 0) {
        if (_nonblocking_writes) {
            chMtxUnlock(&_write_mutex);
            return 0;
        }
        hal.scheduler->delay(1);
    }
    size_t ret = _writebuf.write(&c, 1);
    chMtxUnlock(&_write_mutex);
    return ret;
}

size_t UARTDriver_Generic::write(const uint8_t *buffer, size_t size)
{
    if (!_initialised) {
		return 0;
	}

    if (!chMtxTryLock(&_write_mutex)) {
        return -1;
    }

    if (!_nonblocking_writes) {
        /*
          use the per-byte delay loop in write() above for blocking writes
         */
        chMtxUnlock(&_write_mutex);
        size_t ret = 0;
        while (size--) {
            if (write(*buffer++) != 1) break;
            ret++;
        }
        return ret;
    }

    size_t ret = _writebuf.write(buffer, size);
    chMtxUnlock(&_write_mutex);
    return ret;
}


/*
  push any pending bytes to/from the serial port. This is called at
  1kHz in the timer thread. Doing it this way reduces the system call
  overhead in the main task enormously.
 */
void UARTDriver_Generic::_timer_tick(void)
{
    int ret;
    uint32_t n;

    if (!_initialised) return;

    // don't try IO on a disconnected USB port
    if (_is_usb) {
#if HAL_USE_SERIAL_USB == TRUE
        if (((SerialUSBDriver*)_serial)->config->usbp->state != USB_ACTIVE) {
            return;
        }
#else
        return;
#endif
    }
    if(_is_usb) {
        ((GPIO *)hal.gpio)->set_usb_connected();
    }
    _in_timer = true;

    // write any pending bytes
    n = _writebuf.available();
    if (n > 0) {
        ByteBuffer::IoVec vec[2];
        const auto n_vec = _writebuf.peekiovec(vec, n);
        for (int i = 0; i < n_vec; i++) {
            ret = 0;
            if (_is_usb) {
#if HAL_USE_SERIAL_USB == TRUE
                ret = chnWriteTimeout((SerialUSBDriver*)_serial, vec[i].data, vec[i].len, TIME_IMMEDIATE);
#endif
            } else {
                ret = chnWriteTimeout((SerialDriver*)_serial, vec[i].data, vec[i].len, TIME_IMMEDIATE);
            }
            if (ret < 0) {
                break;
            }
            _writebuf.advance(ret);

            /* We wrote less than we asked for, stop */
            if ((unsigned)ret != vec[i].len) {
                break;
            }
        }
    }

    // try to fill the read buffer
    ByteBuffer::IoVec vec[2];

    const auto n_vec = _readbuf.reserve(vec, _readbuf.space());
    for (int i = 0; i < n_vec; i++) {
        //Do a non-blocking read
        ret = 0;
        if (_is_usb) {
#if HAL_USE_SERIAL_USB == TRUE
            ret = chnReadTimeout((SerialUSBDriver*)_serial, vec[i].data, vec[i].len, TIME_IMMEDIATE);
#endif
        } else {
            ret = chnReadTimeout((SerialDriver*)_serial, vec[i].data, vec[i].len, TIME_IMMEDIATE);
        }
        if (ret < 0) {
            break;
        }
        _readbuf.commit((unsigned)ret);

        /* stop reading as we read less than we asked for */
        if ((unsigned)ret < vec[i].len) {
            break;
        }
    }

    _in_timer = false;
}
#endif //CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#endif //#if HAL_USE_SERIAL == TRUE
