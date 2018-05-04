#pragma once

namespace ChibiOS {
    class AnalogIn;
    class AnalogSource;
    class DigitalSource;
    class GPIO;
    class I2CBus;
    class I2CDevice;
    class I2CDeviceManager;
    class OpticalFlow;
    class PrivateMember;
    class RCInput;
    class RCOutput;
    class Scheduler;
    class Semaphore;
    class SPIBus;
    class SPIDesc;
    class SPIDevice;
    class SPIDeviceDriver;
    class SPIDeviceManager;
    class Storage;
    class UARTDriver_Generic;
    class UARTDriver_STM32F4;
    class Util;
    class Shared_DMA;
    class SoftSigReader;
    class SoftSigReaderInt;
    class CANManager;
#if defined(STM32F4xx_MCUCONF)
    typedef UARTDriver_STM32F4 UARTDriver;
#else
    typedef UARTDriver_Generic UARTDriver;
#endif
}
