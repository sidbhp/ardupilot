#include "hwdef.h"

#ifdef STM32F100_MCUCONF
#include "stm32f1_mcuconf.h"
#endif

#if defined(STM32F7xx_MCUCONF) || defined(STM32F4xx_MCUCONF)
#include "stm32f4f7_mcuconf.h"
#endif