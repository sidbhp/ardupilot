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
  ArduPilot bootloader. This implements the same protocol originally
  developed for PX4, but builds on top of the ChibiOS HAL

  It does not use the full AP_HAL API in order to keep the firmware
  size below the maximum of 16kByte required for F4 based
  boards. Instead it uses the ChibiOS APIs directly
 */

#include <AP_HAL/AP_HAL.h>
#include "ch.h"
#include "hal.h"
#include "hwdef.h"
#include <AP_HAL_ChibiOS/hwdef/common/usbcfg.h>
#include "support.h"
#include "bl_protocol.h"

extern "C" {
    int main(void);
}

struct boardinfo board_info;

volatile unsigned timer[NTIMERS];
/*
  1ms timer tick callback
 */
THD_WORKING_AREA(wasys_tick_handler, 128);
static THD_FUNCTION(sys_tick_handler, arg)
{
    uint8_t i;
    while (true) {
        for (i = 0; i < NTIMERS; i++)
            if (timer[i] > 0) {
                timer[i]--;
            }

        if ((led_state == LED_BLINK) && (timer[TIMER_LED] == 0)) {
            led_toggle(LED_BOOTLOADER);
            timer[TIMER_LED] = 50;
        }
        chThdSleep(MS2ST(1));
    }
}

THD_WORKING_AREA(wabootloader_main, 1024);
static THD_FUNCTION(bootloader_main, arg)
{
    init_uarts();

    board_info.board_type = APJ_BOARD_ID;
    board_info.board_rev = 0;
    board_info.fw_size = (BOARD_FLASH_SIZE - FLASH_BOOTLOADER_LOAD_KB)*1024;
    if (BOARD_FLASH_SIZE > 1024 && check_limit_flash_1M()) {
        board_info.fw_size = (1024 - FLASH_BOOTLOADER_LOAD_KB)*1024;        
    }

    flash_init();

    while (true) {
        bootloader(5000);
        jump_to_app();
    }
}

THD_WORKING_AREA(wausb_lld_pump, 8192);
static THD_FUNCTION(usb_lld_pump_redir, arg)
{
    usb_lld_pump(arg);
}

THD_TABLE_BEGIN
THD_TABLE_ENTRY(wasys_tick_handler, "SysTick_Handler", sys_tick_handler, NULL)
THD_TABLE_ENTRY(wabootloader_main, "BL_Main", bootloader_main, NULL)
THD_TABLE_ENTRY(wausb_lld_pump, "USB_Pump", usb_lld_pump_redir, (void*)&USBD1)
THD_TABLE_END

int main()
{
    //This is the idle thread for NIL
    while(true) {
    }
}