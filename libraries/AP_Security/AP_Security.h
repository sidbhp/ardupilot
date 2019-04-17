/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Siddharth Bharat Purohit
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <stdint.h>
#include <npnt.h>

#define HEADER_REGION_LEN 16
#define KEY_REGION_LEN 488

#if ((HEADER_REGION_LEN + KEY_REGION_LEN) != 504) && ((KEY_REGION_LEN % 4) != 0) && ((KEY_REGION_LEN % 4) != 0)
#error "Invalid Security Key region Lengths"
#endif

#define AP_PUBLIC_KEY_FILE "/APM/pubkey.pem" 
#define AP_MANAGER_PUBLIC_KEY_FILE "man_pubkey.pem"

#ifndef AP_NPNT_PERMART_FILE
#define AP_NPNT_PERMART_FILE "/APM/permart.xml"
#endif
struct PACKED keyregion_struct {
    uint8_t header[HEADER_REGION_LEN];
    union {
        struct PACKED {
            uint16_t key_serial_id;
            uint16_t key_length;
        };
        uint32_t key_details;
    };
    uint8_t key[KEY_REGION_LEN];
    uint32_t hash;
};


class AP_Security {

public:
    AP_Security();

    // get singleton instance
    static AP_Security *get_singleton() {
        return _singleton;
    }
    void init();

    void generate_public_key();

    bool check_npnt_permission();
private:
    static AP_Security *_singleton;
    void init_key();
    void generate_and_store_key();
    bool is_key_setup();
    static struct keyregion_struct *keyregion;
    npnt_s npnt_handle;
};

namespace AP {
    AP_Security &security();
};
