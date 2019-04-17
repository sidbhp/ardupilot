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

#include "AP_Security.h"
#include <AP_Math/crc.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_ROMFS/AP_ROMFS.h>
#if HAL_OS_POSIX_IO
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <assert.h>
#include <stdio.h>
#include <time.h>
#include <dirent.h>
#if defined(__APPLE__) && defined(__MACH__)
#include <sys/param.h>
#include <sys/mount.h>
#else
#include <sys/statfs.h>
#endif
#endif

#if HAL_OS_FATFS_IO
#include <stdio.h>
#endif

extern const AP_HAL::HAL& hal;

#define DVAL_1X     0xFF
#define DVAL_2X     DVAL_1X,  DVAL_1X
#define DVAL_4X     DVAL_2X,  DVAL_2X
#define DVAL_8X     DVAL_4X,  DVAL_4X
#define DVAL_16X    DVAL_8X,  DVAL_8X
#define DVAL_32X    DVAL_16X, DVAL_16X
#define DVAL_64X    DVAL_32X, DVAL_32X
#define DVAL_128X   DVAL_64X, DVAL_64X

const uint8_t keyregion_bytes[512] = {DVAL_128X, DVAL_128X, DVAL_128X, DVAL_128X};
struct keyregion_struct *AP_Security::keyregion = (struct keyregion_struct *)keyregion_bytes;

//Setup Security at the initialisation step
AP_Security::AP_Security()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("Too many security modules");
        return;
    }
    _singleton = this;
}

void AP_Security::init() {
    //We check if controller is secured from sensitive information reads
    // check_security_features_enabled();
    if (!is_key_setup()) {
        generate_and_store_key();
    } else {
        init_key();
    }

    npnt_init_handle(&npnt_handle);
}

// void AP_Security::check_security_features_enabled()
// {
//     //Check Debugging Disabled
//     if (!hal.util->debugging_disabled()) {
//         hal.util->erase_flash_region(&keyregion, 512);
//         AP_HAL::panic("Debugging is Enabled! Cleared Private Key and stopping autopilot...");
//     }
// }

bool AP_Security::is_key_setup()
{
    //Check if Keys Already setup
    for (uint8_t i = 0; i < sizeof(keyregion->header); i++) {
        if (keyregion->header[i] == 0xFF) {
            //The Keys are either not setup or are corrupted
            //Erase Keys for update
            hal.util->erase_flash_region((void*)&keyregion, 512);
            return false;
        }
    }
    //Check if Key CRC Hash matches
    uint32_t crc;
    crc = crc_crc32(0, (const uint8_t*)keyregion_bytes, 508);
    if (crc != keyregion->hash) {
        //The Keys are either not setup or are corrupted
        //Erase Keys for update
        hal.util->erase_flash_region((void*)&keyregion, 512);
        return false;
    }
    return true;
}

void AP_Security::generate_and_store_key()
{
    uint8_t privkey[KEY_REGION_LEN] = {0xFF};
    uint32_t keylen = KEY_REGION_LEN;
    char key_header[HEADER_REGION_LEN] = {0x00};
    union {
        struct PACKED {
            uint16_t key_serial_id;
            uint16_t key_length;
        };
        uint32_t key_details;
    } _key_details;
    //Generate RSA key
    hal.util->generate_ecc_key(privkey, keylen);
    //Write RSA key to flash
    if (!hal.util->write_flash_region((void*)keyregion->key, privkey, KEY_REGION_LEN)) {
        AP_HAL::panic("Failed to write keys!");
    }
    //Generate Key Details
    _key_details.key_serial_id = keyregion->key_serial_id + 1;
    _key_details.key_length = keylen;
    if (!hal.util->write_flash_region((void*)keyregion->key_details, (uint8_t*)_key_details.key_details, sizeof(_key_details))) {
        AP_HAL::panic("Failed to write key details!");
    }

    //Generate Header
    strcpy(key_header, "APS v1.0");

    if (hal.util->write_flash_region((void*)keyregion->header, (uint8_t*)key_header, HEADER_REGION_LEN)) {
        AP_HAL::panic("Failed to write key header!");
    }

    //Generate CRC
    uint32_t crc;
    crc = crc_crc32(0, keyregion_bytes, 508);
    if (hal.util->write_flash_region((void*)keyregion->hash, (uint8_t*)crc, sizeof(crc))) {
        AP_HAL::panic("Failed to write key hash!");
    }
}

void AP_Security::init_key()
{
    hal.util->init_key_from_raw((uint8_t*)keyregion->key, keyregion->key_length);
}

void AP_Security::generate_public_key()
{

}

bool AP_Security::check_npnt_permission()
{
    struct stat st;
    if (::stat(AP_NPNT_PERMART_FILE, &st) != 0) {
        printf("Unable to find Permission Artifact\n", strerror(errno));
        return false;
    }
    char *manager_pubkey_der = ;

    int fd = open(AP_NPNT_PERMART_FILE, O_RDONLY|O_CLOEXEC);
    uint8_t* permart = (uint8_t*)malloc(st.st_size);
    if (!permart) {
        return false;
    }
    
    uint32_t permart_len;
    permart_len = fread(permart, st.st_size, fd);
    if (permart_len != st.st_size) {
        free(permart);
        return false;
    }
    if (npnt_set_permart(&npnt_handle, permart, permart_len, 0) < 0) {
        return false;
    }
}

// singleton instance
AP_Security *AP_Security::_singleton;

namespace AP {

AP_Security &security()
{
    return *AP_Security::get_singleton();
}

}
