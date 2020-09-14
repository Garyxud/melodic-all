/*
 * Copyright (c) 2017, James Jackson
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <string>
#include "system.h"
#include "pwm.h"
#include "led.h"

#include "revo_f4.h"
#include "backup_sram.h"
#include "vcp.h"
#include "printf.h"

struct BackupData
{
    static constexpr uint32_t CHECKSUM_START = 0xA5A5;

    uint32_t reset_count;
    uint32_t some_data;
    uint32_t checksum;

    BackupData()
    {
        memset(this, 0, sizeof(BackupData));
    }

    uint32_t compute_checksum()
    {
        uint32_t crc = CHECKSUM_START;
        for (size_t offset = 0; offset <= sizeof(BackupData) - sizeof(checksum) - sizeof(crc); offset++)
        {
            crc ^= *reinterpret_cast<uint32_t*>(reinterpret_cast<uint8_t*>(this) + offset);
        }
        return crc;
    }
};

void restart()
{
    NVIC_SystemReset();
}

int main()
{
    systemInit();
    VCP vcp;
    vcp.init();
    backup_sram_init();

    // check for and process backup data if it exists
    BackupData read_data;
    backup_sram_read(&read_data, sizeof(read_data));

    uint32_t reset_count = 0;
    if (read_data.checksum == read_data.compute_checksum())
        reset_count = read_data.reset_count;

    // clear the backup data so it doesn't get read on normal startup
    backup_sram_clear(sizeof(BackupData));

    // Pretend we get a hardfault some time in the future; write backup data and reset.
    // Normally this would be done in the hardfault interrupt handler.
    delay(300);

    BackupData write_data;
    write_data.reset_count = ++reset_count; // increment reset counter
    write_data.some_data = 0xDEADBEEF;
    write_data.checksum = write_data.compute_checksum();
    backup_sram_write(&write_data, sizeof(write_data));

    restart();
}
