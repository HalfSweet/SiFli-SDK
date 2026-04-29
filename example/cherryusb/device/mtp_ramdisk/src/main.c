/*
 * Copyright (c) 2026, SiFli Technologies(Nanjing) Co, Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "rtthread.h"
#include "bf0_hal.h"

int main(void)
{
    rt_kprintf("cherryusb device mtp ramdisk demo!\n");

    extern void mtp_ramdisk_init(uint8_t busid, uintptr_t reg_base);
    mtp_ramdisk_init(0, (uintptr_t)USBC_BASE);

    while (1) {
        rt_thread_mdelay(1000);
    }

    return 0;
}
