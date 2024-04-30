/* tpm_io_espressif.c
 *
 * Copyright (C) 2006-2024 wolfSSL Inc.
 *
 * This file is part of wolfTPM.
 *
 * wolfTPM is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * wolfTPM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1335, USA
 */

/* This example shows IO interfaces for Microchip micro-controllers using
 * MPLAB X and Harmony
 */

#ifdef HAVE_CONFIG_H
    #include <config.h>
#endif

#include <wolftpm/tpm2.h>
#include <wolftpm/tpm2_tis.h>
#include "tpm_io.h"

#define TAG "TPM_IO"

/******************************************************************************/
/* --- BEGIN IO Callback Logic -- */
/******************************************************************************/
#ifdef WOLFTPM_I2C
    int TPM2_IoCb_Espressif_I2C(TPM2_CTX* ctx, int isRead, word32 addr,
        byte* buf, word16 size, void* userCtx)
    {
        int ret = TPM_RC_FAILURE;
        if (userCtx != NULL) {
            if (isRead) {
                ESP_LOGE(TAG, "TPM2_IoCb_Espressif_I2C READ not implemented.");
                // ret = tpm_ifx_i2c_read(userCtx, addr, buf, size);
            }
            else {
                ESP_LOGE(TAG, "TPM2_IoCb_Espressif_I2C WRITE not implemented.");
                // ret = tpm_ifx_i2c_write(userCtx, addr, buf, size);
            }
        }
        (void)ctx;
        return ret;
    }
#else /* SPI */
    #ifndef TPM2_SPI_HZ
        /* Use the max speed by default
         * See tpm2_types.h for chip specific max values */
        #define TPM2_SPI_HZ TPM2_SPI_MAX_HZ
    #endif
    #ifdef WOLFTPM_CHECK_WAIT_STATE
        #error SPI check wait state logic not supported
    #endif

    #error "not implemented"
#endif