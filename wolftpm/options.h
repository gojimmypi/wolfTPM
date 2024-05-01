/* options.h for Espressif
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

/* TODO move to wolftpm component directory, as this file is normally excluded
 * from wolftpm */

/* This file if for the Espressif ESP-IDF */
#define WOLFSSL_ESPIDF

/* WOLFTPM_ADV_IO allows callback code in tpm_io_espressif.c */
#define WOLFTPM_ADV_IO

/* Choices are I2C or TPI*/
#define WOLFTPM_I2C

/* Enable the wolfTPM example HAL in tpm_io.h */
#define WOLFTPM_EXAMPLE_HAL

// #define WOLFSSL_STM32_CUBEMX

#define WOLFTPM_INCLUDE_IO_FILE

/* The default TPM_TIMEOUT_TRIES is 1,000,000 but can be overridden: */
#define TPM_TIMEOUT_TRIES 100

#define NO_MAIN_DRIVER

#define I2C_MASTER_FREQ_HZ 100000

// #define WOLFTPM_DEBUG_IO

