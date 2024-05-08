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

/* How many main app test loop interations? */
#define WOLFTPM_MAIN_TEST_ITERATIONS 100

/* WOLFTPM_ADV_IO allows callback code in tpm_io_espressif.c */
#define WOLFTPM_ADV_IO

/* Choices are I2C or SPI*/
/* WOLFTPM_I2C or not; when not defined, assumes SPI. */
#define WOLFTPM_I2C

/* Enable the wolfTPM example HAL in tpm_io.h */
#define WOLFTPM_EXAMPLE_HAL

/* Include the respective hal/tmp_io_NNN file, in our case: tpm_io_espressif */
#define WOLFTPM_INCLUDE_IO_FILE

/* The default TPM_TIMEOUT_TRIES is 1,000,000 but can be overridden.
 * A value of 10000 is much more appropriate for the ESP32: */
#define TPM_TIMEOUT_TRIES 10000

/* If not defined here, TPM_I2C_TRIES is set to a default value of 10 */
/* TPM_I2C_TRIES 10 */

/* Examples may have a main() function, we'll have oour own: */
#define NO_MAIN_DRIVER

/* I2C_MASTER_FREQ_HZ notes:
 *
 * Although the Infineon supports higher speeds, the ESP32 does not.
 *
 * ESP32 supports both I2C Standard-mode (Sm) and Fast-mode (Fm) which
 * can go up to 100KHz and 400KHz respectively.
 *
 * Do not set this value higher than 400000.
 *
 * This is the value assigned to ESP32 i2c_config_t: master.clk_speed */
#define I2C_MASTER_FREQ_HZ 100000

/* Beware that delays in sending data to UART may affect I2C timing and
 * cause errors. Debug with caution. Debug options available: */
/* #define WOLFTPM_DEBUG_IO */


// #define WOLFSSL_NOPRINTF

// #define XPRINTF

#ifdef WOLFSSL_ESPIDF
    #include <esp_log.h>
    #define  ESP_LOG_TAG  "tpm2_tis"
    #ifndef WOLFSSL_NOPRINTF
         __attribute__((unused)) static void do_pause()
        {
            ESP_LOGI(ESP_LOG_TAG, "pause");
        }
        #define XPRINTF(...)         ESP_LOGI("tpm", __VA_ARGS__)
        #define printf_error(...) do { ESP_LOGE("tpm", __VA_ARGS__); do_pause(); } while(0)
    #else
        #define printf(...) {}
        #define printf_error(...) {}
    #endif
#else
    #include <stdio.h>
    #define printf_error(...) printf(__VA_ARGS__)
#endif
