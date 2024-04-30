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

/* Espressif */
/* To use I2C in wolfTPM, be sure the compnent cmake COMPONENT_REQUIRES
 * variable includes "driver" (without quotes) for idf_component_register().
 *
 * See: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/i2c.html */
#if ESP_IDF_VERSION_MAJOR >= 5 && ESP_IDF_VERSION_MINOR >= 3
    #define WOLFSSL_USE_LEGACY_I2C 0
#else
    #define WOLFSSL_USE_LEGACY_I2C 1
#endif

#if WOLFSSL_USE_LEGACY_I2C
    /* Legacy
     * "The legacy driver can't coexist with the new driver. Include i2c.h to
     * use the legacy driver or the other two headers to use the new driver.
     * Please keep in mind that the legacy driver is now deprecated and
     * will be removed in future." */
    #include <driver/i2c.h>
#else
    #include <driver/i2c_types.h>
    #include <driver/i2c_master.h>
#endif

/* GPIO number used for I2C master clock */
#ifdef CONFIG_I2C_MASTER_SCL
    #define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL
#else
    /* There should have been a Kconfig.projbuild file in the ./main
     * directory to set I2C parameters in the sdkconfig project file. */
    #error "Could not find CONFIG_I2C_MASTER_SCL definition."
#endif


/* GPIO number used for I2C master data */
#ifdef CONFIG_I2C_MASTER_SDA
    #define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA
#else
    /* There should have been a Kconfig.projbuild file in the ./main
     * directory to set I2C parameters in the sdkconfig project file. */
    #error "Could not find CONFIG_I2C_MASTER_SDA definition."
#endif

/* I2C master i2c port number,
 * the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_NUM              0

/* I2C master clock frequency */
#define I2C_MASTER_FREQ_HZ          400000

/* I2C master doesn't need buffer, so disabled: */
#define I2C_MASTER_TX_BUF_DISABLE   0

/* I2C master doesn't need buffer, so disabled: */
#define I2C_MASTER_RX_BUF_DISABLE   0

#define I2C_MASTER_TIMEOUT_MS       1000

/* I2C test seneor is an LM75 temperature sensor */
#define LM75_SENSOR_ADDR            0x48

static int is_initialized_i2c = 0;

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    esp_err_t ret = ESP_OK;

#if WOLFSSL_USE_LEGACY_I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);
#else
    ESP_LOGE(TAG, "Need to implement non-legacyupdated ESP-IDF I2C library");
#endif

    ret = i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
    if (ret == ESP_OK) {
        is_initialized_i2c = TRUE;
    }
    else {
        ESP_LOGE(TAG, "Failed to initialize i2c. Error code: %d", ret);
    }

    return ret;
}


/*****************************************************************************/
/* --- BEGIN IO Callback Logic -- */
/*****************************************************************************/
#ifdef WOLFTPM_I2C

#define I2C_READ_WAIT_TICKS  (I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS)
#define I2C_WRITE_WAIT_TICKS (I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS)

static esp_err_t lm75_register_read(uint8_t reg_addr,
                                    uint8_t *data, size_t len)
{
    int ret;
    ret = i2c_master_write_read_device(I2C_MASTER_NUM, LM75_SENSOR_ADDR,
                                        &reg_addr, 1, data, len,
                                        I2C_READ_WAIT_TICKS);
    return ret;
}

static esp_err_t lm75_register_write_byte(uint8_t reg_addr,
                                          uint8_t* data, size_t len)
{
    int ret;
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, LM75_SENSOR_ADDR,
                                     data, len,
                                     I2C_WRITE_WAIT_TICKS);
    return ret;
}

static int tpm_ifx_i2c_read(void* userCtx, word32 reg, byte* data, int len)
{
    int ret;
    ret = lm75_register_read(LM75_SENSOR_ADDR, data, len);
    if (ret == ESP_OK) {
        ESP_LOGV(TAG, "Read successfully");
        ret = TPM_RC_SUCCESS;
    }
    else {
        ret = TPM_RC_FAILURE;
    }
    return ret;
}

static int tpm_ifx_i2c_write(void* userCtx, word32 reg, byte* data, int len)
{
    int ret;
    ret = lm75_register_write_byte(LM75_SENSOR_ADDR, data, len);
    if (ret == ESP_OK) {
        ESP_LOGV(TAG, "Read successfully");
        ret = TPM_RC_SUCCESS;
    }
    else {
        ret = TPM_RC_FAILURE;
    }
    return ret;
}


int TPM2_IoCb_Espressif_I2C(TPM2_CTX* ctx, int isRead, word32 addr,
                            byte* buf, word16 size, void* userCtx)
{
    int ret = TPM_RC_FAILURE;
    if (userCtx != NULL) {
        if (!is_initialized_i2c) {
            ret = i2c_master_init();
        }

        if (isRead) {
            ret = tpm_ifx_i2c_read(userCtx, addr, buf, size);
        }
        else {
            ret = tpm_ifx_i2c_write(userCtx, addr, buf, size);
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