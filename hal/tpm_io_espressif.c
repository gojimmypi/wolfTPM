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

/*****************************************************************************/
/* --- BEGIN IO Callback Logic -- */
/*****************************************************************************/

/* Included via tpm_io.c if WOLFTPM_INCLUDE_IO_FILE is defined */
#ifdef WOLFTPM_INCLUDE_IO_FILE

#ifdef WOLFSSL_ESPIDF

/* Espressif */
#include "sdkconfig.h"

#define TAG "TPM_IO"

#ifdef WOLFTPM_I2C

#define I2C_READ_WAIT_TICKS  (I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS)
#define I2C_WRITE_WAIT_TICKS (I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS)

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
    /* Yellow wire */
    #define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL
#else
    /* There should have been a Kconfig.projbuild file in the ./main
     * directory to set I2C parameters in the sdkconfig project file. */
    #error "Could not find CONFIG_I2C_MASTER_SCL definition."
#endif

/* GPIO number used for I2C master data */
#ifdef CONFIG_I2C_MASTER_SDA
    /* Orange wire */
    #define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA
#else
    /* There should have been a Kconfig.projbuild file in the ./main
     * directory to set I2C parameters in the sdkconfig project file. */
    #error "Could not find CONFIG_I2C_MASTER_SDA definition."
#endif

/* I2C master i2c port number,
 * the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_NUM              0

/* I2C master clock frequency
 *   Typically, an I2C slave device has a 7-bit address or 10-bit address.
 *   ESP32 supports both I2C Standard-mode (Sm) and Fast-mode (Fm) which
 *   can go up to 100KHz and 400KHz respectively.
 *
 *   The clock frequency of SCL in master mode
 *   should not be larger than 400 KHz. */
#ifndef I2C_MASTER_FREQ_HZ
    #define I2C_MASTER_FREQ_HZ          100000
#endif
/* I2C master doesn't need buffer, so disabled: */
#define I2C_MASTER_TX_BUF_DISABLE   0

/* I2C master doesn't need buffer, so disabled: */
#define I2C_MASTER_RX_BUF_DISABLE   0

/* Wait timeout, in millisecondss. Note: -1 means wait forever. */
#define I2C_MASTER_TIMEOUT_MS       10000

/* Infineon 9673 I2C at 0x2e */
#define TPM2_INFINEON_9673_ADDR     0x2e

/* I2C test sensor is an LM75 temperature sensor at 0x48 */
#define LM75_SENSOR_ADDR            0x48

#define DELETE_I2C_ON_ERROR         0

#if 0
    #define TPM2_I2C_ADDR           LM75_SENSOR_ADDR
#else
    #define TPM2_I2C_ADDR           TPM2_INFINEON_9673_ADDR
#endif
static int is_initialized_i2c = 0;
static int is_init_read = 1; /* TODO is this actually helpful? */


/* i2c master initialization */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    esp_err_t ret = ESP_OK;
    ESP_LOGI(TAG, "i2c_master_init");
    ESP_LOGI(TAG, "I2C_MASTER_FREQ_HZ   = %d", (int)I2C_MASTER_FREQ_HZ);
    ESP_LOGI(TAG, "I2C_READ_WAIT_TICKS  = %d", (int)I2C_READ_WAIT_TICKS);
    ESP_LOGI(TAG, "I2C_WRITE_WAIT_TICKS = %d", (int)I2C_WRITE_WAIT_TICKS);


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
    ESP_LOGE(TAG, "Need to implement non-legacy ESP-IDF I2C library");
#endif

    ret = i2c_driver_install(i2c_master_port, conf.mode,
                             I2C_MASTER_RX_BUF_DISABLE,
                             I2C_MASTER_TX_BUF_DISABLE, 0);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "i2c driver install success");
        is_initialized_i2c = TRUE;
    }
    else {
        ESP_LOGE(TAG, "Failed to initialize i2c. Error code: %d", ret);
    }

    return ret;
}

static esp_err_t i2c_master_delete(void)
{
    ESP_LOGI(TAG, "i2c_master_delete");
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    is_initialized_i2c = FALSE;
    return ESP_OK;
}

/* Espressif HAL I2C */
static esp_err_t tpm_register_read(uint32_t reg,
                                    uint8_t *data, size_t len)
{
    int ret;

//esp_err_t i2c_master_write_read_device(i2c_port_t i2c_num, uint8_t device_address,
//                                       const uint8_t* write_buffer, size_t write_size,
//                                       uint8_t* read_buffer, size_t read_size,
//                                       TickType_t ticks_to_wait)
    byte buf[1];
    ESP_LOGI(TAG, "TPM Read init %d ", is_init_read );
    if (is_init_read == 1) {
        buf[0] = (reg & 0xFF); /* convert to simple 8-bit address for I2C */
        ret = i2c_master_write_read_device(I2C_MASTER_NUM, TPM2_I2C_ADDR,
                                            buf, sizeof(buf), /* write buffer, len */
                                            data,     len,
                                            I2C_READ_WAIT_TICKS);
        // is_init_read = 0;
    }
    else {
        ret = i2c_master_read_from_device(I2C_MASTER_NUM, TPM2_I2C_ADDR,
                                          data, len,
                                          I2C_READ_WAIT_TICKS);
    }

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Success! i2c_master_write_read_device");
    }
    else {
        ESP_LOGI(TAG, "ERROR: i2c_master_write_read_device failed with code = %d", ret);
        if (DELETE_I2C_ON_ERROR) {
            i2c_master_delete();
        }
    }
    return ret;
}

static esp_err_t tpm_register_write(uint32_t reg_addr,
                                    uint8_t* data, size_t len)
{
    int ret;
    ESP_LOGI(TAG, "TPM Write init %d", is_init_read);
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, TPM2_I2C_ADDR,
                                     data, len,
                                     I2C_WRITE_WAIT_TICKS);
    is_init_read = 1;

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Success! tpm_register_write");
    }
    else {
        ESP_LOGI(TAG, "ERROR: tpm_register_write failed with code = %d", ret);
        if (DELETE_I2C_ON_ERROR) {
            i2c_master_delete();
        }
    }

    return ret;
}

static int tpm_ifx_i2c_read(void* userCtx, word32 reg, byte* data, int len)
{
    int ret;
    ret = tpm_register_read(reg, data, len);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Read device 0x%x success.", TPM2_I2C_ADDR);
        ret = TPM_RC_SUCCESS;
    }
    else {
        ESP_LOGE(TAG, "Read device 0x%x fail. Error = %d", TPM2_I2C_ADDR, ret);
        ret = TPM_RC_FAILURE;
    }
    return ret;
}

static int tpm_ifx_i2c_write(void* userCtx, word32 reg, byte* data, int len)
{
    int ret;
    ret = tpm_register_write(reg, data, len);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Write device 0x%x success 0x%x len = %d.", TPM2_I2C_ADDR, (word32)data, len);
        ret = TPM_RC_SUCCESS;
    }
    else {
        ESP_LOGE(TAG, "Read device 0x%x fail. Error = %d", TPM2_I2C_ADDR, ret);
        ret = TPM_RC_FAILURE;
    }
    return ret;
}

int TPM2_IoCb_Espressif_I2C(TPM2_CTX* ctx, int isRead, word32 addr,
                            byte* buf, word16 size, void* userCtx)
{
    int ret = TPM_RC_FAILURE;

    //buf[0] = (reg & 0xFF); /* convert to simple 8-bit address for I2C */

    if (userCtx == NULL) {
        ESP_LOGE(TAG, "userCtx cannot be null");
    }
    else {
        if (is_initialized_i2c) {
            ESP_LOGV(TAG, "I2C already initialized");
            ret = ESP_OK;
        }
        else {
            ret = i2c_master_init();
        }

        if (ret == ESP_OK) {
            if (isRead) {
                ret = tpm_ifx_i2c_read(userCtx, addr, buf, size);
            }
            else {
                ret = tpm_ifx_i2c_write(userCtx, addr, buf, size);
            }
        }
        else {
            ESP_LOGE(TAG, "I2C Failed to initialize. Error: %d", ret);
            ret = TPM_RC_FAILURE;
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

    #error TPM2 SPI support on this platform not supported yet
#endif

#endif /* WOLFSSL_ESPIDF */
#endif /* WOLFTPM_INCLUDE_IO_FILE */

/******************************************************************************/
/* --- END IO Callback Logic -- */
/******************************************************************************/