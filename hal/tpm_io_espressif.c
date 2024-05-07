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
#if ESP_IDF_VERSION_MAJOR >= 5 && ESP_IDF_VERSION_MINOR > 0
    /* TODO we are forcing legacy mode, even though using v5.2 */
    #define WOLFSSL_USE_LEGACY_I2C 1
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
    /* Yellow wire Clock */
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

#ifndef TPM_I2C_TRIES
    #define TPM_I2C_TRIES 10
#endif

static int is_initialized_i2c = 0;

static int show_binary(byte* theVar, size_t dataSz) {
    printf("*****************************************************\n");
    word32 i;
    for (i = 0; i < dataSz; i++)
        printf("%02X", theVar[i]);
    printf("\n");
    printf("******************************************************\n");
    return 0;
}
/* i2c master initialization */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    esp_err_t ret = ESP_OK;
    ESP_LOGI(TAG, "i2c_master_init");
    ESP_LOGI(TAG, "I2C_MASTER_FREQ_HZ    = %d", (int)I2C_MASTER_FREQ_HZ);
    ESP_LOGI(TAG, "I2C_READ_WAIT_TICKS   = %d", (int)I2C_READ_WAIT_TICKS);
    ESP_LOGI(TAG, "I2C_WRITE_WAIT_TICKS  = %d", (int)I2C_WRITE_WAIT_TICKS);
    ESP_LOGI(TAG, "I2C_MASTER_TIMEOUT_MS = %d", (int)I2C_MASTER_TIMEOUT_MS);

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

/* Obserfed to have a value of 180 in i2c.c, rouded up for safety */
#define I2C_TRANS_BUF_MINIMUM_SIZE     255

// #define USE_MY_I2C
// #define USE_IDF_WRITE_READ
// #define USE_IDF_READ

#ifdef USE_MY_I2C
esp_err_t my_i2c_master_write_read_device(i2c_port_t i2c_num, uint8_t device_address,
                                       const uint8_t* write_buffer, size_t write_size,
                                       uint8_t* read_buffer, size_t read_size,
                                       TickType_t ticks_to_wait)
{
    uint8_t buffer[I2C_TRANS_BUF_MINIMUM_SIZE] = { 0 };
    i2c_cmd_handle_t handle;
    esp_err_t err = ESP_OK;
    int timeout = 100;

/* wake up*/
#if 0
    do {

        // ESP_LOGW(TAG, "Begin my_i2c_master_write_read_device");
        handle = i2c_cmd_link_create_static(buffer, sizeof(buffer));
        assert(handle != NULL);

        /* start bit */
        if (err == ESP_OK) {
            err = i2c_master_start(handle);
        }

        /* Device address */
        if (err == ESP_OK) {
            err = i2c_master_write_byte(handle, device_address << 1 | I2C_MASTER_WRITE, true);
        }
        if (err == ESP_OK) {
            err = i2c_master_read(handle, read_buffer, read_size, I2C_MASTER_LAST_NACK);
        }
//        /* payload to write */
//        if (err == ESP_OK) {
//            err = i2c_master_write(handle, write_buffer, write_size, true);
//        }

//        if (err == ESP_OK) {
//            err = i2c_master_start(handle); /* start bit */
//        }
        //
        //
        //    if (err == ESP_OK) {
        //        err = i2c_master_write_byte(handle, device_address << 1 | I2C_MASTER_READ, true);
        //    }

        //    if (err == ESP_OK) {
        //        err = i2c_master_read(handle, read_buffer, read_size, I2C_MASTER_LAST_NACK);
        //    }

        if (err == ESP_OK) {
            err = i2c_master_stop(handle);
        }

        if (err == ESP_OK) {
            err = i2c_master_cmd_begin(i2c_num, handle, ticks_to_wait);
        }
        i2c_cmd_link_delete_static(handle);

        vTaskDelay(pdMS_TO_TICKS(1));
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "try again = %d", err);
        }
        else {
            ESP_LOGE(TAG, "try again error = %d", err);
        }
    } while ((err != ESP_OK) && (--timeout > 0));

    if (err == ESP_OK) {
        //
    }
    else {
        ESP_LOGE(TAG, "Error i2c_master_cmd_begin = %d", err);
    }
#endif

    vTaskDelay(pdMS_TO_TICKS(1));
//    handle = i2c_cmd_link_create_static(buffer, sizeof(buffer));

    if (err == ESP_OK) {
    err = i2c_master_read_from_device(I2C_MASTER_NUM, TPM2_I2C_ADDR,
                                          read_buffer, read_size,
                                          I2C_READ_WAIT_TICKS);
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "crap");
    }

//    err = i2c_master_start(handle);
//    if (err != ESP_OK) {
//        goto end;
//    }
//
//    err = i2c_master_write_byte(handle, device_address << 1 | I2C_MASTER_READ, true);
//    if (err != ESP_OK) {
//        goto end;
//    }
//
//    err = i2c_master_read(handle, read_buffer, read_size, I2C_MASTER_LAST_NACK);
//    if (err != ESP_OK) {
//        goto end;
//    }


//     vTaskDelay(pdMS_TO_TICKS(1));
//    i2c_master_stop(handle);
//    err = i2c_master_cmd_begin(i2c_num, handle, ticks_to_wait);

    return err;
}
#endif /* USE_MY_I2C custom */

/* Espressif HAL I2C */
static esp_err_t tpm_register_read(uint32_t reg,
                                    uint8_t *data, size_t len)
{
    int ret;

//esp_err_t i2c_master_write_read_device(i2c_port_t i2c_num, uint8_t device_address,
//                                       const uint8_t* write_buffer, size_t write_size,
//                                       uint8_t* read_buffer, size_t read_size,
//                                       TickType_t ticks_to_wait)

    /* TIS layer should never provide a buffer larger than this,
     * but double check for good coding practice */
    if (len > MAX_SPI_FRAMESIZE)
        return BAD_FUNC_ARG;

    // ESP_LOGI(TAG, "TPM Read init %d len = %d", is_init_read, len);

#ifdef USE_MY_I2C
    byte buf[1];
    buf[0] = (reg & 0xFF); /* convert to simple 8-bit address for I2C */
        ret = my_i2c_master_write_read_device(I2C_MASTER_NUM, TPM2_I2C_ADDR,
                                            buf, sizeof(buf), /* write buffer, len */
                                            data,     len,
                                            I2C_READ_WAIT_TICKS);
#else
    #ifdef USE_IDF_WRITE_READ
        byte buf[1];
        buf[0] = (reg & 0xFF); /* convert to simple 8-bit address for I2C */
        ret = i2c_master_write_read_device(I2C_MASTER_NUM, TPM2_I2C_ADDR,
                                            buf, sizeof(buf), /* write buffer, len */
                                            data,     len,
                                            I2C_READ_WAIT_TICKS);
    #elif defined(USE_IDF_READ)
        ret = i2c_master_read_from_device(I2C_MASTER_NUM, TPM2_I2C_ADDR,
                                          data, len,
                                          I2C_READ_WAIT_TICKS);
    #else
        byte buf[1];
        buf[0] = (reg & 0xFF); /* convert to simple 8-bit address for I2C */
        int timeout = TPM_I2C_TRIES;

        /* The I2C takes about 80us to wake up and will NAK until it is ready */
        do {
            /* Write address to read from - retry until ack  */
            //ret = cyhal_i2c_master_write(i2c, TPM2_I2C_ADDR, buf, sizeof(buf),
            //    0, true);
            ret = i2c_master_write_to_device(I2C_MASTER_NUM, TPM2_I2C_ADDR,
                                     buf, sizeof(buf),
                                     I2C_WRITE_WAIT_TICKS);

            /* for read we always need this guard time (success wake or real read) */
            XSLEEP_MS(1); /* guard time - should be 250us */
        } while (ret != ESP_OK && --timeout > 0);
    #endif
#endif

        if (ret == ESP_OK) {
            timeout = TPM_I2C_TRIES;
            do {
                //ret = cyhal_i2c_master_read(i2c, TPM2_I2C_ADDR, data, len,
                //    0, true);
                ret = i2c_master_read_from_device(I2C_MASTER_NUM, TPM2_I2C_ADDR,
                                                 data, len,
                                                 I2C_READ_WAIT_TICKS);
                if (ret != ESP_OK) {
                    XSLEEP_MS(1); /* guard time - should be 250us */
                }
            } while (ret != ESP_OK && --timeout > 0);
        }



    if (ret == ESP_OK) {
        //ESP_LOGI(TAG, "Success! i2c_master_write_read_device");
        //show_binary(data, len);
    }
    else {
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "ERROR: tpm_register_read failed ESP_ERR_TIMEOUT");
        }
        else {
            ESP_LOGE(TAG, "ERROR: tpm_register_read failed error = %d", ret);
        }
        if (DELETE_I2C_ON_ERROR) {
            i2c_master_delete();
        }
    }
    return ret;
}

static esp_err_t tpm_register_write(uint32_t reg,
                                    uint8_t* data, size_t len)
{
    int result = ESP_FAIL;
    int timeout = TPM_I2C_TRIES;
    byte buf[MAX_SPI_FRAMESIZE+1];

    /* TIS layer should never provide a buffer larger than this,
        * but double check for good coding practice */
    if (len > MAX_SPI_FRAMESIZE)
        return BAD_FUNC_ARG;

    /* Build packet with TPM register and data */
    buf[0] = (reg & 0xFF); /* convert to simple 8-bit address for I2C */
    XMEMCPY(buf + 1, data, len);


    ESP_LOGI(TAG, "TPM Write Len = %d", len);
    show_binary(data, len);

        /* The I2C takes about 80us to wake up and will NAK until it is ready */
        do {
//            result = cyhal_i2c_master_write(i2c, TPM2_I2C_ADDR, buf, len+1,
//                0, true);
            result = i2c_master_write_to_device(I2C_MASTER_NUM, TPM2_I2C_ADDR,
                                     buf, len + 1,
                                     I2C_WRITE_WAIT_TICKS);
            if (result != ESP_OK) {
                XSLEEP_MS(1); /* guard time - should be 250us */
            }
        } while (result != ESP_OK && --timeout > 0);

//    ret = i2c_master_write_to_device(I2C_MASTER_NUM, TPM2_I2C_ADDR,
//                                     data, len,
//                                     I2C_WRITE_WAIT_TICKS);

    if (result == ESP_OK) {
        ESP_LOGI(TAG, "Success! tpm_register_write wrote %d bytes", len);
    }
    else {
        ESP_LOGI(TAG, "ERROR: tpm_register_write failed with code = %d", result);
        if (DELETE_I2C_ON_ERROR) {
            i2c_master_delete();
        }
    }

    return result;
}

static int tpm_ifx_i2c_read(void* userCtx, word32 reg, byte* data, int len)
{
    int ret;
    ret = tpm_register_read(reg, data, len);
    if (ret == ESP_OK) {
        // ESP_LOGI(TAG, "Read device 0x%x success.\n", TPM2_I2C_ADDR);
        ret = TPM_RC_SUCCESS;
    }
    else {
        ESP_LOGE(TAG, "Read device 0x%x fail. Error = %d\n", TPM2_I2C_ADDR, ret);
        ret = TPM_RC_FAILURE;
    }
    return ret;
}

static int tpm_ifx_i2c_write(void* userCtx, word32 reg, byte* data, int len)
{
    int ret;
    ret = tpm_register_write(reg, data, len);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Write device 0x%x success 0x%x len = %d\n", TPM2_I2C_ADDR, (word32)data, len);
        ret = TPM_RC_SUCCESS;
    }
    else {
        ESP_LOGE(TAG, "Read device 0x%x fail. Error = %d\n", TPM2_I2C_ADDR, ret);
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