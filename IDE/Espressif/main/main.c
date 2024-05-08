/* wolftpm_test.c
 *
 * Copyright (C) 2014-2023 wolfSSL Inc.
 *
 * This file is part of wolfTPM.
 *
 * wolfTPM is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * wolfTPM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with wolfTPM.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Espressif */
#include <esp_log.h>

/* wolfSSL */
/* Always include wolfcrypt/settings.h before any other wolfSSL file.    */
/* Reminder: settings.h pulls in user_settings.h; don't include it here. */
#ifdef WOLFSSL_USER_SETTINGS
    #include <wolfssl/wolfcrypt/settings.h>
    #ifndef WOLFSSL_ESPIDF
        #warning "Problem with wolfSSL user_settings."
        #warning "Check components/wolfssl/include"
    #endif
    #include <wolfssl/wolfcrypt/port/Espressif/esp32-crypt.h>
#else
    /* Define WOLFSSL_USER_SETTINGS project wide for settings.h to include   */
    /* wolfSSL user settings in ./components/wolfssl/include/user_settings.h */
    #error "Missing WOLFSSL_USER_SETTINGS in CMakeLists or Makefile:\
    CFLAGS +=-DWOLFSSL_USER_SETTINGS"
#endif


#include <wolftpm/options.h>
// #include "wolftpm_test.h"

#include <wolfssl/internal.h>

/* project */
#include "native_test.h"
#include "main.h"

#ifndef WOLFTPM_MAIN_TEST_ITERATIONS
    #define WOLFTPM_MAIN_TEST_ITERATIONS 1
#endif

static const char* const TAG = "wolfTPM main";

extern int TPM2_Wrapper_Test(void* userCtx);

void app_main(void)
{
    int ret = 0;
    ESP_LOGI(TAG, "Hello wolfTPM!");

#ifdef HAVE_VERSION_EXTENDED_INFO
    ret = esp_ShowExtendedSystemInfo();
#endif

    char mydata[1024];
    int tests = WOLFTPM_MAIN_TEST_ITERATIONS;
    do {
        ret += TPM2_Native_TestArgs(mydata, 0, NULL);
        if (tests > 1) {
            ESP_LOGW(TAG, "*************************************************");
            ESP_LOGW(TAG, "\n\nTest #%d\n\n",
                          WOLFTPM_MAIN_TEST_ITERATIONS - tests + 1);
            ESP_LOGW(TAG, "*************************************************");
            ESP_LOGI(TAG, "Waiting to start next test iteration...\n\n");
            vTaskDelay(5550);
        }
    } while (ret == 0 && (--tests > 0));

//    ret += TPM2_Wrapper_Test(ctx);

#ifdef WOLFSSL_ESPIDF_VERBOSE_EXIT_MESSAGE
    if (ret == 0) {
        ESP_LOGI(TAG, WOLFSSL_ESPIDF_VERBOSE_EXIT_MESSAGE("Success!", ret));
    }
    else {
        ESP_LOGE(TAG, WOLFSSL_ESPIDF_VERBOSE_EXIT_MESSAGE("Failed!", ret));
    }

#elif defined(WOLFSSL_ESPIDF_EXIT_MESSAGE)
    ESP_LOGI(TAG, WOLFSSL_ESPIDF_EXIT_MESSAGE);
#else
    ESP_LOGI(TAG, "\n\nDone!"
                  "If running from idf.py monitor, press twice: Ctrl+]\n\n"
                  "WOLFSSL_COMPLETE\n" /* exit keyword for wolfssl_monitor.py */
            );
#endif
}
