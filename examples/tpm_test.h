/* tpm_test.h
 *
 * Copyright (C) 2006-2020 wolfSSL Inc.
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA
 */


#ifndef _TPM_TEST_H_
#define _TPM_TEST_H_

#include <wolftpm/tpm2.h>

#ifdef __cplusplus
    extern "C" {
#endif

/* Test Configuration */
#define TPM2_DEMO_STORAGE_KEY_HANDLE    0x81000200  /* Persistent Storage Key Handle (RSA) */
#define TPM2_DEMO_STORAGE_EC_KEY_HANDLE 0x81000201  /* Persistent Storage Key Handle (ECC) */

#define TPM2_DEMO_RSA_IDX               0x20        /* offset handle to unused index */
#define TPM2_DEMO_RSA_KEY_HANDLE        (0x81000000 + TPM2_DEMO_RSA_IDX) /* Persistent Key Handle */
#define TPM2_DEMO_RSA_CERT_HANDLE       (0x01800000 + TPM2_DEMO_RSA_IDX) /* NV Handle */

#define TPM2_DEMO_ECC_IDX               0x21        /* offset handle to unused index */
#define TPM2_DEMO_ECC_KEY_HANDLE        (0x81000000 + TPM2_DEMO_ECC_IDX) /* Persistent Key Handle */
#define TPM2_DEMO_ECC_CERT_HANDLE       (0x01800000 + TPM2_DEMO_ECC_IDX) /* NV Handle */

#define TPM2_DEMO_NV_TEST_INDEX         0x01800200
#define TPM2_DEMO_NV_TEST_AUTH_INDEX    0x01800201
#define TPM2_DEMO_NV_TEST_SIZE          1024 /* max size on Infineon SLB9670 is 1664 */

static const char gStorageKeyAuth[] = "ThisIsMyStorageKeyAuth";
static const char gAiKeyAuth[] =      "ThisIsMyAiKeyAuth";
static const char gKeyAuth[] =        "ThisIsMyKeyAuth";
static const char gKeyAuthAlt[] =     "ThisIsMyKeyAltAuth";
static const char gUsageAuth[] =      "ThisIsASecretUsageAuth";
static const char gNvAuth[] =         "ThisIsMyNvAuth";

/* Default Test PCR */
/* PCR16 is for DEBUG purposes, thus safe to use */
#define TPM2_TEST_PCR 16

/* CFB is the more common mode on the TPM because it is used by
 * parameter encryption. Most TPM's don't enable the
 * TPM2_EncryptDecrypt(2) API's because of export controls.
 */
#if 1
#define TEST_AES_MODE TPM_ALG_CFB
#define TEST_AES_KEY kTestAesCfb128Key
#define TEST_AES_IV kTestAesCfb128Iv
#define TEST_AES_MSG kTestAesCfb128Msg
#define TEST_AES_VERIFY kTestAesCfb128Cipher
#else
#define TEST_AES_MODE TPM_ALG_CBC
#define TEST_AES_KEY kTestAesCbc128Key
#define TEST_AES_IV kTestAesCbc128Iv
#define TEST_AES_MSG kTestAesCbc128Msg
#define TEST_AES_VERIFY kTestAesCbc128Verify
#endif

#ifdef WOLFTPM_MCHP
    /* workaround due to issue with older firmware */
    #define TEST_WRAP_DIGEST TPM_ALG_SHA1
#else
    #define TEST_WRAP_DIGEST TPM_ALG_SHA256
#endif


#ifndef NO_TPM_BENCH
    #ifndef WOLFSSL_USER_CURRTIME
        #include <sys/time.h>
    #endif
    static inline double gettime_secs(int reset)
    {
    #ifdef WOLFSSL_USER_CURRTIME
        extern double current_time(int reset);
        return current_time(reset);
    #else
        struct timeval tv;
        gettimeofday(&tv, 0);
        (void)reset;
        return (double)tv.tv_sec + (double)tv.tv_usec / 1000000;
    #endif
    }
#endif /* !NO_TPM_BENCH */


/* RAW KEY MATERIAL */

/* from wolfSSL ./certs/client-key.der */
static const byte kRsaKeyPubModulus[] = {
    0xC3, 0x03, 0xD1, 0x2B, 0xFE, 0x39, 0xA4, 0x32, 0x45, 0x3B, 0x53, 0xC8, 0x84,
    0x2B, 0x2A, 0x7C, 0x74, 0x9A, 0xBD, 0xAA, 0x2A, 0x52, 0x07, 0x47, 0xD6, 0xA6,
    0x36, 0xB2, 0x07, 0x32, 0x8E, 0xD0, 0xBA, 0x69, 0x7B, 0xC6, 0xC3, 0x44, 0x9E,
    0xD4, 0x81, 0x48, 0xFD, 0x2D, 0x68, 0xA2, 0x8B, 0x67, 0xBB, 0xA1, 0x75, 0xC8,
    0x36, 0x2C, 0x4A, 0xD2, 0x1B, 0xF7, 0x8B, 0xBA, 0xCF, 0x0D, 0xF9, 0xEF, 0xEC,
    0xF1, 0x81, 0x1E, 0x7B, 0x9B, 0x03, 0x47, 0x9A, 0xBF, 0x65, 0xCC, 0x7F, 0x65,
    0x24, 0x69, 0xA6, 0xE8, 0x14, 0x89, 0x5B, 0xE4, 0x34, 0xF7, 0xC5, 0xB0, 0x14,
    0x93, 0xF5, 0x67, 0x7B, 0x3A, 0x7A, 0x78, 0xE1, 0x01, 0x56, 0x56, 0x91, 0xA6,
    0x13, 0x42, 0x8D, 0xD2, 0x3C, 0x40, 0x9C, 0x4C, 0xEF, 0xD1, 0x86, 0xDF, 0x37,
    0x51, 0x1B, 0x0C, 0xA1, 0x3B, 0xF5, 0xF1, 0xA3, 0x4A, 0x35, 0xE4, 0xE1, 0xCE,
    0x96, 0xDF, 0x1B, 0x7E, 0xBF, 0x4E, 0x97, 0xD0, 0x10, 0xE8, 0xA8, 0x08, 0x30,
    0x81, 0xAF, 0x20, 0x0B, 0x43, 0x14, 0xC5, 0x74, 0x67, 0xB4, 0x32, 0x82, 0x6F,
    0x8D, 0x86, 0xC2, 0x88, 0x40, 0x99, 0x36, 0x83, 0xBA, 0x1E, 0x40, 0x72, 0x22,
    0x17, 0xD7, 0x52, 0x65, 0x24, 0x73, 0xB0, 0xCE, 0xEF, 0x19, 0xCD, 0xAE, 0xFF,
    0x78, 0x6C, 0x7B, 0xC0, 0x12, 0x03, 0xD4, 0x4E, 0x72, 0x0D, 0x50, 0x6D, 0x3B,
    0xA3, 0x3B, 0xA3, 0x99, 0x5E, 0x9D, 0xC8, 0xD9, 0x0C, 0x85, 0xB3, 0xD9, 0x8A,
    0xD9, 0x54, 0x26, 0xDB, 0x6D, 0xFA, 0xAC, 0xBB, 0xFF, 0x25, 0x4C, 0xC4, 0xD1,
    0x79, 0xF4, 0x71, 0xD3, 0x86, 0x40, 0x18, 0x13, 0xB0, 0x63, 0xB5, 0x72, 0x4E,
    0x30, 0xC4, 0x97, 0x84, 0x86, 0x2D, 0x56, 0x2F, 0xD7, 0x15, 0xF7, 0x7F, 0xC0,
    0xAE, 0xF5, 0xFC, 0x5B, 0xE5, 0xFB, 0xA1, 0xBA, 0xD3
};

static const word32 kRsaKeyPubExponent = 0x10001; /* {0x01, 0x00, 0x01} */

static const byte kRsaKeyPrivQ[] = {
    0xD5, 0x38, 0x1B, 0xC3, 0x8F, 0xC5, 0x93, 0x0C, 0x47, 0x0B, 0x6F, 0x35, 0x92,
    0xC5, 0xB0, 0x8D, 0x46, 0xC8, 0x92, 0x18, 0x8F, 0xF5, 0x80, 0x0A, 0xF7, 0xEF,
    0xA1, 0xFE, 0x80, 0xB9, 0xB5, 0x2A, 0xBA, 0xCA, 0x18, 0xB0, 0x5D, 0xA5, 0x07,
    0xD0, 0x93, 0x8D, 0xD8, 0x9C, 0x04, 0x1C, 0xD4, 0x62, 0x8E, 0xA6, 0x26, 0x81,
    0x01, 0xFF, 0xCE, 0x8A, 0x2A, 0x63, 0x34, 0x35, 0x40, 0xAA, 0x6D, 0x80, 0xDE,
    0x89, 0x23, 0x6A, 0x57, 0x4D, 0x9E, 0x6E, 0xAD, 0x93, 0x4E, 0x56, 0x90, 0x0B,
    0x6D, 0x9D, 0x73, 0x8B, 0x0C, 0xAE, 0x27, 0x3D, 0xDE, 0x4E, 0xF0, 0xAA, 0xC5,
    0x6C, 0x78, 0x67, 0x6C, 0x94, 0x52, 0x9C, 0x37, 0x67, 0x6C, 0x2D, 0xEF, 0xBB,
    0xAF, 0xDF, 0xA6, 0x90, 0x3C, 0xC4, 0x47, 0xCF, 0x8D, 0x96, 0x9E, 0x98, 0xA9,
    0xB4, 0x9F, 0xC5, 0xA6, 0x50, 0xDC, 0xB3, 0xF0, 0xFB, 0x74, 0x17
};


/* Public X/Y and Private D from wolfSSL ./certs/ecc-key.der */
static const byte kEccKeyPubXRaw[] = {
    0xBB, 0x33, 0xAC, 0x4C, 0x27, 0x50, 0x4A, 0xC6, 0x4A, 0xA5, 0x04, 0xC3, 0x3C,
    0xDE, 0x9F, 0x36, 0xDB, 0x72, 0x2D, 0xCE, 0x94, 0xEA, 0x2B, 0xFA, 0xCB, 0x20,
    0x09, 0x39, 0x2C, 0x16, 0xE8, 0x61
};
static const byte kEccKeyPubYRaw[] = {
    0x02, 0xE9, 0xAF, 0x4D, 0xD3, 0x02, 0x93, 0x9A, 0x31, 0x5B, 0x97, 0x92, 0x21,
    0x7F, 0xF0, 0xCF, 0x18, 0xDA, 0x91, 0x11, 0x02, 0x34, 0x86, 0xE8, 0x20, 0x58,
    0x33, 0x0B, 0x80, 0x34, 0x89, 0xD8
};
static const byte kEccKeyPrivD[] = {
    0x45, 0xB6, 0x69, 0x02, 0x73, 0x9C, 0x6C, 0x85, 0xA1, 0x38, 0x5B, 0x72, 0xE8,
    0xE8, 0xC7, 0xAC, 0xC4, 0x03, 0x8D, 0x53, 0x35, 0x04, 0xFA, 0x6C, 0x28, 0xDC,
    0x34, 0x8D, 0xE1, 0xA8, 0x09, 0x8C
};



/* DER (ASN.1) ENCODED KEYS */

#ifndef WOLFTPM2_NO_WOLFCRYPT

/* from wolfSSL ./certs/client-keyPub.der */
static const byte kRsaKeyPubDer[] = {
    0x30, 0x82, 0x01, 0x22, 0x30, 0x0D, 0x06, 0x09, 0x2A, 0x86, 0x48, 0x86, 0xF7,
    0x0D, 0x01, 0x01, 0x01, 0x05, 0x00, 0x03, 0x82, 0x01, 0x0F, 0x00, 0x30, 0x82,
    0x01, 0x0A, 0x02, 0x82, 0x01, 0x01, 0x00, 0xC3, 0x03, 0xD1, 0x2B, 0xFE, 0x39,
    0xA4, 0x32, 0x45, 0x3B, 0x53, 0xC8, 0x84, 0x2B, 0x2A, 0x7C, 0x74, 0x9A, 0xBD,
    0xAA, 0x2A, 0x52, 0x07, 0x47, 0xD6, 0xA6, 0x36, 0xB2, 0x07, 0x32, 0x8E, 0xD0,
    0xBA, 0x69, 0x7B, 0xC6, 0xC3, 0x44, 0x9E, 0xD4, 0x81, 0x48, 0xFD, 0x2D, 0x68,
    0xA2, 0x8B, 0x67, 0xBB, 0xA1, 0x75, 0xC8, 0x36, 0x2C, 0x4A, 0xD2, 0x1B, 0xF7,
    0x8B, 0xBA, 0xCF, 0x0D, 0xF9, 0xEF, 0xEC, 0xF1, 0x81, 0x1E, 0x7B, 0x9B, 0x03,
    0x47, 0x9A, 0xBF, 0x65, 0xCC, 0x7F, 0x65, 0x24, 0x69, 0xA6, 0xE8, 0x14, 0x89,
    0x5B, 0xE4, 0x34, 0xF7, 0xC5, 0xB0, 0x14, 0x93, 0xF5, 0x67, 0x7B, 0x3A, 0x7A,
    0x78, 0xE1, 0x01, 0x56, 0x56, 0x91, 0xA6, 0x13, 0x42, 0x8D, 0xD2, 0x3C, 0x40,
    0x9C, 0x4C, 0xEF, 0xD1, 0x86, 0xDF, 0x37, 0x51, 0x1B, 0x0C, 0xA1, 0x3B, 0xF5,
    0xF1, 0xA3, 0x4A, 0x35, 0xE4, 0xE1, 0xCE, 0x96, 0xDF, 0x1B, 0x7E, 0xBF, 0x4E,
    0x97, 0xD0, 0x10, 0xE8, 0xA8, 0x08, 0x30, 0x81, 0xAF, 0x20, 0x0B, 0x43, 0x14,
    0xC5, 0x74, 0x67, 0xB4, 0x32, 0x82, 0x6F, 0x8D, 0x86, 0xC2, 0x88, 0x40, 0x99,
    0x36, 0x83, 0xBA, 0x1E, 0x40, 0x72, 0x22, 0x17, 0xD7, 0x52, 0x65, 0x24, 0x73,
    0xB0, 0xCE, 0xEF, 0x19, 0xCD, 0xAE, 0xFF, 0x78, 0x6C, 0x7B, 0xC0, 0x12, 0x03,
    0xD4, 0x4E, 0x72, 0x0D, 0x50, 0x6D, 0x3B, 0xA3, 0x3B, 0xA3, 0x99, 0x5E, 0x9D,
    0xC8, 0xD9, 0x0C, 0x85, 0xB3, 0xD9, 0x8A, 0xD9, 0x54, 0x26, 0xDB, 0x6D, 0xFA,
    0xAC, 0xBB, 0xFF, 0x25, 0x4C, 0xC4, 0xD1, 0x79, 0xF4, 0x71, 0xD3, 0x86, 0x40,
    0x18, 0x13, 0xB0, 0x63, 0xB5, 0x72, 0x4E, 0x30, 0xC4, 0x97, 0x84, 0x86, 0x2D,
    0x56, 0x2F, 0xD7, 0x15, 0xF7, 0x7F, 0xC0, 0xAE, 0xF5, 0xFC, 0x5B, 0xE5, 0xFB,
    0xA1, 0xBA, 0xD3, 0x02, 0x03, 0x01, 0x00, 0x01
};

/* from wolfSSL ./certs/client-key.der */
static const byte kRsaKeyPrivDer[] = {
    0x30, 0x82, 0x04, 0xA4, 0x02, 0x01, 0x00, 0x02, 0x82, 0x01, 0x01, 0x00, 0xC3,
    0x03, 0xD1, 0x2B, 0xFE, 0x39, 0xA4, 0x32, 0x45, 0x3B, 0x53, 0xC8, 0x84, 0x2B,
    0x2A, 0x7C, 0x74, 0x9A, 0xBD, 0xAA, 0x2A, 0x52, 0x07, 0x47, 0xD6, 0xA6, 0x36,
    0xB2, 0x07, 0x32, 0x8E, 0xD0, 0xBA, 0x69, 0x7B, 0xC6, 0xC3, 0x44, 0x9E, 0xD4,
    0x81, 0x48, 0xFD, 0x2D, 0x68, 0xA2, 0x8B, 0x67, 0xBB, 0xA1, 0x75, 0xC8, 0x36,
    0x2C, 0x4A, 0xD2, 0x1B, 0xF7, 0x8B, 0xBA, 0xCF, 0x0D, 0xF9, 0xEF, 0xEC, 0xF1,
    0x81, 0x1E, 0x7B, 0x9B, 0x03, 0x47, 0x9A, 0xBF, 0x65, 0xCC, 0x7F, 0x65, 0x24,
    0x69, 0xA6, 0xE8, 0x14, 0x89, 0x5B, 0xE4, 0x34, 0xF7, 0xC5, 0xB0, 0x14, 0x93,
    0xF5, 0x67, 0x7B, 0x3A, 0x7A, 0x78, 0xE1, 0x01, 0x56, 0x56, 0x91, 0xA6, 0x13,
    0x42, 0x8D, 0xD2, 0x3C, 0x40, 0x9C, 0x4C, 0xEF, 0xD1, 0x86, 0xDF, 0x37, 0x51,
    0x1B, 0x0C, 0xA1, 0x3B, 0xF5, 0xF1, 0xA3, 0x4A, 0x35, 0xE4, 0xE1, 0xCE, 0x96,
    0xDF, 0x1B, 0x7E, 0xBF, 0x4E, 0x97, 0xD0, 0x10, 0xE8, 0xA8, 0x08, 0x30, 0x81,
    0xAF, 0x20, 0x0B, 0x43, 0x14, 0xC5, 0x74, 0x67, 0xB4, 0x32, 0x82, 0x6F, 0x8D,
    0x86, 0xC2, 0x88, 0x40, 0x99, 0x36, 0x83, 0xBA, 0x1E, 0x40, 0x72, 0x22, 0x17,
    0xD7, 0x52, 0x65, 0x24, 0x73, 0xB0, 0xCE, 0xEF, 0x19, 0xCD, 0xAE, 0xFF, 0x78,
    0x6C, 0x7B, 0xC0, 0x12, 0x03, 0xD4, 0x4E, 0x72, 0x0D, 0x50, 0x6D, 0x3B, 0xA3,
    0x3B, 0xA3, 0x99, 0x5E, 0x9D, 0xC8, 0xD9, 0x0C, 0x85, 0xB3, 0xD9, 0x8A, 0xD9,
    0x54, 0x26, 0xDB, 0x6D, 0xFA, 0xAC, 0xBB, 0xFF, 0x25, 0x4C, 0xC4, 0xD1, 0x79,
    0xF4, 0x71, 0xD3, 0x86, 0x40, 0x18, 0x13, 0xB0, 0x63, 0xB5, 0x72, 0x4E, 0x30,
    0xC4, 0x97, 0x84, 0x86, 0x2D, 0x56, 0x2F, 0xD7, 0x15, 0xF7, 0x7F, 0xC0, 0xAE,
    0xF5, 0xFC, 0x5B, 0xE5, 0xFB, 0xA1, 0xBA, 0xD3, 0x02, 0x03, 0x01, 0x00, 0x01,
    0x02, 0x82, 0x01, 0x01, 0x00, 0xA2, 0xE6, 0xD8, 0x5F, 0x10, 0x71, 0x64, 0x08,
    0x9E, 0x2E, 0x6D, 0xD1, 0x6D, 0x1E, 0x85, 0xD2, 0x0A, 0xB1, 0x8C, 0x47, 0xCE,
    0x2C, 0x51, 0x6A, 0xA0, 0x12, 0x9E, 0x53, 0xDE, 0x91, 0x4C, 0x1D, 0x6D, 0xEA,
    0x59, 0x7B, 0xF2, 0x77, 0xAA, 0xD9, 0xC6, 0xD9, 0x8A, 0xAB, 0xD8, 0xE1, 0x16,
    0xE4, 0x63, 0x26, 0xFF, 0xB5, 0x6C, 0x13, 0x59, 0xB8, 0xE3, 0xA5, 0xC8, 0x72,
    0x17, 0x2E, 0x0C, 0x9F, 0x6F, 0xE5, 0x59, 0x3F, 0x76, 0x6F, 0x49, 0xB1, 0x11,
    0xC2, 0x5A, 0x2E, 0x16, 0x29, 0x0D, 0xDE, 0xB7, 0x8E, 0xDC, 0x40, 0xD5, 0xA2,
    0xEE, 0xE0, 0x1E, 0xA1, 0xF4, 0xBE, 0x97, 0xDB, 0x86, 0x63, 0x96, 0x14, 0xCD,
    0x98, 0x09, 0x60, 0x2D, 0x30, 0x76, 0x9C, 0x3C, 0xCD, 0xE6, 0x88, 0xEE, 0x47,
    0x92, 0x79, 0x0B, 0x5A, 0x00, 0xE2, 0x5E, 0x5F, 0x11, 0x7C, 0x7D, 0xF9, 0x08,
    0xB7, 0x20, 0x06, 0x89, 0x2A, 0x5D, 0xFD, 0x00, 0xAB, 0x22, 0xE1, 0xF0, 0xB3,
    0xBC, 0x24, 0xA9, 0x5E, 0x26, 0x0E, 0x1F, 0x00, 0x2D, 0xFE, 0x21, 0x9A, 0x53,
    0x5B, 0x6D, 0xD3, 0x2B, 0xAB, 0x94, 0x82, 0x68, 0x43, 0x36, 0xD8, 0xF6, 0x2F,
    0xC6, 0x22, 0xFC, 0xB5, 0x41, 0x5D, 0x0D, 0x33, 0x60, 0xEA, 0xA4, 0x7D, 0x7E,
    0xE8, 0x4B, 0x55, 0x91, 0x56, 0xD3, 0x5C, 0x57, 0x8F, 0x1F, 0x94, 0x17, 0x2F,
    0xAA, 0xDE, 0xE9, 0x9E, 0xA8, 0xF4, 0xCF, 0x8A, 0x4C, 0x8E, 0xA0, 0xE4, 0x56,
    0x73, 0xB2, 0xCF, 0x4F, 0x86, 0xC5, 0x69, 0x3C, 0xF3, 0x24, 0x20, 0x8B, 0x5C,
    0x96, 0x0C, 0xFA, 0x6B, 0x12, 0x3B, 0x9A, 0x67, 0xC1, 0xDF, 0xC6, 0x96, 0xB2,
    0xA5, 0xD5, 0x92, 0x0D, 0x9B, 0x09, 0x42, 0x68, 0x24, 0x10, 0x45, 0xD4, 0x50,
    0xE4, 0x17, 0x39, 0x48, 0xD0, 0x35, 0x8B, 0x94, 0x6D, 0x11, 0xDE, 0x8F, 0xCA,
    0x59, 0x02, 0x81, 0x81, 0x00, 0xEA, 0x24, 0xA7, 0xF9, 0x69, 0x33, 0xE9, 0x71,
    0xDC, 0x52, 0x7D, 0x88, 0x21, 0x28, 0x2F, 0x49, 0xDE, 0xBA, 0x72, 0x16, 0xE9,
    0xCC, 0x47, 0x7A, 0x88, 0x0D, 0x94, 0x57, 0x84, 0x58, 0x16, 0x3A, 0x81, 0xB0,
    0x3F, 0xA2, 0xCF, 0xA6, 0x6C, 0x1E, 0xB0, 0x06, 0x29, 0x00, 0x8F, 0xE7, 0x77,
    0x76, 0xAC, 0xDB, 0xCA, 0xC7, 0xD9, 0x5E, 0x9B, 0x3F, 0x26, 0x90, 0x52, 0xAE,
    0xFC, 0x38, 0x90, 0x00, 0x14, 0xBB, 0xB4, 0x0F, 0x58, 0x94, 0xE7, 0x2F, 0x6A,
    0x7E, 0x1C, 0x4F, 0x41, 0x21, 0xD4, 0x31, 0x59, 0x1F, 0x4E, 0x8A, 0x1A, 0x8D,
    0xA7, 0x57, 0x6C, 0x22, 0xD8, 0xE5, 0xF4, 0x7E, 0x32, 0xA6, 0x10, 0xCB, 0x64,
    0xA5, 0x55, 0x03, 0x87, 0xA6, 0x27, 0x05, 0x8C, 0xC3, 0xD7, 0xB6, 0x27, 0xB2,
    0x4D, 0xBA, 0x30, 0xDA, 0x47, 0x8F, 0x54, 0xD3, 0x3D, 0x8B, 0x84, 0x8D, 0x94,
    0x98, 0x58, 0xA5, 0x02, 0x81, 0x81, 0x00, 0xD5, 0x38, 0x1B, 0xC3, 0x8F, 0xC5,
    0x93, 0x0C, 0x47, 0x0B, 0x6F, 0x35, 0x92, 0xC5, 0xB0, 0x8D, 0x46, 0xC8, 0x92,
    0x18, 0x8F, 0xF5, 0x80, 0x0A, 0xF7, 0xEF, 0xA1, 0xFE, 0x80, 0xB9, 0xB5, 0x2A,
    0xBA, 0xCA, 0x18, 0xB0, 0x5D, 0xA5, 0x07, 0xD0, 0x93, 0x8D, 0xD8, 0x9C, 0x04,
    0x1C, 0xD4, 0x62, 0x8E, 0xA6, 0x26, 0x81, 0x01, 0xFF, 0xCE, 0x8A, 0x2A, 0x63,
    0x34, 0x35, 0x40, 0xAA, 0x6D, 0x80, 0xDE, 0x89, 0x23, 0x6A, 0x57, 0x4D, 0x9E,
    0x6E, 0xAD, 0x93, 0x4E, 0x56, 0x90, 0x0B, 0x6D, 0x9D, 0x73, 0x8B, 0x0C, 0xAE,
    0x27, 0x3D, 0xDE, 0x4E, 0xF0, 0xAA, 0xC5, 0x6C, 0x78, 0x67, 0x6C, 0x94, 0x52,
    0x9C, 0x37, 0x67, 0x6C, 0x2D, 0xEF, 0xBB, 0xAF, 0xDF, 0xA6, 0x90, 0x3C, 0xC4,
    0x47, 0xCF, 0x8D, 0x96, 0x9E, 0x98, 0xA9, 0xB4, 0x9F, 0xC5, 0xA6, 0x50, 0xDC,
    0xB3, 0xF0, 0xFB, 0x74, 0x17, 0x02, 0x81, 0x80, 0x5E, 0x83, 0x09, 0x62, 0xBD,
    0xBA, 0x7C, 0xA2, 0xBF, 0x42, 0x74, 0xF5, 0x7C, 0x1C, 0xD2, 0x69, 0xC9, 0x04,
    0x0D, 0x85, 0x7E, 0x3E, 0x3D, 0x24, 0x12, 0xC3, 0x18, 0x7B, 0xF3, 0x29, 0xF3,
    0x5F, 0x0E, 0x76, 0x6C, 0x59, 0x75, 0xE4, 0x41, 0x84, 0x69, 0x9D, 0x32, 0xF3,
    0xCD, 0x22, 0xAB, 0xB0, 0x35, 0xBA, 0x4A, 0xB2, 0x3C, 0xE5, 0xD9, 0x58, 0xB6,
    0x62, 0x4F, 0x5D, 0xDE, 0xE5, 0x9E, 0x0A, 0xCA, 0x53, 0xB2, 0x2C, 0xF7, 0x9E,
    0xB3, 0x6B, 0x0A, 0x5B, 0x79, 0x65, 0xEC, 0x6E, 0x91, 0x4E, 0x92, 0x20, 0xF6,
    0xFC, 0xFC, 0x16, 0xED, 0xD3, 0x76, 0x0C, 0xE2, 0xEC, 0x7F, 0xB2, 0x69, 0x13,
    0x6B, 0x78, 0x0E, 0x5A, 0x46, 0x64, 0xB4, 0x5E, 0xB7, 0x25, 0xA0, 0x5A, 0x75,
    0x3A, 0x4B, 0xEF, 0xC7, 0x3C, 0x3E, 0xF7, 0xFD, 0x26, 0xB8, 0x20, 0xC4, 0x99,
    0x0A, 0x9A, 0x73, 0xBE, 0xC3, 0x19, 0x02, 0x81, 0x81, 0x00, 0xBA, 0x44, 0x93,
    0x14, 0xAC, 0x34, 0x19, 0x3B, 0x5F, 0x91, 0x60, 0xAC, 0xF7, 0xB4, 0xD6, 0x81,
    0x05, 0x36, 0x51, 0x53, 0x3D, 0xE8, 0x65, 0xDC, 0xAF, 0x2E, 0xDC, 0x61, 0x3E,
    0xC9, 0x7D, 0xB8, 0x7F, 0x87, 0xF0, 0x3B, 0x9B, 0x03, 0x82, 0x29, 0x37, 0xCE,
    0x72, 0x4E, 0x11, 0xD5, 0xB1, 0xC1, 0x0C, 0x07, 0xA0, 0x99, 0x91, 0x4A, 0x8D,
    0x7F, 0xEC, 0x79, 0xCF, 0xF1, 0x39, 0xB5, 0xE9, 0x85, 0xEC, 0x62, 0xF7, 0xDA,
    0x7D, 0xBC, 0x64, 0x4D, 0x22, 0x3C, 0x0E, 0xF2, 0xD6, 0x51, 0xF5, 0x87, 0xD8,
    0x99, 0xC0, 0x11, 0x20, 0x5D, 0x0F, 0x29, 0xFD, 0x5B, 0xE2, 0xAE, 0xD9, 0x1C,
    0xD9, 0x21, 0x56, 0x6D, 0xFC, 0x84, 0xD0, 0x5F, 0xED, 0x10, 0x15, 0x1C, 0x18,
    0x21, 0xE7, 0xC4, 0x3D, 0x4B, 0xD7, 0xD0, 0x9E, 0x6A, 0x95, 0xCF, 0x22, 0xC9,
    0x03, 0x7B, 0x9E, 0xE3, 0x60, 0x01, 0xFC, 0x2F, 0x02, 0x81, 0x80, 0x11, 0xD0,
    0x4B, 0xCF, 0x1B, 0x67, 0xB9, 0x9F, 0x10, 0x75, 0x47, 0x86, 0x65, 0xAE, 0x31,
    0xC2, 0xC6, 0x30, 0xAC, 0x59, 0x06, 0x50, 0xD9, 0x0F, 0xB5, 0x70, 0x06, 0xF7,
    0xF0, 0xD3, 0xC8, 0x62, 0x7C, 0xA8, 0xDA, 0x6E, 0xF6, 0x21, 0x3F, 0xD3, 0x7F,
    0x5F, 0xEA, 0x8A, 0xAB, 0x3F, 0xD9, 0x2A, 0x5E, 0xF3, 0x51, 0xD2, 0xC2, 0x30,
    0x37, 0xE3, 0x2D, 0xA3, 0x75, 0x0D, 0x1E, 0x4D, 0x21, 0x34, 0xD5, 0x57, 0x70,
    0x5C, 0x89, 0xBF, 0x72, 0xEC, 0x4A, 0x6E, 0x68, 0xD5, 0xCD, 0x18, 0x74, 0x33,
    0x4E, 0x8C, 0x3A, 0x45, 0x8F, 0xE6, 0x96, 0x40, 0xEB, 0x63, 0xF9, 0x19, 0x86,
    0x3A, 0x51, 0xDD, 0x89, 0x4B, 0xB0, 0xF3, 0xF9, 0x9F, 0x5D, 0x28, 0x95, 0x38,
    0xBE, 0x35, 0xAB, 0xCA, 0x5C, 0xE7, 0x93, 0x53, 0x34, 0xA1, 0x45, 0x5D, 0x13,
    0x39, 0x65, 0x42, 0x46, 0xA1, 0x9F, 0xCD, 0xF5, 0xBF
};

/* from wolfSSL ./certs/ecc-key-pub.der */
static const byte kEccKeyPubDer[] = {
    0x30, 0x59, 0x30, 0x13, 0x06, 0x07, 0x2A, 0x86, 0x48, 0xCE, 0x3D, 0x02, 0x01,
    0x06, 0x08, 0x2A, 0x86, 0x48, 0xCE, 0x3D, 0x03, 0x01, 0x07, 0x03, 0x42, 0x00,
    0x04, 0x55, 0xBF, 0xF4, 0x0F, 0x44, 0x50, 0x9A, 0x3D, 0xCE, 0x9B, 0xB7, 0xF0,
    0xC5, 0x4D, 0xF5, 0x70, 0x7B, 0xD4, 0xEC, 0x24, 0x8E, 0x19, 0x80, 0xEC, 0x5A,
    0x4C, 0xA2, 0x24, 0x03, 0x62, 0x2C, 0x9B, 0xDA, 0xEF, 0xA2, 0x35, 0x12, 0x43,
    0x84, 0x76, 0x16, 0xC6, 0x56, 0x95, 0x06, 0xCC, 0x01, 0xA9, 0xBD, 0xF6, 0x75,
    0x1A, 0x42, 0xF7, 0xBD, 0xA9, 0xB2, 0x36, 0x22, 0x5F, 0xC7, 0x5D, 0x7F, 0xB4
};

/* from wolfSSL ./certs/ecc-key.der */
static const byte kEccKeyPrivDer[] = {
    0x30, 0x77, 0x02, 0x01, 0x01, 0x04, 0x20, 0x45, 0xB6, 0x69, 0x02, 0x73, 0x9C,
    0x6C, 0x85, 0xA1, 0x38, 0x5B, 0x72, 0xE8, 0xE8, 0xC7, 0xAC, 0xC4, 0x03, 0x8D,
    0x53, 0x35, 0x04, 0xFA, 0x6C, 0x28, 0xDC, 0x34, 0x8D, 0xE1, 0xA8, 0x09, 0x8C,
    0xA0, 0x0A, 0x06, 0x08, 0x2A, 0x86, 0x48, 0xCE, 0x3D, 0x03, 0x01, 0x07, 0xA1,
    0x44, 0x03, 0x42, 0x00, 0x04, 0xBB, 0x33, 0xAC, 0x4C, 0x27, 0x50, 0x4A, 0xC6,
    0x4A, 0xA5, 0x04, 0xC3, 0x3C, 0xDE, 0x9F, 0x36, 0xDB, 0x72, 0x2D, 0xCE, 0x94,
    0xEA, 0x2B, 0xFA, 0xCB, 0x20, 0x09, 0x39, 0x2C, 0x16, 0xE8, 0x61, 0x02, 0xE9,
    0xAF, 0x4D, 0xD3, 0x02, 0x93, 0x9A, 0x31, 0x5B, 0x97, 0x92, 0x21, 0x7F, 0xF0,
    0xCF, 0x18, 0xDA, 0x91, 0x11, 0x02, 0x34, 0x86, 0xE8, 0x20, 0x58, 0x33, 0x0B,
    0x80, 0x34, 0x89, 0xD8
};

#endif /* !WOLFTPM2_NO_WOLFCRYPT */


/* [P-256,SHA-1] vector from FIPS 186-3 NIST vectors */
static const byte kEccTestMsg[] =   {
    /* Test messsage */
    0xa3, 0xf9, 0x1a, 0xe2, 0x1b, 0xa6, 0xb3, 0x03, 0x98, 0x64, 0x47,
    0x2f, 0x18, 0x41, 0x44, 0xc6, 0xaf, 0x62, 0xcd, 0x0e
};
static const byte kEccTestPubQX[] = {
    /* Public ECC Key X */
    0xFA, 0x27, 0x37, 0xFB, 0x93, 0x48, 0x8D, 0x19, 0xCA, 0xEF, 0x11,
    0xAE, 0x7F, 0xAF, 0x6B, 0x7F, 0x4B, 0xCD, 0x67, 0xB2, 0x86, 0xE3,
    0xFC, 0x54, 0xE8, 0xA6, 0x5C, 0x2B, 0x74, 0xAE, 0xCC, 0xB0
};
static const byte kEccTestPubQY[] = {
    /* Public ECC Key Y */
    0xD4, 0xCC, 0xD6, 0xDA, 0xE6, 0x98, 0x20, 0x8A, 0xA8, 0xC3, 0xA6,
    0xF3, 0x9E, 0x45, 0x51, 0x0D, 0x03, 0xBE, 0x09, 0xB2, 0xF1, 0x24,
    0xBF, 0xC0, 0x67, 0x85, 0x6C, 0x32, 0x4F, 0x9B, 0x4D, 0x09
};
static const byte kEccTestSigRS[] = {
    /* Signature R */
    0x2B, 0x82, 0x6F, 0x5D, 0x44, 0xE2, 0xD0, 0xB6, 0xDE, 0x53, 0x1A,
    0xD9, 0x6B, 0x51, 0xE8, 0xF0, 0xC5, 0x6F, 0xDF, 0xEA, 0xD3, 0xC2,
    0x36, 0x89, 0x2E, 0x4D, 0x84, 0xEA, 0xCF, 0xC3, 0xB7, 0x5C,
    /* Signature S */
    0xA2, 0x24, 0x8B, 0x62, 0xC0, 0x3D, 0xB3, 0x5A, 0x7C, 0xD6, 0x3E,
    0x8A, 0x12, 0x0A, 0x35, 0x21, 0xA8, 0x9D, 0x3D, 0x2F, 0x61, 0xFF,
    0x99, 0x03, 0x5A, 0x21, 0x48, 0xAE, 0x32, 0xE3, 0xA2, 0x48
};



/* from certs/dummy-ecc.pem (as DER) */
static const unsigned char DUMMY_ECC_KEY[] = {
    0x30, 0x77, 0x02, 0x01, 0x01, 0x04, 0x20, 0x05, 0x0F, 0xEA, 0xB6, 0x2C, 0x7C,
    0xD3, 0x3C, 0x66, 0x3D, 0x6B, 0x44, 0xD5, 0x8A, 0xD4, 0x1C, 0xF6, 0x2A, 0x35,
    0x49, 0xB2, 0x36, 0x7D, 0xEC, 0xD4, 0xB3, 0x9A, 0x2B, 0x4F, 0x71, 0xC8, 0xD3,
    0xA0, 0x0A, 0x06, 0x08, 0x2A, 0x86, 0x48, 0xCE, 0x3D, 0x03, 0x01, 0x07, 0xA1,
    0x44, 0x03, 0x42, 0x00, 0x04, 0x43, 0x98, 0xF7, 0x33, 0x77, 0xB4, 0x55, 0x02,
    0xF1, 0xF3, 0x79, 0x97, 0x67, 0xED, 0xB5, 0x3A, 0x7A, 0xE1, 0x7C, 0xC6, 0xA8,
    0x23, 0x8B, 0x3A, 0x68, 0x42, 0xDD, 0x68, 0x4F, 0x48, 0x6F, 0x2D, 0x9A, 0x7C,
    0x47, 0x20, 0x1F, 0x13, 0x69, 0x71, 0x05, 0x42, 0x5B, 0x9F, 0x23, 0x7D, 0xE0,
    0xA6, 0x5D, 0xD4, 0x11, 0x44, 0xB1, 0x91, 0x66, 0x50, 0xC0, 0x2C, 0x8C, 0x71,
    0x35, 0x0E, 0x28, 0xB4
};

/* from certs/dummy-rsa.pem (as DER) */
static const unsigned char DUMMY_RSA_KEY[] = {
    0x30, 0x82, 0x04, 0xA3, 0x02, 0x01, 0x00, 0x02, 0x82, 0x01, 0x01, 0x00, 0xCF,
    0xDD, 0xB2, 0x17, 0x49, 0xEB, 0xBF, 0xFB, 0xC5, 0x19, 0x13, 0x63, 0x86, 0x49,
    0xBC, 0xFE, 0x8E, 0xED, 0x21, 0x6E, 0x53, 0x18, 0x9C, 0x41, 0xD5, 0xEC, 0x12,
    0x31, 0xF0, 0xF9, 0x90, 0x08, 0x15, 0x68, 0x2F, 0x00, 0x9C, 0xAC, 0x36, 0x28,
    0xF6, 0xD8, 0x50, 0xA0, 0xD4, 0x7C, 0xDF, 0xE7, 0x0F, 0xE1, 0x36, 0xD1, 0xDD,
    0xC0, 0x2B, 0xF0, 0x3D, 0xC9, 0xF0, 0x5B, 0xE4, 0x76, 0x48, 0x91, 0xF0, 0x92,
    0x29, 0x82, 0x75, 0x7F, 0x0B, 0x41, 0x39, 0x77, 0x52, 0xCD, 0x1F, 0x30, 0xA3,
    0xC3, 0x79, 0x92, 0xBD, 0x0A, 0x7F, 0x16, 0xB2, 0x06, 0xFD, 0x49, 0xC5, 0x4D,
    0x34, 0x26, 0xDB, 0x49, 0x06, 0xDB, 0x49, 0x63, 0xB6, 0xE5, 0xA4, 0xEC, 0xC0,
    0x6D, 0x24, 0xF1, 0x82, 0x0F, 0x83, 0x1B, 0xB1, 0x0D, 0xA3, 0x8B, 0x6A, 0x39,
    0x39, 0xB6, 0xB3, 0xA3, 0xE1, 0x77, 0x69, 0x8C, 0xC7, 0x83, 0xE1, 0xBE, 0x9E,
    0xF9, 0xB7, 0xDB, 0xDF, 0xF8, 0x98, 0x7C, 0x9D, 0xC8, 0x72, 0x78, 0xBF, 0x13,
    0x62, 0x27, 0xA1, 0xBF, 0x4B, 0x2B, 0x04, 0x18, 0xCD, 0x2C, 0x10, 0x7E, 0xA5,
    0x33, 0x08, 0xD4, 0x49, 0xF1, 0xEC, 0x99, 0x6F, 0x2E, 0x0B, 0xB4, 0xD3, 0xB3,
    0xC2, 0x20, 0x02, 0xE9, 0x3A, 0xA1, 0xB3, 0x81, 0x9B, 0x0C, 0x02, 0xB0, 0xDE,
    0x9E, 0xEF, 0x0A, 0x47, 0x6E, 0xFA, 0xDB, 0x4D, 0x13, 0x1E, 0x1F, 0xD2, 0x7B,
    0xC6, 0x48, 0xE8, 0x27, 0xDE, 0xBC, 0x8D, 0x4C, 0x60, 0x5A, 0x71, 0xB5, 0xC3,
    0x7F, 0xFC, 0x7C, 0x28, 0xC1, 0x99, 0xF2, 0x7A, 0x3B, 0xCD, 0x6A, 0x76, 0xFE,
    0xA8, 0x9B, 0xDD, 0x03, 0x1E, 0xEB, 0xB4, 0x9D, 0x70, 0x5C, 0x7A, 0x1F, 0xB6,
    0x12, 0xEC, 0xCD, 0xAC, 0x6A, 0xCD, 0x2C, 0x25, 0x53, 0xEF, 0x34, 0xD5, 0xC5,
    0x97, 0x14, 0xD4, 0xB2, 0x86, 0x03, 0x5F, 0x89, 0x02, 0x03, 0x01, 0x00, 0x01,
    0x02, 0x82, 0x01, 0x00, 0x5D, 0x9E, 0xBF, 0x10, 0x48, 0x25, 0xDB, 0x00, 0xFD,
    0x43, 0x8E, 0xFC, 0xFB, 0x45, 0x88, 0xCE, 0xA9, 0xF6, 0xD9, 0x60, 0xC4, 0x22,
    0x48, 0x76, 0x4A, 0x70, 0x19, 0xBD, 0xCE, 0x87, 0xC8, 0x3C, 0x2B, 0xD0, 0x11,
    0xA3, 0x57, 0xED, 0x24, 0x33, 0x8D, 0x01, 0xDE, 0x46, 0xA1, 0x8D, 0x60, 0x96,
    0xC4, 0x0B, 0x2E, 0x52, 0x95, 0x6A, 0x71, 0x1F, 0xB1, 0xE4, 0x9A, 0xD1, 0xF8,
    0x72, 0xE1, 0xBA, 0x81, 0x3C, 0x83, 0x5F, 0x93, 0xA5, 0xD5, 0x9E, 0xD9, 0xD0,
    0x09, 0x46, 0x03, 0x6F, 0x37, 0xC2, 0xD9, 0xA5, 0xA2, 0x68, 0xF0, 0xD6, 0x7A,
    0xF6, 0x34, 0xEC, 0x1D, 0xE5, 0xE8, 0xC0, 0x3B, 0x71, 0x87, 0x9A, 0x0A, 0x52,
    0xD3, 0xD4, 0x58, 0x54, 0x9D, 0x52, 0x4B, 0x1A, 0x4E, 0xF6, 0xC7, 0x99, 0x18,
    0x44, 0x49, 0x4D, 0x88, 0x59, 0x1F, 0xCA, 0x4E, 0xDC, 0x57, 0xB7, 0x1D, 0x9D,
    0xDF, 0x59, 0x91, 0xD9, 0x2E, 0xE0, 0x54, 0xAA, 0x4E, 0x8F, 0x92, 0x82, 0x85,
    0x70, 0xF9, 0x93, 0x90, 0x3A, 0x30, 0xCD, 0xB3, 0x73, 0x81, 0x93, 0xE7, 0xF9,
    0x1F, 0xF6, 0xA9, 0xA9, 0xD4, 0xAE, 0x89, 0x0E, 0x38, 0x11, 0x61, 0xF7, 0xF7,
    0xDC, 0x9B, 0x99, 0x4B, 0xFE, 0xC0, 0x71, 0x78, 0x53, 0x18, 0x0F, 0x23, 0xA9,
    0x11, 0xA0, 0xAA, 0x57, 0xEE, 0x39, 0xAA, 0xEA, 0x2A, 0x7A, 0x8D, 0x12, 0x69,
    0x2C, 0x82, 0x4D, 0xA0, 0xE5, 0x1C, 0xB3, 0x69, 0x9D, 0xA1, 0x30, 0xA3, 0x40,
    0xFA, 0x86, 0x40, 0xD3, 0x8B, 0xF9, 0xAF, 0x98, 0x7D, 0x17, 0x07, 0xA3, 0x29,
    0xE2, 0x57, 0xEF, 0x47, 0xCF, 0x81, 0x22, 0x4D, 0x47, 0x63, 0xA4, 0x2F, 0x1A,
    0x8F, 0xC3, 0x26, 0x1F, 0xF6, 0xC5, 0x81, 0xFF, 0x14, 0xA9, 0x87, 0x56, 0x18,
    0x8A, 0x18, 0xFD, 0x37, 0xC3, 0x4B, 0x8E, 0xE0, 0x6B, 0x2C, 0x07, 0x4B, 0x05,
    0x02, 0x81, 0x81, 0x00, 0xED, 0x51, 0x06, 0x91, 0xB8, 0x94, 0x5E, 0x17, 0x9B,
    0x22, 0x25, 0xEF, 0x23, 0x76, 0x61, 0x26, 0xFA, 0xAC, 0xEE, 0xC1, 0x99, 0x8E,
    0x55, 0x38, 0x85, 0xD2, 0x15, 0x06, 0x6E, 0xBB, 0x45, 0xBB, 0xFE, 0x5F, 0xF7,
    0xD2, 0xA4, 0x41, 0x15, 0x24, 0x67, 0x8E, 0xA2, 0x6B, 0xBA, 0xAA, 0x28, 0x84,
    0x22, 0x63, 0xEE, 0xA8, 0xA0, 0xD0, 0xEA, 0x47, 0x8C, 0xAC, 0x4E, 0x98, 0x18,
    0x8A, 0xF2, 0x19, 0x76, 0x50, 0x9D, 0xFE, 0xD1, 0x59, 0xC3, 0xC1, 0x23, 0x3B,
    0x31, 0x73, 0xC7, 0x71, 0x3E, 0x94, 0xC8, 0x6D, 0x7F, 0xBA, 0x30, 0xF2, 0x4C,
    0x1A, 0x7E, 0x74, 0x52, 0x78, 0xA0, 0xAB, 0x69, 0x0C, 0x44, 0x59, 0xD0, 0xB0,
    0xFE, 0x2F, 0xE8, 0xC2, 0x18, 0xE7, 0x24, 0x8B, 0x73, 0xDF, 0x4F, 0x40, 0x92,
    0x0C, 0x8C, 0x6C, 0x92, 0x27, 0xBC, 0x3F, 0x5B, 0x79, 0x44, 0xC1, 0x32, 0xCC,
    0xA2, 0xB7, 0x02, 0x81, 0x81, 0x00, 0xE0, 0x3B, 0x1C, 0xC4, 0xC4, 0x69, 0x0D,
    0x6B, 0x57, 0x19, 0xB7, 0x59, 0x18, 0x92, 0xB2, 0x09, 0x39, 0x66, 0x97, 0xD1,
    0x18, 0xDE, 0x6B, 0x5F, 0xC5, 0x9B, 0x11, 0x47, 0x1E, 0xEA, 0xE3, 0xAC, 0x36,
    0x2B, 0x30, 0x99, 0x81, 0x00, 0x3D, 0x39, 0x41, 0x03, 0x90, 0x77, 0xDE, 0x4A,
    0xE1, 0x48, 0xDF, 0x98, 0x86, 0x03, 0x3B, 0xEA, 0xAF, 0xC8, 0xF6, 0xD7, 0x4F,
    0xE6, 0xAE, 0x70, 0xF2, 0x3D, 0xBB, 0xF2, 0x63, 0xB9, 0x2D, 0x3C, 0x08, 0xB3,
    0x10, 0x9E, 0x97, 0x6C, 0x8D, 0x28, 0x34, 0xAE, 0xDA, 0xD9, 0xA1, 0x8E, 0x3A,
    0x51, 0x7A, 0xA1, 0x14, 0x3F, 0xFB, 0xEA, 0x3B, 0xB4, 0x93, 0xAA, 0x14, 0x7A,
    0xB4, 0xD7, 0xCA, 0x7B, 0x61, 0xAF, 0xF5, 0x87, 0x1A, 0x64, 0xA9, 0x3E, 0x3C,
    0x7A, 0xDD, 0x11, 0x7F, 0x01, 0x2D, 0xA6, 0x91, 0xED, 0x3D, 0x28, 0x9C, 0x67,
    0xC2, 0x5C, 0xCF, 0xBF, 0x02, 0x81, 0x81, 0x00, 0xCE, 0x0C, 0x59, 0xCD, 0xD0,
    0x1B, 0x52, 0x0E, 0xE0, 0xED, 0x27, 0x4E, 0x98, 0xD5, 0xC1, 0xC8, 0x9C, 0x41,
    0xE6, 0x13, 0x46, 0x06, 0x24, 0xCC, 0x2C, 0xB4, 0x98, 0xF8, 0xBA, 0xCF, 0xF2,
    0xDE, 0x25, 0x20, 0xA2, 0x05, 0xCC, 0x03, 0x8E, 0x1D, 0xCB, 0xA4, 0x36, 0x35,
    0x9F, 0x1E, 0xFA, 0x8A, 0xAF, 0x69, 0x60, 0xE0, 0x1C, 0xB1, 0x07, 0x99, 0x13,
    0xF4, 0xCF, 0x50, 0x93, 0x8E, 0xA0, 0x61, 0xA7, 0x2E, 0x9B, 0xDF, 0x91, 0x59,
    0x84, 0xF3, 0x7E, 0x69, 0x78, 0xA8, 0x73, 0xF4, 0x49, 0x47, 0xD9, 0x35, 0xE9,
    0x7E, 0x79, 0xDD, 0x06, 0x62, 0xC2, 0x84, 0xB0, 0xCE, 0x77, 0x82, 0x1C, 0x75,
    0x40, 0x2B, 0x53, 0x5D, 0x39, 0x75, 0xD3, 0x7C, 0x23, 0x2F, 0x1D, 0xB5, 0xCE,
    0xE7, 0x86, 0xE2, 0x23, 0x6C, 0xAD, 0xC7, 0xDE, 0xA6, 0x8D, 0x75, 0xDD, 0x30,
    0x4F, 0x98, 0x07, 0x49, 0x51, 0xC5, 0x02, 0x81, 0x80, 0x46, 0x19, 0x34, 0xBD,
    0x2E, 0xC9, 0xC8, 0xB0, 0x2D, 0xE2, 0x94, 0x36, 0xFE, 0x3F, 0x9D, 0xF8, 0xD4,
    0x41, 0x06, 0x65, 0x0F, 0xE9, 0x38, 0x98, 0x10, 0x26, 0x92, 0x18, 0x31, 0xCA,
    0x2C, 0xB2, 0xC1, 0x9C, 0x6E, 0xED, 0x0E, 0x2F, 0x0C, 0xF4, 0xC1, 0x26, 0x64,
    0x1B, 0x95, 0x1A, 0xC3, 0xA3, 0x0C, 0x83, 0x9A, 0x21, 0x98, 0xB1, 0x9D, 0x92,
    0xAD, 0xD8, 0x51, 0xDA, 0x43, 0xDE, 0x7B, 0x5C, 0x61, 0x4D, 0x3D, 0x6F, 0xBE,
    0x7C, 0x6E, 0x1B, 0xCC, 0xAE, 0x47, 0x98, 0x5F, 0xE8, 0x99, 0xCF, 0xB0, 0x0B,
    0x29, 0x3E, 0x55, 0x6C, 0xF3, 0x71, 0x37, 0xEB, 0x68, 0xCD, 0xA9, 0x2C, 0xA2,
    0x9D, 0x21, 0x19, 0xDB, 0x3F, 0x3A, 0xC5, 0xA7, 0x9C, 0x62, 0x9D, 0x81, 0xDA,
    0xC6, 0x2D, 0xF6, 0xAA, 0x52, 0x42, 0x0D, 0xFA, 0x48, 0x53, 0x32, 0x7B, 0x80,
    0x0B, 0x1A, 0x1A, 0x35, 0xE0, 0xDD, 0xF1, 0x02, 0x81, 0x80, 0x76, 0x46, 0xB9,
    0x57, 0x91, 0x3F, 0x64, 0x5D, 0x42, 0x37, 0x70, 0x9D, 0x44, 0x38, 0x09, 0x09,
    0x42, 0x3E, 0x2E, 0x8A, 0x7A, 0xA4, 0x57, 0x4B, 0x81, 0x95, 0x65, 0x47, 0x3C,
    0xF3, 0x77, 0x54, 0xE3, 0x7D, 0xEC, 0x06, 0xC9, 0x26, 0xAB, 0xDD, 0x66, 0x73,
    0x54, 0x86, 0x31, 0x26, 0x75, 0x5B, 0x84, 0xAB, 0xD2, 0xA2, 0x6A, 0x9B, 0x6E,
    0xDD, 0x45, 0xAE, 0x81, 0x49, 0x12, 0x8D, 0x03, 0x1C, 0x1B, 0x6B, 0x5B, 0x37,
    0xFA, 0xE7, 0x05, 0x9F, 0xBD, 0x66, 0xDD, 0x6C, 0xD7, 0x16, 0x0D, 0xCC, 0x64,
    0x19, 0xC2, 0xCD, 0xC3, 0xA9, 0xED, 0x70, 0xFA, 0x75, 0xD8, 0x41, 0xF7, 0xC6,
    0x84, 0xE8, 0x40, 0xF0, 0xE5, 0x93, 0x88, 0xE2, 0x4E, 0x4F, 0xE4, 0x5F, 0xDF,
    0x53, 0xAB, 0xA7, 0x06, 0xDC, 0x64, 0x7E, 0x51, 0xE8, 0x7E, 0x1C, 0x33, 0x9F,
    0xBF, 0x5E, 0x58, 0xBC, 0x7D, 0xA3, 0x80, 0x84
};


static const byte kTestAesCbc128Key[] = {
    '0', '1', '2', '3', '4', '5', '6', '7',
    '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'
};
static const byte kTestAesCbc128Iv[]  = {
    '0', '1', '2', '3', '4', '5', '6', '7',
    '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'
};

static const byte kTestAesCbc128Msg[] = { /* "Now is the time for all " */
    0x6e,0x6f,0x77,0x20,0x69,0x73,0x20,0x74,
    0x68,0x65,0x20,0x74,0x69,0x6d,0x65,0x20
};
static const byte kTestAesCbc128Verify[] = {
    0x95,0x94,0x92,0x57,0x5f,0x42,0x81,0x53,
    0x2c,0xcc,0x9d,0x46,0x77,0xa2,0x33,0xcb
};

static const byte kTestAesCfb128Iv[] = {
    0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
    0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f
};

static const byte kTestAesCfb128Key[] = {
    0x2b,0x7e,0x15,0x16,0x28,0xae,0xd2,0xa6,
    0xab,0xf7,0x15,0x88,0x09,0xcf,0x4f,0x3c
};

static const byte kTestAesCfb128Cipher[] = {
    0x3b,0x3f,0xd9,0x2e,0xb7,0x2d,0xad,0x20,
    0x33,0x34,0x49,0xf8,0xe8,0x3c,0xfb,0x4a,
    0xc8,0xa6,0x45,0x37,0xa0,0xb3,0xa9,0x3f,
    0xcd,0xe3,0xcd,0xad,0x9f,0x1c,0xe5,0x8b,
    0x26,0x75,0x1f,0x67,0xa3,0xcb,0xb1,0x40,
    0xb1,0x80,0x8c,0xf1,0x87,0xa4,0xf4,0xdf
};

static const byte kTestAesCfb128Msg[] = {
    0x6b,0xc1,0xbe,0xe2,0x2e,0x40,0x9f,0x96,
    0xe9,0x3d,0x7e,0x11,0x73,0x93,0x17,0x2a,
    0xae,0x2d,0x8a,0x57,0x1e,0x03,0xac,0x9c,
    0x9e,0xb7,0x6f,0xac,0x45,0xaf,0x8e,0x51,
    0x30,0xc8,0x1c,0x46,0xa3,0x5c,0xe4,0x11,
    0xe5,0xfb,0xc1,0x19,0x1a,0x0a,0x52,0xef
};

#ifdef __cplusplus
    }  /* extern "C" */
#endif

#endif /* _TPM_TEST_H_ */
