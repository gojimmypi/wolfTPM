/* tpm2_wrap.c
 *
 * Copyright (C) 2006-2018 wolfSSL Inc.
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

#include <wolftpm/tpm2_wrap.h>

#ifndef WOLFTPM2_NO_WRAPPER

/* For some struct to buffer conversions */
#include <wolftpm/tpm2_packet.h>


/******************************************************************************/
/* --- BEGIN Wrapper Device Functions -- */
/******************************************************************************/

int wolfTPM2_Init(WOLFTPM2_DEV* dev, TPM2HalIoCb ioCb, void* userCtx)
{
    int rc;
    Startup_In startupIn;

    if (dev == NULL)
        return BAD_FUNC_ARG;

    rc = TPM2_Init(&dev->ctx, ioCb, userCtx);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_Init failed %d: %s\n", rc, wolfTPM2_GetRCString(rc));
    #endif
        return rc;
    }

#ifdef DEBUG_WOLFTPM
    printf("TPM2: Caps 0x%08x, Did 0x%04x, Vid 0x%04x, Rid 0x%2x \n",
        dev->ctx.caps,
        dev->ctx.did_vid >> 16,
        dev->ctx.did_vid & 0xFFFF,
        dev->ctx.rid);
#endif

    /* define the default session auth */
    XMEMSET(dev->session, 0, sizeof(dev->session));
    wolfTPM2_SetAuth(dev, 0, TPM_RS_PW, NULL, 0);

    /* startup */
    XMEMSET(&startupIn, 0, sizeof(Startup_In));
    startupIn.startupType = TPM_SU_CLEAR;
    rc = TPM2_Startup(&startupIn);
    if (rc != TPM_RC_SUCCESS &&
        rc != TPM_RC_INITIALIZE /* TPM_RC_INITIALIZE = Already started */ ) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_Startup failed %d: %s\n", rc, wolfTPM2_GetRCString(rc));
    #endif
        return rc;
    }
#ifdef DEBUG_WOLFTPM
    printf("TPM2_Startup pass\n");
#endif

    return TPM_RC_SUCCESS;
}

int wolfTPM2_GetTpmDevId(WOLFTPM2_DEV* dev)
{
    if (dev == NULL) {
        return BAD_FUNC_ARG;
    }

    return dev->ctx.did_vid; /* not INVALID_DEVID */
}


/* Infineon SLB9670
 *  TPM_PT_MANUFACTURER     "IFX"
 *  TPM_PT_VENDOR_STRING_1  "SLB9"
 *  TPM_PT_VENDOR_STRING_2  "670 "
 *  TPM_PT_FIRMWARE_VERSION_1 0x00070055 = v7.85
 *  TPM_PT_FIRMWARE_VERSION_2 0x0011CB02
 *      Byte  1: reserved.
 *      Bytes 2-3: build num = 11CB,
 *      Byte  4: 0x00 (TPM CC), 0x02 (no CC)
 *  TPM_PT_MODES = Bit 0 = FIPS_140_2
 */

/* ST33TP
 *  TPM_PT_MANUFACTURER 0x53544D20: “STM”
 *  TPM_PT_FIRMWARE_VERSION_1 TPM FW version: 0x00006400
 *  TPM_PT_VENDOR_TPM_TYPE 1: TPM 2.0
 *  TPM_PT_MODES: BIT 0 SET (1): indicates that the TPM is designed to
 *      comply with all of the FIPS 140-2 requirements at Level 1 or higher.
 *   TPM_PT_FIRMWARE_VERSION_2: ST Internal Additional Version
 */
static int wolfTPM2_ParseCapabilities(WOLFTPM2_CAPS* caps,
    TPML_TAGGED_TPM_PROPERTY* props)
{
    int rc = 0;
    word32 i, val, len;

    for (i=0; i<props->count && i<MAX_TPM_PROPERTIES; i++) {
        val = props->tpmProperty[i].value;
        switch (props->tpmProperty[i].property) {
            case TPM_PT_MANUFACTURER:
                val = TPM2_Packet_SwapU32(val); /* swap for little endian */
                XMEMCPY(&caps->mfgStr, &val, sizeof(UINT32));
                if (XMEMCMP(&caps->mfgStr, "IFX", 3) == 0) {
                    caps->mfg = TPM_MFG_INFINEON;
                }
                else if (XMEMCMP(&caps->mfgStr, "STM", 3) == 0) {
                    caps->mfg = TPM_MFG_STM;
                }
                else if (XMEMCMP(&caps->mfgStr, "MCHP", 4) == 0) {
                    caps->mfg = TPM_MFG_MCHP;
                }
                break;
            case TPM_PT_VENDOR_STRING_1:
            case TPM_PT_VENDOR_STRING_2:
            case TPM_PT_VENDOR_STRING_3:
            case TPM_PT_VENDOR_STRING_4:
                val = TPM2_Packet_SwapU32(val); /* swap for little endian */
                len = (word32)XSTRLEN(caps->vendorStr); /* add to existing string */
                if (len + sizeof(UINT32) < sizeof(caps->vendorStr)) {
                    XMEMCPY(&caps->vendorStr[len], &val, sizeof(UINT32));
                }
                if (val == 0x46495053) { /* FIPS */
                    caps->fips140_2 = 1;
                }
                break;
            case TPM_PT_VENDOR_TPM_TYPE:
                caps->tpmType = val;
                break;
            case TPM_PT_FIRMWARE_VERSION_1:
                caps->fwVerMajor = val >> 16;
                caps->fwVerMinor = val & 0xFFFF;
                break;
            case TPM_PT_FIRMWARE_VERSION_2:
                if (caps->mfg == TPM_MFG_INFINEON) {
                    caps->fwVerVendor = val >> 8;
                    caps->cc_eal4 = (val & 0x00000002) ? 0 : 1;
                }
                else {
                    caps->fwVerVendor = val;
                }
                break;
            case TPM_PT_MODES:
                caps->fips140_2 = (val & 0x00000001) ? 1: 0;
                break;
            default:
                break;
        }
    }
    return rc;
}

int wolfTPM2_GetCapabilities(WOLFTPM2_DEV* dev, WOLFTPM2_CAPS* cap)
{
    int rc;
    GetCapability_In  in;
    GetCapability_Out out;

    if (dev == NULL || cap == NULL)
        return BAD_FUNC_ARG;

    /* clear caps */
    XMEMSET(cap, 0, sizeof(WOLFTPM2_CAPS));

    /* Get Capabilities TPM_PT_MANUFACTURER thru TPM_PT_FIRMWARE_VERSION_2 */
    XMEMSET(&in, 0, sizeof(in));
    in.capability = TPM_CAP_TPM_PROPERTIES;
    in.property = TPM_PT_MANUFACTURER;
    in.propertyCount = 8;
    rc = TPM2_GetCapability(&in, &out);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_GetCapability failed 0x%x: %s\n", rc,
            TPM2_GetRCString(rc));
    #endif
        return rc;
    }
    rc = wolfTPM2_ParseCapabilities(cap, &out.capabilityData.data.tpmProperties);
    if (rc != 0)
        return rc;

    /* Get Capability TPM_PT_MODES */
    XMEMSET(&in, 0, sizeof(in));
    in.capability = TPM_CAP_TPM_PROPERTIES;
    in.property = TPM_PT_MODES;
    in.propertyCount = 1;
    rc = TPM2_GetCapability(&in, &out);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_GetCapability failed 0x%x: %s\n", rc,
            TPM2_GetRCString(rc));
    #endif
        return rc;
    }
    rc = wolfTPM2_ParseCapabilities(cap, &out.capabilityData.data.tpmProperties);

    return rc;
}


int wolfTPM2_SetAuth(WOLFTPM2_DEV* dev, int index,
    TPM_HANDLE sessionHandle, const byte* auth, int authSz)
{
    if (dev == NULL || index >= MAX_SESSION_NUM) {
        return BAD_FUNC_ARG;
    }

    /* define the default session auth */
    XMEMSET(&dev->session[index], 0, sizeof(dev->session[index]));
    dev->session[index].sessionHandle = sessionHandle;
    dev->session[index].auth.size = authSz;
    if (auth && authSz > 0)
        XMEMCPY(dev->session[index].auth.buffer, auth, authSz);

    TPM2_SetSessionAuth(dev->session);

    return 0;
}

int wolfTPM2_Cleanup(WOLFTPM2_DEV* dev)
{
    int rc;
    Shutdown_In shutdownIn;

    if (dev == NULL)
        return BAD_FUNC_ARG;

    shutdownIn.shutdownType = TPM_SU_CLEAR;
    rc = TPM2_Shutdown(&shutdownIn);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_Shutdown failed %d: %s\n", rc, wolfTPM2_GetRCString(rc));
    #endif
        return rc;
    }

    TPM2_Cleanup(&dev->ctx);

    return rc;
}


int wolfTPM2_StartSession(WOLFTPM2_DEV* dev, WOLFTPM2_SESSION* session,
    WOLFTPM2_KEY* tpmKey, WOLFTPM2_HANDLE* bind, TPM_SE sesType,
    int useEncrypDecrypt)
{
    int rc;
    StartAuthSession_In  authSesIn;
    StartAuthSession_Out authSesOut;

    if (dev == NULL || session == NULL)
        return BAD_FUNC_ARG;

    XMEMSET(&authSesIn, 0, sizeof(authSesIn));
    authSesIn.tpmKey = tpmKey ? tpmKey->handle.hndl : TPM_RH_NULL;
    authSesIn.bind =     bind ? bind->hndl          : TPM_RH_NULL;
    authSesIn.sessionType = sesType;
    if (useEncrypDecrypt) {
        authSesIn.symmetric.algorithm = TPM_ALG_AES;
        authSesIn.symmetric.keyBits.aes = 128;
        authSesIn.symmetric.mode.aes = TPM_ALG_CFB;
    }
    else {
        authSesIn.symmetric.algorithm = TPM_ALG_NULL;
    }
    authSesIn.authHash = WOLFTPM2_WRAP_DIGEST;
    authSesIn.nonceCaller.size = TPM2_GetHashDigestSize(WOLFTPM2_WRAP_DIGEST);
    rc = TPM2_GetNonce(authSesIn.nonceCaller.buffer,
                       authSesIn.nonceCaller.size);
    if (rc < 0) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_GetNonce failed %d: %s\n", rc, wolfTPM2_GetRCString(rc));
    #endif
        return rc;
    }
    rc = TPM2_StartAuthSession(&authSesIn, &authSesOut);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_StartAuthSession failed %d: %s\n", rc,
            wolfTPM2_GetRCString(rc));
    #endif
        return rc;
    }

    session->handle.dev = dev;
    session->handle.hndl = authSesOut.sessionHandle;
    session->nonceTPM = authSesOut.nonceTPM;

#ifdef DEBUG_WOLFTPM
    printf("TPM2_StartAuthSession: handle 0x%x\n",
        (word32)session->handle.hndl);
#endif

    return rc;
}


int wolfTPM2_CreatePrimaryKey(WOLFTPM2_DEV* dev, WOLFTPM2_KEY* key,
    TPM_HANDLE primaryHandle, TPMT_PUBLIC* publicTemplate,
    const byte* auth, int authSz)
{
    int rc;
    CreatePrimary_In  createPriIn;
    CreatePrimary_Out createPriOut;

    if (dev == NULL || key == NULL || publicTemplate == NULL)
        return BAD_FUNC_ARG;

    /* clear output key buffer */
    XMEMSET(key, 0, sizeof(WOLFTPM2_KEY));

    /* setup create primary command */
    XMEMSET(&createPriIn, 0, sizeof(createPriIn));
    /* TPM_RH_OWNER, TPM_RH_ENDORSEMENT, TPM_RH_PLATFORM or TPM_RH_NULL */
    createPriIn.primaryHandle = primaryHandle;
    if (auth && authSz > 0) {
        int nameAlgDigestSz = TPM2_GetHashDigestSize(publicTemplate->nameAlg);
        /* truncate if longer than name size */
        if (nameAlgDigestSz > 0 && authSz > nameAlgDigestSz)
            authSz = nameAlgDigestSz;
        XMEMCPY(createPriIn.inSensitive.sensitive.userAuth.buffer, auth, authSz);
        /* make sure auth is same size as nameAlg digest size */
        if (nameAlgDigestSz > 0 && authSz < nameAlgDigestSz)
            authSz = nameAlgDigestSz;
        createPriIn.inSensitive.sensitive.userAuth.size = authSz;
    }
    XMEMCPY(&createPriIn.inPublic.publicArea, publicTemplate,
        sizeof(TPMT_PUBLIC));
    rc = TPM2_CreatePrimary(&createPriIn, &createPriOut);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_CreatePrimary: failed %d: %s\n", rc,
            wolfTPM2_GetRCString(rc));
    #endif
        return rc;
    }
    key->handle.dev  = dev;
    key->handle.hndl = createPriOut.objectHandle;
    key->handle.auth = createPriIn.inSensitive.sensitive.userAuth;

    key->pub = createPriOut.outPublic;
    key->name = createPriOut.name;

#ifdef DEBUG_WOLFTPM
    printf("TPM2_CreatePrimary: 0x%x (%d bytes)\n",
		(word32)key->handle.hndl, key->pub.size);
#endif

    return rc;
}

int wolfTPM2_CreateAndLoadKey(WOLFTPM2_DEV* dev, WOLFTPM2_KEY* key,
    WOLFTPM2_HANDLE* parent, TPMT_PUBLIC* publicTemplate,
    const byte* auth, int authSz)
{
    int rc;
    Create_In  createIn;
    Create_Out createOut;
    Load_In  loadIn;
    Load_Out loadOut;

    if (dev == NULL || key == NULL || parent == NULL || publicTemplate == NULL)
        return BAD_FUNC_ARG;

    /* clear output key buffer */
    XMEMSET(key, 0, sizeof(WOLFTPM2_KEY));

    /* set session auth for key */
    dev->session[0].auth = parent->auth;

    XMEMSET(&createIn, 0, sizeof(createIn));
    createIn.parentHandle = parent->hndl;
    if (auth) {
        createIn.inSensitive.sensitive.userAuth.size = authSz;
        XMEMCPY(createIn.inSensitive.sensitive.userAuth.buffer, auth,
            createIn.inSensitive.sensitive.userAuth.size);
    }
    XMEMCPY(&createIn.inPublic.publicArea, publicTemplate, sizeof(TPMT_PUBLIC));

#if 0
    /* Optional creation nonce */
    createIn.outsideInfo.size = createNoneSz;
    XMEMCPY(createIn.outsideInfo.buffer, createNonce, createIn.outsideInfo.size);
#endif

    rc = TPM2_Create(&createIn, &createOut);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_Create key failed %d: %s\n", rc, wolfTPM2_GetRCString(rc));
    #endif
        return rc;
    }

#ifdef DEBUG_WOLFTPM
    printf("TPM2_Create key: pub %d, priv %d\n", createOut.outPublic.size,
        createOut.outPrivate.size);
#endif
    key->pub = createOut.outPublic;

    /* Load new key */
    XMEMSET(&loadIn, 0, sizeof(loadIn));
    loadIn.parentHandle = parent->hndl;
    loadIn.inPrivate = createOut.outPrivate;
    loadIn.inPublic = key->pub;
    rc = TPM2_Load(&loadIn, &loadOut);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_Load key failed %d: %s\n", rc, wolfTPM2_GetRCString(rc));
    #endif
        return rc;
    }
    key->handle.dev  = dev;
    key->handle.hndl = loadOut.objectHandle;
    key->handle.auth = createIn.inSensitive.sensitive.userAuth;

#ifdef DEBUG_WOLFTPM
    printf("TPM2_Load Key Handle 0x%x\n", (word32)key->handle.hndl);
#endif

    return rc;
}

int wolfTPM2_LoadPublicKey(WOLFTPM2_DEV* dev, WOLFTPM2_KEY* key,
    const TPM2B_PUBLIC* pub)
{
    int rc;
    LoadExternal_In  loadExtIn;
    LoadExternal_Out loadExtOut;

    if (dev == NULL || key == NULL || pub == NULL)
        return BAD_FUNC_ARG;

    /* Loading public key */
    XMEMSET(&loadExtIn, 0, sizeof(loadExtIn));
    loadExtIn.inPublic = *pub;
    loadExtIn.hierarchy = TPM_RH_NULL;
    rc = TPM2_LoadExternal(&loadExtIn, &loadExtOut);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_LoadExternal: failed %d: %s\n", rc,
            wolfTPM2_GetRCString(rc));
    #endif
        return rc;
    }
    key->handle.dev = dev;
    key->handle.hndl = loadExtOut.objectHandle;
    key->pub = loadExtIn.inPublic;

#ifdef DEBUG_WOLFTPM
    printf("TPM2_LoadExternal: 0x%x\n", (word32)loadExtOut.objectHandle);
#endif

    return rc;
}

int wolfTPM2_ComputeName(const TPM2B_PUBLIC* pub, TPM2B_NAME* out)
{
    int rc;
    TPMI_ALG_HASH nameAlg;
#ifndef WOLFTPM2_NO_WOLFCRYPT
    TPM2_Packet packet;
    TPM2B_DATA data;
    wc_HashAlg hash;
    enum wc_HashType hashType;
    int hashSz;
#endif

    if (pub == NULL || out == NULL)
        return BAD_FUNC_ARG;

    nameAlg = pub->publicArea.nameAlg;
    if (nameAlg == TPM_ALG_NULL)
        return TPM_RC_SUCCESS;

#ifndef WOLFTPM2_NO_WOLFCRYPT
    /* Encode public into buffer */
    XMEMSET(&packet, 0, sizeof(packet));
    packet.buf = data.buffer;
    packet.size = sizeof(data.buffer);
    TPM2_Packet_AppendPublic(&packet, (TPM2B_PUBLIC*)pub);
    data.size = packet.pos;

    /* Hash data - first two bytes are TPM_ALG_ID */
    rc = TPM2_GetHashType(nameAlg);
    hashType = (enum wc_HashType)rc;
    rc = wc_HashGetDigestSize(hashType);
    if (rc < 0)
        return rc;
    hashSz = rc;

    /* Encode hash algorithm in first 2 bytes */
    nameAlg = TPM2_Packet_SwapU16(nameAlg);
    XMEMCPY(&out->name[0], &nameAlg, sizeof(UINT16));

    /* Hash of data (name) goes into remainder */
    rc = wc_HashInit(&hash, hashType);
    if (rc == 0) {
        rc = wc_HashUpdate(&hash, hashType, data.buffer, data.size);
        if (rc == 0)
            rc = wc_HashFinal(&hash, hashType, &out->name[sizeof(UINT16)]);

        wc_HashFree(&hash, hashType);
    }

    /* compute final size */
    out->size = hashSz + (int)sizeof(UINT16);

#else
    (void)out;
    rc = NOT_COMPILED_IN;
#endif
    return rc;
}

/* Convert TPM2B_SENSITIVE to TPM2B_PRIVATE */
int wolfTPM2_SensitiveToPrivate(TPM2B_SENSITIVE* sens, TPM2B_PRIVATE* priv,
    TPMI_ALG_HASH nameAlg, TPM2B_NAME* name, const WOLFTPM2_KEY* parentKey,
    TPMT_SYM_DEF_OBJECT* sym, TPM2B_ENCRYPTED_SECRET* symSeed)
{
    int innerWrap  = 0;
    int outerWrap = 0;
    TPMI_ALG_HASH innerAlg, outerAlg;
    TPM2_Packet packet;
    int pos = 0;
    int digestSz;

    if (sens == NULL || priv == NULL)
        return BAD_FUNC_ARG;

    digestSz = TPM2_GetHashDigestSize(nameAlg);

    if (sym && sym->algorithm != TPM_ALG_NULL) {
        innerWrap = 1;

        innerAlg = nameAlg;
        pos += sizeof(word16) + digestSz;
    }

    if (symSeed && symSeed->size > 0 && parentKey) {
        outerWrap = 1;

        outerAlg = parentKey->pub.publicArea.nameAlg;
        pos += sizeof(word16) + TPM2_GetHashDigestSize(outerAlg);
    }

    /* Encode sensitive into private buffer */
    XMEMSET(&packet, 0, sizeof(packet));
    packet.buf = &priv->buffer[pos];
    packet.size = sizeof(priv->buffer) - pos;
    TPM2_Packet_AppendSensitive(&packet, sens);
    priv->size = packet.pos;

    /* TODO: Add support for innerWrap */
    if (innerWrap) {
        (void)innerAlg;
    }
    /* TODO: Add support for outerWrap */
    if (outerWrap) {
        (void)outerAlg;
    }

    (void)name;
    (void)innerWrap;
    (void)outerWrap;

    return 0;
}

int wolfTPM2_LoadPrivateKey(WOLFTPM2_DEV* dev, const WOLFTPM2_KEY* parentKey,
    WOLFTPM2_KEY* key, const TPM2B_PUBLIC* pub, TPM2B_SENSITIVE* sens)
{
    int rc;
    Import_In  importIn;
    Import_Out importOut;
    Load_In  loadIn;
    Load_Out loadOut;
    TPM2B_NAME name;
    TPM_HANDLE parentHandle;

    if (dev == NULL || key == NULL || pub == NULL ||
            sens == NULL) {
        return BAD_FUNC_ARG;
    }

    /* set session auth for key */
    if (parentKey != NULL) {
        dev->session[0].auth = parentKey->handle.auth;
        dev->session[0].symmetric =
            parentKey->pub.publicArea.parameters.rsaDetail.symmetric;
        parentHandle = parentKey->handle.hndl;
    }
    else {
        /* clear auth */
        XMEMSET(&dev->session[0].auth, 0, sizeof(dev->session[0].auth));
        parentHandle = TPM_RH_OWNER;
    }

    /* Import private key */
    XMEMSET(&importIn, 0, sizeof(importIn));
    importIn.parentHandle = parentHandle;
    importIn.objectPublic = *pub;
    importIn.symmetricAlg.algorithm = TPM_ALG_NULL;
    rc = wolfTPM2_ComputeName(pub, &name);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("wolfTPM2_ComputeName: failed %d: %s\n", rc,
            wolfTPM2_GetRCString(rc));
    #endif
        return rc;
    }
    rc = wolfTPM2_SensitiveToPrivate(sens, &importIn.duplicate,
        pub->publicArea.nameAlg, &name, parentKey, &importIn.symmetricAlg,
        &importIn.inSymSeed);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("wolfTPM2_SensitiveToPrivate: failed %d: %s\n", rc,
            wolfTPM2_GetRCString(rc));
    #endif
        return rc;
    }
    rc = TPM2_Import(&importIn, &importOut);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_Import: failed %d: %s\n", rc,
            wolfTPM2_GetRCString(rc));
    #endif
        return rc;
    }

    key->handle.dev = dev;
    key->pub = importIn.objectPublic;

    /* Load key to get handle */
    XMEMSET(&loadIn, 0, sizeof(loadIn));
    loadIn.parentHandle = parentKey->handle.hndl;
    loadIn.inPrivate = importOut.outPrivate;
    loadIn.inPublic = key->pub;
    rc = TPM2_Load(&loadIn, &loadOut);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_Load key failed %d: %s\n", rc, wolfTPM2_GetRCString(rc));
    #endif
        return rc;
    }
    key->handle.hndl = loadOut.objectHandle;
    key->handle.auth = sens->sensitiveArea.authValue;

#ifdef DEBUG_WOLFTPM
    printf("TPM2_Load Key Handle 0x%x\n", (word32)key->handle.hndl);
#endif

    return rc;
}

int wolfTPM2_LoadRsaPublicKey(WOLFTPM2_DEV* dev, WOLFTPM2_KEY* key,
    const byte* rsaPub, word32 rsaPubSz, word32 exponent)
{
    TPM2B_PUBLIC pub;

    if (dev == NULL || key == NULL || rsaPub == NULL)
        return BAD_FUNC_ARG;
    if (rsaPubSz > sizeof(pub.publicArea.unique.rsa.buffer))
        return BUFFER_E;

    /* To support TPM hardware and firmware versions that do not allow
        small exponents */
#ifndef WOLFTPM_NO_SOFTWARE_RSA
    /* The TPM reference implementation does not support an exponent size
       smaller than 7 nor does it allow keys to be created on the TPM with a
       public exponent less than 2^16 + 1. */
    if (exponent < 7) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM based RSA with exponent %u not allowed! Using soft RSA\n",
            exponent);
    #endif
        return TPM_RC_KEY;
    }
#endif

    XMEMSET(&pub, 0, sizeof(pub));
    pub.publicArea.type = TPM_ALG_RSA;
    pub.publicArea.nameAlg = TPM_ALG_NULL;
    pub.publicArea.objectAttributes = TPMA_OBJECT_decrypt;
    pub.publicArea.parameters.rsaDetail.symmetric.algorithm = TPM_ALG_NULL;
    pub.publicArea.parameters.rsaDetail.keyBits = rsaPubSz * 8;
    pub.publicArea.parameters.rsaDetail.exponent = exponent;
    pub.publicArea.parameters.rsaDetail.scheme.scheme = TPM_ALG_NULL;
    pub.publicArea.unique.rsa.size = rsaPubSz;
    XMEMCPY(pub.publicArea.unique.rsa.buffer, rsaPub, rsaPubSz);

    return wolfTPM2_LoadPublicKey(dev, key, &pub);
}

int wolfTPM2_LoadRsaPrivateKey(WOLFTPM2_DEV* dev, const WOLFTPM2_KEY* parentKey,
    WOLFTPM2_KEY* key, const byte* rsaPub, word32 rsaPubSz, word32 exponent,
    const byte* rsaPriv, word32 rsaPrivSz)
{
    TPM2B_PUBLIC pub;
    TPM2B_SENSITIVE sens;

    if (dev == NULL || key == NULL || rsaPub == NULL || rsaPriv == NULL)
        return BAD_FUNC_ARG;
    if (rsaPubSz > sizeof(pub.publicArea.unique.rsa.buffer))
        return BUFFER_E;
    if (rsaPrivSz > sizeof(sens.sensitiveArea.sensitive.rsa.buffer))
        return BUFFER_E;

    /* Set up public key */
    XMEMSET(&pub, 0, sizeof(pub));
    pub.publicArea.type = TPM_ALG_RSA;
    pub.publicArea.nameAlg = WOLFTPM2_WRAP_DIGEST;
    pub.publicArea.objectAttributes = TPMA_OBJECT_sign | TPMA_OBJECT_decrypt |
        TPMA_OBJECT_userWithAuth | TPMA_OBJECT_noDA;
    pub.publicArea.parameters.rsaDetail.symmetric.algorithm = TPM_ALG_NULL;
    pub.publicArea.parameters.rsaDetail.keyBits = rsaPubSz * 8;
    pub.publicArea.parameters.rsaDetail.exponent = exponent;
    pub.publicArea.parameters.rsaDetail.scheme.scheme = TPM_ALG_NULL;
    pub.publicArea.unique.rsa.size = rsaPubSz;
    XMEMCPY(pub.publicArea.unique.rsa.buffer, rsaPub, rsaPubSz);

    /* Set up private key */
    XMEMSET(&sens, 0, sizeof(sens));
    sens.sensitiveArea.sensitiveType = TPM_ALG_RSA;
    if (key->handle.auth.size > 0) {
        sens.sensitiveArea.authValue.size = key->handle.auth.size;
        XMEMCPY(sens.sensitiveArea.authValue.buffer, key->handle.auth.buffer,
            key->handle.auth.size);
    }
    sens.sensitiveArea.sensitive.rsa.size = rsaPrivSz;
    XMEMCPY(sens.sensitiveArea.sensitive.rsa.buffer, rsaPriv, rsaPrivSz);

    return wolfTPM2_LoadPrivateKey(dev, parentKey, key, &pub, &sens);
}

int wolfTPM2_LoadEccPublicKey(WOLFTPM2_DEV* dev, WOLFTPM2_KEY* key, int curveId,
    const byte* eccPubX, word32 eccPubXSz, const byte* eccPubY, word32 eccPubYSz)
{
    TPM2B_PUBLIC pub;

    if (dev == NULL || key == NULL || eccPubX == NULL || eccPubY == NULL)
        return BAD_FUNC_ARG;
    if (eccPubXSz > sizeof(pub.publicArea.unique.ecc.x.buffer))
        return BUFFER_E;
    if (eccPubYSz > sizeof(pub.publicArea.unique.ecc.y.buffer))
        return BUFFER_E;

    XMEMSET(&pub, 0, sizeof(pub));
    pub.publicArea.type = TPM_ALG_ECC;
    pub.publicArea.nameAlg = TPM_ALG_NULL;
    pub.publicArea.objectAttributes = TPMA_OBJECT_sign | TPMA_OBJECT_noDA;
    pub.publicArea.parameters.eccDetail.symmetric.algorithm = TPM_ALG_NULL;
    pub.publicArea.parameters.eccDetail.scheme.scheme = TPM_ALG_ECDSA;
    pub.publicArea.parameters.eccDetail.scheme.details.ecdsa.hashAlg =
        WOLFTPM2_WRAP_DIGEST;
    pub.publicArea.parameters.eccDetail.curveID = curveId;
    pub.publicArea.parameters.eccDetail.kdf.scheme = TPM_ALG_NULL;
    pub.publicArea.unique.ecc.x.size = eccPubXSz;
    XMEMCPY(pub.publicArea.unique.ecc.x.buffer, eccPubX, eccPubXSz);
    pub.publicArea.unique.ecc.y.size = eccPubYSz;
    XMEMCPY(pub.publicArea.unique.ecc.y.buffer, eccPubY, eccPubYSz);

    return wolfTPM2_LoadPublicKey(dev, key, &pub);
}

int wolfTPM2_LoadEccPrivateKey(WOLFTPM2_DEV* dev, const WOLFTPM2_KEY* parentKey,
    WOLFTPM2_KEY* key, int curveId,
    const byte* eccPubX, word32 eccPubXSz,
    const byte* eccPubY, word32 eccPubYSz,
    const byte* eccPriv, word32 eccPrivSz)
{
    TPM2B_PUBLIC pub;
    TPM2B_SENSITIVE sens;

    if (dev == NULL || key == NULL || eccPubX == NULL || eccPubY == NULL ||
        eccPriv == NULL) {
        return BAD_FUNC_ARG;
    }
    if (eccPubXSz > sizeof(pub.publicArea.unique.ecc.x.buffer))
        return BUFFER_E;
    if (eccPubYSz > sizeof(pub.publicArea.unique.ecc.y.buffer))
        return BUFFER_E;
    if (eccPrivSz > sizeof(sens.sensitiveArea.sensitive.rsa.buffer))
        return BUFFER_E;

    /* Set up public key */
    XMEMSET(&pub, 0, sizeof(pub));
    pub.publicArea.type = TPM_ALG_ECC;
    pub.publicArea.nameAlg = WOLFTPM2_WRAP_DIGEST;
    pub.publicArea.objectAttributes = TPMA_OBJECT_sign | TPMA_OBJECT_decrypt |
        TPMA_OBJECT_userWithAuth | TPMA_OBJECT_noDA;
    pub.publicArea.parameters.eccDetail.symmetric.algorithm = TPM_ALG_NULL;
    pub.publicArea.parameters.eccDetail.scheme.scheme = TPM_ALG_NULL;
    pub.publicArea.parameters.eccDetail.scheme.details.ecdsa.hashAlg =
        WOLFTPM2_WRAP_DIGEST;
    pub.publicArea.parameters.eccDetail.curveID = curveId;
    pub.publicArea.parameters.eccDetail.kdf.scheme = TPM_ALG_NULL;
    pub.publicArea.unique.ecc.x.size = eccPubXSz;
    XMEMCPY(pub.publicArea.unique.ecc.x.buffer, eccPubX, eccPubXSz);
    pub.publicArea.unique.ecc.y.size = eccPubYSz;
    XMEMCPY(pub.publicArea.unique.ecc.y.buffer, eccPubY, eccPubYSz);

    /* Set up private key */
    XMEMSET(&sens, 0, sizeof(sens));
    sens.sensitiveArea.sensitiveType = TPM_ALG_ECC;
    if (key->handle.auth.size > 0) {
        sens.sensitiveArea.authValue.size = key->handle.auth.size;
        XMEMCPY(sens.sensitiveArea.authValue.buffer, key->handle.auth.buffer,
            key->handle.auth.size);
    }
    sens.sensitiveArea.sensitive.ecc.size = eccPrivSz;
    XMEMCPY(sens.sensitiveArea.sensitive.ecc.buffer, eccPriv, eccPrivSz);

    return wolfTPM2_LoadPrivateKey(dev, parentKey, key, &pub, &sens);
}

int wolfTPM2_ReadPublicKey(WOLFTPM2_DEV* dev, WOLFTPM2_KEY* key,
    const TPM_HANDLE handle)
{
    int rc;
    ReadPublic_In  readPubIn;
    ReadPublic_Out readPubOut;

    if (dev == NULL || key == NULL)
        return BAD_FUNC_ARG;

    /* Read public key */
    XMEMSET(&readPubIn, 0, sizeof(readPubIn));
    readPubIn.objectHandle = handle;
    rc = TPM2_ReadPublic(&readPubIn, &readPubOut);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_ReadPublic failed %d: %s\n", rc, wolfTPM2_GetRCString(rc));
    #endif
        return rc;
    }

    key->handle.dev = dev;
    key->handle.hndl = readPubIn.objectHandle;
    key->pub = readPubOut.outPublic;

#ifdef DEBUG_WOLFTPM
    printf("TPM2_ReadPublic Handle 0x%x: pub %d, name %d, qualifiedName %d\n",
		(word32)readPubIn.objectHandle,
        readPubOut.outPublic.size, readPubOut.name.size,
        readPubOut.qualifiedName.size);
#endif

    return rc;
}

#ifndef WOLFTPM2_NO_WOLFCRYPT
#ifndef NO_RSA
int wolfTPM2_RsaKey_TpmToWolf(WOLFTPM2_DEV* dev, WOLFTPM2_KEY* tpmKey,
    RsaKey* wolfKey)
{
    int rc;
    word32  exponent;
    byte    e[sizeof(exponent)];
    byte    n[WOLFTPM2_WRAP_RSA_KEY_BITS / 8];
    word32  eSz = sizeof(e);
    word32  nSz = sizeof(n);

    if (dev == NULL || tpmKey == NULL || wolfKey == NULL)
        return BAD_FUNC_ARG;

    XMEMSET(e, 0, sizeof(e));
    XMEMSET(n, 0, sizeof(n));

    /* load exponent */
    exponent = tpmKey->pub.publicArea.parameters.rsaDetail.exponent;
    if (exponent == 0)
        exponent = RSA_DEFAULT_PUBLIC_EXPONENT;
    e[3] = (exponent >> 24) & 0xFF;
    e[2] = (exponent >> 16) & 0xFF;
    e[1] = (exponent >> 8)  & 0xFF;
    e[0] =  exponent        & 0xFF;
    eSz = e[3] ? 4 : e[2] ? 3 : e[1] ? 2 : e[0] ? 1 : 0; /* calc size */

    /* load public key */
    nSz = tpmKey->pub.publicArea.unique.rsa.size;
    XMEMCPY(n, tpmKey->pub.publicArea.unique.rsa.buffer, nSz);

    /* load public key portion into wolf RsaKey */
    rc = wc_RsaPublicKeyDecodeRaw(n, nSz, e, eSz, wolfKey);

    return rc;
}

static word32 wolfTPM2_RsaKey_Exponent(byte* e, word32 eSz)
{
    word32 exponent = 0, i;
    for (i=0; i<eSz && i<sizeof(word32); i++) {
        exponent |= ((word32)e[i]) << (i*8);
    }
    return exponent;
}

int wolfTPM2_RsaKey_WolfToTpm_ex(WOLFTPM2_DEV* dev, const WOLFTPM2_KEY* parentKey,
    RsaKey* wolfKey, WOLFTPM2_KEY* tpmKey)
{
    int rc;
    word32  exponent;
    byte    e[sizeof(exponent)];
    byte    n[WOLFTPM2_WRAP_RSA_KEY_BITS / 8];
    word32  eSz = sizeof(e);
    word32  nSz = sizeof(n);

    if (dev == NULL || tpmKey == NULL || wolfKey == NULL)
        return BAD_FUNC_ARG;

    XMEMSET(e, 0, sizeof(e));
    XMEMSET(n, 0, sizeof(n));

    if (parentKey && wolfKey->type == RSA_PRIVATE) {
        byte    d[WOLFTPM2_WRAP_RSA_KEY_BITS / 8];
        byte    p[WOLFTPM2_WRAP_RSA_KEY_BITS / 8];
        byte    q[WOLFTPM2_WRAP_RSA_KEY_BITS / 8];
        word32  dSz = sizeof(d);
        word32  pSz = sizeof(p);
        word32  qSz = sizeof(q);

        XMEMSET(d, 0, sizeof(d));
        XMEMSET(p, 0, sizeof(p));
        XMEMSET(q, 0, sizeof(q));

        /* export the raw private and public RSA as unsigned binary */
        rc = wc_RsaExportKey(wolfKey, e, &eSz, n, &nSz,
            d, &dSz, p, &pSz, q, &qSz);
        if (rc == 0) {
            exponent = wolfTPM2_RsaKey_Exponent(e, eSz);
            rc = wolfTPM2_LoadRsaPrivateKey(dev, parentKey, tpmKey, n, nSz,
                exponent, q, qSz);
        }

        /* not used */
        (void)p;
    }
    else {
        /* export the raw public RSA portion */
        rc = wc_RsaFlattenPublicKey(wolfKey, e, &eSz, n, &nSz);
        if (rc == 0) {
            exponent = wolfTPM2_RsaKey_Exponent(e, eSz);
            rc = wolfTPM2_LoadRsaPublicKey(dev, tpmKey, n, nSz, exponent);
        }
    }

    return rc;
}
int wolfTPM2_RsaKey_WolfToTpm(WOLFTPM2_DEV* dev, RsaKey* wolfKey,
    WOLFTPM2_KEY* tpmKey)
{
    return wolfTPM2_RsaKey_WolfToTpm_ex(dev, NULL, wolfKey, tpmKey);
}
#endif /* !NO_RSA */

#ifdef HAVE_ECC
int wolfTPM2_EccKey_TpmToWolf(WOLFTPM2_DEV* dev, WOLFTPM2_KEY* tpmKey,
    ecc_key* wolfKey)
{
    int rc, curve_id;
    byte    qx[WOLFTPM2_WRAP_ECC_KEY_BITS / 8];
    byte    qy[WOLFTPM2_WRAP_ECC_KEY_BITS / 8];
    word32  qxSz = sizeof(qx);
    word32  qySz = sizeof(qy);

    if (dev == NULL || tpmKey == NULL || wolfKey == NULL)
        return BAD_FUNC_ARG;

    XMEMSET(qx, 0, sizeof(qx));
    XMEMSET(qy, 0, sizeof(qy));

    /* load curve type */
    curve_id = tpmKey->pub.publicArea.parameters.eccDetail.curveID;
    rc = TPM2_GetWolfCurve(curve_id);
    if (rc < 0)
        return rc;
    curve_id = rc;

    /* load public key */
    qxSz = tpmKey->pub.publicArea.unique.ecc.x.size;
    XMEMCPY(qx, tpmKey->pub.publicArea.unique.ecc.x.buffer, qxSz);
    qySz = tpmKey->pub.publicArea.unique.ecc.y.size;
    XMEMCPY(qy, tpmKey->pub.publicArea.unique.ecc.y.buffer, qySz);

    /* load public key portion into wolf ecc_key */
    rc = wc_ecc_import_unsigned(wolfKey, qx, qy, NULL, curve_id);

    return rc;
}

int wolfTPM2_EccKey_WolfToTpm_ex(WOLFTPM2_DEV* dev, WOLFTPM2_KEY* parentKey,
    ecc_key* wolfKey, WOLFTPM2_KEY* tpmKey)
{
    int rc, curve_id = 0;
    byte    qx[WOLFTPM2_WRAP_ECC_KEY_BITS / 8];
    byte    qy[WOLFTPM2_WRAP_ECC_KEY_BITS / 8];
    word32  qxSz = sizeof(qx);
    word32  qySz = sizeof(qy);

    if (dev == NULL || tpmKey == NULL || wolfKey == NULL)
        return BAD_FUNC_ARG;

    XMEMSET(qx, 0, sizeof(qx));
    XMEMSET(qy, 0, sizeof(qy));

    if (wolfKey->dp)
        curve_id = wolfKey->dp->id;

    rc = TPM2_GetTpmCurve(curve_id);
    if (rc < 0)
        return rc;
    curve_id = rc;

    if (parentKey && wolfKey->type == ECC_PRIVATEKEY) {
        byte    d[WOLFTPM2_WRAP_ECC_KEY_BITS / 8];
        word32  dSz = sizeof(d);

        XMEMSET(d, 0, sizeof(d));

        /* export the raw private/public ECC portions */
        rc = wc_ecc_export_private_raw(wolfKey, qx, &qxSz, qy, &qySz, d, &dSz);
        if (rc == 0) {
            rc = wolfTPM2_LoadEccPrivateKey(dev, parentKey, tpmKey, curve_id,
                qx, qxSz, qy, qySz, d, dSz);
        }
    }
    else {
        /* export the raw public ECC portion */
        rc = wc_ecc_export_public_raw(wolfKey, qx, &qxSz, qy, &qySz);
        if (rc == 0) {
            rc = wolfTPM2_LoadEccPublicKey(dev, tpmKey, curve_id, qx, qxSz, qy, qySz);
        }
    }

    return rc;
}
int wolfTPM2_EccKey_WolfToTpm(WOLFTPM2_DEV* dev, ecc_key* wolfKey,
    WOLFTPM2_KEY* tpmKey)
{
    return wolfTPM2_EccKey_WolfToTpm_ex(dev, NULL, wolfKey, tpmKey);
}

int wolfTPM2_EccKey_WolfToPubPoint(WOLFTPM2_DEV* dev, ecc_key* wolfKey,
    TPM2B_ECC_POINT* pubPoint)
{
    int rc;
    word32 xSz, ySz;

    if (dev == NULL || wolfKey == NULL || pubPoint == NULL)
        return BAD_FUNC_ARG;

    xSz = sizeof(pubPoint->point.x.buffer);;
    ySz = sizeof(pubPoint->point.y.buffer);;

    /* load wolf ECC public key into TPM2B_ECC_POINT */
    rc = wc_ecc_export_public_raw(wolfKey,
        pubPoint->point.x.buffer, &xSz,
        pubPoint->point.y.buffer, &ySz);
    if (rc == 0) {
        pubPoint->point.x.size = xSz;
        pubPoint->point.y.size = ySz;
    }

    return rc;
}
#endif /* HAVE_ECC */
#endif /* !WOLFTPM2_NO_WOLFCRYPT */


/* primaryHandle must be owner or platform hierarchy */
/* Owner    Persistent Handle Range: 0x81000000 to 0x817FFFFF */
/* Platform Persistent Handle Range: 0x81800000 to 0x81FFFFFF */
int wolfTPM2_NVStoreKey(WOLFTPM2_DEV* dev, TPM_HANDLE primaryHandle,
    WOLFTPM2_KEY* key, TPM_HANDLE persistentHandle)
{
    int rc;
    EvictControl_In in;

    if (dev == NULL || key == NULL ||
        (primaryHandle != TPM_RH_OWNER && primaryHandle != TPM_RH_PLATFORM) ||
        persistentHandle < PERSISTENT_FIRST ||
        persistentHandle > PERSISTENT_LAST) {
        return BAD_FUNC_ARG;
    }

    /* if key is already persistent then just return success */
    if (key->handle.hndl == persistentHandle)
        return TPM_RC_SUCCESS;

    /* clear auth */
    XMEMSET(&dev->session[0].auth, 0, sizeof(dev->session[0].auth));

    /* Move key into NV to persist */
    XMEMSET(&in, 0, sizeof(in));
    in.auth = primaryHandle;
    in.objectHandle = key->handle.hndl;
    in.persistentHandle = persistentHandle;

    rc = TPM2_EvictControl(&in);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_EvictControl failed %d: %s\n", rc,
            wolfTPM2_GetRCString(rc));
    #endif
        return rc;
    }

#ifdef DEBUG_WOLFTPM
    printf("TPM2_EvictControl Auth 0x%x, Key 0x%x, Persistent 0x%x\n",
    	(word32)in.auth, (word32)in.objectHandle, (word32)in.persistentHandle);
#endif

    /* unload transient handle */
    wolfTPM2_UnloadHandle(dev, &key->handle);

    /* replace handle with persistent one */
    key->handle.hndl = persistentHandle;

    return rc;
}

int wolfTPM2_NVDeleteKey(WOLFTPM2_DEV* dev, TPM_HANDLE primaryHandle,
    WOLFTPM2_KEY* key)
{
    int rc;
    EvictControl_In in;

    if (dev == NULL || key == NULL || primaryHandle == 0) {
        return BAD_FUNC_ARG;
    }

    /* if key is not persistent then just return success */
    if (key->handle.hndl < PERSISTENT_FIRST ||
            key->handle.hndl > PERSISTENT_LAST)
        return TPM_RC_SUCCESS;

    /* clear auth */
    XMEMSET(&dev->session[0].auth, 0, sizeof(dev->session[0].auth));

    /* remove key from NV */
    XMEMSET(&in, 0, sizeof(in));
    in.auth = primaryHandle;
    in.objectHandle = key->handle.hndl;
    in.persistentHandle = key->handle.hndl;

    rc = TPM2_EvictControl(&in);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_EvictControl failed %d: %s\n", rc,
            wolfTPM2_GetRCString(rc));
    #endif
        return rc;
    }

#ifdef DEBUG_WOLFTPM
    printf("TPM2_EvictControl Auth 0x%x, Key 0x%x, Persistent 0x%x\n",
    	(word32)in.auth, (word32)in.objectHandle, (word32)in.persistentHandle);
#endif

    /* indicate no handle */
    key->handle.hndl = TPM_RH_NULL;

    return rc;
}


int wolfTPM2_SignHash(WOLFTPM2_DEV* dev, WOLFTPM2_KEY* key,
    const byte* digest, int digestSz, byte* sig, int* sigSz)
{
    int rc;
    Sign_In  signIn;
    Sign_Out signOut;
    int curveSize;

    if (dev == NULL || key == NULL || digest == NULL || sig == NULL ||
                                                            sigSz == NULL) {
        return BAD_FUNC_ARG;
    }

    /* get curve size */
    curveSize = wolfTPM2_GetCurveSize(
        key->pub.publicArea.parameters.eccDetail.curveID);
    if (curveSize <= 0 || *sigSz < (curveSize * 2)) {
        return BAD_FUNC_ARG;
    }

    /* set session auth for key */
    dev->session[0].auth = key->handle.auth;
    dev->session[0].symmetric =
        key->pub.publicArea.parameters.eccDetail.symmetric;

    /* Sign with ECC key */
    XMEMSET(&signIn, 0, sizeof(signIn));
    signIn.keyHandle = key->handle.hndl;
    signIn.digest.size = digestSz;
    XMEMCPY(signIn.digest.buffer, digest, signIn.digest.size);
    signIn.inScheme.scheme = TPM_ALG_ECDSA;
    signIn.inScheme.details.ecdsa.hashAlg = WOLFTPM2_WRAP_DIGEST;
    signIn.validation.tag = TPM_ST_HASHCHECK;
    signIn.validation.hierarchy = TPM_RH_NULL;
    rc = TPM2_Sign(&signIn, &signOut);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_Sign failed %d: %s\n", rc, wolfTPM2_GetRCString(rc));
    #endif
        return rc;
    }

    /* Assemble R and S into signature (R then S) */
    *sigSz = signOut.signature.signature.ecdsa.signatureR.size +
             signOut.signature.signature.ecdsa.signatureS.size;
    XMEMCPY(sig, signOut.signature.signature.ecdsa.signatureR.buffer,
        signOut.signature.signature.ecdsa.signatureR.size);
    XMEMCPY(sig + signOut.signature.signature.ecdsa.signatureR.size,
        signOut.signature.signature.ecdsa.signatureS.buffer,
        signOut.signature.signature.ecdsa.signatureS.size);

#ifdef DEBUG_WOLFTPM
    printf("TPM2_Sign: ECC R %d, S %d\n",
        signOut.signature.signature.ecdsa.signatureR.size,
        signOut.signature.signature.ecdsa.signatureS.size);
#endif

    return rc;
}

static TPMI_ALG_HASH wolfTPM2_GetHashType(int digestSz)
{
    switch (digestSz) {
        case TPM_SHA_DIGEST_SIZE:
            return TPM_ALG_SHA1;
        case TPM_SHA256_DIGEST_SIZE:
            return TPM_ALG_SHA256;
        case TPM_SHA384_DIGEST_SIZE:
            return TPM_ALG_SHA384;
        case TPM_SHA512_DIGEST_SIZE:
            return TPM_ALG_SHA512;
        default:
            break;
    }
    return TPM_ALG_NULL;
}

int wolfTPM2_VerifyHash(WOLFTPM2_DEV* dev, WOLFTPM2_KEY* key,
    const byte* sig, int sigSz, const byte* digest, int digestSz)
{
    int rc;
    VerifySignature_In  verifySigIn;
    VerifySignature_Out verifySigOut;
    int curveSize;

    if (dev == NULL || key == NULL || digest == NULL || sig == NULL) {
        return BAD_FUNC_ARG;
    }

    /* get curve size */
    curveSize = wolfTPM2_GetCurveSize(
        key->pub.publicArea.parameters.eccDetail.curveID);
    if (curveSize <= 0 || sigSz < (curveSize * 2)) {
        return BAD_FUNC_ARG;
    }

    /* set session auth for key */
    dev->session[0].auth = key->handle.auth;
    dev->session[0].symmetric =
        key->pub.publicArea.parameters.eccDetail.symmetric;

    XMEMSET(&verifySigIn, 0, sizeof(verifySigIn));
    verifySigIn.keyHandle = key->handle.hndl;
    verifySigIn.digest.size = digestSz;
    XMEMCPY(verifySigIn.digest.buffer, digest, digestSz);
    verifySigIn.signature.sigAlgo =
        key->pub.publicArea.parameters.eccDetail.scheme.scheme;
    verifySigIn.signature.signature.ecdsa.hash = wolfTPM2_GetHashType(digestSz);
    if (verifySigIn.signature.signature.ecdsa.hash == TPM_ALG_NULL)
        verifySigIn.signature.signature.ecdsa.hash = WOLFTPM2_WRAP_DIGEST;

    /* Signature is R then S */
    verifySigIn.signature.signature.ecdsa.signatureR.size = curveSize;
    XMEMCPY(verifySigIn.signature.signature.ecdsa.signatureR.buffer,
        sig, curveSize);
    verifySigIn.signature.signature.ecdsa.signatureS.size = curveSize;
    XMEMCPY(verifySigIn.signature.signature.ecdsa.signatureS.buffer,
        sig + curveSize, curveSize);

    rc = TPM2_VerifySignature(&verifySigIn, &verifySigOut);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_VerifySignature failed %d: %s\n", rc,
            wolfTPM2_GetRCString(rc));
    #endif
        return rc;
    }
#ifdef DEBUG_WOLFTPM
    printf("TPM2_VerifySignature: Tag %d\n", verifySigOut.validation.tag);
#endif

    return rc;
}

/* Generate ECC key-pair with NULL hierarchy and load (populates handle) */
int wolfTPM2_ECDHGenKey(WOLFTPM2_DEV* dev, WOLFTPM2_KEY* ecdhKey, int curve_id,
    const byte* auth, int authSz)
{
    int rc;
    TPMT_PUBLIC publicTemplate;
    WOLFTPM2_HANDLE nullParent;

    if (dev == NULL || ecdhKey == NULL) {
        return BAD_FUNC_ARG;
    }

    /* clear auth */
    XMEMSET(&dev->session[0].auth, 0, sizeof(dev->session[0].auth));

    XMEMSET(&nullParent, 0, sizeof(nullParent));
    nullParent.hndl = TPM_RH_NULL;

    /* Create and load ECC key for DH */
    rc = wolfTPM2_GetKeyTemplate_ECC(&publicTemplate,
        TPMA_OBJECT_sensitiveDataOrigin | TPMA_OBJECT_userWithAuth |
        TPMA_OBJECT_decrypt | TPMA_OBJECT_noDA,
        curve_id, TPM_ALG_ECDH);
    if (rc == 0) {
        rc = wolfTPM2_CreatePrimaryKey(dev, ecdhKey, TPM_RH_NULL,
            &publicTemplate, auth, authSz);
    }

    return rc;
}

/* Generate ephemeral key and compute Z (shared secret) */
/* One shot API using private key handle to generate key-pair and return
    pub-point and shared secret */
int wolfTPM2_ECDHGen(WOLFTPM2_DEV* dev, WOLFTPM2_KEY* privKey,
    TPM2B_ECC_POINT* pubPoint, byte* out, int* outSz)
{
    int rc;
    ECDH_KeyGen_In  ecdhIn;
    ECDH_KeyGen_Out ecdhOut;
    int curveSize;

    if (dev == NULL || privKey == NULL || pubPoint == NULL || out == NULL ||
                                                                outSz == NULL) {
        return BAD_FUNC_ARG;
    }

    /* get curve size to verify output is large enough */
    curveSize = wolfTPM2_GetCurveSize(
        privKey->pub.publicArea.parameters.eccDetail.curveID);
    if (curveSize <= 0 || *outSz < curveSize) {
        return BAD_FUNC_ARG;
    }

    /* set session auth for key */
    dev->session[0].auth = privKey->handle.auth;
    dev->session[0].symmetric =
        privKey->pub.publicArea.parameters.eccDetail.symmetric;

    XMEMSET(&ecdhIn, 0, sizeof(ecdhIn));
    ecdhIn.keyHandle = privKey->handle.hndl;
    rc = TPM2_ECDH_KeyGen(&ecdhIn, &ecdhOut);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_ECDH_KeyGen failed %d: %s\n", rc,
            wolfTPM2_GetRCString(rc));
    #endif
        return rc;
    }

    *pubPoint = ecdhOut.pubPoint;
    *outSz = ecdhOut.zPoint.point.x.size;
    XMEMCPY(out, ecdhOut.zPoint.point.x.buffer, ecdhOut.zPoint.point.x.size);

#ifdef DEBUG_WOLFTPM
    printf("TPM2_ECDH_KeyGen: zPt %d, pubPt %d\n",
        ecdhOut.zPoint.size,
        ecdhOut.pubPoint.size);
#endif

    return rc;
}

/* Compute Z (shared secret) using pubPoint and loaded private ECC key */
int wolfTPM2_ECDHGenZ(WOLFTPM2_DEV* dev, WOLFTPM2_KEY* privKey,
    const TPM2B_ECC_POINT* pubPoint, byte* out, int* outSz)
{
    int rc;
    ECDH_ZGen_In  ecdhZIn;
    ECDH_ZGen_Out ecdhZOut;
    int curveSize;

    if (dev == NULL || privKey == NULL || pubPoint == NULL || out == NULL ||
                                                                outSz == NULL) {
        return BAD_FUNC_ARG;
    }

    /* get curve size to verify output is large enough */
    curveSize = wolfTPM2_GetCurveSize(
        privKey->pub.publicArea.parameters.eccDetail.curveID);
    if (curveSize <= 0 || *outSz < curveSize) {
        return BAD_FUNC_ARG;
    }

    /* set session auth for key */
    dev->session[0].auth = privKey->handle.auth;
    dev->session[0].symmetric =
        privKey->pub.publicArea.parameters.eccDetail.symmetric;

    XMEMSET(&ecdhZIn, 0, sizeof(ecdhZIn));
    ecdhZIn.keyHandle = privKey->handle.hndl;
    ecdhZIn.inPoint = *pubPoint;
    rc = TPM2_ECDH_ZGen(&ecdhZIn, &ecdhZOut);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_ECDH_ZGen failed %d: %s\n", rc,
            wolfTPM2_GetRCString(rc));
    #endif
        return rc;
    }

    *outSz = ecdhZOut.outPoint.point.x.size;
    XMEMCPY(out, ecdhZOut.outPoint.point.x.buffer,
        ecdhZOut.outPoint.point.x.size);

#ifdef DEBUG_WOLFTPM
    printf("TPM2_ECDH_ZGen: zPt %d\n", ecdhZOut.outPoint.size);
#endif

    return rc;
}


/* Generate ephemeral ECC key and return array index (2 phase method) */
/* One time use key */
int wolfTPM2_ECDHEGenKey(WOLFTPM2_DEV* dev, WOLFTPM2_KEY* ecdhKey, int curve_id)
{
    int rc;
    EC_Ephemeral_In in;
    EC_Ephemeral_Out out;

    if (dev == NULL || ecdhKey == NULL) {
        return BAD_FUNC_ARG;
    }

    XMEMSET(&in, 0, sizeof(in));
    in.curveID = curve_id;
    rc = TPM2_EC_Ephemeral(&in, &out);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_EC_Ephemeral failed %d: %s\n", rc,
            wolfTPM2_GetRCString(rc));
    #endif
        return rc;
    }

    /* Save the point and counter (commit ID) into ecdhKey */
    ecdhKey->pub.publicArea.unique.ecc = out.Q.point;
    ecdhKey->handle.hndl = (UINT32)out.counter;

    return rc;
}

/* Compute Z (shared secret) using pubPoint and counter (2 phase method) */
/* The counter / array ID can only be used one time */
int wolfTPM2_ECDHEGenZ(WOLFTPM2_DEV* dev, WOLFTPM2_KEY* parentKey,
    WOLFTPM2_KEY* ecdhKey, const TPM2B_ECC_POINT* pubPoint,
    byte* out, int* outSz)
{
    int rc;
    ZGen_2Phase_In  inZGen2Ph;
    ZGen_2Phase_Out outZGen2Ph;
    int curveSize;

    if (dev == NULL || parentKey == NULL || ecdhKey == NULL ||
        pubPoint == NULL || out == NULL || outSz == NULL) {
        return BAD_FUNC_ARG;
    }

    /* get curve size to verify output is large enough */
    curveSize = wolfTPM2_GetCurveSize(
        parentKey->pub.publicArea.parameters.eccDetail.curveID);
    if (curveSize <= 0 || *outSz < curveSize) {
        return BAD_FUNC_ARG;
    }

    /* set session auth for key */
    dev->session[0].auth = parentKey->handle.auth;
    dev->session[0].symmetric =
        parentKey->pub.publicArea.parameters.eccDetail.symmetric;

    XMEMSET(&inZGen2Ph, 0, sizeof(inZGen2Ph));
    inZGen2Ph.keyA = ecdhKey->handle.hndl;
    ecdhKey->handle.hndl = TPM_RH_NULL;
    inZGen2Ph.inQsB = *pubPoint;
    inZGen2Ph.inQeB = *pubPoint;
    inZGen2Ph.inScheme = TPM_ALG_ECDH;
    inZGen2Ph.counter = (UINT16)ecdhKey->handle.hndl;

    rc = TPM2_ZGen_2Phase(&inZGen2Ph, &outZGen2Ph);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_ZGen_2Phase failed %d: %s\n", rc,
            wolfTPM2_GetRCString(rc));
    #endif
        return rc;
    }

    *outSz = outZGen2Ph.outZ2.point.x.size;
    XMEMCPY(out, outZGen2Ph.outZ2.point.x.buffer,
        outZGen2Ph.outZ2.point.x.size);

#ifdef DEBUG_WOLFTPM
    printf("TPM2_ZGen_2Phase: zPt %d\n", outZGen2Ph.outZ2.size);
#endif

    return rc;
}


int wolfTPM2_RsaEncrypt(WOLFTPM2_DEV* dev, WOLFTPM2_KEY* key,
    TPM_ALG_ID padScheme, const byte* msg, int msgSz, byte* out, int* outSz)
{
    int rc;
    RSA_Encrypt_In  rsaEncIn;
    RSA_Encrypt_Out rsaEncOut;

    if (dev == NULL || key == NULL || msg == NULL || out == NULL ||
                                                                outSz == NULL) {
        return BAD_FUNC_ARG;
    }

    /* set session auth for key */
    dev->session[0].auth = key->handle.auth;
    dev->session[0].symmetric =
        key->pub.publicArea.parameters.rsaDetail.symmetric;

    /* RSA Encrypt */
    XMEMSET(&rsaEncIn, 0, sizeof(rsaEncIn));
    rsaEncIn.keyHandle = key->handle.hndl;
    rsaEncIn.message.size = msgSz;
    XMEMCPY(rsaEncIn.message.buffer, msg, msgSz);
    /* TPM_ALG_NULL, TPM_ALG_OAEP, TPM_ALG_RSASSA or TPM_ALG_RSAPSS */
    rsaEncIn.inScheme.scheme = padScheme;
    rsaEncIn.inScheme.details.anySig.hashAlg = WOLFTPM2_WRAP_DIGEST;

#if 0
    /* Optional label */
    rsaEncIn.label.size = sizeof(label); /* Null term required */
    XMEMCPY(rsaEncIn.label.buffer, label, rsaEncIn.label.size);
#endif

    rc = TPM2_RSA_Encrypt(&rsaEncIn, &rsaEncOut);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_RSA_Encrypt failed %d: %s\n", rc,
            wolfTPM2_GetRCString(rc));
    #endif
        return rc;
    }

    *outSz = rsaEncOut.outData.size;
    XMEMCPY(out, rsaEncOut.outData.buffer, *outSz);

#ifdef DEBUG_WOLFTPM
    printf("TPM2_RSA_Encrypt: %d\n", rsaEncOut.outData.size);
#endif

    return rc;
}

int wolfTPM2_RsaDecrypt(WOLFTPM2_DEV* dev, WOLFTPM2_KEY* key,
    TPM_ALG_ID padScheme, const byte* in, int inSz, byte* msg, int* msgSz)
{
    int rc;
    RSA_Decrypt_In  rsaDecIn;
    RSA_Decrypt_Out rsaDecOut;

    if (dev == NULL || key == NULL || in == NULL || msg == NULL ||
                                                                msgSz == NULL) {
        return BAD_FUNC_ARG;
    }

    /* set session auth for key */
    dev->session[0].auth = key->handle.auth;
    dev->session[0].symmetric =
        key->pub.publicArea.parameters.rsaDetail.symmetric;

    /* RSA Decrypt */
    XMEMSET(&rsaDecIn, 0, sizeof(rsaDecIn));
    rsaDecIn.keyHandle = key->handle.hndl;
    rsaDecIn.cipherText.size = inSz;
    XMEMCPY(rsaDecIn.cipherText.buffer, in, inSz);
    /* TPM_ALG_NULL, TPM_ALG_OAEP, TPM_ALG_RSASSA or TPM_ALG_RSAPSS */
    rsaDecIn.inScheme.scheme = padScheme;
    rsaDecIn.inScheme.details.anySig.hashAlg = WOLFTPM2_WRAP_DIGEST;

#if 0
    /* Optional label */
    rsaDecIn.label.size = sizeof(label); /* Null term required */
    XMEMCPY(rsaDecIn.label.buffer, label, rsaEncIn.label.size);
#endif

    rc = TPM2_RSA_Decrypt(&rsaDecIn, &rsaDecOut);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_RSA_Decrypt failed %d: %s\n", rc,
            wolfTPM2_GetRCString(rc));
    #endif
        return rc;
    }

    *msgSz = rsaDecOut.message.size;
    XMEMCPY(msg, rsaDecOut.message.buffer, *msgSz);

#ifdef DEBUG_WOLFTPM
    printf("TPM2_RSA_Decrypt: %d\n", rsaDecOut.message.size);
#endif
    return rc;
}


int wolfTPM2_ReadPCR(WOLFTPM2_DEV* dev, int pcrIndex, int alg, byte* digest,
    int* p_digest_len)
{
    int rc;
    PCR_Read_In  pcrReadIn;
    PCR_Read_Out pcrReadOut;
    int digest_len;

    if (dev == NULL)
        return BAD_FUNC_ARG;

    wolfTPM2_SetupPCRSel(&pcrReadIn.pcrSelectionIn, alg, pcrIndex);
    rc = TPM2_PCR_Read(&pcrReadIn, &pcrReadOut);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_PCR_Read failed %d: %s\n", rc, wolfTPM2_GetRCString(rc));
    #endif
        return rc;
    }

    digest_len = (int)pcrReadOut.pcrValues.digests[0].size;
    if (digest)
        XMEMCPY(digest, pcrReadOut.pcrValues.digests[0].buffer, digest_len);

#ifdef DEBUG_WOLFTPM
    printf("TPM2_PCR_Read: Index %d, Digest Sz %d, Update Counter %d\n",
        pcrIndex, digest_len, (int)pcrReadOut.pcrUpdateCounter);
    TPM2_PrintBin(digest, digest_len);
#endif

    if (p_digest_len)
        *p_digest_len = digest_len;

    return rc;
}

int wolfTPM2_UnloadHandle(WOLFTPM2_DEV* dev, WOLFTPM2_HANDLE* handle)
{
    int rc;
    FlushContext_In in;

    if (dev == NULL || handle == NULL)
        return BAD_FUNC_ARG;

    /* don't try and unload null or persistent handles */
    if (handle->hndl == 0 || handle->hndl == TPM_RH_NULL ||
        (handle->hndl >= PERSISTENT_FIRST && handle->hndl <= PERSISTENT_LAST)) {
        return TPM_RC_SUCCESS;
    }

    XMEMSET(&in, 0, sizeof(in));
    in.flushHandle = handle->hndl;
    rc = TPM2_FlushContext(&in);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_FlushContext failed %d: %s\n", rc,
            wolfTPM2_GetRCString(rc));
    #endif
        return rc;
    }

#ifdef DEBUG_WOLFTPM
    printf("TPM2_FlushContext: Closed handle 0x%x\n", (word32)handle->hndl);
#endif

    handle->hndl = TPM_RH_NULL;

    return TPM_RC_SUCCESS;
}


int wolfTPM2_NVCreate(WOLFTPM2_DEV* dev, TPM_HANDLE authHandle,
    word32 nvIndex, word32 nvAttributes, word32 maxSize,
    const byte* auth, int authSz)
{
    int rc;
    NV_DefineSpace_In in;

    if (dev == NULL)
        return BAD_FUNC_ARG;

    /* clear auth */
    XMEMSET(&dev->session[0].auth, 0, sizeof(dev->session[0].auth));

    XMEMSET(&in, 0, sizeof(in));
    in.authHandle = authHandle;
    if (auth && authSz > 0) {
        in.auth.size = authSz;
        XMEMCPY(in.auth.buffer, auth, in.auth.size);
    }

    in.publicInfo.nvPublic.nvIndex = nvIndex;
    in.publicInfo.nvPublic.nameAlg = TPM_ALG_SHA256;
    in.publicInfo.nvPublic.attributes = nvAttributes;
    in.publicInfo.nvPublic.dataSize = (UINT16)maxSize;

    rc = TPM2_NV_DefineSpace(&in);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_NV_DefineSpace failed %d: %s\n", rc,
            wolfTPM2_GetRCString(rc));
    #endif
        return rc;
    }

#ifdef DEBUG_WOLFTPM
    printf("TPM2_NV_DefineSpace: Auth 0x%x, Idx 0x%x, Attribs 0x%d, Size %d\n",
        (word32)in.authHandle,
		(word32)in.publicInfo.nvPublic.nvIndex,
		(word32)in.publicInfo.nvPublic.attributes,
        in.publicInfo.nvPublic.dataSize);
#endif

    return rc;
}

int wolfTPM2_NVWrite(WOLFTPM2_DEV* dev, TPM_HANDLE authHandle,
    word32 nvIndex, byte* dataBuf, word32 dataSz, word32 offset)
{
    int rc = TPM_RC_SUCCESS;
    word32 pos = 0, towrite;
    NV_Write_In in;

    if (dev == NULL)
        return BAD_FUNC_ARG;

    /* clear auth */
    XMEMSET(&dev->session[0].auth, 0, sizeof(dev->session[0].auth));

    while (dataSz > 0) {
        towrite = dataSz;
        if (towrite > MAX_NV_BUFFER_SIZE)
            towrite = MAX_NV_BUFFER_SIZE;

        XMEMSET(&in, 0, sizeof(in));
        in.authHandle = authHandle;
        in.nvIndex = nvIndex;
        in.offset = offset+pos;
        in.data.size = towrite;
        if (dataBuf)
            XMEMCPY(in.data.buffer, &dataBuf[pos], towrite);

        rc = TPM2_NV_Write(&in);
        if (rc != TPM_RC_SUCCESS) {
        #ifdef DEBUG_WOLFTPM
            printf("TPM2_NV_Write failed %d: %s\n", rc,
                wolfTPM2_GetRCString(rc));
        #endif
            return rc;
        }

    #ifdef DEBUG_WOLFTPM
        printf("TPM2_NV_Write: Auth 0x%x, Idx 0x%x, Offset %d, Size %d\n",
            (word32)in.authHandle, (word32)in.nvIndex, in.offset, in.data.size);
    #endif

        pos += towrite;
        dataSz -= towrite;
    }

    return rc;

}

int wolfTPM2_NVRead(WOLFTPM2_DEV* dev, TPM_HANDLE authHandle,
    word32 nvIndex, byte* dataBuf, word32* pDataSz, word32 offset)
{
    int rc = TPM_RC_SUCCESS;
    word32 pos = 0, toread, dataSz;
    NV_Read_In in;
    NV_Read_Out out;

    if (dev == NULL || pDataSz == NULL)
        return BAD_FUNC_ARG;

    dataSz = *pDataSz;

    /* clear auth */
    XMEMSET(&dev->session[0].auth, 0, sizeof(dev->session[0].auth));

    while (dataSz > 0) {
        toread = dataSz;
        if (toread > MAX_NV_BUFFER_SIZE)
            toread = MAX_NV_BUFFER_SIZE;

        XMEMSET(&in, 0, sizeof(in));
        in.authHandle = authHandle;
        in.nvIndex = nvIndex;
        in.offset = offset+pos;
        in.size = toread;

        rc = TPM2_NV_Read(&in, &out);
        if (rc != TPM_RC_SUCCESS) {
        #ifdef DEBUG_WOLFTPM
            printf("TPM2_NV_Read failed %d: %s\n", rc,
                wolfTPM2_GetRCString(rc));
        #endif
            return rc;
        }

        toread = out.data.size;
        if (dataBuf) {
            XMEMCPY(&dataBuf[pos], out.data.buffer, toread);
        }

    #ifdef DEBUG_WOLFTPM
        printf("TPM2_NV_Read: Auth 0x%x, Idx 0x%x, Offset %d, Size %d\n",
            (word32)in.authHandle, (word32)in.nvIndex, in.offset, out.data.size);
    #endif

        /* if we are done reading, exit loop */
        if (toread == 0)
            break;

        pos += toread;
        dataSz -= toread;
    }

    *pDataSz = pos;

    return rc;
}

int wolfTPM2_NVReadPublic(WOLFTPM2_DEV* dev, word32 nvIndex,
    TPMS_NV_PUBLIC* nvPublic)
{
    int rc;
    NV_ReadPublic_In  in;
    NV_ReadPublic_Out out;

    if (dev == NULL)
        return BAD_FUNC_ARG;

    XMEMSET(&in, 0, sizeof(in));
    in.nvIndex = nvIndex;
    rc = TPM2_NV_ReadPublic(&in, &out);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_NV_ReadPublic failed %d: %s\n", rc,
            wolfTPM2_GetRCString(rc));
    #endif
        return rc;
    }

#ifdef DEBUG_WOLFTPM
    printf("TPM2_NV_ReadPublic: Sz %d, Idx 0x%x, nameAlg %d, Attr 0x%x, "
            "authPol %d, dataSz %d, name %d\n",
        out.nvPublic.size,
		(word32)out.nvPublic.nvPublic.nvIndex,
        out.nvPublic.nvPublic.nameAlg,
        (word32)out.nvPublic.nvPublic.attributes,
        out.nvPublic.nvPublic.authPolicy.size,
        out.nvPublic.nvPublic.dataSize,
        out.nvName.size);
#endif

    if (nvPublic) {
        XMEMCPY(nvPublic, &out.nvPublic.nvPublic, sizeof(*nvPublic));
    }

    return rc;
}

int wolfTPM2_NVDelete(WOLFTPM2_DEV* dev, TPM_HANDLE authHandle,
    word32 nvIndex)
{
    int rc;
    NV_UndefineSpace_In in;

    if (dev == NULL)
        return BAD_FUNC_ARG;

    /* clear auth */
    XMEMSET(&dev->session[0].auth, 0, sizeof(dev->session[0].auth));

    XMEMSET(&in, 0, sizeof(in));
    in.authHandle = authHandle;
    in.nvIndex = nvIndex;

    rc = TPM2_NV_UndefineSpace(&in);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_NV_UndefineSpace failed %d: %s\n", rc,
            wolfTPM2_GetRCString(rc));
    #endif
        return rc;
    }

#ifdef DEBUG_WOLFTPM
    printf("TPM2_NV_UndefineSpace: Auth 0x%x, Idx 0x%x\n",
        (word32)in.authHandle, (word32)in.nvIndex);
#endif

    return rc;
}

#ifndef WOLFTPM2_NO_WOLFCRYPT
struct WC_RNG* wolfTPM2_GetRng(WOLFTPM2_DEV* dev)
{
#ifndef WC_NO_RNG
    if (dev)
        return &dev->ctx.rng;
#endif
    return NULL;
}
#endif

int wolfTPM2_GetRandom(WOLFTPM2_DEV* dev, byte* buf, word32 len)
{
    int rc = TPM_RC_SUCCESS;
    GetRandom_In in;
    GetRandom_Out out;
    word32 sz, pos = 0;

    if (dev == NULL || buf == NULL)
        return BAD_FUNC_ARG;

    while (pos < len) {
        /* caclulate size to get */
        sz = len - pos;
        if (sz > MAX_RNG_REQ_SIZE)
            sz = MAX_RNG_REQ_SIZE;

        XMEMSET(&in, 0, sizeof(in));
        in.bytesRequested = sz;
        rc = TPM2_GetRandom(&in, &out);
        if (rc != TPM_RC_SUCCESS) {
        #ifdef DEBUG_WOLFTPM
            printf("TPM2_GetRandom failed 0x%x: %s\n", rc,
                TPM2_GetRCString(rc));
        #endif
            break;
        }

        sz = out.randomBytes.size; /* use actual returned size */
        if (sz > MAX_RNG_REQ_SIZE) {
        #ifdef DEBUG_WOLFTPM
            printf("wolfTPM2_GetRandom out size error\n");
        #endif
            rc = BAD_FUNC_ARG;
            break;
        }

        XMEMCPY(&buf[pos], out.randomBytes.buffer, sz);
        pos += sz;
    }
    return rc;
}

int wolfTPM2_Clear(WOLFTPM2_DEV* dev)
{
    int rc;
    Clear_In in;

    if (dev == NULL)
        return BAD_FUNC_ARG;

    XMEMSET(&in, 0, sizeof(in));
    in.authHandle = TPM_RH_LOCKOUT;

    rc = TPM2_Clear(&in);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_Clear failed %d: %s\n", rc,
            wolfTPM2_GetRCString(rc));
    #endif
        return rc;
    }

#ifdef DEBUG_WOLFTPM
    printf("TPM2_Clear Auth 0x%x\n", (word32)in.authHandle);
#endif

    return rc;
}

/* Hashing */
/* usageAuth: Optional auth for handle */
int wolfTPM2_HashStart(WOLFTPM2_DEV* dev, WOLFTPM2_HASH* hash,
    TPMI_ALG_HASH hashAlg, const byte* usageAuth, word32 usageAuthSz)
{
    int rc;
    HashSequenceStart_In in;
    HashSequenceStart_Out out;

    if (dev == NULL || hash == NULL || hashAlg == TPM_ALG_NULL) {
        return BAD_FUNC_ARG;
    }

    /* Capture usage auth */
    if (usageAuthSz > sizeof(hash->handle.auth.buffer))
        usageAuthSz = sizeof(hash->handle.auth.buffer);
    hash->handle.auth.size = usageAuthSz;
    XMEMCPY(hash->handle.auth.buffer, usageAuth, usageAuthSz);

    XMEMSET(&in, 0, sizeof(in));
    in.auth = hash->handle.auth;
    in.hashAlg = hashAlg;
    rc = TPM2_HashSequenceStart(&in, &out);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_HashSequenceStart failed 0x%x: %s\n", rc,
            TPM2_GetRCString(rc));
    #endif
        return rc;
    }

    /* Capture hash sequence handle */
    hash->handle.hndl = out.sequenceHandle;

#ifdef DEBUG_WOLFTPM
    printf("wolfTPM2_HashStart: Handle 0x%x\n",
        (word32)out.sequenceHandle);
#endif

    return rc;
}

int wolfTPM2_HashUpdate(WOLFTPM2_DEV* dev, WOLFTPM2_HASH* hash,
    const byte* data, word32 dataSz)
{
    int rc = TPM_RC_SUCCESS;
    SequenceUpdate_In in;
    word32 pos = 0, hashSz;

    if (dev == NULL || hash == NULL || (data == NULL && dataSz > 0)) {
        return BAD_FUNC_ARG;
    }

    /* set session auth for hash handle */
    dev->session[0].auth = hash->handle.auth;

    XMEMSET(&in, 0, sizeof(in));
    in.sequenceHandle = hash->handle.hndl;

    while (pos < dataSz) {
        hashSz = dataSz - pos;
        if (hashSz > sizeof(in.buffer.buffer))
            hashSz = sizeof(in.buffer.buffer);

        in.buffer.size = hashSz;
        XMEMCPY(in.buffer.buffer, &data[pos], hashSz);
        rc = TPM2_SequenceUpdate(&in);
        if (rc != TPM_RC_SUCCESS) {
        #ifdef DEBUG_WOLFTPM
            printf("TPM2_SequenceUpdate failed 0x%x: %s\n", rc,
                TPM2_GetRCString(rc));
        #endif
            return rc;
        }
        pos += hashSz;
    }

#ifdef DEBUG_WOLFTPM
    printf("wolfTPM2_HashUpdate: Handle 0x%x, DataSz %d\n",
        (word32)in.sequenceHandle, dataSz);
#endif

    return rc;
}

int wolfTPM2_HashFinish(WOLFTPM2_DEV* dev, WOLFTPM2_HASH* hash,
    byte* digest, word32* digestSz)
{
    int rc;
    SequenceComplete_In in;
    SequenceComplete_Out out;

    if (dev == NULL || hash == NULL || digest == NULL || digestSz == NULL) {
        return BAD_FUNC_ARG;
    }

    /* set session auth for hash handle */
    dev->session[0].auth = hash->handle.auth;

    XMEMSET(&in, 0, sizeof(in));
    in.sequenceHandle = hash->handle.hndl;
    in.hierarchy = TPM_RH_NULL;
    rc = TPM2_SequenceComplete(&in, &out);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_SequenceComplete failed 0x%x: %s\n", rc,
            TPM2_GetRCString(rc));
    #endif
        return rc;
    }

    if (out.result.size > *digestSz)
        out.result.size = *digestSz;
    *digestSz = out.result.size;
    XMEMCPY(digest, out.result.buffer, *digestSz);

#ifdef DEBUG_WOLFTPM
    printf("wolfTPM2_HashFinish: Handle 0x%x, DigestSz %d\n",
        (word32)in.sequenceHandle, *digestSz);
#endif

    return rc;
}

int wolfTPM2_LoadSymmetricKey(WOLFTPM2_DEV* dev, const WOLFTPM2_KEY* parentKey,
    WOLFTPM2_KEY* key, int alg, const byte* keyBuf, word32 keySz)
{
    int rc;
    TPM2B_PUBLIC pub;
    TPM2B_SENSITIVE sens;

    if (dev == NULL || key == NULL || keyBuf == NULL || (keySz != 16 && keySz != 32)) {
        return BAD_FUNC_ARG;
    }
    if (keySz > sizeof(sens.sensitiveArea.sensitive.sym.buffer))
        return BUFFER_E;

    /* Set up public key */
    rc = wolfTPM2_GetKeyTemplate_Symmetric(&pub.publicArea, keySz * 8, alg, YES, YES);
    if (rc != 0)
        return rc;
    pub.publicArea.nameAlg = keySz == 32 ? TPM_ALG_SHA256 : TPM_ALG_SHA1;

    /* Set up private key */
    XMEMSET(&sens, 0, sizeof(sens));
    sens.sensitiveArea.sensitiveType = TPM_ALG_SYMCIPHER;
    if (key->handle.auth.size > 0) {
        sens.sensitiveArea.authValue.size = key->handle.auth.size;
        XMEMCPY(sens.sensitiveArea.authValue.buffer, key->handle.auth.buffer,
            key->handle.auth.size);
    }
    sens.sensitiveArea.sensitive.sym.size = keySz;
    XMEMCPY(sens.sensitiveArea.sensitive.sym.buffer, keyBuf, keySz);

    return wolfTPM2_LoadPrivateKey(dev, parentKey, key, &pub, &sens);
}

/* EncryptDecrypt */
int wolfTPM2_EncryptDecryptBlock(WOLFTPM2_DEV* dev, WOLFTPM2_KEY* key,
    const byte* in, byte* out, word32 inOutSz, const byte* iv, word32 ivSz,
    int isDecrypt)
{
    int rc;
#ifdef WOLFTPM_MCHP
    EncryptDecrypt_In encDecIn;
    EncryptDecrypt_Out encDecOut;
#else
    EncryptDecrypt2_In encDecIn;
    EncryptDecrypt2_Out encDecOut;
#endif

    if (dev == NULL || key == NULL || in == NULL || out == NULL || inOutSz == 0) {
        return BAD_FUNC_ARG;
    }

    /* set session auth for key */
    dev->session[0].auth = key->handle.auth;

    XMEMSET(&encDecIn, 0, sizeof(encDecIn));
    encDecIn.keyHandle = key->handle.hndl;
    if (iv == NULL || ivSz == 0) {
        encDecIn.ivIn.size = MAX_AES_BLOCK_SIZE_BYTES; /* zeros */
    }
    else {
        encDecIn.ivIn.size = ivSz;
        XMEMCPY(encDecIn.ivIn.buffer, iv, ivSz);
    }
    encDecIn.decrypt = isDecrypt;
    /* use symmetric algorithm from key */
    encDecIn.mode = key->pub.publicArea.parameters.symDetail.sym.mode.aes;

    encDecIn.inData.size = inOutSz;
    XMEMCPY(encDecIn.inData.buffer, in, inOutSz);

    /* make sure its multiple of block size */
    encDecIn.inData.size = (encDecIn.inData.size +
        MAX_AES_BLOCK_SIZE_BYTES - 1) & ~(MAX_AES_BLOCK_SIZE_BYTES - 1);

#ifdef WOLFTPM_MCHP
    rc = TPM2_EncryptDecrypt(&encDecIn, &encDecOut);
#else
    rc = TPM2_EncryptDecrypt2(&encDecIn, &encDecOut);
#endif
    if (rc == TPM_RC_COMMAND_CODE) { /* some TPM's may not support command */
        /* try to enable support */
        rc = wolfTPM2_SetCommand(dev, TPM_CC_EncryptDecrypt2, YES);
        if (rc == TPM_RC_SUCCESS) {
            /* try command again */
        #ifdef WOLFTPM_MCHP
            rc = TPM2_EncryptDecrypt(&encDecIn, &encDecOut);
        #else
            rc = TPM2_EncryptDecrypt2(&encDecIn, &encDecOut);
        #endif
        }
    }

    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_EncryptDecrypt2 failed 0x%x: %s\n", rc,
            TPM2_GetRCString(rc));
    #endif
        return rc;
    }

    /* return block */
    if (inOutSz > encDecOut.outData.size)
        inOutSz = encDecOut.outData.size;
    XMEMCPY(out, encDecOut.outData.buffer, inOutSz);

    return rc;
}

int wolfTPM2_EncryptDecrypt(WOLFTPM2_DEV* dev, WOLFTPM2_KEY* key,
    const byte* in, byte* out, word32 inOutSz,
    const byte* iv, word32 ivSz, int isDecrypt)
{
    int rc;
    word32 pos = 0, xfer;

    while (pos < inOutSz) {
        xfer = inOutSz - pos;
        if (xfer > MAX_DIGEST_BUFFER)
            xfer = MAX_DIGEST_BUFFER;

        rc = wolfTPM2_EncryptDecryptBlock(dev, key, &in[pos], &out[pos],
            xfer, iv, ivSz, isDecrypt);
        if (rc != TPM_RC_SUCCESS)
            break;

        pos += xfer;
    }

#ifdef DEBUG_WOLFTPM
    printf("wolfTPM2_EncryptDecrypt: 0x%x: %s, %d bytes\n",
        rc, TPM2_GetRCString(rc), inOutSz);
#endif

    return rc;
}


int wolfTPM2_SetCommand(WOLFTPM2_DEV* dev, TPM_CC commandCode, int enableFlag)
{
    int rc;
#ifdef WOLFTPM_ST33
    SetCommandSet_In in;

    /* Enable TPM2_EncryptDecrypt2 command */
    XMEMSET(&in, 0, sizeof(in));
    in.authHandle = TPM_RH_PLATFORM;
    in.commandCode = commandCode;
    in.enableFlag = enableFlag;
    rc = TPM2_SetCommandSet(&in);
    if (rc != TPM_RC_SUCCESS) {
    #ifdef DEBUG_WOLFTPM
        printf("TPM2_SetCommandSet failed 0x%x: %s\n", rc,
            TPM2_GetRCString(rc));
    #endif
    }
#else
    (void)commandCode;
    (void)enableFlag;
    rc = TPM_RC_COMMAND_CODE; /* not supported */
#endif
    (void)dev;
    return rc;
}

/******************************************************************************/
/* --- END Wrapper Device Functions-- */
/******************************************************************************/


/******************************************************************************/
/* --- BEGIN Utility Functions -- */
/******************************************************************************/

int wolfTPM2_GetKeyTemplate_RSA(TPMT_PUBLIC* publicTemplate,
    TPMA_OBJECT objectAttributes)
{
    if (publicTemplate == NULL)
        return BAD_FUNC_ARG;

    XMEMSET(publicTemplate, 0, sizeof(TPMT_PUBLIC));
    publicTemplate->type = TPM_ALG_RSA;
    publicTemplate->unique.rsa.size = WOLFTPM2_WRAP_RSA_KEY_BITS / 8;
    publicTemplate->nameAlg = WOLFTPM2_WRAP_DIGEST;
    publicTemplate->objectAttributes = objectAttributes;
    publicTemplate->parameters.rsaDetail.keyBits = WOLFTPM2_WRAP_RSA_KEY_BITS;
    publicTemplate->parameters.rsaDetail.exponent = WOLFTPM2_WRAP_RSA_EXPONENT;
    publicTemplate->parameters.rsaDetail.scheme.scheme = TPM_ALG_NULL;
    if (objectAttributes & TPMA_OBJECT_fixedTPM) {
        publicTemplate->parameters.rsaDetail.symmetric.algorithm = TPM_ALG_AES;
        publicTemplate->parameters.rsaDetail.symmetric.keyBits.aes = 128;
        publicTemplate->parameters.rsaDetail.symmetric.mode.aes = TPM_ALG_CFB;
    }
    else {
        publicTemplate->parameters.rsaDetail.symmetric.algorithm = TPM_ALG_NULL;
    }

    return 0;
}

int wolfTPM2_GetKeyTemplate_ECC(TPMT_PUBLIC* publicTemplate,
    TPMA_OBJECT objectAttributes, TPM_ECC_CURVE curve, TPM_ALG_ID sigScheme)
{
    if (publicTemplate == NULL)
        return BAD_FUNC_ARG;

    XMEMSET(publicTemplate, 0, sizeof(TPMT_PUBLIC));
    publicTemplate->type = TPM_ALG_ECC;
    publicTemplate->nameAlg = WOLFTPM2_WRAP_DIGEST;
    publicTemplate->objectAttributes = objectAttributes;
    if (objectAttributes & TPMA_OBJECT_fixedTPM) {
        publicTemplate->parameters.rsaDetail.symmetric.algorithm = TPM_ALG_AES;
        publicTemplate->parameters.rsaDetail.symmetric.keyBits.aes = 128;
        publicTemplate->parameters.rsaDetail.symmetric.mode.aes = TPM_ALG_CFB;
    }
    else {
        publicTemplate->parameters.rsaDetail.symmetric.algorithm = TPM_ALG_NULL;
    }
    publicTemplate->parameters.eccDetail.scheme.scheme = sigScheme;
                                            /* TPM_ALG_ECDSA or TPM_ALG_ECDH */
    publicTemplate->parameters.eccDetail.scheme.details.ecdsa.hashAlg =
        WOLFTPM2_WRAP_DIGEST;
    publicTemplate->parameters.eccDetail.curveID = curve;
    publicTemplate->parameters.eccDetail.kdf.scheme = TPM_ALG_NULL;

    return 0;
}

int wolfTPM2_GetKeyTemplate_Symmetric(TPMT_PUBLIC* publicTemplate, int keyBits,
    TPM_ALG_ID algMode, int isSign, int isDecrypt)
{
    if (publicTemplate == NULL)
        return BAD_FUNC_ARG;

#ifdef WOLFTPM_MCHP
    /* workaround for issue with both sign and decrypt being set */
    if (isSign && isDecrypt) {
        isSign = NO;
    }
#endif

    XMEMSET(publicTemplate, 0, sizeof(TPMT_PUBLIC));
    publicTemplate->type = TPM_ALG_SYMCIPHER;
    publicTemplate->nameAlg = WOLFTPM2_WRAP_DIGEST;
    publicTemplate->unique.sym.size = keyBits / 8;
    publicTemplate->objectAttributes = (
        TPMA_OBJECT_sensitiveDataOrigin | TPMA_OBJECT_userWithAuth |
        TPMA_OBJECT_noDA | (isSign ? TPMA_OBJECT_sign : 0) |
        (isDecrypt ? TPMA_OBJECT_decrypt : 0));
    publicTemplate->parameters.symDetail.sym.algorithm = TPM_ALG_AES;
    publicTemplate->parameters.symDetail.sym.keyBits.sym = keyBits;
    publicTemplate->parameters.symDetail.sym.mode.sym = algMode;

    return 0;
}

int wolfTPM2_GetKeyTemplate_RSA_EK(TPMT_PUBLIC* publicTemplate)
{
    if (publicTemplate == NULL)
        return BAD_FUNC_ARG;

    XMEMSET(publicTemplate, 0, sizeof(TPMT_PUBLIC));
    publicTemplate->type = TPM_ALG_RSA;
    publicTemplate->unique.rsa.size = 256;
    publicTemplate->nameAlg = TPM_ALG_SHA256;
    publicTemplate->objectAttributes = (
        TPMA_OBJECT_fixedTPM | TPMA_OBJECT_fixedParent |
        TPMA_OBJECT_sensitiveDataOrigin | TPMA_OBJECT_adminWithPolicy |
        TPMA_OBJECT_restricted | TPMA_OBJECT_decrypt);
    publicTemplate->parameters.rsaDetail.keyBits = 2048;
    publicTemplate->parameters.rsaDetail.exponent = 0;
    publicTemplate->parameters.rsaDetail.scheme.scheme = TPM_ALG_NULL;
    publicTemplate->parameters.rsaDetail.symmetric.algorithm = TPM_ALG_AES;
    publicTemplate->parameters.rsaDetail.symmetric.keyBits.aes = 128;
    publicTemplate->parameters.rsaDetail.symmetric.mode.aes = TPM_ALG_CFB;
    publicTemplate->authPolicy.size = sizeof(TPM_20_EK_AUTH_POLICY);
    XMEMCPY(publicTemplate->authPolicy.buffer,
        TPM_20_EK_AUTH_POLICY, publicTemplate->authPolicy.size);

    return 0;
}

int wolfTPM2_GetKeyTemplate_ECC_EK(TPMT_PUBLIC* publicTemplate)
{
    if (publicTemplate == NULL)
        return BAD_FUNC_ARG;

    XMEMSET(publicTemplate, 0, sizeof(TPMT_PUBLIC));
    publicTemplate->type = TPM_ALG_ECC;
    publicTemplate->unique.ecc.x.size = 32;
    publicTemplate->unique.ecc.y.size = 32;
    publicTemplate->nameAlg = TPM_ALG_SHA256;
    publicTemplate->objectAttributes = (
        TPMA_OBJECT_fixedTPM | TPMA_OBJECT_fixedParent |
        TPMA_OBJECT_sensitiveDataOrigin | TPMA_OBJECT_adminWithPolicy |
        TPMA_OBJECT_restricted | TPMA_OBJECT_decrypt);
    publicTemplate->parameters.eccDetail.symmetric.algorithm = TPM_ALG_AES;
    publicTemplate->parameters.eccDetail.symmetric.keyBits.aes = 128;
    publicTemplate->parameters.eccDetail.symmetric.mode.aes = TPM_ALG_CFB;
    publicTemplate->parameters.eccDetail.scheme.scheme = TPM_ALG_NULL;
    publicTemplate->parameters.eccDetail.scheme.details.ecdsa.hashAlg =
        TPM_ALG_SHA256;
    publicTemplate->parameters.eccDetail.curveID = TPM_ECC_NIST_P256;
    publicTemplate->parameters.eccDetail.kdf.scheme = TPM_ALG_NULL;
    publicTemplate->authPolicy.size = sizeof(TPM_20_EK_AUTH_POLICY);
    XMEMCPY(publicTemplate->authPolicy.buffer,
        TPM_20_EK_AUTH_POLICY, publicTemplate->authPolicy.size);

    return 0;
}

int wolfTPM2_GetNvAttributesTemplate(TPM_HANDLE auth, word32* nvAttributes)
{
    if (nvAttributes == NULL)
        return BAD_FUNC_ARG;

    *nvAttributes = (
        TPMA_NV_AUTHWRITE | TPMA_NV_OWNERWRITE |    /* write allowed */
        TPMA_NV_AUTHREAD |  TPMA_NV_OWNERREAD |     /* read allowed */
        TPMA_NV_NO_DA                               /* no dictionary attack */
    );

    if (auth == TPM_RH_PLATFORM) {
        *nvAttributes |= (
            TPMA_NV_PPWRITE | TPMA_NV_PPREAD
        );
    }

    return 0;
}

/******************************************************************************/
/* --- END Utility Functions -- */
/******************************************************************************/


#ifdef WOLF_CRYPTO_DEV
/******************************************************************************/
/* --- BEGIN wolf Crypto Device Support -- */
/******************************************************************************/

int wolfTPM2_CryptoDevCb(int devId, wc_CryptoInfo* info, void* ctx)
{
    int rc = NOT_COMPILED_IN; /* return this to bypass HW and use SW */
    TpmCryptoDevCtx* tlsCtx = (TpmCryptoDevCtx*)ctx;

    if (info == NULL || ctx == NULL || tlsCtx->dev == NULL)
        return BAD_FUNC_ARG;

    (void)devId;

    if (info->algo_type == WC_ALGO_TYPE_RNG) {
    #ifndef WC_NO_RNG
    #ifdef DEBUG_WOLFTPM
        printf("CryptoDevCb RNG: Sz %d\n", info->rng.sz);
    #endif
        rc = wolfTPM2_GetRandom(tlsCtx->dev, info->rng.out, info->rng.sz);
    #endif
    }
    else if (info->algo_type == WC_ALGO_TYPE_PK) {
        int isWolfKeyValid = 1;

    #ifdef DEBUG_WOLFTPM
        printf("CryptoDevCb Pk: Type %d\n", info->pk.type);
    #endif

        /* optional callback to check key to determine if TPM should be used */
        if (tlsCtx->checkKeyCb) {
            /* this is useful to check the provided key for dummy key
                cases like TLS server */
            if (tlsCtx->checkKeyCb(info, tlsCtx) != 0) {
                isWolfKeyValid = 0;
            }
        }

    #ifndef NO_RSA
        /* RSA */
        if (info->pk.type == WC_PK_TYPE_RSA_KEYGEN) {
            /* TODO: */
            #if 0
            RsaKey* key;
            int     size;
            long    e;
            WC_RNG* rng;
            #endif
            rc = NOT_COMPILED_IN;
        }
        else if (info->pk.type == WC_PK_TYPE_RSA) {
            switch (info->pk.rsa.type) {
                case RSA_PUBLIC_ENCRYPT:
                case RSA_PUBLIC_DECRYPT:
                {
                    /* public operations */
                    WOLFTPM2_KEY rsaPub;

                    if (!isWolfKeyValid && tlsCtx->rsaKey) {
                        /* use already loaded TPM handle for operation */
                        rc = wolfTPM2_RsaEncrypt(tlsCtx->dev, tlsCtx->rsaKey,
                            TPM_ALG_NULL, /* no padding */
                            info->pk.rsa.in, info->pk.rsa.inLen,
                            info->pk.rsa.out, (int*)info->pk.rsa.outLen);
                        break;
                    }
                    /* otherwise load public key and perform public op */

                    /* load public key into TPM */
                    rc = wolfTPM2_RsaKey_WolfToTpm(tlsCtx->dev,
                        info->pk.rsa.key, &rsaPub);
                    if (rc != 0) {
                        /* A failure of TPM_RC_KEY can happen due to unsupported
                            RSA exponents. In those cases return NOT_COMPILED_IN
                            and use software */
                        rc = NOT_COMPILED_IN;
                        break;
                    }

                    /* public operations */
                    rc = wolfTPM2_RsaEncrypt(tlsCtx->dev, &rsaPub,
                        TPM_ALG_NULL, /* no padding */
                        info->pk.rsa.in, info->pk.rsa.inLen,
                        info->pk.rsa.out, (int*)info->pk.rsa.outLen);

                    wolfTPM2_UnloadHandle(tlsCtx->dev, &rsaPub.handle);
                    break;
                }
                case RSA_PRIVATE_ENCRYPT:
                case RSA_PRIVATE_DECRYPT:
                {
                    /* private operations */
                    rc = wolfTPM2_RsaDecrypt(tlsCtx->dev, tlsCtx->rsaKey,
                        TPM_ALG_NULL, /* no padding */
                        info->pk.rsa.in, info->pk.rsa.inLen,
                        info->pk.rsa.out, (int*)info->pk.rsa.outLen);
                    break;
                }
            }
        }
    #endif /* !NO_RSA */
    #ifdef HAVE_ECC
        if (info->pk.type == WC_PK_TYPE_EC_KEYGEN) {
        #ifdef WOLFTPM2_USE_SW_ECDHE
            rc = NOT_COMPILED_IN;
        #else
            int curve_id;

            /* Make sure an ECDH key has been set and curve is supported */
            rc = TPM2_GetTpmCurve(info->pk.eckg.curveId);
            if (rc < 0 || tlsCtx->ecdhKey == NULL || tlsCtx->eccKey == NULL) {
                return NOT_COMPILED_IN;
            }
            curve_id = rc;

            /* Generate ephemeral key */
            rc = wolfTPM2_ECDHGenKey(tlsCtx->dev, tlsCtx->ecdhKey, curve_id,
                (byte*)tlsCtx->eccKey->handle.auth.buffer,
                tlsCtx->eccKey->handle.auth.size);
            if (rc == 0) {
                /* Export public key info to wolf ecc_key */
                rc = wolfTPM2_EccKey_TpmToWolf(tlsCtx->dev, tlsCtx->ecdhKey,
                    info->pk.eckg.key);
                if (rc != 0) {
                    /* if failure, release key */
                    wolfTPM2_UnloadHandle(tlsCtx->dev, &tlsCtx->ecdhKey->handle);
                }
            }
        #endif /* WOLFTPM2_USE_SW_ECDHE */
        }
        else if (info->pk.type == WC_PK_TYPE_ECDSA_SIGN) {
            byte sigRS[MAX_ECC_BYTES*2];
            byte *r = sigRS, *s;
            word32 rsLen = sizeof(sigRS), rLen, sLen;
            word32 inlen = info->pk.eccsign.inlen;

            /* truncate input to match key size */
            rLen = wc_ecc_size(info->pk.eccsign.key);
            if (inlen > rLen)
                inlen = rLen;

            rc = wolfTPM2_SignHash(tlsCtx->dev, tlsCtx->eccKey,
                info->pk.eccsign.in, inlen, sigRS, (int*)&rsLen);
            if (rc == 0) {
                /* Encode ECDSA Header */
                rLen = sLen = rsLen / 2;
                s = &sigRS[rLen];
                rc = wc_ecc_rs_raw_to_sig(r, rLen, s, sLen,
                    info->pk.eccsign.out, info->pk.eccsign.outlen);
            }
        }
        else if (info->pk.type == WC_PK_TYPE_ECDSA_VERIFY) {
            WOLFTPM2_KEY eccPub;
            byte sigRS[MAX_ECC_BYTES*2];
            byte *r = sigRS, *s = &sigRS[MAX_ECC_BYTES];
            word32 rLen = MAX_ECC_BYTES, sLen = MAX_ECC_BYTES;

            /* Decode ECDSA Header */
            rc = wc_ecc_sig_to_rs(info->pk.eccverify.sig,
                info->pk.eccverify.siglen, r, &rLen, s, &sLen);
            if (rc == 0) {
                /* load public key into TPM */
                rc = wolfTPM2_EccKey_WolfToTpm(tlsCtx->dev,
                    info->pk.eccverify.key, &eccPub);
                if (rc == 0) {
                    rc = wolfTPM2_VerifyHash(tlsCtx->dev, &eccPub,
                        sigRS, rLen + sLen,
                        info->pk.eccverify.hash, info->pk.eccverify.hashlen);

                    if (rc == 0 && info->pk.eccverify.res) {
                        *info->pk.eccverify.res = 1;
                    }

                    wolfTPM2_UnloadHandle(tlsCtx->dev, &eccPub.handle);
                }
            }
        }
        else if (info->pk.type == WC_PK_TYPE_ECDH) {
        #ifdef WOLFTPM2_USE_SW_ECDHE
            rc = NOT_COMPILED_IN;
        #else
            TPM2B_ECC_POINT pubPoint;

            /* Make sure an ECDH key has been set */
            if (tlsCtx->ecdhKey == NULL || tlsCtx->eccKey == NULL) {
                return NOT_COMPILED_IN;
            }

            rc = wolfTPM2_EccKey_WolfToPubPoint(tlsCtx->dev,
                info->pk.ecdh.public_key, &pubPoint);
            if (rc == 0) {
                /* Compute shared secret and compare results */
                rc = wolfTPM2_ECDHGenZ(tlsCtx->dev, tlsCtx->ecdhKey,
                    &pubPoint, info->pk.ecdh.out, (int*)info->pk.ecdh.outlen);
            }

            /* done with ephemeral key */
            wolfTPM2_UnloadHandle(tlsCtx->dev, &tlsCtx->ecdhKey->handle);
        #endif /* !WOLFTPM2_USE_SW_ECDHE */
        }
    #endif
    }
    else if (info->algo_type == WC_ALGO_TYPE_CIPHER) {
    #ifndef NO_AES
        #ifdef DEBUG_WOLFTPM
        printf("CryptoDevCb Cipher: Type %d\n", info->cipher.type);
        #endif
        if (info->cipher.type != WC_CIPHER_AES_CBC) {
            return NOT_COMPILED_IN;
        }

        if (info->cipher.aescbc.aes) {
            WOLFTPM2_KEY symKey;
            Aes* aes = info->cipher.aescbc.aes;

            /* load key */
            rc = wolfTPM2_LoadSymmetricKey(tlsCtx->dev, NULL, &symKey,
                TPM_ALG_CBC, (byte*)aes->devKey, aes->keylen);
            if (rc == 0) {
                /* perform symmetric encrypt/decrypt */
                rc = wolfTPM2_EncryptDecrypt(tlsCtx->dev, &symKey,
                    info->cipher.aescbc.in,
                    info->cipher.aescbc.out,
                    info->cipher.aescbc.sz,
                    NULL, 0,
                    info->cipher.enc ? WOLFTPM2_ENCRYPT : WOLFTPM2_DECRYPT);

                /* done with handle */
                wolfTPM2_UnloadHandle(tlsCtx->dev, &symKey.handle);
            }
        }

    #endif /* !NO_AES */
    }
    else if (info->algo_type == WC_ALGO_TYPE_HASH) {
    #if !defined(NO_SHA) || !defined(NO_SHA256)
        WOLFTPM2_HASH hashCtx;
        TPM_HANDLE* hashHandle;
        TPM_ALG_ID hashAlg = TPM_ALG_ERROR;
        #ifdef DEBUG_WOLFTPM
        printf("CryptoDevCb Hash: Type %d\n", info->hash.type);
        #endif
        if (info->hash.type != WC_HASH_TYPE_SHA &&
            info->hash.type != WC_HASH_TYPE_SHA256) {
            return NOT_COMPILED_IN;
        }

        XMEMSET(&hashCtx, 0, sizeof(hashCtx));
    #ifndef NO_SHA
        if (info->hash.type == WC_HASH_TYPE_SHA && info->hash.sha1 == NULL) {
            return NOT_COMPILED_IN;
        }
        else {
            hashHandle = (TPM_HANDLE*)&info->hash.sha1->devCtx;
            hashAlg = TPM_ALG_SHA1;
        }
    #endif
    #ifndef NO_SHA256
        if (info->hash.type == WC_HASH_TYPE_SHA256 && info->hash.sha256 == NULL) {
            return NOT_COMPILED_IN;
        }
        else {
            hashHandle = (TPM_HANDLE*)&info->hash.sha256->devCtx;
            hashAlg = TPM_ALG_SHA256;
        }
    #endif
        if (hashAlg == TPM_ALG_ERROR) {
            return NOT_COMPILED_IN;
        }
        hashCtx.handle.hndl = *hashHandle;

        if (info->hash.in != NULL) { /* Update */
            rc = 0;
            if (*hashHandle == 0) {
                rc = wolfTPM2_HashStart(tlsCtx->dev, &hashCtx, hashAlg, NULL, 0);
                if (rc == 0) {
                    /* save new handle to hash context */
                    *hashHandle = hashCtx.handle.hndl;
                }
            }
            if (rc == 0) {
                rc = wolfTPM2_HashUpdate(tlsCtx->dev, &hashCtx,
                    info->hash.in, info->hash.inSz);
            }
        }
        else if (info->hash.digest != NULL) { /* Final */
            word32 digestSz;
            if (hashCtx.handle.hndl == 0) {
            #ifdef DEBUG_WOLFTPM
                printf("Error: Hash final without context!\n");
                return NOT_COMPILED_IN;
            #endif
            }
            digestSz = TPM2_GetHashDigestSize(hashAlg);
            rc = wolfTPM2_HashFinish(tlsCtx->dev, &hashCtx, info->hash.digest,
                &digestSz);
        }
    #endif /* !NO_SHA || !NO_SHA256 */
    }

    /* need to return negative here for error */
    if (rc != TPM_RC_SUCCESS && rc != NOT_COMPILED_IN) {
    #ifdef DEBUG_WOLFTPM
        printf("wolfTPM2_CryptoDevCb failed rc = %d\n", rc);
    #endif
        rc = WC_HW_E;
    }

    return rc;
}

int wolfTPM2_SetCryptoDevCb(WOLFTPM2_DEV* dev, CryptoDevCallbackFunc cb,
    TpmCryptoDevCtx* tpmCtx, int* pDevId)
{
    int rc;
    int devId = INVALID_DEVID;

    if (dev == NULL || cb == NULL || tpmCtx == NULL) {
        return BAD_FUNC_ARG;
    }

    /* register a crypto device callback for TPM private key */
    rc = wolfTPM2_GetTpmDevId(dev);
    if (rc >= 0) {
        devId = rc;
        tpmCtx->dev = dev;

        rc = wc_CryptoDev_RegisterDevice(devId, cb, tpmCtx);
    }

    if (pDevId) {
        *pDevId = devId;
    }

    return rc;
}

/******************************************************************************/
/* --- END wolf Crypto Device Support -- */
/******************************************************************************/

#endif /* WOLF_CRYPTO_DEV */


#endif /* !WOLFTPM2_NO_WRAPPER */
