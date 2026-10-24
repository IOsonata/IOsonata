/* Renesas SCE5 primitives, BSD-3-Clause (see crypto_sce5-LICENSE.txt).
 * Source: renesas/fsp 6e2675336b37fbd805f0c01d5134adfdab4bf294,
 * ra/fsp/src/r_sce/crypto_procedures/src/sce5/plainkey/primitive.
 * IOsonata changes: local linkage, explicit register offsets, bounded waits,
 * sticky fault propagation and a small serialized caller interface.
 * Command words, register order and vendor algorithm branches are preserved.
 */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "crypto_sce5.h"
#ifndef SCE_REG
#define SCE_REG(offset) (*(volatile uint32_t *)(uintptr_t)(0x400C0000U + (offset)))
#endif
#ifndef SCE_POLL_MAX
#define SCE_POLL_MAX 1000000U
#endif

typedef int fsp_err_t;
enum { FSP_SUCCESS, FSP_ERR_CRYPTO_SCE_FAIL, FSP_ERR_CRYPTO_SCE_RESOURCE_CONFLICT,
    FSP_ERR_CRYPTO_SCE_RETRY, FSP_ERR_CRYPTO_SCE_KEY_SET_FAIL };
static bool s_Fault;
static bool s_Ready;
static uint32_t S_RAM[8];
static uint32_t change_endian_long(uint32_t value) { return __builtin_bswap32(value); }
static bool SceWait(unsigned offset, unsigned bit, unsigned expected)
{
    uint32_t polls = SCE_POLL_MAX;
    while (!s_Fault && polls-- > 0U) {
        if (((SCE_REG(offset) >> bit) & 1U) == expected) return true;
    }
    s_Fault = true;
    return false;
}
static void Wipe(void *data, size_t size)
{
    volatile uint8_t *p = (volatile uint8_t *)data;
    while (size-- != 0U) *p++ = 0U;
}
// Adapted from s_flash.c
/*
* Copyright (c) 2020 - 2026 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/**********************************************************************************************************************
 * File Name    : s_flash.c
 * Version      : 1.09
 * Description  : Key information file.
 *********************************************************************************************************************/
/**********************************************************************************************************************
 * History : DD.MM.YYYY Version  Description
 *         : 31.03.2020 1.09     First Release
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Includes   <System Includes> , "Project Includes"
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Macro definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Typedef definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Exported global variables
 *********************************************************************************************************************/
static uint32_t const S_FLASH[] =
{
#ifndef __ARMEB__   /* Little endian */
    0x92f41cea, 0x2309b379, 0xb7d5e7b4, 0x0e53336d,
    0xe8cf625a, 0x19e79b10, 0x9dc85a10, 0x4d197093,
    0xbb94e956, 0xe38beca2, 0x9ed529ec, 0xb40e4b25,
    0x7fff4a80, 0x37c2251a, 0x0819a70c, 0x03666eba,
    0x74c55cd1, 0x9a0bc71d, 0x9cdafc90, 0xdef69ba5,
    0xb5431194, 0x006473e9, 0xb1acfd4a, 0xfd7dc888,
    0xb9cdc199, 0x32bd21b0, 0x6dd06b62, 0x88a15abb,
    0x5f9e4d6d, 0xa680b579, 0x702a5be7, 0x7f05e917,
    0x67902610, 0x5ddd6b35, 0xba3443c1, 0x5daa8d40,
    0xbecae6a6, 0x62584c47, 0x89a99ac7, 0x4d76f91a,
    0x313af776, 0x91e9d697, 0x57af6a0a, 0xd4c7e3e7,
    0xd09c715f, 0x22e3f7f0, 0x7cac0867, 0xdf8bd4d9,
    0x04e246b3, 0x1d50adaa, 0xd7404f94, 0x60fd642f,
    0xc8f3b8a4, 0x0cc8d4ef, 0x8c81684b, 0x4480724a,
    0x5569ded3, 0x322b8e85, 0x6187bfc5, 0x47166c34,
    0x1b9cfd57, 0xa092fbf0, 0x7b152f0e, 0x187731c0,
    0x819460ae, 0xb195c4d5, 0xc89dce93, 0xb1f082c2,
    0xebe23482, 0xf7ba9c09, 0x0ed87573, 0x629450be,
    0x2679d43f, 0xeffad241, 0xb2a2f72a, 0x79f04a49,
    0x2ec41794, 0xaa2de611, 0x56019ccb, 0x9d05fefe,
    0x626a52aa, 0x65051b73, 0x3ff7b5b8, 0x4258f3b1,
    0xb2a6abac, 0xc766eedb, 0x685deafc, 0xe32e8e7f,
    0xe4cab55d, 0x266ada78, 0x596f5930, 0x06c09e9e,
    0x81a822b1, 0xe93f12b9, 0x7b0f6ad9, 0x26cea2f4,
    0x23b058e7, 0x8117b8d9, 0xa64088d0, 0xe07fd1fb,
    0x21024258, 0xb59ba1a2, 0xf6c40124, 0x8c74e284,
    0x2309cf83, 0xe2a0cb8b, 0xaf024e64, 0x796e29ed,
    0xe48c48a9, 0x67e9630b, 0x47cf8506, 0x54879af5,
    0xebb7e35b, 0xd16f2922, 0x810768ec, 0x96c94bf3,
    0xf82acd5e, 0x3e0e9a66, 0xdd76d024, 0x6b931381,
    0xe8f1075e, 0x602a4438, 0x37328b5b, 0x45d2343b,
    0x5ccab479, 0x9940f6c0, 0xe4b10d54, 0x8c508659,
#else               /* Big endian */
    0xea1cf492, 0x79b30923, 0xb4e7d5b7, 0x6d33530e,
    0x5a62cfe8, 0x109be719, 0x105ac89d, 0x9370194d,
    0x56e994bb, 0xa2ec8be3, 0xec29d59e, 0x254b0eb4,
    0x804aff7f, 0x1a25c237, 0x0ca71908, 0xba6e6603,
    0xd15cc574, 0x1dc70b9a, 0x90fcda9c, 0xa59bf6de,
    0x941143b5, 0xe9736400, 0x4afdacb1, 0x88c87dfd,
    0x99c1cdb9, 0xb021bd32, 0x626bd06d, 0xbb5aa188,
    0x6d4d9e5f, 0x79b580a6, 0xe75b2a70, 0x17e9057f,
    0x10269067, 0x356bdd5d, 0xc14334ba, 0x408daa5d,
    0xa6e6cabe, 0x474c5862, 0xc79aa989, 0x1af9764d,
    0x76f73a31, 0x97d6e991, 0x0a6aaf57, 0xe7e3c7d4,
    0x5f719cd0, 0xf0f7e322, 0x6708ac7c, 0xd9d48bdf,
    0xb346e204, 0xaaad501d, 0x944f40d7, 0x2f64fd60,
    0xa4b8f3c8, 0xefd4c80c, 0x4b68818c, 0x4a728044,
    0xd3de6955, 0x858e2b32, 0xc5bf8761, 0x346c1647,
    0x57fd9c1b, 0xf0fb92a0, 0x0e2f157b, 0xc0317718,
    0xae609481, 0xd5c495b1, 0x93ce9dc8, 0xc282f0b1,
    0x8234e2eb, 0x099cbaf7, 0x7375d80e, 0xbe509462,
    0x3fd47926, 0x41d2faef, 0x2af7a2b2, 0x494af079,
    0x9417c42e, 0x11e62daa, 0xcb9c0156, 0xfefe059d,
    0xaa526a62, 0x731b0565, 0xb8b5f73f, 0xb1f35842,
    0xacaba6b2, 0xdbee66c7, 0xfcea5d68, 0x7f8e2ee3,
    0x5db5cae4, 0x78da6a26, 0x30596f59, 0x9e9ec006,
    0xb122a881, 0xb9123fe9, 0xd96a0f7b, 0xf4a2ce26,
    0xe758b023, 0xd9b81781, 0xd08840a6, 0xfbd17fe0,
    0x58420221, 0xa2a19bb5, 0x2401c4f6, 0x84e2748c,
    0x83cf0923, 0x8bcba0e2, 0x644e02af, 0xed296e79,
    0xa9488ce4, 0x0b63e967, 0x0685cf47, 0xf59a8754,
    0x5be3b7eb, 0x22296fd1, 0xec680781, 0xf34bc996,
    0x5ecd2af8, 0x669a0e3e, 0x24d076dd, 0x8113936b,
    0x5e07f1e8, 0x38442a60, 0x5b8b3237, 0x3b34d245,
    0x79b4ca5c, 0xc0f64099, 0x540db1e4, 0x5986508c,
#endif  /* defined __ARMEB__ */
};

/**********************************************************************************************************************
 Private (static) variables and functions
 *********************************************************************************************************************/


static void HW_SCE_SoftwareResetSub(void);
static fsp_err_t HW_SCE_SelfCheck1Sub(void);
static fsp_err_t HW_SCE_SelfCheck2Sub(void);
static fsp_err_t HW_SCE_GenerateRandomNumberSub(uint32_t *OutData_Text);
static fsp_err_t HW_SCE_Aes128EncryptDecryptInitSub (const uint32_t *InData_KeyType,
                                                 const uint32_t * InData_Cmd,
                                                 const uint32_t * InData_KeyIndex,
                                                 const uint32_t * InData_IV);
static void HW_SCE_Aes128EncryptDecryptUpdateSub (const uint32_t * InData_Text, uint32_t * OutData_Text,
                                           const uint32_t MAX_CNT);
static fsp_err_t HW_SCE_Aes128EncryptDecryptFinalSub(void);
static fsp_err_t HW_SCE_Aes256EncryptDecryptInitSub (const uint32_t *InData_KeyType,
                                                 const uint32_t * InData_Cmd,
                                                 const uint32_t * InData_KeyIndex,
                                                 const uint32_t * InData_IV);
static void HW_SCE_Aes256EncryptDecryptUpdateSub (const uint32_t * InData_Text, uint32_t * OutData_Text,
                                           const uint32_t MAX_CNT);
static fsp_err_t HW_SCE_Aes256EncryptDecryptFinalSub(void);
static fsp_err_t HW_SCE_GenerateAes128PlainKeyIndexSub(uint32_t *InData_KeyType, uint32_t *InData_SharedKeyIndex, uint32_t *InData_SessionKey, uint32_t *InData_IV, uint32_t *InData_InstData, uint32_t *OutData_KeyIndex);
static fsp_err_t HW_SCE_GenerateAes256PlainKeyIndexSub(uint32_t *InData_KeyType, uint32_t *InData_SharedKeyIndex, uint32_t *InData_SessionKey, uint32_t *InData_IV, uint32_t *InData_InstData, uint32_t *OutData_KeyIndex);
static void HW_SCE_func001(uint32_t ARG1, uint32_t ARG2, uint32_t ARG3, uint32_t ARG4);
static void HW_SCE_func002(uint32_t ARG1, uint32_t ARG2, uint32_t ARG3, uint32_t ARG4);
static void HW_SCE_func003(uint32_t ARG1, uint32_t ARG2, uint32_t ARG3, uint32_t ARG4);
static void HW_SCE_func206(void);
static void HW_SCE_SelfCheck1SubSub(void);
static void HW_SCE_SelfCheck2SubSub(void);
// Adapted from hw_sce_p00.c
/*
* Copyright (c) 2020 - 2026 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/***********************************************************************************************************************
 * History : DD.MM.YYYY Version Description
 *         : 05.10.2020 1.00        First Release.
 *         : 02.12.2020 1.01        Added new functions such as the Brainpool curve.
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Imported global variables and functions (from other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Exported global variables (to be accessed by other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Private global variables and functions
***********************************************************************************************************************/

static void HW_SCE_SoftwareResetSub(void)
{
    uint32_t iLoop    = 0U;
    uint32_t iLoop1   = 0U;
    uint32_t iLoop2   = 0U;
    int32_t  jLoop    = 0U;
    uint32_t kLoop    = 0U;
    uint32_t oLoop    = 0U;
    uint32_t oLoop1   = 0U;
    uint32_t oLoop2   = 0U;
    uint32_t dummy    = 0U;
    uint32_t KEY_ADR  = 0U;
    uint32_t OFS_ADR  = 0U;
    uint32_t MAX_CNT2 = 0U;
    (void)iLoop;
    (void)iLoop1;
    (void)iLoop2;
    (void)jLoop;
    (void)kLoop;
    (void)oLoop;
    (void)oLoop1;
    (void)oLoop2;
    (void)dummy;
    (void)KEY_ADR;
    (void)OFS_ADR;
    (void)MAX_CNT2;
    SCE_REG(0x138U) = 0x00000000U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0x1D0U) = 0x00000000U;
}

/***********************************************************************************************************************
End of function ./input_dir/HW_SCE_Sec_200408/200408/RA4M1/Cryptographic/HW_SCE_p00.prc
***********************************************************************************************************************/

// Adapted from hw_sce_p01.c
/*
* Copyright (c) 2020 - 2026 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/***********************************************************************************************************************
 * History : DD.MM.YYYY Version Description
 *         : 05.10.2020 1.00        First Release.
 *         : 02.12.2020 1.01        Added new functions such as the Brainpool curve.
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Imported global variables and functions (from other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Exported global variables (to be accessed by other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Private global variables and functions
***********************************************************************************************************************/

static fsp_err_t HW_SCE_SelfCheck1Sub(void)
{
    uint32_t iLoop    = 0U;
    uint32_t iLoop1   = 0U;
    uint32_t iLoop2   = 0U;
    int32_t  jLoop    = 0U;
    uint32_t kLoop    = 0U;
    uint32_t oLoop    = 0U;
    uint32_t oLoop1   = 0U;
    uint32_t oLoop2   = 0U;
    uint32_t dummy    = 0U;
    uint32_t KEY_ADR  = 0U;
    uint32_t OFS_ADR  = 0U;
    uint32_t MAX_CNT2 = 0U;
    (void)iLoop;
    (void)iLoop1;
    (void)iLoop2;
    (void)jLoop;
    (void)kLoop;
    (void)oLoop;
    (void)oLoop1;
    (void)oLoop2;
    (void)dummy;
    (void)KEY_ADR;
    (void)OFS_ADR;
    (void)MAX_CNT2;
    SCE_REG(0x1D0U) = 0x00000000U;
    HW_SCE_SelfCheck1SubSub();

    SCE_REG(0x138U) = 0xf597806AU;
    SCE_REG(0xF0U) = 0x00000000U;
    SCE_REG(0x04U) = 0x00000001U;
    if (0x0U != (SCE_REG(0x1BCU) & 0x1fU))
    {
        return FSP_ERR_CRYPTO_SCE_RESOURCE_CONFLICT;
    }
    SCE_REG(0x80U) = 0x00000001U;
    /* WAIT_LOOP */
    if (!SceWait(0x64U, 11U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    SCE_REG(0x64U) = 0x00000008U;
    SCE_REG(0x84U) = 0x00010101U;
    SCE_REG(0x13CU) = 0x00000F00U;
    SCE_REG(0x88U) = 0x00008003U;
    SCE_REG(0x104U) = 0x00000351U;
    HW_SCE_func002(change_endian_long(0x4c1d2affU), change_endian_long(0x60ebc80aU), change_endian_long(0xdf041ee8U), change_endian_long(0x394548fdU));
    if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
    SCE_REG(0x88U) = 0x00000000U;
    SCE_REG(0x104U) = 0x00000351U;
    SCE_REG(0xA4U) = 0x400f380dU;
    /* WAIT_LOOP */
    if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    SCE_REG(0x100U) = 0xe6be653cU;
    SCE_REG(0x100U) = 0xcfa7022cU;
    SCE_REG(0x100U) = 0xb7f12cbfU;
    SCE_REG(0x100U) = 0x7eb98ec1U;
    HW_SCE_func002(change_endian_long(0x851be22bU), change_endian_long(0xa6e93a47U), change_endian_long(0x5714620aU), change_endian_long(0x5f2d7b1fU));
    if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
    HW_SCE_func001(change_endian_long(0xc0a8e4c8U), change_endian_long(0xb08ee757U), change_endian_long(0x6bd1dd36U), change_endian_long(0x11fc0e56U));
    if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
    SCE_REG(0x04U) = 0x00001001U;
    SCE_REG(0x1D0U) = 0x00000000U;
    if (0U == (((SCE_REG(0x18U) >> 13U) & 1U)))
    {
        SCE_REG(0x1BCU) = 0x00000020U;
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    else
    {
        SCE_REG(0x13CU) = 0x00000201U;
        HW_SCE_func003(change_endian_long(0x7153c22dU), change_endian_long(0xb402a12dU), change_endian_long(0x355fb000U), change_endian_long(0x55e578a7U));
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        SCE_REG(0x1BCU) = 0x00000040U;
        /* WAIT_LOOP */
        if (!SceWait(0x18U, 12U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        return FSP_SUCCESS;
    }
}

/***********************************************************************************************************************
End of function ./input_dir/HW_SCE_Sec_200408/200408/RA4M1/Cryptographic/HW_SCE_p01.prc
***********************************************************************************************************************/

// Adapted from hw_sce_p02.c
/*
* Copyright (c) 2020 - 2026 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/***********************************************************************************************************************
 * History : DD.MM.YYYY Version Description
 *         : 05.10.2020 1.00        First Release.
 *         : 02.12.2020 1.01        Added new functions such as the Brainpool curve.
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Imported global variables and functions (from other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Exported global variables (to be accessed by other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Private global variables and functions
***********************************************************************************************************************/

static fsp_err_t HW_SCE_SelfCheck2Sub(void)
{
    uint32_t iLoop    = 0U;
    uint32_t iLoop1   = 0U;
    uint32_t iLoop2   = 0U;
    int32_t  jLoop    = 0U;
    uint32_t kLoop    = 0U;
    uint32_t oLoop    = 0U;
    uint32_t oLoop1   = 0U;
    uint32_t oLoop2   = 0U;
    uint32_t dummy    = 0U;
    uint32_t KEY_ADR  = 0U;
    uint32_t OFS_ADR  = 0U;
    uint32_t MAX_CNT2 = 0U;
    (void)iLoop;
    (void)iLoop1;
    (void)iLoop2;
    (void)jLoop;
    (void)kLoop;
    (void)oLoop;
    (void)oLoop1;
    (void)oLoop2;
    (void)dummy;
    (void)KEY_ADR;
    (void)OFS_ADR;
    (void)MAX_CNT2;
    if (0x0U != (SCE_REG(0x1BCU) & 0x1fU))
    {
        return FSP_ERR_CRYPTO_SCE_RESOURCE_CONFLICT;
    }
    SCE_REG(0x84U) = 0x00000201U;
    SCE_REG(0x108U) = 0x00000000U;
    SCE_REG(0xECU) = 0x3000a800U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0xECU) = 0x00000003U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0xECU) = 0x00010020U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0xECU) = 0x00000800U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0xECU) = 0x00000080U;
    SCE_REG(0x1D0U) = 0x00000000U;
    HW_SCE_func001(0x0ffbd2c9U, 0xaa8ff051U, 0x6a474c72U, 0x43516ad7U);
    if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
    SCE_REG(0xECU) = 0x00007c00U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0x1CU) = 0x00600000U;
    SCE_REG(0x1D0U) = 0x00000000U;
    if (0x00000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        /* WAIT_LOOP */
        if (!SceWait(0x64U, 11U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x64U) = 0x00000008U;
        /* WAIT_LOOP */
        if (!SceWait(0x64U, 11U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x64U) = 0x0000005CU;
        HW_SCE_func002(0x3c4a5c81U, 0x208257cdU, 0x5b474dd8U, 0x01b5a09dU);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
    }
    else if (0x01000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        /* WAIT_LOOP */
        if (!SceWait(0x64U, 11U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x64U) = 0x00000008U;
        /* WAIT_LOOP */
        if (!SceWait(0x64U, 11U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x64U) = 0x0000005eU;
        HW_SCE_func002(0x41db671fU, 0x82356453U, 0xb3918a3bU, 0x3859a49cU);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
    }
    else if (0x02000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        /* WAIT_LOOP */
        if (!SceWait(0x64U, 11U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x64U) = 0x00000008U;
        /* WAIT_LOOP */
        if (!SceWait(0x64U, 11U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x64U) = 0x0000005dU;
        HW_SCE_func002(0x33c7fc89U, 0xb0cd3eb8U, 0xa2a73968U, 0xd8ba9e19U);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
    }
    SCE_REG(0xA4U) = 0x00040804U;
    SCE_REG(0xE0U) = 0x81010000U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0x00U) = 0x00001807U;
    /* WAIT_LOOP */
    if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    SCE_REG(0x1CU) = 0x00001800U;
    HW_SCE_SelfCheck2SubSub();

    /* WAIT_LOOP */
    if (!SceWait(0x64U, 11U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    SCE_REG(0x104U) = 0x00000051U;
    SCE_REG(0xA4U) = 0x00000684U;
    /* WAIT_LOOP */
    if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    SCE_REG(0x100U) = change_endian_long(0x00000000U);
    SCE_REG(0xE0U) = 0x80010000U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0x00U) = 0x00008107U;
    /* WAIT_LOOP */
    if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    SCE_REG(0x1CU) = 0x00001800U;
    SCE_REG(0x00U) = 0x0000010fU;
    /* WAIT_LOOP */
    if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    SCE_REG(0x1CU) = 0x00001800U;
    SCE_REG(0xECU) = 0x00002c00U;
    SCE_REG(0x1D0U) = 0x00000000U;
    HW_SCE_func001(0xb9375233U, 0x86b67a72U, 0x6f0ee8f9U, 0x71ff9697U);
    if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
    SCE_REG(0x1CU) = 0x00400000U;
    SCE_REG(0x1D0U) = 0x00000000U;
    if (1U == (((SCE_REG(0x1CU) >> 22U) & 1U)))
    {
        HW_SCE_func003(0x48ed8c84U, 0xb2d01eceU, 0xa0661789U, 0xdbb74e30U);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        SCE_REG(0x1BCU) = 0x00000040U;
        /* WAIT_LOOP */
        if (!SceWait(0x18U, 12U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        return FSP_ERR_CRYPTO_SCE_RETRY;
    }
    else
    {
        /* WAIT_LOOP */
        if (!SceWait(0x64U, 11U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x64U) = 0x0000001cU;
        SCE_REG(0x104U) = 0x00000051U;
        SCE_REG(0xA4U) = 0x00200684U;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = change_endian_long(0x00000000U);
        SCE_REG(0xE0U) = 0x80040000U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0x00U) = 0x00008113U;
        /* WAIT_LOOP */
        if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x1CU) = 0x00001800U;
        SCE_REG(0xA4U) = 0xc0200c9dU;
        SCE_REG(0x00U) = 0x00001413U;
        /* WAIT_LOOP */
        if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x1CU) = 0x00001800U;
        /* WAIT_LOOP */
        if (!SceWait(0x64U, 11U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x64U) = 0x00000000U;
        SCE_REG(0xE0U) = 0x80040080U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0x00U) = 0x00008113U;
        /* WAIT_LOOP */
        if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x1CU) = 0x00001800U;
        HW_SCE_func001(0x8627d9acU, 0x6d2369bfU, 0x733c08b5U, 0x97487af9U);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        SCE_REG(0x104U) = 0x00000051U;
        SCE_REG(0xA4U) = 0x000b0804U;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = change_endian_long(0x00000000U);
        SCE_REG(0x104U) = 0x00000351U;
        SCE_REG(0xA4U) = 0x00010805U;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = change_endian_long(0x00000000U);
        SCE_REG(0x100U) = change_endian_long(0x00000000U);
        SCE_REG(0x100U) = change_endian_long(0x00000000U);
        SCE_REG(0x100U) = change_endian_long(0x00000001U);
        HW_SCE_func001(0x5a86b499U, 0x9e5788daU, 0x09eddf92U, 0xba75eec3U);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        SCE_REG(0xB0U) = 0x00000100U;
        SCE_REG(0xA4U) = 0x0020b6bfU;
        SCE_REG(0xE0U) = 0x81080000U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0x00U) = 0x00001823U;
        /* WAIT_LOOP */
        if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x1CU) = 0x00001800U;
        SCE_REG(0xA4U) = 0x000b0805U;
        SCE_REG(0x00U) = 0x00001113U;
        /* WAIT_LOOP */
        if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x1CU) = 0x00001800U;
        SCE_REG(0xA4U) = 0x00010805U;
        SCE_REG(0x00U) = 0x00001113U;
        /* WAIT_LOOP */
        if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x1CU) = 0x00001800U;
        HW_SCE_func001(0xa0a89032U, 0x36a7c63dU, 0x750cd4f0U, 0x8740312fU);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        SCE_REG(0x104U) = 0x00000251U;
        SCE_REG(0xA4U) = 0x0020363cU;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = change_endian_long(0x00000000U);
        SCE_REG(0xA4U) = 0x002036bcU;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = change_endian_long(0x00000000U);
        SCE_REG(0xA4U) = 0x002036bcU;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = change_endian_long(0x00000000U);
        SCE_REG(0xA4U) = 0x00060805U;
        SCE_REG(0x00U) = 0x00001113U;
        /* WAIT_LOOP */
        if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x1CU) = 0x00001800U;
        SCE_REG(0x104U) = 0x00000151U;
        SCE_REG(0xA4U) = 0x0021340cU;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = change_endian_long(0x00000000U);
        SCE_REG(0xA4U) = 0x000b0805U;
        SCE_REG(0x00U) = 0x00001113U;
        /* WAIT_LOOP */
        if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x1CU) = 0x00001800U;
        SCE_REG(0xA4U) = 0x01000684U;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = change_endian_long(0x00000000U);
        HW_SCE_func001(0x3788782dU, 0x0b828de3U, 0xbd8bcd51U, 0xbd9bc999U);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        SCE_REG(0xA4U) = 0x000d0805U;
        SCE_REG(0x00U) = 0x00001113U;
        /* WAIT_LOOP */
        if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x1CU) = 0x00001800U;
        HW_SCE_func001(0xbebbf494U, 0x95c03fa1U, 0x5ae6458cU, 0xc0016389U);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        SCE_REG(0x104U) = 0x00000251U;
        SCE_REG(0xA4U) = 0x0020363cU;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = change_endian_long(0x00000000U);
        SCE_REG(0xA4U) = 0x002036bcU;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = change_endian_long(0x00000000U);
        SCE_REG(0xA4U) = 0x002036bcU;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = change_endian_long(0x00000000U);
        SCE_REG(0xA4U) = 0x00060805U;
        SCE_REG(0x00U) = 0x00001113U;
        /* WAIT_LOOP */
        if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x1CU) = 0x00001800U;
        SCE_REG(0x104U) = 0x00000151U;
        SCE_REG(0xA4U) = 0x0021340cU;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = change_endian_long(0x00000000U);
        SCE_REG(0xA4U) = 0x000b0805U;
        SCE_REG(0x00U) = 0x00001113U;
        /* WAIT_LOOP */
        if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x1CU) = 0x00001800U;
        SCE_REG(0xA4U) = 0x01000684U;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = change_endian_long(0x00000000U);
        HW_SCE_func001(0xa6b90b54U, 0x61ab03a7U, 0x2ce6e649U, 0x19ed264bU);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        SCE_REG(0x04U) = 0x00000113U;
        /* WAIT_LOOP */
        if (!SceWait(0x04U, 30U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        S_RAM[4+0 + 0] = SCE_REG(0x100U);
        S_RAM[4+0 + 1] = SCE_REG(0x100U);
        S_RAM[4+0 + 2] = SCE_REG(0x100U);
        S_RAM[4+0 + 3] = SCE_REG(0x100U);
        SCE_REG(0x13CU) = 0x00000202U;
        HW_SCE_func003(0xe729aae7U, 0x00446087U, 0x8445ae02U, 0x63abd977U);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        SCE_REG(0x1BCU) = 0x00000040U;
        /* WAIT_LOOP */
        if (!SceWait(0x18U, 12U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        return FSP_SUCCESS;
    }
}

/***********************************************************************************************************************
End of function ./input_dir/HW_SCE_Sec_200408/200408/RA4M1/Cryptographic/HW_SCE_p02.prc
***********************************************************************************************************************/

// Adapted from hw_sce_p09.c
/*
* Copyright (c) 2020 - 2026 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/***********************************************************************************************************************
 * History : DD.MM.YYYY Version Description
 *         : 05.10.2020 1.00        First Release.
 *         : 02.12.2020 1.01        Added new functions such as the Brainpool curve.
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Imported global variables and functions (from other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Exported global variables (to be accessed by other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Private global variables and functions
***********************************************************************************************************************/

static fsp_err_t HW_SCE_GenerateRandomNumberSub(uint32_t *OutData_Text)
{
    uint32_t iLoop    = 0U;
    uint32_t iLoop1   = 0U;
    uint32_t iLoop2   = 0U;
    int32_t  jLoop    = 0U;
    uint32_t kLoop    = 0U;
    uint32_t oLoop    = 0U;
    uint32_t oLoop1   = 0U;
    uint32_t oLoop2   = 0U;
    uint32_t dummy    = 0U;
    uint32_t KEY_ADR  = 0U;
    uint32_t OFS_ADR  = 0U;
    uint32_t MAX_CNT2 = 0U;
    (void)iLoop;
    (void)iLoop1;
    (void)iLoop2;
    (void)jLoop;
    (void)kLoop;
    (void)oLoop;
    (void)oLoop1;
    (void)oLoop2;
    (void)dummy;
    (void)KEY_ADR;
    (void)OFS_ADR;
    (void)MAX_CNT2;
    if (0x0U != (SCE_REG(0x1BCU) & 0x1fU))
    {
        return FSP_ERR_CRYPTO_SCE_RESOURCE_CONFLICT;
    }
    SCE_REG(0x84U) = 0x00000901U;
    SCE_REG(0x108U) = 0x00000000U;
    HW_SCE_func001(0x3f063139U, 0xcb903ce7U, 0x4c5616a7U, 0x4970827aU);
    if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
    SCE_REG(0x104U) = 0x00000251U;
    SCE_REG(0xA4U) = 0x0020363cU;
    /* WAIT_LOOP */
    if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    SCE_REG(0x100U) = change_endian_long(0x00000000U);
    SCE_REG(0xA4U) = 0x002036bcU;
    /* WAIT_LOOP */
    if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    SCE_REG(0x100U) = change_endian_long(0x00000000U);
    SCE_REG(0xA4U) = 0x002036bcU;
    /* WAIT_LOOP */
    if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    SCE_REG(0x100U) = change_endian_long(0x00000000U);
    SCE_REG(0xA4U) = 0x00060805U;
    SCE_REG(0x00U) = 0x00001113U;
    /* WAIT_LOOP */
    if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    SCE_REG(0x1CU) = 0x00001800U;
    SCE_REG(0x104U) = 0x00000151U;
    SCE_REG(0xA4U) = 0x0021340cU;
    /* WAIT_LOOP */
    if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    SCE_REG(0x100U) = change_endian_long(0x00000000U);
    SCE_REG(0xA4U) = 0x000b0805U;
    SCE_REG(0x00U) = 0x00001113U;
    /* WAIT_LOOP */
    if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    SCE_REG(0x1CU) = 0x00001800U;
    SCE_REG(0xA4U) = 0x01000684U;
    /* WAIT_LOOP */
    if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    SCE_REG(0x100U) = change_endian_long(0x00000000U);
    HW_SCE_func001(0x19e279b0U, 0x849def2cU, 0xc9afeb32U, 0xc5e5e8c0U);
    if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
    SCE_REG(0x04U) = 0x00000112U;
    /* WAIT_LOOP */
    if (!SceWait(0x04U, 30U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    OutData_Text[0] = SCE_REG(0x100U);
    OutData_Text[1] = SCE_REG(0x100U);
    OutData_Text[2] = SCE_REG(0x100U);
    OutData_Text[3] = SCE_REG(0x100U);
    HW_SCE_func003(0x3471330bU, 0x4e06dd6cU, 0x6e2f6a05U, 0x1300bca9U);
    if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
    SCE_REG(0x1BCU) = 0x00000040U;
    /* WAIT_LOOP */
    if (!SceWait(0x18U, 12U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    return FSP_SUCCESS;
}

/***********************************************************************************************************************
End of function ./input_dir/HW_SCE_Sec_200408/200408/RA4M1/Cryptographic/HW_SCE_p09.prc
***********************************************************************************************************************/

// Adapted from hw_sce_p60.c
/*
* Copyright (c) 2020 - 2026 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/***********************************************************************************************************************
 * History : DD.MM.YYYY Version Description
 *         : 05.10.2020 1.00        First Release.
 *         : 02.12.2020 1.01        Added new functions such as the Brainpool curve.
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Imported global variables and functions (from other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Exported global variables (to be accessed by other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Private global variables and functions
***********************************************************************************************************************/

static fsp_err_t HW_SCE_Aes128EncryptDecryptInitSub (const uint32_t *InData_KeyType,
                                                 const uint32_t * InData_Cmd,
                                                 const uint32_t * InData_KeyIndex,
                                                 const uint32_t * InData_IV)
{
    uint32_t iLoop    = 0U;
    uint32_t iLoop1   = 0U;
    uint32_t iLoop2   = 0U;
    int32_t  jLoop    = 0U;
    uint32_t kLoop    = 0U;
    uint32_t oLoop    = 0U;
    uint32_t oLoop1   = 0U;
    uint32_t oLoop2   = 0U;
    uint32_t dummy    = 0U;
    uint32_t KEY_ADR  = 0U;
    uint32_t OFS_ADR  = 0U;
    uint32_t MAX_CNT2 = 0U;
    (void)iLoop;
    (void)iLoop1;
    (void)iLoop2;
    (void)jLoop;
    (void)kLoop;
    (void)oLoop;
    (void)oLoop1;
    (void)oLoop2;
    (void)dummy;
    (void)KEY_ADR;
    (void)OFS_ADR;
    (void)MAX_CNT2;
    (void)InData_KeyType;
    if (0x0U != (SCE_REG(0x1BCU) & 0x1fU))
    {
        return FSP_ERR_CRYPTO_SCE_RESOURCE_CONFLICT;
    }
    SCE_REG(0x84U) = 0x00006001U;
    SCE_REG(0x108U) = 0x00000000U;
    HW_SCE_func001(0xed19ab94U, 0x93c9c505U, 0x1d873666U, 0x20ab5f24U);
    if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
    SCE_REG(0xA4U) = 0x600c3a0dU;
    /* WAIT_LOOP */
    if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    SCE_REG(0x100U) = InData_KeyIndex[0];
    SCE_REG(0x100U) = InData_KeyIndex[1];
    SCE_REG(0x100U) = InData_KeyIndex[2];
    SCE_REG(0x100U) = InData_KeyIndex[3];
    SCE_REG(0xA4U) = 0x400c0a0cU;
    /* WAIT_LOOP */
    if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    SCE_REG(0x100U) = change_endian_long(0xef138998U);
    SCE_REG(0xA4U) = 0x42f8063dU;
    /* WAIT_LOOP */
    if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    SCE_REG(0x100U) = InData_KeyIndex[4];
    SCE_REG(0x100U) = InData_KeyIndex[5];
    SCE_REG(0x100U) = InData_KeyIndex[6];
    SCE_REG(0x100U) = InData_KeyIndex[7];
    SCE_REG(0xA4U) = 0x400006bdU;
    /* WAIT_LOOP */
    if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    SCE_REG(0x100U) = InData_KeyIndex[8];
    SCE_REG(0x100U) = InData_KeyIndex[9];
    SCE_REG(0x100U) = InData_KeyIndex[10];
    SCE_REG(0x100U) = InData_KeyIndex[11];
    SCE_REG(0xA4U) = 0x00900c45U;
    SCE_REG(0x00U) = 0x00001113U;
    /* WAIT_LOOP */
    if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    SCE_REG(0x1CU) = 0x00001800U;
    HW_SCE_func001(0x136006ddU, 0x08a23e9dU, 0x63289167U, 0x1a14175dU);
    if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
    SCE_REG(0x1CU) = 0x00400000U;
    SCE_REG(0x1D0U) = 0x00000000U;
    if (1U == (((SCE_REG(0x1CU) >> 22U) & 1U)))
    {
        HW_SCE_func003(0xb48f6d49U, 0xf6c5502dU, 0x36addd72U, 0xc7209976U);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        SCE_REG(0x1BCU) = 0x00000040U;
        /* WAIT_LOOP */
        if (!SceWait(0x18U, 12U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        return FSP_ERR_CRYPTO_SCE_KEY_SET_FAIL;
    }
    else
    {
        HW_SCE_func001(0x1329ae05U, 0xb174626dU, 0xf42fe21bU, 0x66e9d647U);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        SCE_REG(0xE0U) = 0x80010000U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0x104U) = 0x00000068U;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = InData_Cmd[0];
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0xECU) = 0x3000a800U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0xECU) = 0x00000004U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0xECU) = 0x00010020U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0xECU) = 0x0000b400U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0xECU) = 0x00000004U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0xECU) = 0x00000080U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0xECU) = 0x00007c00U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0x1CU) = 0x00600000U;
        SCE_REG(0x1D0U) = 0x00000000U;
        if (0x00000000U == (SCE_REG(0x1CU) & 0xff000000U))
        {
            HW_SCE_func002(0x56a4bb4eU, 0x2c0f47d0U, 0x2241eef6U, 0x6234eac4U);
            if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        }
        else if (0x01000000U == (SCE_REG(0x1CU) & 0xff000000U))
        {
            HW_SCE_func002(0xd0665c19U, 0x04772908U, 0xad4d20a4U, 0xe41fde9aU);
            if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        }
        else if (0x02000000U == (SCE_REG(0x1CU) & 0xff000000U))
        {
            SCE_REG(0x104U) = 0x00000361U;
            SCE_REG(0xA4U) = 0x00040805U;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = InData_IV[0];
            SCE_REG(0x100U) = InData_IV[1];
            SCE_REG(0x100U) = InData_IV[2];
            SCE_REG(0x100U) = InData_IV[3];
            HW_SCE_func002(0xcb399d18U, 0x4323efb7U, 0x1a22dfa6U, 0xf473e241U);
            if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        }
        else if (0x03000000U == (SCE_REG(0x1CU) & 0xff000000U))
        {
            SCE_REG(0x104U) = 0x00000361U;
            SCE_REG(0xA4U) = 0x00040805U;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = InData_IV[0];
            SCE_REG(0x100U) = InData_IV[1];
            SCE_REG(0x100U) = InData_IV[2];
            SCE_REG(0x100U) = InData_IV[3];
            HW_SCE_func002(0x2e60444bU, 0x7c03b08bU, 0xe6c02ef7U, 0xde95dca4U);
            if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        }
        else if (0x04000000U == (SCE_REG(0x1CU) & 0xff000000U))
        {
            SCE_REG(0x104U) = 0x00000361U;
            SCE_REG(0xA4U) = 0x00040805U;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = InData_IV[0];
            SCE_REG(0x100U) = InData_IV[1];
            SCE_REG(0x100U) = InData_IV[2];
            SCE_REG(0x100U) = InData_IV[3];
            HW_SCE_func002(0x6d7ec53dU, 0xf03ac2a7U, 0xb47c9921U, 0x2cc5b591U);
            if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        }
        return FSP_SUCCESS;
    }
}

/***********************************************************************************************************************
End of function ./input_dir/HW_SCE_Sec_200408/200408/RA4M1/Cryptographic/HW_SCE_p60.prc
***********************************************************************************************************************/

// Adapted from hw_sce_p61.c
/*
* Copyright (c) 2020 - 2026 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/***********************************************************************************************************************
 * History : DD.MM.YYYY Version Description
 *         : 05.10.2020 1.00        First Release.
 *         : 02.12.2020 1.01        Added new functions such as the Brainpool curve.
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Imported global variables and functions (from other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Exported global variables (to be accessed by other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Private global variables and functions
***********************************************************************************************************************/

static void HW_SCE_Aes128EncryptDecryptUpdateSub (const uint32_t * InData_Text, uint32_t * OutData_Text,
                                           const uint32_t MAX_CNT)
{
    uint32_t iLoop    = 0U;
    uint32_t iLoop1   = 0U;
    uint32_t iLoop2   = 0U;
    int32_t  jLoop    = 0U;
    uint32_t kLoop    = 0U;
    uint32_t oLoop    = 0U;
    uint32_t oLoop1   = 0U;
    uint32_t oLoop2   = 0U;
    uint32_t dummy    = 0U;
    uint32_t KEY_ADR  = 0U;
    uint32_t OFS_ADR  = 0U;
    uint32_t MAX_CNT2 = 0U;
    (void)iLoop;
    (void)iLoop1;
    (void)iLoop2;
    (void)jLoop;
    (void)kLoop;
    (void)oLoop;
    (void)oLoop1;
    (void)oLoop2;
    (void)dummy;
    (void)KEY_ADR;
    (void)OFS_ADR;
    (void)MAX_CNT2;
    if (0x00000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        SCE_REG(0x104U) = 0x00000361U;
        HW_SCE_func001(0xc099fa80U, 0x55f8d7e7U, 0xb1b81cd1U, 0xe45dc05cU);
        if (s_Fault) { return; }
        SCE_REG(0x104U) = 0x000000b1U;
        SCE_REG(0xA4U) = 0x0000088eU;
        SCE_REG(0x04U) = 0x0000c100U;
    }
    else if (0x01000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        SCE_REG(0x104U) = 0x00000361U;
        HW_SCE_func001(0xc4afbe43U, 0xfbe23476U, 0x3869f8fbU, 0xa1ddf110U);
        if (s_Fault) { return; }
        SCE_REG(0x104U) = 0x000000b1U;
        SCE_REG(0xA4U) = 0x0000488eU;
        SCE_REG(0x04U) = 0x0000c100U;
    }
    else if (0x02000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        SCE_REG(0x104U) = 0x00000361U;
        HW_SCE_func001(0x222a7ebeU, 0xfa5080beU, 0x9dc5000cU, 0x2cb50d11U);
        if (s_Fault) { return; }
        SCE_REG(0x104U) = 0x000000b1U;
        SCE_REG(0xA4U) = 0x00000c9eU;
        SCE_REG(0x04U) = 0x0000c100U;
    }
    else if (0x03000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        SCE_REG(0x104U) = 0x00000361U;
        HW_SCE_func001(0x96b0d912U, 0x5f0ba5e6U, 0x3ffcce56U, 0xbd9481eeU);
        if (s_Fault) { return; }
        SCE_REG(0x104U) = 0x000000b1U;
        SCE_REG(0xA4U) = 0x000049aeU;
        SCE_REG(0x04U) = 0x0000c100U;
    }
    else if (0x04000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        SCE_REG(0x104U) = 0x00000361U;
        HW_SCE_func001(0xec8fa4d2U, 0x1a6dc8d2U, 0x4067dba7U, 0x266ae636U);
        if (s_Fault) { return; }
        SCE_REG(0x104U) = 0x000000b1U;
        SCE_REG(0xA4U) = 0x000006beU;
        SCE_REG(0x04U) = 0x0000c100U;
    }
    /* WAIT_LOOP */
    if (!SceWait(0x104U, 31U, 1U))
    {
        return;
    }
    SCE_REG(0x100U) = InData_Text[0];
    SCE_REG(0x100U) = InData_Text[1];
    SCE_REG(0x100U) = InData_Text[2];
    SCE_REG(0x100U) = InData_Text[3];
    for (iLoop = 4; iLoop < MAX_CNT ; iLoop = iLoop + 4)
    {
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return;
    }
        SCE_REG(0x100U) = InData_Text[iLoop + 0];
        SCE_REG(0x100U) = InData_Text[iLoop + 1];
        SCE_REG(0x100U) = InData_Text[iLoop + 2];
        SCE_REG(0x100U) = InData_Text[iLoop + 3];
        /* WAIT_LOOP */
        if (!SceWait(0x04U, 30U, 1U))
    {
        return;
    }
        OutData_Text[iLoop-4 + 0] = SCE_REG(0x100U);
        OutData_Text[iLoop-4 + 1] = SCE_REG(0x100U);
        OutData_Text[iLoop-4 + 2] = SCE_REG(0x100U);
        OutData_Text[iLoop-4 + 3] = SCE_REG(0x100U);
    }
    /* WAIT_LOOP */
    if (!SceWait(0x04U, 30U, 1U))
    {
        return;
    }
    OutData_Text[MAX_CNT-4 + 0] = SCE_REG(0x100U);
    OutData_Text[MAX_CNT-4 + 1] = SCE_REG(0x100U);
    OutData_Text[MAX_CNT-4 + 2] = SCE_REG(0x100U);
    OutData_Text[MAX_CNT-4 + 3] = SCE_REG(0x100U);
    if (0x00000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        HW_SCE_func206();
        HW_SCE_func002(0x6d31fdbeU, 0xf4147098U, 0x7a374888U, 0xbb9e479cU);
        if (s_Fault) { return; }
    }
    else if (0x01000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        HW_SCE_func206();
        HW_SCE_func002(0x797ba3d2U, 0x4de21909U, 0x07e3a19dU, 0xb981a166U);
        if (s_Fault) { return; }
    }
    else if (0x02000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        HW_SCE_func206();
        HW_SCE_func002(0x2debffd0U, 0x75734739U, 0xf9eabd2dU, 0x004be10eU);
        if (s_Fault) { return; }
    }
    else if (0x03000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        HW_SCE_func206();
        HW_SCE_func002(0x4cfe8eb3U, 0xb71cf565U, 0x8714d05fU, 0x093a1f3dU);
        if (s_Fault) { return; }
    }
    else if (0x04000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        HW_SCE_func206();
        HW_SCE_func002(0xa919894bU, 0x90f11cf5U, 0x3cdb87c5U, 0x8882d91fU);
        if (s_Fault) { return; }
    }
}

/***********************************************************************************************************************
End of function ./input_dir/HW_SCE_Sec_200408/200408/RA4M1/Cryptographic/HW_SCE_p61.prc
***********************************************************************************************************************/

// Adapted from hw_sce_p62.c
/*
* Copyright (c) 2020 - 2026 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/***********************************************************************************************************************
 * History : DD.MM.YYYY Version Description
 *         : 05.10.2020 1.00        First Release.
 *         : 02.12.2020 1.01        Added new functions such as the Brainpool curve.
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Imported global variables and functions (from other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Exported global variables (to be accessed by other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Private global variables and functions
***********************************************************************************************************************/

static fsp_err_t HW_SCE_Aes128EncryptDecryptFinalSub(void)
{
    uint32_t iLoop    = 0U;
    uint32_t iLoop1   = 0U;
    uint32_t iLoop2   = 0U;
    int32_t  jLoop    = 0U;
    uint32_t kLoop    = 0U;
    uint32_t oLoop    = 0U;
    uint32_t oLoop1   = 0U;
    uint32_t oLoop2   = 0U;
    uint32_t dummy    = 0U;
    uint32_t KEY_ADR  = 0U;
    uint32_t OFS_ADR  = 0U;
    uint32_t MAX_CNT2 = 0U;
    (void)iLoop;
    (void)iLoop1;
    (void)iLoop2;
    (void)jLoop;
    (void)kLoop;
    (void)oLoop;
    (void)oLoop1;
    (void)oLoop2;
    (void)dummy;
    (void)KEY_ADR;
    (void)OFS_ADR;
    (void)MAX_CNT2;
    if (0x00000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        HW_SCE_func002(0x04806bcfU, 0xb5854414U, 0xfbf66061U, 0xa4c71fc9U);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
    }
    else if (0x01000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        HW_SCE_func002(0x3784ca5eU, 0x302e119eU, 0xa3fc989cU, 0x09811f45U);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
    }
    else if (0x02000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        HW_SCE_func002(0x087df25eU, 0x779ae4feU, 0xe50d522eU, 0x3dbe8185U);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
    }
    else if (0x03000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        HW_SCE_func002(0xe40057a3U, 0xead57b67U, 0xc952cde2U, 0x9d709058U);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
    }
    else if (0x04000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        HW_SCE_func002(0xf9c9f3f5U, 0xc7667faeU, 0xf459287cU, 0xfa7570ccU);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
    }
    HW_SCE_func001(0x69c4bcb1U, 0xb00dc893U, 0x4623fdf8U, 0x6e8dd8dbU);
    if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
    SCE_REG(0x1CU) = 0x00400000U;
    SCE_REG(0x1D0U) = 0x00000000U;
    if (1U == (((SCE_REG(0x1CU) >> 22U) & 1U)))
    {
        SCE_REG(0x1CU) = 0x002d0000U;
        HW_SCE_func001(0x52d2dcbbU, 0x678c2567U, 0x797c8252U, 0xdf776986U);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        SCE_REG(0x1CU) = 0x00400000U;
        SCE_REG(0x1D0U) = 0x00000000U;
        if (1U == (((SCE_REG(0x1CU) >> 22U) & 1U)))
        {
            SCE_REG(0x00U) = 0x00000113U;
            /* WAIT_LOOP */
            if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x1CU) = 0x00001800U;
            HW_SCE_func002(0xca3b5acfU, 0x43d40531U, 0x1bf0b353U, 0xd6c5b6cdU);
            if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        }
        else
        {
            HW_SCE_func002(0xfcb4c6feU, 0x5d137a36U, 0xf872aea4U, 0xa77ebf44U);
            if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        }
        HW_SCE_func003(0x05a69693U, 0x5a5507b3U, 0xbceaeed1U, 0x4b659931U);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        SCE_REG(0x1BCU) = 0x00000040U;
        /* WAIT_LOOP */
        if (!SceWait(0x18U, 12U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    else
    {
        HW_SCE_func003(0xc4af11caU, 0xbdaa0ea4U, 0xd888cdb6U, 0xe9083d8aU);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        SCE_REG(0x1BCU) = 0x00000040U;
        /* WAIT_LOOP */
        if (!SceWait(0x18U, 12U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        return FSP_SUCCESS;
    }
}

/***********************************************************************************************************************
End of function ./input_dir/HW_SCE_Sec_200408/200408/RA4M1/Cryptographic/HW_SCE_p62.prc
***********************************************************************************************************************/

// Adapted from hw_sce_p63.c
/*
* Copyright (c) 2020 - 2026 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/***********************************************************************************************************************
 * History : DD.MM.YYYY Version Description
 *         : 05.10.2020 1.00        First Release.
 *         : 02.12.2020 1.01        Added new functions such as the Brainpool curve.
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Imported global variables and functions (from other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Exported global variables (to be accessed by other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Private global variables and functions
***********************************************************************************************************************/

static fsp_err_t HW_SCE_Aes256EncryptDecryptInitSub (const uint32_t *InData_KeyType,
                                                 const uint32_t * InData_Cmd,
                                                 const uint32_t * InData_KeyIndex,
                                                 const uint32_t * InData_IV)
{
    uint32_t iLoop    = 0U;
    uint32_t iLoop1   = 0U;
    uint32_t iLoop2   = 0U;
    int32_t  jLoop    = 0U;
    uint32_t kLoop    = 0U;
    uint32_t oLoop    = 0U;
    uint32_t oLoop1   = 0U;
    uint32_t oLoop2   = 0U;
    uint32_t dummy    = 0U;
    uint32_t KEY_ADR  = 0U;
    uint32_t OFS_ADR  = 0U;
    uint32_t MAX_CNT2 = 0U;
    (void)iLoop;
    (void)iLoop1;
    (void)iLoop2;
    (void)jLoop;
    (void)kLoop;
    (void)oLoop;
    (void)oLoop1;
    (void)oLoop2;
    (void)dummy;
    (void)KEY_ADR;
    (void)OFS_ADR;
    (void)MAX_CNT2;
    (void)InData_KeyType;
    if (0x0U != (SCE_REG(0x1BCU) & 0x1fU))
    {
        return FSP_ERR_CRYPTO_SCE_RESOURCE_CONFLICT;
    }
    SCE_REG(0x84U) = 0x00006301U;
    SCE_REG(0x108U) = 0x00000000U;
    HW_SCE_func001(0x1181b7efU, 0x556d5729U, 0x8e42ba80U, 0xfc72400aU);
    if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
    SCE_REG(0xA4U) = 0x600c3a0dU;
    /* WAIT_LOOP */
    if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    SCE_REG(0x100U) = InData_KeyIndex[0];
    SCE_REG(0x100U) = InData_KeyIndex[1];
    SCE_REG(0x100U) = InData_KeyIndex[2];
    SCE_REG(0x100U) = InData_KeyIndex[3];
    SCE_REG(0xA4U) = 0x400c0a0cU;
    /* WAIT_LOOP */
    if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    SCE_REG(0x100U) = change_endian_long(0x213f32e4U);
    SCE_REG(0xA4U) = 0x42f8063dU;
    /* WAIT_LOOP */
    if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    SCE_REG(0x100U) = InData_KeyIndex[4];
    SCE_REG(0x100U) = InData_KeyIndex[5];
    SCE_REG(0x100U) = InData_KeyIndex[6];
    SCE_REG(0x100U) = InData_KeyIndex[7];
    HW_SCE_func001(0x0f6f0746U, 0x8225ff4bU, 0x73e11648U, 0x142bdf5eU);
    if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
    SCE_REG(0xA4U) = 0x40f9063dU;
    /* WAIT_LOOP */
    if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    SCE_REG(0x100U) = InData_KeyIndex[8];
    SCE_REG(0x100U) = InData_KeyIndex[9];
    SCE_REG(0x100U) = InData_KeyIndex[10];
    SCE_REG(0x100U) = InData_KeyIndex[11];
    SCE_REG(0xA4U) = 0x400006bdU;
    /* WAIT_LOOP */
    if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    SCE_REG(0x100U) = InData_KeyIndex[12];
    SCE_REG(0x100U) = InData_KeyIndex[13];
    SCE_REG(0x100U) = InData_KeyIndex[14];
    SCE_REG(0x100U) = InData_KeyIndex[15];
    SCE_REG(0xA4U) = 0x00900c45U;
    SCE_REG(0x00U) = 0x00001113U;
    /* WAIT_LOOP */
    if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    SCE_REG(0x1CU) = 0x00001800U;
    HW_SCE_func001(0xa2b7f698U, 0x3fc26c77U, 0xf118c4a2U, 0x7f6a6cb7U);
    if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
    SCE_REG(0x1CU) = 0x00400000U;
    SCE_REG(0x1D0U) = 0x00000000U;
    if (1U == (((SCE_REG(0x1CU) >> 22U) & 1U)))
    {
        HW_SCE_func003(0x5c586ce9U, 0xa51ddc06U, 0xff572e79U, 0x132f7c1dU);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        SCE_REG(0x1BCU) = 0x00000040U;
        /* WAIT_LOOP */
        if (!SceWait(0x18U, 12U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        return FSP_ERR_CRYPTO_SCE_KEY_SET_FAIL;
    }
    else
    {
        HW_SCE_func001(0x6aaef7e4U, 0x64119651U, 0xa6b9e1aaU, 0xbe50041cU);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        SCE_REG(0xE0U) = 0x80010000U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0x104U) = 0x00000068U;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = InData_Cmd[0];
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0xECU) = 0x3000a800U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0xECU) = 0x00000004U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0xECU) = 0x00010020U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0xECU) = 0x0000b400U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0xECU) = 0x00000004U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0xECU) = 0x00000080U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0xECU) = 0x00007c00U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0x1CU) = 0x00600000U;
        SCE_REG(0x1D0U) = 0x00000000U;
        if (0x00000000U == (SCE_REG(0x1CU) & 0xff000000U))
        {
            HW_SCE_func002(0x68565bc4U, 0x5f1eb3deU, 0xfc951fb7U, 0xd93bf4eeU);
            if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        }
        else if (0x01000000U == (SCE_REG(0x1CU) & 0xff000000U))
        {
            HW_SCE_func002(0x7da76e5aU, 0xc1494e81U, 0xd6661801U, 0x5e0ec87cU);
            if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        }
        else if (0x02000000U == (SCE_REG(0x1CU) & 0xff000000U))
        {
            SCE_REG(0x104U) = 0x00000361U;
            SCE_REG(0xA4U) = 0x00040805U;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = InData_IV[0];
            SCE_REG(0x100U) = InData_IV[1];
            SCE_REG(0x100U) = InData_IV[2];
            SCE_REG(0x100U) = InData_IV[3];
            HW_SCE_func002(0x70291ae4U, 0xcf68385aU, 0x7af0b727U, 0xffb9508eU);
            if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        }
        else if (0x03000000U == (SCE_REG(0x1CU) & 0xff000000U))
        {
            SCE_REG(0x104U) = 0x00000361U;
            SCE_REG(0xA4U) = 0x00040805U;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = InData_IV[0];
            SCE_REG(0x100U) = InData_IV[1];
            SCE_REG(0x100U) = InData_IV[2];
            SCE_REG(0x100U) = InData_IV[3];
            HW_SCE_func002(0x2a2948b6U, 0xd36e6ccdU, 0x1ed0187eU, 0x2772eba4U);
            if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        }
        else if (0x04000000U == (SCE_REG(0x1CU) & 0xff000000U))
        {
            SCE_REG(0x104U) = 0x00000361U;
            SCE_REG(0xA4U) = 0x00040805U;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = InData_IV[0];
            SCE_REG(0x100U) = InData_IV[1];
            SCE_REG(0x100U) = InData_IV[2];
            SCE_REG(0x100U) = InData_IV[3];
            HW_SCE_func002(0xc5a7b7ebU, 0x9b24214bU, 0xe12cdb8fU, 0x16062e27U);
            if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        }
        return FSP_SUCCESS;
    }
}

/***********************************************************************************************************************
End of function ./input_dir/HW_SCE_Sec_200408/200408/RA4M1/Cryptographic/HW_SCE_p63.prc
***********************************************************************************************************************/

// Adapted from hw_sce_p64.c
/*
* Copyright (c) 2020 - 2026 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/***********************************************************************************************************************
 * History : DD.MM.YYYY Version Description
 *         : 05.10.2020 1.00        First Release.
 *         : 02.12.2020 1.01        Added new functions such as the Brainpool curve.
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Imported global variables and functions (from other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Exported global variables (to be accessed by other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Private global variables and functions
***********************************************************************************************************************/

static void HW_SCE_Aes256EncryptDecryptUpdateSub (const uint32_t * InData_Text, uint32_t * OutData_Text,
                                           const uint32_t MAX_CNT)
{
    uint32_t iLoop    = 0U;
    uint32_t iLoop1   = 0U;
    uint32_t iLoop2   = 0U;
    int32_t  jLoop    = 0U;
    uint32_t kLoop    = 0U;
    uint32_t oLoop    = 0U;
    uint32_t oLoop1   = 0U;
    uint32_t oLoop2   = 0U;
    uint32_t dummy    = 0U;
    uint32_t KEY_ADR  = 0U;
    uint32_t OFS_ADR  = 0U;
    uint32_t MAX_CNT2 = 0U;
    (void)iLoop;
    (void)iLoop1;
    (void)iLoop2;
    (void)jLoop;
    (void)kLoop;
    (void)oLoop;
    (void)oLoop1;
    (void)oLoop2;
    (void)dummy;
    (void)KEY_ADR;
    (void)OFS_ADR;
    (void)MAX_CNT2;
    if (0x00000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        SCE_REG(0x104U) = 0x00000361U;
        HW_SCE_func001(0x624aadcbU, 0xe50f8155U, 0x4711c39cU, 0xfd2ef7b1U);
        if (s_Fault) { return; }
        SCE_REG(0x104U) = 0x000000b1U;
        SCE_REG(0xB0U) = 0x40000000U;
        SCE_REG(0xA4U) = 0x0000888eU;
        SCE_REG(0x04U) = 0x0000c100U;
    }
    else if (0x01000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        SCE_REG(0x104U) = 0x00000361U;
        HW_SCE_func001(0xc4422ed7U, 0x384f3f12U, 0x38eb866fU, 0x68b545e7U);
        if (s_Fault) { return; }
        SCE_REG(0x104U) = 0x000000b1U;
        SCE_REG(0xB0U) = 0x40000000U;
        SCE_REG(0xA4U) = 0x0000c88eU;
        SCE_REG(0x04U) = 0x0000c100U;
    }
    else if (0x02000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        SCE_REG(0x104U) = 0x00000361U;
        HW_SCE_func001(0x32d07d47U, 0x3dd04bfbU, 0x9032f3fdU, 0x62191c6eU);
        if (s_Fault) { return; }
        SCE_REG(0x104U) = 0x000000b1U;
        SCE_REG(0xB0U) = 0x40000000U;
        SCE_REG(0xA4U) = 0x00008c9eU;
        SCE_REG(0x04U) = 0x0000c100U;
    }
    else if (0x03000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        SCE_REG(0x104U) = 0x00000361U;
        HW_SCE_func001(0x99a6263aU, 0xe91d3aa8U, 0x104e62bcU, 0xcd57cfa6U);
        if (s_Fault) { return; }
        SCE_REG(0x104U) = 0x000000b1U;
        SCE_REG(0xB0U) = 0x40000000U;
        SCE_REG(0xA4U) = 0x0000c9aeU;
        SCE_REG(0x04U) = 0x0000c100U;
    }
    else if (0x04000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        SCE_REG(0x104U) = 0x00000361U;
        HW_SCE_func001(0x5dffe167U, 0x69f359d3U, 0xb3fa7680U, 0x8792a6a3U);
        if (s_Fault) { return; }
        SCE_REG(0x104U) = 0x000000b1U;
        SCE_REG(0xB0U) = 0x40000000U;
        SCE_REG(0xA4U) = 0x000086beU;
        SCE_REG(0x04U) = 0x0000c100U;
    }
    /* WAIT_LOOP */
    if (!SceWait(0x104U, 31U, 1U))
    {
        return;
    }
    SCE_REG(0x100U) = InData_Text[0];
    SCE_REG(0x100U) = InData_Text[1];
    SCE_REG(0x100U) = InData_Text[2];
    SCE_REG(0x100U) = InData_Text[3];
    for (iLoop = 4; iLoop < MAX_CNT ; iLoop = iLoop + 4)
    {
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return;
    }
        SCE_REG(0x100U) = InData_Text[iLoop + 0];
        SCE_REG(0x100U) = InData_Text[iLoop + 1];
        SCE_REG(0x100U) = InData_Text[iLoop + 2];
        SCE_REG(0x100U) = InData_Text[iLoop + 3];
        /* WAIT_LOOP */
        if (!SceWait(0x04U, 30U, 1U))
    {
        return;
    }
        OutData_Text[iLoop-4 + 0] = SCE_REG(0x100U);
        OutData_Text[iLoop-4 + 1] = SCE_REG(0x100U);
        OutData_Text[iLoop-4 + 2] = SCE_REG(0x100U);
        OutData_Text[iLoop-4 + 3] = SCE_REG(0x100U);
    }
    /* WAIT_LOOP */
    if (!SceWait(0x04U, 30U, 1U))
    {
        return;
    }
    OutData_Text[MAX_CNT-4 + 0] = SCE_REG(0x100U);
    OutData_Text[MAX_CNT-4 + 1] = SCE_REG(0x100U);
    OutData_Text[MAX_CNT-4 + 2] = SCE_REG(0x100U);
    OutData_Text[MAX_CNT-4 + 3] = SCE_REG(0x100U);
    if (0x00000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        HW_SCE_func206();
        HW_SCE_func002(0xff48cf0eU, 0xc56d4fb1U, 0x690b30c9U, 0xec43e750U);
        if (s_Fault) { return; }
    }
    else if (0x01000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        HW_SCE_func206();
        HW_SCE_func002(0xd3fc3930U, 0x1ad2ed42U, 0xaf63893cU, 0x7cf9d506U);
        if (s_Fault) { return; }
    }
    else if (0x02000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        HW_SCE_func206();
        HW_SCE_func002(0xfcb04506U, 0xdfd7fbebU, 0xac25a3fbU, 0x3025878eU);
        if (s_Fault) { return; }
    }
    else if (0x03000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        HW_SCE_func206();
        HW_SCE_func002(0xe54e9a91U, 0x5cf3e032U, 0x7fd66d73U, 0x2b438f10U);
        if (s_Fault) { return; }
    }
    else if (0x04000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        HW_SCE_func206();
        HW_SCE_func002(0x144f6b8bU, 0x0696d7b6U, 0x13c8804cU, 0x40bbcfbdU);
        if (s_Fault) { return; }
    }
}

/***********************************************************************************************************************
End of function ./input_dir/HW_SCE_Sec_200408/200408/RA4M1/Cryptographic/HW_SCE_p64.prc
***********************************************************************************************************************/

// Adapted from hw_sce_p65.c
/*
* Copyright (c) 2020 - 2026 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/***********************************************************************************************************************
 * History : DD.MM.YYYY Version Description
 *         : 05.10.2020 1.00        First Release.
 *         : 02.12.2020 1.01        Added new functions such as the Brainpool curve.
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Imported global variables and functions (from other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Exported global variables (to be accessed by other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Private global variables and functions
***********************************************************************************************************************/

static fsp_err_t HW_SCE_Aes256EncryptDecryptFinalSub(void)
{
    uint32_t iLoop    = 0U;
    uint32_t iLoop1   = 0U;
    uint32_t iLoop2   = 0U;
    int32_t  jLoop    = 0U;
    uint32_t kLoop    = 0U;
    uint32_t oLoop    = 0U;
    uint32_t oLoop1   = 0U;
    uint32_t oLoop2   = 0U;
    uint32_t dummy    = 0U;
    uint32_t KEY_ADR  = 0U;
    uint32_t OFS_ADR  = 0U;
    uint32_t MAX_CNT2 = 0U;
    (void)iLoop;
    (void)iLoop1;
    (void)iLoop2;
    (void)jLoop;
    (void)kLoop;
    (void)oLoop;
    (void)oLoop1;
    (void)oLoop2;
    (void)dummy;
    (void)KEY_ADR;
    (void)OFS_ADR;
    (void)MAX_CNT2;
    if (0x00000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        HW_SCE_func002(0x877ac003U, 0x1f55a80eU, 0x56f67726U, 0xb2fb1b27U);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
    }
    else if (0x01000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        HW_SCE_func002(0x020800e5U, 0x3739cd04U, 0x46c9610cU, 0xafb0bc84U);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
    }
    else if (0x02000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        HW_SCE_func002(0xba294c51U, 0x14afd546U, 0x1c1f8d84U, 0xb8b3e317U);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
    }
    else if (0x03000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        HW_SCE_func002(0x93006859U, 0x575f3d59U, 0xc776b5e7U, 0xd92e9ecbU);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
    }
    else if (0x04000000U == (SCE_REG(0x1CU) & 0xff000000U))
    {
        HW_SCE_func002(0xc74f2e16U, 0x4e88787cU, 0x52b4731cU, 0x6d9858d0U);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
    }
    HW_SCE_func001(0x0fd7d09bU, 0xa7b8d27cU, 0x46c75262U, 0x8075b7beU);
    if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
    SCE_REG(0x1CU) = 0x00400000U;
    SCE_REG(0x1D0U) = 0x00000000U;
    if (1U == (((SCE_REG(0x1CU) >> 22U) & 1U)))
    {
        SCE_REG(0x1CU) = 0x002d0000U;
        HW_SCE_func001(0x4037084eU, 0xff71ad85U, 0x1b8b2e00U, 0x99824528U);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        SCE_REG(0x1CU) = 0x00400000U;
        SCE_REG(0x1D0U) = 0x00000000U;
        if (1U == (((SCE_REG(0x1CU) >> 22U) & 1U)))
        {
            SCE_REG(0x00U) = 0x00000113U;
            /* WAIT_LOOP */
            if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x1CU) = 0x00001800U;
            HW_SCE_func002(0x050a08feU, 0x4fc6390eU, 0xa111da36U, 0x975d32a8U);
            if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        }
        else
        {
            HW_SCE_func002(0x0d3b9483U, 0x0aa68236U, 0x866fe1a6U, 0xee77a58dU);
            if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        }
        HW_SCE_func003(0xcd084c94U, 0xd2324d31U, 0x5dd83952U, 0xcebaf319U);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        SCE_REG(0x1BCU) = 0x00000040U;
        /* WAIT_LOOP */
        if (!SceWait(0x18U, 12U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    else
    {
        HW_SCE_func003(0x35ca3d10U, 0xbe886693U, 0xae5873e3U, 0x2477eafbU);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        SCE_REG(0x1BCU) = 0x00000040U;
        /* WAIT_LOOP */
        if (!SceWait(0x18U, 12U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        return FSP_SUCCESS;
    }
}

/***********************************************************************************************************************
End of function ./input_dir/HW_SCE_Sec_200408/200408/RA4M1/Cryptographic/HW_SCE_p65.prc
***********************************************************************************************************************/

// Adapted from hw_sce_p84.c
/*
* Copyright (c) 2020 - 2026 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/***********************************************************************************************************************
 * History : DD.MM.YYYY Version Description
 *         : 05.10.2020 1.00        First Release.
 *         : 02.12.2020 1.01        Added new functions such as the Brainpool curve.
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Imported global variables and functions (from other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Exported global variables (to be accessed by other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Private global variables and functions
***********************************************************************************************************************/

static fsp_err_t HW_SCE_GenerateAes128PlainKeyIndexSub(uint32_t *InData_KeyType, uint32_t *InData_SharedKeyIndex, uint32_t *InData_SessionKey, uint32_t *InData_IV, uint32_t *InData_InstData, uint32_t *OutData_KeyIndex)
{
    uint32_t iLoop    = 0U;
    uint32_t iLoop1   = 0U;
    uint32_t iLoop2   = 0U;
    int32_t  jLoop    = 0U;
    uint32_t kLoop    = 0U;
    uint32_t oLoop    = 0U;
    uint32_t oLoop1   = 0U;
    uint32_t oLoop2   = 0U;
    uint32_t dummy    = 0U;
    uint32_t KEY_ADR  = 0U;
    uint32_t OFS_ADR  = 0U;
    uint32_t MAX_CNT2 = 0U;
    (void)iLoop;
    (void)iLoop1;
    (void)iLoop2;
    (void)jLoop;
    (void)kLoop;
    (void)oLoop;
    (void)oLoop1;
    (void)oLoop2;
    (void)dummy;
    (void)KEY_ADR;
    (void)OFS_ADR;
    (void)MAX_CNT2;
    if (0x0U != (SCE_REG(0x1BCU) & 0x1fU))
    {
        return FSP_ERR_CRYPTO_SCE_RESOURCE_CONFLICT;
    }
    SCE_REG(0x84U) = 0x00008401U;
    SCE_REG(0x108U) = 0x00000000U;
    SCE_REG(0x104U) = 0x00000068U;
    SCE_REG(0xE0U) = 0x800103e0U;
    SCE_REG(0x1D0U) = 0x00000000U;
    /* WAIT_LOOP */
    if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    SCE_REG(0x100U) = InData_KeyType[0];
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0xECU) = 0x38000fffU;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0xE0U) = 0x00000080U;
    SCE_REG(0x1CU) = 0x00260000U;
    HW_SCE_func001(0x2b38f6e1U, 0xb2ee9d83U, 0x265b79ceU, 0x4527c8b5U);
    if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
    SCE_REG(0x1CU) = 0x00400000U;
    SCE_REG(0x1D0U) = 0x00000000U;
    if (1U == (((SCE_REG(0x1CU) >> 22U) & 1U)))
    {
        SCE_REG(0x104U) = 0x00000068U;
        SCE_REG(0xE0U) = 0x80010000U;
        SCE_REG(0x1D0U) = 0x00000000U;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = change_endian_long(InData_SharedKeyIndex[0]);
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0xECU) = 0x38008c00U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0xECU) = 0xfffffff0U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0xE0U) = 0x00000080U;
        SCE_REG(0x1CU) = 0x00A60000U;
        HW_SCE_func001(0x2d26a53cU, 0x3411202dU, 0x4edbbc4cU, 0x25fd475dU);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        SCE_REG(0x1CU) = 0x00400000U;
        SCE_REG(0x1D0U) = 0x00000000U;
        if (1U == (((SCE_REG(0x1CU) >> 22U) & 1U)))
        {
            HW_SCE_func003(0xdf748238U, 0x22304026U, 0x928b4178U, 0x645fcbe6U);
            if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
            SCE_REG(0x1BCU) = 0x00000040U;
            /* WAIT_LOOP */
            if (!SceWait(0x18U, 12U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            return FSP_ERR_CRYPTO_SCE_FAIL;
        }
        else
        {
            HW_SCE_func001(0xccbad478U, 0xf3238b4cU, 0xba462dc1U, 0x695d7d82U);
            if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
            OFS_ADR = InData_SharedKeyIndex[0]*8;
            SCE_REG(0xA4U) = 0x400c3a0cU;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = change_endian_long(0x129e32e8U);
            SCE_REG(0xA4U) = 0x400c0a0cU;
            SCE_REG(0xE0U) = 0x81010000U;
            SCE_REG(0x1D0U) = 0x00000000U;
            SCE_REG(0x00U) = 0x00001807U;
            /* WAIT_LOOP */
            if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x1CU) = 0x00001800U;
            SCE_REG(0xA4U) = 0x42fa063dU;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = S_FLASH[OFS_ADR + 0];
            SCE_REG(0x100U) = S_FLASH[OFS_ADR + 1];
            SCE_REG(0x100U) = S_FLASH[OFS_ADR + 2];
            SCE_REG(0x100U) = S_FLASH[OFS_ADR + 3];
            HW_SCE_func001(0xeb350f83U, 0x7d2faa02U, 0xb4a60d94U, 0x16b362b3U);
            if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
            SCE_REG(0xA4U) = 0x400006bdU;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = S_FLASH[OFS_ADR+4 + 0];
            SCE_REG(0x100U) = S_FLASH[OFS_ADR+4 + 1];
            SCE_REG(0x100U) = S_FLASH[OFS_ADR+4 + 2];
            SCE_REG(0x100U) = S_FLASH[OFS_ADR+4 + 3];
            SCE_REG(0xA4U) = 0x00800c45U;
            SCE_REG(0x00U) = 0x00001113U;
            /* WAIT_LOOP */
            if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x1CU) = 0x00001800U;
            SCE_REG(0x104U) = 0x00000361U;
            SCE_REG(0xA4U) = 0x0008680dU;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = InData_SessionKey[0];
            SCE_REG(0x100U) = InData_SessionKey[1];
            SCE_REG(0x100U) = InData_SessionKey[2];
            SCE_REG(0x100U) = InData_SessionKey[3];
            HW_SCE_func001(0x928739beU, 0x12238d6cU, 0x73cf8e31U, 0xceac6ccaU);
            if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
            SCE_REG(0x104U) = 0x00000361U;
            SCE_REG(0xA4U) = 0x0009680dU;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = InData_SessionKey[4];
            SCE_REG(0x100U) = InData_SessionKey[5];
            SCE_REG(0x100U) = InData_SessionKey[6];
            SCE_REG(0x100U) = InData_SessionKey[7];
            HW_SCE_func001(0xefd962f4U, 0x56443df8U, 0xf6f69197U, 0x54f495a6U);
            if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
            SCE_REG(0x104U) = 0x00000361U;
            SCE_REG(0xA4U) = 0x00040805U;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = InData_IV[0];
            SCE_REG(0x100U) = InData_IV[1];
            SCE_REG(0x100U) = InData_IV[2];
            SCE_REG(0x100U) = InData_IV[3];
            SCE_REG(0x104U) = 0x00000051U;
            SCE_REG(0xA4U) = 0x00050804U;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = change_endian_long(0x00000000U);
            SCE_REG(0x104U) = 0x00000251U;
            SCE_REG(0xA4U) = 0x0020363cU;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = change_endian_long(0x00000000U);
            SCE_REG(0xA4U) = 0x002036bcU;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = change_endian_long(0x00000000U);
            SCE_REG(0xA4U) = 0x002036bcU;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = change_endian_long(0x00000000U);
            SCE_REG(0xA4U) = 0x00060805U;
            SCE_REG(0x00U) = 0x00001113U;
            /* WAIT_LOOP */
            if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x1CU) = 0x00001800U;
            SCE_REG(0x104U) = 0x00000151U;
            SCE_REG(0xA4U) = 0x0021340cU;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = change_endian_long(0x00000000U);
            SCE_REG(0xA4U) = 0x000b0805U;
            SCE_REG(0x00U) = 0x00001113U;
            /* WAIT_LOOP */
            if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x1CU) = 0x00001800U;
            SCE_REG(0xA4U) = 0x01000684U;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = change_endian_long(0x00000000U);
            HW_SCE_func001(0x35cf0f9dU, 0xa0fe8cf1U, 0xddee3d74U, 0xfbe6f5a5U);
            if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
            SCE_REG(0xE0U) = 0x80040000U;
            SCE_REG(0x1D0U) = 0x00000000U;
            SCE_REG(0x00U) = 0x00008113U;
            /* WAIT_LOOP */
            if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x1CU) = 0x00001800U;
            SCE_REG(0xA4U) = 0x600c3a0dU;
            SCE_REG(0xE0U) = 0x81040000U;
            SCE_REG(0x1D0U) = 0x00000000U;
            SCE_REG(0x00U) = 0x00001813U;
            /* WAIT_LOOP */
            if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x1CU) = 0x00001800U;
            SCE_REG(0xA4U) = 0x400c0a0cU;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = change_endian_long(0xef138998U);
            SCE_REG(0x104U) = 0x00000361U;
            SCE_REG(0xA4U) = 0x00d049adU;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = InData_InstData[0];
            SCE_REG(0x100U) = InData_InstData[1];
            SCE_REG(0x100U) = InData_InstData[2];
            SCE_REG(0x100U) = InData_InstData[3];
            SCE_REG(0xA4U) = 0x43e006bdU;
            SCE_REG(0x00U) = 0x00001113U;
            /* WAIT_LOOP */
            if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x1CU) = 0x00001800U;
            SCE_REG(0x04U) = 0x00000112U;
            /* WAIT_LOOP */
            if (!SceWait(0x04U, 30U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            OutData_KeyIndex[4] = SCE_REG(0x100U);
            OutData_KeyIndex[5] = SCE_REG(0x100U);
            OutData_KeyIndex[6] = SCE_REG(0x100U);
            OutData_KeyIndex[7] = SCE_REG(0x100U);
            HW_SCE_func001(0x65a6e055U, 0x939551e8U, 0xe6b5702bU, 0x6b3bbc7cU);
            if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
            SCE_REG(0x104U) = 0x00000051U;
            SCE_REG(0xA4U) = 0x01000684U;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = change_endian_long(0x00000000U);
            SCE_REG(0xA4U) = 0x410009cdU;
            SCE_REG(0x00U) = 0x00001113U;
            /* WAIT_LOOP */
            if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x1CU) = 0x00001800U;
            SCE_REG(0x04U) = 0x00000112U;
            /* WAIT_LOOP */
            if (!SceWait(0x04U, 30U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            OutData_KeyIndex[8] = SCE_REG(0x100U);
            OutData_KeyIndex[9] = SCE_REG(0x100U);
            OutData_KeyIndex[10] = SCE_REG(0x100U);
            OutData_KeyIndex[11] = SCE_REG(0x100U);
            SCE_REG(0x104U) = 0x00000361U;
            SCE_REG(0xA4U) = 0x000049adU;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = InData_InstData[4];
            SCE_REG(0x100U) = InData_InstData[5];
            SCE_REG(0x100U) = InData_InstData[6];
            SCE_REG(0x100U) = InData_InstData[7];
            SCE_REG(0xA4U) = 0x00900c45U;
            SCE_REG(0x00U) = 0x00001113U;
            /* WAIT_LOOP */
            if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x1CU) = 0x00001800U;
            HW_SCE_func001(0x076faf67U, 0xc204b8f4U, 0x263186b8U, 0xc6195441U);
            if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
            SCE_REG(0x1CU) = 0x00400000U;
            SCE_REG(0x1D0U) = 0x00000000U;
            if (1U == (((SCE_REG(0x1CU) >> 22U) & 1U)))
            {
                HW_SCE_func003(0x10be6df0U, 0x4489ee44U, 0x461e50caU, 0xb639e428U);
                if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
                SCE_REG(0x1BCU) = 0x00000040U;
                /* WAIT_LOOP */
                if (!SceWait(0x18U, 12U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
                return FSP_ERR_CRYPTO_SCE_FAIL;
            }
            else
            {
                HW_SCE_func001(0x1e59e0abU, 0xd0a58cfaU, 0xedde2681U, 0x7f213335U);
                if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
                SCE_REG(0xE0U) = 0x81040000U;
                SCE_REG(0x1D0U) = 0x00000000U;
                SCE_REG(0x04U) = 0x00000612U;
                /* WAIT_LOOP */
                if (!SceWait(0x04U, 30U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
                OutData_KeyIndex[0] = SCE_REG(0x100U);
                /* WAIT_LOOP */
                if (!SceWait(0x04U, 30U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
                OutData_KeyIndex[1] = SCE_REG(0x100U);
                /* WAIT_LOOP */
                if (!SceWait(0x04U, 30U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
                OutData_KeyIndex[2] = SCE_REG(0x100U);
                /* WAIT_LOOP */
                if (!SceWait(0x04U, 30U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
                OutData_KeyIndex[3] = SCE_REG(0x100U);
                HW_SCE_func003(0xdb4518b5U, 0x83279165U, 0x5eb6fbc7U, 0x93bcd37cU);
                if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
                SCE_REG(0x1BCU) = 0x00000040U;
                /* WAIT_LOOP */
                if (!SceWait(0x18U, 12U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
                return FSP_SUCCESS;
            }
        }
    }
    else
    {
        HW_SCE_func001(0x3f1e4052U, 0x7830c312U, 0x6af66249U, 0x088be5baU);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        SCE_REG(0x104U) = 0x00000251U;
        SCE_REG(0xA4U) = 0x0020363cU;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = change_endian_long(0x00000000U);
        SCE_REG(0xA4U) = 0x002036bcU;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = change_endian_long(0x00000000U);
        SCE_REG(0xA4U) = 0x002036bcU;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = change_endian_long(0x00000000U);
        SCE_REG(0xA4U) = 0x00060805U;
        SCE_REG(0x00U) = 0x00001113U;
        /* WAIT_LOOP */
        if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x1CU) = 0x00001800U;
        SCE_REG(0x104U) = 0x00000151U;
        SCE_REG(0xA4U) = 0x0021340cU;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = change_endian_long(0x00000000U);
        SCE_REG(0xA4U) = 0x000b0805U;
        SCE_REG(0x00U) = 0x00001113U;
        /* WAIT_LOOP */
        if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x1CU) = 0x00001800U;
        SCE_REG(0xA4U) = 0x01000684U;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = change_endian_long(0x00000000U);
        HW_SCE_func001(0x5a847decU, 0xa628fa18U, 0x79d8e198U, 0x403cd5dfU);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        SCE_REG(0xE0U) = 0x80040000U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0x00U) = 0x00008113U;
        /* WAIT_LOOP */
        if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x1CU) = 0x00001800U;
        SCE_REG(0xA4U) = 0x600c3a0dU;
        SCE_REG(0xE0U) = 0x81040000U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0x00U) = 0x00001813U;
        /* WAIT_LOOP */
        if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x1CU) = 0x00001800U;
        SCE_REG(0xA4U) = 0x400c0a0cU;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = change_endian_long(0xef138998U);
        SCE_REG(0x104U) = 0x00000361U;
        SCE_REG(0xA4U) = 0x00000885U;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = InData_InstData[0];
        SCE_REG(0x100U) = InData_InstData[1];
        SCE_REG(0x100U) = InData_InstData[2];
        SCE_REG(0x100U) = InData_InstData[3];
        SCE_REG(0xA4U) = 0x43e006bdU;
        SCE_REG(0x00U) = 0x00001113U;
        /* WAIT_LOOP */
        if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x1CU) = 0x00001800U;
        SCE_REG(0x04U) = 0x00000112U;
        /* WAIT_LOOP */
        if (!SceWait(0x04U, 30U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        OutData_KeyIndex[4] = SCE_REG(0x100U);
        OutData_KeyIndex[5] = SCE_REG(0x100U);
        OutData_KeyIndex[6] = SCE_REG(0x100U);
        OutData_KeyIndex[7] = SCE_REG(0x100U);
        HW_SCE_func001(0x452ebb38U, 0xad4b097aU, 0x10e2d767U, 0xca22d27fU);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        SCE_REG(0x104U) = 0x00000051U;
        SCE_REG(0xA4U) = 0x01000684U;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = change_endian_long(0x00000000U);
        SCE_REG(0xA4U) = 0x410009cdU;
        SCE_REG(0x00U) = 0x00001113U;
        /* WAIT_LOOP */
        if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x1CU) = 0x00001800U;
        SCE_REG(0x04U) = 0x00000112U;
        /* WAIT_LOOP */
        if (!SceWait(0x04U, 30U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        OutData_KeyIndex[8] = SCE_REG(0x100U);
        OutData_KeyIndex[9] = SCE_REG(0x100U);
        OutData_KeyIndex[10] = SCE_REG(0x100U);
        OutData_KeyIndex[11] = SCE_REG(0x100U);
        HW_SCE_func001(0xc2df453bU, 0xf7cd3032U, 0x420bbc49U, 0x8b8267d5U);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        SCE_REG(0xE0U) = 0x81040000U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0x04U) = 0x00000612U;
        /* WAIT_LOOP */
        if (!SceWait(0x04U, 30U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        OutData_KeyIndex[0] = SCE_REG(0x100U);
        /* WAIT_LOOP */
        if (!SceWait(0x04U, 30U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        OutData_KeyIndex[1] = SCE_REG(0x100U);
        /* WAIT_LOOP */
        if (!SceWait(0x04U, 30U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        OutData_KeyIndex[2] = SCE_REG(0x100U);
        /* WAIT_LOOP */
        if (!SceWait(0x04U, 30U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        OutData_KeyIndex[3] = SCE_REG(0x100U);
        HW_SCE_func003(0x06ecda62U, 0x02879bd3U, 0x8fb2beafU, 0x414f899dU);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        SCE_REG(0x1BCU) = 0x00000040U;
        /* WAIT_LOOP */
        if (!SceWait(0x18U, 12U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        return FSP_SUCCESS;
    }
}

/***********************************************************************************************************************
End of function ./input_dir/HW_SCE_Sec_200408/200408/RA4M1/Cryptographic_PlainKey/HW_SCEp_p84.prc
***********************************************************************************************************************/

// Adapted from hw_sce_p85.c
/*
* Copyright (c) 2020 - 2026 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/***********************************************************************************************************************
 * History : DD.MM.YYYY Version Description
 *         : 05.10.2020 1.00        First Release.
 *         : 02.12.2020 1.01        Added new functions such as the Brainpool curve.
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Imported global variables and functions (from other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Exported global variables (to be accessed by other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Private global variables and functions
***********************************************************************************************************************/

static fsp_err_t HW_SCE_GenerateAes256PlainKeyIndexSub(uint32_t *InData_KeyType, uint32_t *InData_SharedKeyIndex, uint32_t *InData_SessionKey, uint32_t *InData_IV, uint32_t *InData_InstData, uint32_t *OutData_KeyIndex)
{
    uint32_t iLoop    = 0U;
    uint32_t iLoop1   = 0U;
    uint32_t iLoop2   = 0U;
    int32_t  jLoop    = 0U;
    uint32_t kLoop    = 0U;
    uint32_t oLoop    = 0U;
    uint32_t oLoop1   = 0U;
    uint32_t oLoop2   = 0U;
    uint32_t dummy    = 0U;
    uint32_t KEY_ADR  = 0U;
    uint32_t OFS_ADR  = 0U;
    uint32_t MAX_CNT2 = 0U;
    (void)iLoop;
    (void)iLoop1;
    (void)iLoop2;
    (void)jLoop;
    (void)kLoop;
    (void)oLoop;
    (void)oLoop1;
    (void)oLoop2;
    (void)dummy;
    (void)KEY_ADR;
    (void)OFS_ADR;
    (void)MAX_CNT2;
    if (0x0U != (SCE_REG(0x1BCU) & 0x1fU))
    {
        return FSP_ERR_CRYPTO_SCE_RESOURCE_CONFLICT;
    }
    SCE_REG(0x84U) = 0x00008501U;
    SCE_REG(0x108U) = 0x00000000U;
    SCE_REG(0x104U) = 0x00000068U;
    SCE_REG(0xE0U) = 0x800103e0U;
    SCE_REG(0x1D0U) = 0x00000000U;
    /* WAIT_LOOP */
    if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
    SCE_REG(0x100U) = InData_KeyType[0];
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0xECU) = 0x38000fffU;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0xE0U) = 0x00000080U;
    SCE_REG(0x1CU) = 0x00260000U;
    HW_SCE_func001(0x046050b8U, 0x5f4634c3U, 0xeaef7838U, 0x27b7e43aU);
    if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
    SCE_REG(0x1CU) = 0x00400000U;
    SCE_REG(0x1D0U) = 0x00000000U;
    if (1U == (((SCE_REG(0x1CU) >> 22U) & 1U)))
    {
        SCE_REG(0x104U) = 0x00000068U;
        SCE_REG(0xE0U) = 0x80010000U;
        SCE_REG(0x1D0U) = 0x00000000U;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = change_endian_long(InData_SharedKeyIndex[0]);
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0xECU) = 0x38008c00U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0xECU) = 0xfffffff0U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0xE0U) = 0x00000080U;
        SCE_REG(0x1CU) = 0x00A60000U;
        HW_SCE_func001(0x52981ea7U, 0x2f8abb24U, 0x9d8e5cbbU, 0x032ecf88U);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        SCE_REG(0x1CU) = 0x00400000U;
        SCE_REG(0x1D0U) = 0x00000000U;
        if (1U == (((SCE_REG(0x1CU) >> 22U) & 1U)))
        {
            HW_SCE_func003(0xd86d5ebdU, 0x2d9e1a60U, 0x69e14652U, 0xef15c0d7U);
            if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
            SCE_REG(0x1BCU) = 0x00000040U;
            /* WAIT_LOOP */
            if (!SceWait(0x18U, 12U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            return FSP_ERR_CRYPTO_SCE_FAIL;
        }
        else
        {
            HW_SCE_func001(0x995b2953U, 0x9c9489b4U, 0xd336e83cU, 0x0c49eadeU);
            if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
            OFS_ADR = InData_SharedKeyIndex[0]*8;
            SCE_REG(0xA4U) = 0x400c3a0cU;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = change_endian_long(0x129e32e8U);
            SCE_REG(0xA4U) = 0x400c0a0cU;
            SCE_REG(0xE0U) = 0x81010000U;
            SCE_REG(0x1D0U) = 0x00000000U;
            SCE_REG(0x00U) = 0x00001807U;
            /* WAIT_LOOP */
            if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x1CU) = 0x00001800U;
            SCE_REG(0xA4U) = 0x42fa063dU;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = S_FLASH[OFS_ADR + 0];
            SCE_REG(0x100U) = S_FLASH[OFS_ADR + 1];
            SCE_REG(0x100U) = S_FLASH[OFS_ADR + 2];
            SCE_REG(0x100U) = S_FLASH[OFS_ADR + 3];
            HW_SCE_func001(0xd24fe8a7U, 0xa1caffafU, 0x739b49c1U, 0x6efa0559U);
            if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
            SCE_REG(0xA4U) = 0x400006bdU;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = S_FLASH[OFS_ADR+4 + 0];
            SCE_REG(0x100U) = S_FLASH[OFS_ADR+4 + 1];
            SCE_REG(0x100U) = S_FLASH[OFS_ADR+4 + 2];
            SCE_REG(0x100U) = S_FLASH[OFS_ADR+4 + 3];
            SCE_REG(0xA4U) = 0x00800c45U;
            SCE_REG(0x00U) = 0x00001113U;
            /* WAIT_LOOP */
            if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x1CU) = 0x00001800U;
            SCE_REG(0x104U) = 0x00000361U;
            SCE_REG(0xA4U) = 0x0008680dU;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = InData_SessionKey[0];
            SCE_REG(0x100U) = InData_SessionKey[1];
            SCE_REG(0x100U) = InData_SessionKey[2];
            SCE_REG(0x100U) = InData_SessionKey[3];
            HW_SCE_func001(0x67b3e11aU, 0x2b37be6eU, 0x90bd0173U, 0xa1b8a807U);
            if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
            SCE_REG(0x104U) = 0x00000361U;
            SCE_REG(0xA4U) = 0x0009680dU;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = InData_SessionKey[4];
            SCE_REG(0x100U) = InData_SessionKey[5];
            SCE_REG(0x100U) = InData_SessionKey[6];
            SCE_REG(0x100U) = InData_SessionKey[7];
            HW_SCE_func001(0x615968c2U, 0xd2fdab2aU, 0x0bcb1933U, 0xd2395d81U);
            if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
            SCE_REG(0x104U) = 0x00000361U;
            SCE_REG(0xA4U) = 0x00040805U;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = InData_IV[0];
            SCE_REG(0x100U) = InData_IV[1];
            SCE_REG(0x100U) = InData_IV[2];
            SCE_REG(0x100U) = InData_IV[3];
            SCE_REG(0x104U) = 0x00000051U;
            SCE_REG(0xA4U) = 0x00050804U;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = change_endian_long(0x00000000U);
            SCE_REG(0x104U) = 0x00000251U;
            SCE_REG(0xA4U) = 0x0020363cU;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = change_endian_long(0x00000000U);
            SCE_REG(0xA4U) = 0x002036bcU;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = change_endian_long(0x00000000U);
            SCE_REG(0xA4U) = 0x002036bcU;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = change_endian_long(0x00000000U);
            SCE_REG(0xA4U) = 0x00060805U;
            SCE_REG(0x00U) = 0x00001113U;
            /* WAIT_LOOP */
            if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x1CU) = 0x00001800U;
            SCE_REG(0x104U) = 0x00000151U;
            SCE_REG(0xA4U) = 0x0021340cU;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = change_endian_long(0x00000000U);
            SCE_REG(0xA4U) = 0x000b0805U;
            SCE_REG(0x00U) = 0x00001113U;
            /* WAIT_LOOP */
            if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x1CU) = 0x00001800U;
            SCE_REG(0xA4U) = 0x01000684U;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = change_endian_long(0x00000000U);
            HW_SCE_func001(0xe0860099U, 0x9b0e3201U, 0xef96a873U, 0x9ec8430eU);
            if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
            SCE_REG(0xE0U) = 0x80040000U;
            SCE_REG(0x1D0U) = 0x00000000U;
            SCE_REG(0x00U) = 0x00008113U;
            /* WAIT_LOOP */
            if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x1CU) = 0x00001800U;
            SCE_REG(0xA4U) = 0x600c3a0dU;
            SCE_REG(0xE0U) = 0x81040000U;
            SCE_REG(0x1D0U) = 0x00000000U;
            SCE_REG(0x00U) = 0x00001813U;
            /* WAIT_LOOP */
            if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x1CU) = 0x00001800U;
            SCE_REG(0xA4U) = 0x400c0a0cU;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = change_endian_long(0x213f32e4U);
            SCE_REG(0x104U) = 0x00000361U;
            SCE_REG(0xA4U) = 0x00d049adU;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = InData_InstData[0];
            SCE_REG(0x100U) = InData_InstData[1];
            SCE_REG(0x100U) = InData_InstData[2];
            SCE_REG(0x100U) = InData_InstData[3];
            SCE_REG(0xA4U) = 0x43e006bdU;
            SCE_REG(0x00U) = 0x00001113U;
            /* WAIT_LOOP */
            if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x1CU) = 0x00001800U;
            SCE_REG(0x04U) = 0x00000112U;
            /* WAIT_LOOP */
            if (!SceWait(0x04U, 30U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            OutData_KeyIndex[4] = SCE_REG(0x100U);
            OutData_KeyIndex[5] = SCE_REG(0x100U);
            OutData_KeyIndex[6] = SCE_REG(0x100U);
            OutData_KeyIndex[7] = SCE_REG(0x100U);
            HW_SCE_func001(0x8525eb6cU, 0x8fb5ac0fU, 0x0aa7a378U, 0x014e7acdU);
            if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
            SCE_REG(0x104U) = 0x00000361U;
            SCE_REG(0xA4U) = 0x00d049adU;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = InData_InstData[4];
            SCE_REG(0x100U) = InData_InstData[5];
            SCE_REG(0x100U) = InData_InstData[6];
            SCE_REG(0x100U) = InData_InstData[7];
            SCE_REG(0xA4U) = 0x41e006bdU;
            SCE_REG(0x00U) = 0x00001113U;
            /* WAIT_LOOP */
            if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x1CU) = 0x00001800U;
            SCE_REG(0x04U) = 0x00000112U;
            /* WAIT_LOOP */
            if (!SceWait(0x04U, 30U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            OutData_KeyIndex[8] = SCE_REG(0x100U);
            OutData_KeyIndex[9] = SCE_REG(0x100U);
            OutData_KeyIndex[10] = SCE_REG(0x100U);
            OutData_KeyIndex[11] = SCE_REG(0x100U);
            HW_SCE_func001(0x890c7546U, 0xcd12f341U, 0x8a61e47bU, 0xa58fb1e8U);
            if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
            SCE_REG(0x104U) = 0x00000051U;
            SCE_REG(0xA4U) = 0x01000684U;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = change_endian_long(0x00000000U);
            SCE_REG(0xA4U) = 0x410009cdU;
            SCE_REG(0x00U) = 0x00001113U;
            /* WAIT_LOOP */
            if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x1CU) = 0x00001800U;
            SCE_REG(0x04U) = 0x00000112U;
            /* WAIT_LOOP */
            if (!SceWait(0x04U, 30U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            OutData_KeyIndex[12] = SCE_REG(0x100U);
            OutData_KeyIndex[13] = SCE_REG(0x100U);
            OutData_KeyIndex[14] = SCE_REG(0x100U);
            OutData_KeyIndex[15] = SCE_REG(0x100U);
            SCE_REG(0x104U) = 0x00000361U;
            SCE_REG(0xA4U) = 0x000049adU;
            /* WAIT_LOOP */
            if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x100U) = InData_InstData[8];
            SCE_REG(0x100U) = InData_InstData[9];
            SCE_REG(0x100U) = InData_InstData[10];
            SCE_REG(0x100U) = InData_InstData[11];
            SCE_REG(0xA4U) = 0x00900c45U;
            SCE_REG(0x00U) = 0x00001113U;
            /* WAIT_LOOP */
            if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
            SCE_REG(0x1CU) = 0x00001800U;
            HW_SCE_func001(0xc3e7a455U, 0x0d1c8705U, 0x16d865efU, 0xd502572aU);
            if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
            SCE_REG(0x1CU) = 0x00400000U;
            SCE_REG(0x1D0U) = 0x00000000U;
            if (1U == (((SCE_REG(0x1CU) >> 22U) & 1U)))
            {
                HW_SCE_func003(0x28782014U, 0xca7d9527U, 0x66211539U, 0x7825e0c6U);
                if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
                SCE_REG(0x1BCU) = 0x00000040U;
                /* WAIT_LOOP */
                if (!SceWait(0x18U, 12U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
                return FSP_ERR_CRYPTO_SCE_FAIL;
            }
            else
            {
                HW_SCE_func001(0x747dfec6U, 0xe6140a23U, 0x465afcc8U, 0xa0b72700U);
                if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
                SCE_REG(0xE0U) = 0x81040000U;
                SCE_REG(0x1D0U) = 0x00000000U;
                SCE_REG(0x04U) = 0x00000612U;
                /* WAIT_LOOP */
                if (!SceWait(0x04U, 30U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
                OutData_KeyIndex[0] = SCE_REG(0x100U);
                /* WAIT_LOOP */
                if (!SceWait(0x04U, 30U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
                OutData_KeyIndex[1] = SCE_REG(0x100U);
                /* WAIT_LOOP */
                if (!SceWait(0x04U, 30U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
                OutData_KeyIndex[2] = SCE_REG(0x100U);
                /* WAIT_LOOP */
                if (!SceWait(0x04U, 30U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
                OutData_KeyIndex[3] = SCE_REG(0x100U);
                HW_SCE_func003(0x792cb424U, 0xa0e331e9U, 0xa4c7f197U, 0xb90337e2U);
                if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
                SCE_REG(0x1BCU) = 0x00000040U;
                /* WAIT_LOOP */
                if (!SceWait(0x18U, 12U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
                return FSP_SUCCESS;
            }
        }
    }
    else
    {
        HW_SCE_func001(0x9ee5d86bU, 0x1bc1ef1bU, 0xf118b20aU, 0xb0b6751cU);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        SCE_REG(0x104U) = 0x00000251U;
        SCE_REG(0xA4U) = 0x0020363cU;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = change_endian_long(0x00000000U);
        SCE_REG(0xA4U) = 0x002036bcU;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = change_endian_long(0x00000000U);
        SCE_REG(0xA4U) = 0x002036bcU;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = change_endian_long(0x00000000U);
        SCE_REG(0xA4U) = 0x00060805U;
        SCE_REG(0x00U) = 0x00001113U;
        /* WAIT_LOOP */
        if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x1CU) = 0x00001800U;
        SCE_REG(0x104U) = 0x00000151U;
        SCE_REG(0xA4U) = 0x0021340cU;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = change_endian_long(0x00000000U);
        SCE_REG(0xA4U) = 0x000b0805U;
        SCE_REG(0x00U) = 0x00001113U;
        /* WAIT_LOOP */
        if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x1CU) = 0x00001800U;
        SCE_REG(0xA4U) = 0x01000684U;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = change_endian_long(0x00000000U);
        HW_SCE_func001(0x2590ac62U, 0x0cebe7e6U, 0x9d4592f6U, 0x152beb4dU);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        SCE_REG(0xE0U) = 0x80040000U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0x00U) = 0x00008113U;
        /* WAIT_LOOP */
        if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x1CU) = 0x00001800U;
        SCE_REG(0xA4U) = 0x600c3a0dU;
        SCE_REG(0xE0U) = 0x81040000U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0x00U) = 0x00001813U;
        /* WAIT_LOOP */
        if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x1CU) = 0x00001800U;
        SCE_REG(0xA4U) = 0x400c0a0cU;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = change_endian_long(0x213f32e4U);
        SCE_REG(0x104U) = 0x00000761U;
        SCE_REG(0xB0U) = 0x00000100U;
        SCE_REG(0xA4U) = 0x00008887U;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = InData_InstData[0];
        SCE_REG(0x100U) = InData_InstData[1];
        SCE_REG(0x100U) = InData_InstData[2];
        SCE_REG(0x100U) = InData_InstData[3];
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = InData_InstData[4];
        SCE_REG(0x100U) = InData_InstData[5];
        SCE_REG(0x100U) = InData_InstData[6];
        SCE_REG(0x100U) = InData_InstData[7];
        SCE_REG(0xB0U) = 0x00000100U;
        SCE_REG(0xA4U) = 0x43e086bfU;
        SCE_REG(0x00U) = 0x00001123U;
        /* WAIT_LOOP */
        if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x1CU) = 0x00001800U;
        SCE_REG(0x04U) = 0x00000122U;
        /* WAIT_LOOP */
        if (!SceWait(0x04U, 30U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        OutData_KeyIndex[4] = SCE_REG(0x100U);
        OutData_KeyIndex[5] = SCE_REG(0x100U);
        OutData_KeyIndex[6] = SCE_REG(0x100U);
        OutData_KeyIndex[7] = SCE_REG(0x100U);
        /* WAIT_LOOP */
        if (!SceWait(0x04U, 30U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        OutData_KeyIndex[8] = SCE_REG(0x100U);
        OutData_KeyIndex[9] = SCE_REG(0x100U);
        OutData_KeyIndex[10] = SCE_REG(0x100U);
        OutData_KeyIndex[11] = SCE_REG(0x100U);
        HW_SCE_func001(0x028b7994U, 0xe2e79faaU, 0x45a1cbdeU, 0x99a2c6d1U);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        SCE_REG(0x104U) = 0x00000051U;
        SCE_REG(0xA4U) = 0x01000684U;
        /* WAIT_LOOP */
        if (!SceWait(0x104U, 31U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x100U) = change_endian_long(0x00000000U);
        SCE_REG(0xA4U) = 0x410009cdU;
        SCE_REG(0x00U) = 0x00001113U;
        /* WAIT_LOOP */
        if (!SceWait(0x00U, 25U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        SCE_REG(0x1CU) = 0x00001800U;
        SCE_REG(0x04U) = 0x00000112U;
        /* WAIT_LOOP */
        if (!SceWait(0x04U, 30U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        OutData_KeyIndex[12] = SCE_REG(0x100U);
        OutData_KeyIndex[13] = SCE_REG(0x100U);
        OutData_KeyIndex[14] = SCE_REG(0x100U);
        OutData_KeyIndex[15] = SCE_REG(0x100U);
        HW_SCE_func001(0xc076a3cdU, 0x8ddadcfcU, 0x5751d101U, 0xd1ebb33cU);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        SCE_REG(0xE0U) = 0x81040000U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0x04U) = 0x00000612U;
        /* WAIT_LOOP */
        if (!SceWait(0x04U, 30U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        OutData_KeyIndex[0] = SCE_REG(0x100U);
        /* WAIT_LOOP */
        if (!SceWait(0x04U, 30U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        OutData_KeyIndex[1] = SCE_REG(0x100U);
        /* WAIT_LOOP */
        if (!SceWait(0x04U, 30U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        OutData_KeyIndex[2] = SCE_REG(0x100U);
        /* WAIT_LOOP */
        if (!SceWait(0x04U, 30U, 1U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        OutData_KeyIndex[3] = SCE_REG(0x100U);
        HW_SCE_func003(0x1af3fa71U, 0xafcf6218U, 0x86941d73U, 0x72f24b45U);
        if (s_Fault) { return FSP_ERR_CRYPTO_SCE_FAIL; }
        SCE_REG(0x1BCU) = 0x00000040U;
        /* WAIT_LOOP */
        if (!SceWait(0x18U, 12U, 0U))
    {
        return FSP_ERR_CRYPTO_SCE_FAIL;
    }
        return FSP_SUCCESS;
    }
}

/***********************************************************************************************************************
End of function ./input_dir/HW_SCE_Sec_230630/230630/RA4M1/Cryptographic_PlainKey/HW_SCEp_p85_r1.prc
***********************************************************************************************************************/

// Adapted from hw_sce_func001.c
/*
* Copyright (c) 2020 - 2026 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/***********************************************************************************************************************
 * History : DD.MM.YYYY Version Description
 *         : 05.10.2020 1.00        First Release.
 *         : 02.12.2020 1.01        Added new functions such as the Brainpool curve.
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Imported global variables and functions (from other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Exported global variables (to be accessed by other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Private global variables and functions
***********************************************************************************************************************/

static void HW_SCE_func001(uint32_t ARG1, uint32_t ARG2, uint32_t ARG3, uint32_t ARG4)
{
    uint32_t iLoop    = 0U;
    uint32_t iLoop1   = 0U;
    uint32_t iLoop2   = 0U;
    int32_t  jLoop    = 0U;
    uint32_t kLoop    = 0U;
    uint32_t oLoop    = 0U;
    uint32_t oLoop1   = 0U;
    uint32_t oLoop2   = 0U;
    uint32_t dummy    = 0U;
    uint32_t KEY_ADR  = 0U;
    uint32_t OFS_ADR  = 0U;
    uint32_t MAX_CNT2 = 0U;
    (void)iLoop;
    (void)iLoop1;
    (void)iLoop2;
    (void)jLoop;
    (void)kLoop;
    (void)oLoop;
    (void)oLoop1;
    (void)oLoop2;
    (void)dummy;
    (void)KEY_ADR;
    (void)OFS_ADR;
    (void)MAX_CNT2;
    SCE_REG(0xA4U) = 0x400f388dU;
    /* WAIT_LOOP */
    if (!SceWait(0x104U, 31U, 1U))
    {
        return;
    }
    SCE_REG(0x100U) = change_endian_long(ARG1);
    SCE_REG(0x100U) = change_endian_long(ARG2);
    SCE_REG(0x100U) = change_endian_long(ARG3);
    SCE_REG(0x100U) = change_endian_long(ARG4);
    /* WAIT_LOOP */
    if (!SceWait(0xA8U, 16U, 0U))
    {
        return;
    }
}

/***********************************************************************************************************************
End of function ./input_dir/HW_SCE_Sec_200408/200408/RA4M1/Cryptographic/HW_SCE_func001.prc
***********************************************************************************************************************/

// Adapted from hw_sce_func002.c
/*
* Copyright (c) 2020 - 2026 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/***********************************************************************************************************************
 * History : DD.MM.YYYY Version Description
 *         : 05.10.2020 1.00        First Release.
 *         : 02.12.2020 1.01        Added new functions such as the Brainpool curve.
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Imported global variables and functions (from other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Exported global variables (to be accessed by other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Private global variables and functions
***********************************************************************************************************************/

static void HW_SCE_func002(uint32_t ARG1, uint32_t ARG2, uint32_t ARG3, uint32_t ARG4)
{
    uint32_t iLoop    = 0U;
    uint32_t iLoop1   = 0U;
    uint32_t iLoop2   = 0U;
    int32_t  jLoop    = 0U;
    uint32_t kLoop    = 0U;
    uint32_t oLoop    = 0U;
    uint32_t oLoop1   = 0U;
    uint32_t oLoop2   = 0U;
    uint32_t dummy    = 0U;
    uint32_t KEY_ADR  = 0U;
    uint32_t OFS_ADR  = 0U;
    uint32_t MAX_CNT2 = 0U;
    (void)iLoop;
    (void)iLoop1;
    (void)iLoop2;
    (void)jLoop;
    (void)kLoop;
    (void)oLoop;
    (void)oLoop1;
    (void)oLoop2;
    (void)dummy;
    (void)KEY_ADR;
    (void)OFS_ADR;
    (void)MAX_CNT2;
    SCE_REG(0xA4U) = 0x400e388dU;
    /* WAIT_LOOP */
    if (!SceWait(0x104U, 31U, 1U))
    {
        return;
    }
    SCE_REG(0x100U) = change_endian_long(ARG1);
    SCE_REG(0x100U) = change_endian_long(ARG2);
    SCE_REG(0x100U) = change_endian_long(ARG3);
    SCE_REG(0x100U) = change_endian_long(ARG4);
    /* WAIT_LOOP */
    if (!SceWait(0xA8U, 17U, 0U))
    {
        return;
    }
}

/***********************************************************************************************************************
End of function ./input_dir/HW_SCE_Sec_200408/200408/RA4M1/Cryptographic/HW_SCE_func002.prc
***********************************************************************************************************************/

// Adapted from hw_sce_func003.c
/*
* Copyright (c) 2020 - 2026 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/***********************************************************************************************************************
 * History : DD.MM.YYYY Version Description
 *         : 05.10.2020 1.00        First Release.
 *         : 02.12.2020 1.01        Added new functions such as the Brainpool curve.
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Imported global variables and functions (from other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Exported global variables (to be accessed by other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Private global variables and functions
***********************************************************************************************************************/

static void HW_SCE_func003(uint32_t ARG1, uint32_t ARG2, uint32_t ARG3, uint32_t ARG4)
{
    uint32_t iLoop    = 0U;
    uint32_t iLoop1   = 0U;
    uint32_t iLoop2   = 0U;
    int32_t  jLoop    = 0U;
    uint32_t kLoop    = 0U;
    uint32_t oLoop    = 0U;
    uint32_t oLoop1   = 0U;
    uint32_t oLoop2   = 0U;
    uint32_t dummy    = 0U;
    uint32_t KEY_ADR  = 0U;
    uint32_t OFS_ADR  = 0U;
    uint32_t MAX_CNT2 = 0U;
    (void)iLoop;
    (void)iLoop1;
    (void)iLoop2;
    (void)jLoop;
    (void)kLoop;
    (void)oLoop;
    (void)oLoop1;
    (void)oLoop2;
    (void)dummy;
    (void)KEY_ADR;
    (void)OFS_ADR;
    (void)MAX_CNT2;
    SCE_REG(0xA4U) = 0x400d388dU;
    /* WAIT_LOOP */
    if (!SceWait(0x104U, 31U, 1U))
    {
        return;
    }
    SCE_REG(0x100U) = change_endian_long(ARG1);
    SCE_REG(0x100U) = change_endian_long(ARG2);
    SCE_REG(0x100U) = change_endian_long(ARG3);
    SCE_REG(0x100U) = change_endian_long(ARG4);
}

/***********************************************************************************************************************
End of function ./input_dir/HW_SCE_Sec_200408/200408/RA4M1/Cryptographic/HW_SCE_func003.prc
***********************************************************************************************************************/

// Adapted from hw_sce_func206.c
/*
* Copyright (c) 2020 - 2026 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/***********************************************************************************************************************
 * History : DD.MM.YYYY Version Description
 *         : 05.10.2020 1.00        First Release.
 *         : 02.12.2020 1.01        Added new functions such as the Brainpool curve.
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Imported global variables and functions (from other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Exported global variables (to be accessed by other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Private global variables and functions
***********************************************************************************************************************/

static void HW_SCE_func206(void)
{
    uint32_t iLoop    = 0U;
    uint32_t iLoop1   = 0U;
    uint32_t iLoop2   = 0U;
    int32_t  jLoop    = 0U;
    uint32_t kLoop    = 0U;
    uint32_t oLoop    = 0U;
    uint32_t oLoop1   = 0U;
    uint32_t oLoop2   = 0U;
    uint32_t dummy    = 0U;
    uint32_t KEY_ADR  = 0U;
    uint32_t OFS_ADR  = 0U;
    uint32_t MAX_CNT2 = 0U;
    (void)iLoop;
    (void)iLoop1;
    (void)iLoop2;
    (void)jLoop;
    (void)kLoop;
    (void)oLoop;
    (void)oLoop1;
    (void)oLoop2;
    (void)dummy;
    (void)KEY_ADR;
    (void)OFS_ADR;
    (void)MAX_CNT2;
    SCE_REG(0x04U) = 0x00000000U;
    SCE_REG(0x104U) = 0x00000000U;
    SCE_REG(0x1CU) = 0x002c0000U;
    SCE_REG(0x1CU) = 0x002d0000U;
    SCE_REG(0xB0U) = 0x00000001U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0xA4U) = 0x00000000U;
    SCE_REG(0x1D0U) = 0x00000000U;
}

/***********************************************************************************************************************
End of function ./input_dir/HW_SCE_Sec_200408/200408/RA4M1/Cryptographic/HW_SCE_func206.prc
***********************************************************************************************************************/

// Adapted from hw_sce_subprc01.c
/*
* Copyright (c) 2020 - 2026 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/***********************************************************************************************************************
 * History : DD.MM.YYYY Version Description
 *         : 05.10.2020 1.00        First Release.
 *         : 02.12.2020 1.01        Added new functions such as the Brainpool curve.
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Imported global variables and functions (from other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Exported global variables (to be accessed by other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Private global variables and functions
***********************************************************************************************************************/

static void HW_SCE_SelfCheck1SubSub(void)
{
    uint32_t iLoop    = 0U;
    uint32_t iLoop1   = 0U;
    uint32_t iLoop2   = 0U;
    int32_t  jLoop    = 0U;
    uint32_t kLoop    = 0U;
    uint32_t oLoop    = 0U;
    uint32_t oLoop1   = 0U;
    uint32_t oLoop2   = 0U;
    uint32_t dummy    = 0U;
    uint32_t KEY_ADR  = 0U;
    uint32_t OFS_ADR  = 0U;
    uint32_t MAX_CNT2 = 0U;
    (void)iLoop;
    (void)iLoop1;
    (void)iLoop2;
    (void)jLoop;
    (void)kLoop;
    (void)oLoop;
    (void)oLoop1;
    (void)oLoop2;
    (void)dummy;
    (void)KEY_ADR;
    (void)OFS_ADR;
    (void)MAX_CNT2;
    SCE_REG(0x138U) = 0xf597806AU;
    SCE_REG(0x10CU) = 0x00010001U;
    SCE_REG(0x80U) = 0x00000001U;
    SCE_REG(0xA4U) = 0x00000008U;
    SCE_REG(0x8CU) = 0x00000100U;
    /* WAIT_LOOP */
    if (!SceWait(0x64U, 11U, 0U))
    {
        return;
    }
    SCE_REG(0x64U) = 0x00000008U;
    SCE_REG(0x7CU) = 0x00000001U;
    SCE_REG(0x14U) = 0x10103005U;
    SCE_REG(0x10U) = 0x01234567U;
    SCE_REG(0x14U) = 0x10103015U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0x1D0U) = 0x00000000U;
    /* WAIT_LOOP */
    if (!SceWait(0x00U, 31U, 0U))
    {
        return;
    }
    SCE_REG(0x138U) = 0xf597806AU;
    SCE_REG(0xF0U) = 0x00000000U;
    SCE_REG(0x10CU) = 0x00010001U;
    SCE_REG(0x80U) = 0x00000001U;
    SCE_REG(0xA4U) = 0x00000008U;
    SCE_REG(0x8CU) = 0x00000102U;
    /* WAIT_LOOP */
    if (!SceWait(0x64U, 11U, 0U))
    {
        return;
    }
    SCE_REG(0x64U) = 0x00000008U;
    SCE_REG(0x7CU) = 0x00000001U;
    SCE_REG(0x14U) = 0x10303005U;
    SCE_REG(0x10U) = 0xefa59826U;
    SCE_REG(0x14U) = 0x10303015U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0x1D0U) = 0x00000000U;
    /* WAIT_LOOP */
    if (!SceWait(0x00U, 31U, 0U))
    {
        return;
    }
    SCE_REG(0x138U) = 0xf597806AU;
    SCE_REG(0xF0U) = 0x00000000U;
    SCE_REG(0x10CU) = 0x00010001U;
    SCE_REG(0x80U) = 0x00000001U;
    SCE_REG(0xA4U) = 0x00000008U;
    /* WAIT_LOOP */
    if (!SceWait(0x64U, 11U, 0U))
    {
        return;
    }
    SCE_REG(0x64U) = 0x00000008U;
    SCE_REG(0x8CU) = 0x00000100U;
    SCE_REG(0x7CU) = 0x00000001U;
    SCE_REG(0x78U) = 0x00000001U;
    SCE_REG(0x14U) = 0x10303005U;
    SCE_REG(0x10U) = 0x6159465eU;
    SCE_REG(0x14U) = 0x10303015U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0x1D0U) = 0x00000000U;
    /* WAIT_LOOP */
    if (!SceWait(0x00U, 31U, 0U))
    {
        return;
    }
    SCE_REG(0x138U) = 0xf597806AU;
    SCE_REG(0xF0U) = 0x00000000U;
    SCE_REG(0x10CU) = 0x00010001U;
    SCE_REG(0x80U) = 0x00000001U;
    SCE_REG(0xA4U) = 0x00000008U;
    SCE_REG(0x8CU) = 0x00000103U;
    /* WAIT_LOOP */
    if (!SceWait(0x64U, 11U, 0U))
    {
        return;
    }
    SCE_REG(0x64U) = 0x00000008U;
    SCE_REG(0x68U) = 0x00000005U;
    SCE_REG(0x7CU) = 0x00000001U;
    SCE_REG(0x78U) = 0x00000005U;
    SCE_REG(0x14U) = 0x70303005U;
    SCE_REG(0x10U) = 0xb785467cU;
    SCE_REG(0x14U) = 0x70303015U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0x1D0U) = 0x00000000U;
    /* WAIT_LOOP */
    if (!SceWait(0x00U, 31U, 0U))
    {
        return;
    }
    SCE_REG(0x138U) = 0xf597806AU;
    SCE_REG(0xF0U) = 0x00000000U;
    SCE_REG(0x10CU) = 0x00010001U;
    SCE_REG(0xECU) = 0x00000c00U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0x00U) = 0x08008802U;
    SCE_REG(0x80U) = 0x00000001U;
    SCE_REG(0xA4U) = 0x00000008U;
    SCE_REG(0x8CU) = 0xfffff1dfU;
    /* WAIT_LOOP */
    if (!SceWait(0x64U, 11U, 0U))
    {
        return;
    }
    SCE_REG(0x64U) = 0x00000008U;
    SCE_REG(0x68U) = 0x00000005U;
    SCE_REG(0x7CU) = 0x00000001U;
    SCE_REG(0x78U) = 0x00000005U;
    SCE_REG(0x14U) = 0x10300007U;
    SCE_REG(0x10U) = 0x9a59a424U;
    SCE_REG(0x14U) = 0x10300017U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0x1D0U) = 0x00000000U;
    /* WAIT_LOOP */
    if (!SceWait(0x00U, 31U, 0U))
    {
        return;
    }
}

/***********************************************************************************************************************
End of function ./input_dir/HW_SCE_Sec_200408/200408/RA4M1/Cryptographic/HW_SCE_subprc01.prc
***********************************************************************************************************************/

// Adapted from hw_sce_subprc02.c
/*
* Copyright (c) 2020 - 2026 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/***********************************************************************************************************************
 * History : DD.MM.YYYY Version Description
 *         : 05.10.2020 1.00        First Release.
 *         : 02.12.2020 1.01        Added new functions such as the Brainpool curve.
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Imported global variables and functions (from other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Exported global variables (to be accessed by other files)
***********************************************************************************************************************/

/***********************************************************************************************************************
Private global variables and functions
***********************************************************************************************************************/

static void HW_SCE_SelfCheck2SubSub(void)
{
    uint32_t iLoop    = 0U;
    uint32_t iLoop1   = 0U;
    uint32_t iLoop2   = 0U;
    int32_t  jLoop    = 0U;
    uint32_t kLoop    = 0U;
    uint32_t oLoop    = 0U;
    uint32_t oLoop1   = 0U;
    uint32_t oLoop2   = 0U;
    uint32_t dummy    = 0U;
    uint32_t KEY_ADR  = 0U;
    uint32_t OFS_ADR  = 0U;
    uint32_t MAX_CNT2 = 0U;
    (void)iLoop;
    (void)iLoop1;
    (void)iLoop2;
    (void)jLoop;
    (void)kLoop;
    (void)oLoop;
    (void)oLoop1;
    (void)oLoop2;
    (void)dummy;
    (void)KEY_ADR;
    (void)OFS_ADR;
    (void)MAX_CNT2;
    SCE_REG(0x104U) = 0x00000051U;
    SCE_REG(0xA4U) = 0x40083a8cU;
    /* WAIT_LOOP */
    if (!SceWait(0x104U, 31U, 1U))
    {
        return;
    }
    SCE_REG(0x100U) = change_endian_long(0x63a83a19U);
    SCE_REG(0x104U) = 0x00000051U;
    SCE_REG(0xA4U) = 0x00010804U;
    /* WAIT_LOOP */
    if (!SceWait(0x104U, 31U, 1U))
    {
        return;
    }
    SCE_REG(0x100U) = change_endian_long(0x00000000U);
    SCE_REG(0xECU) = 0x00000800U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0xE0U) = 0x80880000U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0x00U) = 0x00008023U;
    /* WAIT_LOOP */
    if (!SceWait(0x00U, 25U, 0U))
    {
        return;
    }
    SCE_REG(0x1CU) = 0x00001800U;
    SCE_REG(0xECU) = 0x0000b460U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0xECU) = 0x00000002U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0xECU) = 0x0000b4e0U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0xECU) = 0x00400003U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0xA4U) = 0xc0200c1eU;
    for(iLoop=0; iLoop<1024; iLoop=iLoop+1)
    {
        SCE_REG(0xE0U) = 0x80010020U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0x00U) = 0x00008407U;
        /* WAIT_LOOP */
        if (!SceWait(0x00U, 25U, 0U))
    {
        return;
    }
        SCE_REG(0x1CU) = 0x00001800U;
        SCE_REG(0xECU) = 0x000034c1U;
        SCE_REG(0x1D0U) = 0x00000000U;
        for(jLoop=0; jLoop<32; jLoop=jLoop+1)
        {
            SCE_REG(0xECU) = 0x0c016cc6U;
            SCE_REG(0x1D0U) = 0x00000000U;
            SCE_REG(0xECU) = 0x100008c7U;
            SCE_REG(0x1D0U) = 0x00000000U;
        }
        SCE_REG(0xE0U) = 0x810100c0U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0x00U) = 0x00001807U;
        /* WAIT_LOOP */
        if (!SceWait(0x00U, 25U, 0U))
    {
        return;
    }
        SCE_REG(0x1CU) = 0x00001800U;
        for(jLoop=0; jLoop<8; jLoop=jLoop+1)
        {
            SCE_REG(0xECU) = 0x00003441U;
            SCE_REG(0x1D0U) = 0x00000000U;
            SCE_REG(0xECU) = 0x00008c40U;
            SCE_REG(0x1D0U) = 0x00000000U;
            SCE_REG(0xECU) = 0x0000000fU;
            SCE_REG(0x1D0U) = 0x00000000U;
            SCE_REG(0xECU) = 0x00000800U;
            SCE_REG(0x1D0U) = 0x00000000U;
            SCE_REG(0xECU) = 0x00000884U;
            SCE_REG(0x1D0U) = 0x00000000U;
            for(kLoop=0; kLoop<16; kLoop=kLoop+1)
            {
                SCE_REG(0xECU) = 0x010038a0U;
                SCE_REG(0x1D0U) = 0x00000000U;
                SCE_REG(0xECU) = 0x38000844U;
                SCE_REG(0x1D0U) = 0x00000000U;
                SCE_REG(0xECU) = 0x10002ca0U;
                SCE_REG(0x1D0U) = 0x00000000U;
                SCE_REG(0xECU) = 0x01003ca0U;
                SCE_REG(0x1D0U) = 0x00000000U;
                SCE_REG(0xECU) = 0x00002403U;
                SCE_REG(0x1D0U) = 0x00000000U;
                SCE_REG(0xECU) = 0x00002c80U;
                SCE_REG(0x1D0U) = 0x00000000U;
            }
            SCE_REG(0xECU) = 0x00046821U;
            SCE_REG(0x1D0U) = 0x00000000U;
        }
    }
    /* WAIT_LOOP */
    if (!SceWait(0xA8U, 6U, 0U))
    {
        return;
    }
    SCE_REG(0xA4U) = 0x00000000U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0x1CU) = 0x00000400U;
    SCE_REG(0xECU) = 0x00000821U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0xECU) = 0x0000b480U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0xECU) = 0x00000B50U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0xECU) = 0x00000800U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0xECU) = 0x00000863U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0xECU) = 0x000008c6U;
    SCE_REG(0x1D0U) = 0x00000000U;
    for(iLoop=0; iLoop<16; iLoop=iLoop+1)
    {
        SCE_REG(0xECU) = 0x010038a0U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0xECU) = 0x30002885U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0xECU) = 0x00070020U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0xECU) = 0x00005023U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0xECU) = 0x00000080U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0xECU) = 0x000024c5U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0xECU) = 0x0000a400U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0xECU) = 0x00000002U;
        SCE_REG(0x1D0U) = 0x00000000U;
        SCE_REG(0xECU) = 0x00002c60U;
        SCE_REG(0x1D0U) = 0x00000000U;
    }
    SCE_REG(0xECU) = 0x380088c0U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0xECU) = 0x00002000U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0xE0U) = 0x00000080U;
    SCE_REG(0x1CU) = 0x00260000U;
    SCE_REG(0x1CU) = 0x00402000U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0xECU) = 0x38008820U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0xECU) = 0x00000000U;
    SCE_REG(0x1D0U) = 0x00000000U;
    SCE_REG(0xE0U) = 0x00000080U;
    SCE_REG(0x1CU) = 0x00A60000U;
}

/***********************************************************************************************************************
End of function ./input_dir/HW_SCE_Sec_200408/200408/RA4M1/Cryptographic/HW_SCE_subprc02.prc
***********************************************************************************************************************/


static bool Finish(fsp_err_t status)
{
	if (s_Fault || status != FSP_SUCCESS)
	{
		s_Ready = false;
		HW_SCE_SoftwareResetSub();
		Wipe(S_RAM, sizeof(S_RAM));
		return false;
	}
	return true;
}

bool Sce5Init(void)
{
	fsp_err_t status;
	unsigned attempts;

	if (s_Ready)
	{
		return true;
	}
	s_Fault = false;
	HW_SCE_SoftwareResetSub();
	status = HW_SCE_SelfCheck1Sub();
	if (!Finish(status))
	{
		return false;
	}
	SCE_REG(0x1D4U) = 0x0000FFFFU;
	for (attempts = 0U; attempts < 3U; attempts++)
	{
		status = HW_SCE_SelfCheck2Sub();
		if (s_Fault || status != FSP_ERR_CRYPTO_SCE_RETRY)
		{
			break;
		}
	}
	s_Ready = Finish(status);
	return s_Ready;
}

bool Sce5Ready(void)
{
	return s_Ready && !s_Fault;
}

bool Sce5RandomWords(uint32_t output[4])
{
	bool ok;

	if (!Sce5Ready())
	{
		Wipe(output, 16U);
		return false;
	}
	ok = Finish(HW_SCE_GenerateRandomNumberSub(output));
	if (!ok)
	{
		Wipe(output, 16U);
	}
	return ok;
}

bool Sce5AesBlock(const uint8_t *key, size_t keyLen, bool encrypt,
	const uint8_t input[16], uint8_t output[16])
{
	uint32_t plain[8] = {0};
	uint32_t wrapped[16] = {0};
	uint32_t block[4] = {0};
	uint32_t result[4] = {0};
	uint32_t unused[8] = {0};
	uint32_t keyType = 1U;
	uint32_t command = change_endian_long(encrypt ? 0U : 1U);
	fsp_err_t status = FSP_ERR_CRYPTO_SCE_FAIL;
	bool ok = false;

	if (!Sce5Ready() || (keyLen != 16U && keyLen != 32U))
	{
		Wipe(output, 16U);
		return false;
	}
	memcpy(plain, key, keyLen);
	memcpy(block, input, 16U);
	if (keyLen == 16U)
	{
		status = HW_SCE_GenerateAes128PlainKeyIndexSub(&keyType, unused, unused,
			unused, plain, wrapped);
		if (status == FSP_SUCCESS && !s_Fault)
		{
			status = HW_SCE_Aes128EncryptDecryptInitSub(&keyType, &command, wrapped, unused);
		}
		if (status == FSP_SUCCESS && !s_Fault)
		{
			HW_SCE_Aes128EncryptDecryptUpdateSub(block, result, 4U);
			if (!s_Fault)
			{
				status = HW_SCE_Aes128EncryptDecryptFinalSub();
			}
		}
	}
	else
	{
		status = HW_SCE_GenerateAes256PlainKeyIndexSub(&keyType, unused, unused,
			unused, plain, wrapped);
		if (status == FSP_SUCCESS && !s_Fault)
		{
			status = HW_SCE_Aes256EncryptDecryptInitSub(&keyType, &command, wrapped, unused);
		}
		if (status == FSP_SUCCESS && !s_Fault)
		{
			HW_SCE_Aes256EncryptDecryptUpdateSub(block, result, 4U);
			if (!s_Fault)
			{
				status = HW_SCE_Aes256EncryptDecryptFinalSub();
			}
		}
	}
	ok = Finish(status);
	if (ok)
	{
		memcpy(output, result, 16U);
	}
	else
	{
		Wipe(output, 16U);
	}
	Wipe(plain, sizeof(plain));
	Wipe(wrapped, sizeof(wrapped));
	Wipe(block, sizeof(block));
	Wipe(result, sizeof(result));
	return ok;
}
