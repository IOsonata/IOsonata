#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "bluetooth/bt_pds.h"

static uint8_t g_Nvm[4 * 512];
static int g_FailAt = -1;
static int g_OpCount;

static bool FailNow()
{
    if (g_FailAt >= 0 && g_OpCount++ == g_FailAt)
    {
        return true;
    }
    return false;
}

static int NvmRead(uint32_t Off, void *pBuf, uint32_t Len)
{
    if (Off > sizeof(g_Nvm) || Len > sizeof(g_Nvm) - Off)
    {
        return -EINVAL;
    }
    if (Len > 0)
    {
        memcpy(pBuf, &g_Nvm[Off], Len);
    }
    return 0;
}

static int NvmWrite(uint32_t Off, const void *pData, uint32_t Len)
{
    if (Off > sizeof(g_Nvm) || Len > sizeof(g_Nvm) - Off ||
        (Off & 3U) != 0 || (Len & 3U) != 0)
    {
        return -EINVAL;
    }
    if (FailNow())
    {
        return -EIO;
    }
    memcpy(&g_Nvm[Off], pData, Len);
    return 0;
}

static int NvmErase(uint32_t Off)
{
    if ((Off % 512U) != 0 || Off > sizeof(g_Nvm) - 512U)
    {
        return -EINVAL;
    }
    if (FailNow())
    {
        return -EIO;
    }
    memset(&g_Nvm[Off], 0xFF, 512);
    return 0;
}

static const BtPdsNvm_t g_Impl = {
    0, sizeof(g_Nvm), 512, 4, NvmRead, NvmWrite, NvmErase
};

static void ReadCheck(uint32_t Id, uint8_t Value)
{
    uint8_t data[32];
    memset(data, 0, sizeof(data));
    ssize_t len = BtPdsRead(Id, data, sizeof(data));
    assert(len == (ssize_t)sizeof(data));
    for (uint8_t b : data)
    {
        assert(b == Value);
    }
}

static void WriteValue(uint32_t Id, uint8_t Value)
{
    uint8_t data[32];
    memset(data, Value, sizeof(data));
    assert(BtPdsWrite(Id, data, sizeof(data)) == (ssize_t)sizeof(data));
}

int main()
{
    memset(g_Nvm, 0xFF, sizeof(g_Nvm));
    assert(BtPdsInit(&g_Impl) == 0);

    uint8_t expected[6] = {};
    for (int i = 0; i < 30; i++)
    {
        uint32_t id = (uint32_t)(i % 6) + 1U;
        uint8_t value = (uint8_t)(i + 1);
        WriteValue(id, value);
        expected[id - 1U] = value;
    }

    uint8_t snapshot[sizeof(g_Nvm)];
    memcpy(snapshot, g_Nvm, sizeof(snapshot));

    // Exercise every medium-operation failure point around the first GC. The
    // pre-existing live values must survive whether the new write committed or
    // not. Remount performs interrupted-GC recovery.
    for (int fail = 0; fail < 16; fail++)
    {
        memcpy(g_Nvm, snapshot, sizeof(snapshot));
        g_OpCount = 0;
        g_FailAt = fail;

        uint8_t data[32];
        memset(data, 0xA5, sizeof(data));
        (void)BtPdsWrite(99U, data, sizeof(data));

        g_FailAt = -1;
        assert(BtPdsInit(&g_Impl) == 0);
        for (uint32_t id = 1; id <= 6; id++)
        {
            ReadCheck(id, expected[id - 1U]);
        }
    }

    memcpy(g_Nvm, snapshot, sizeof(snapshot));
    g_FailAt = -1;
    assert(BtPdsInit(&g_Impl) == 0);
    WriteValue(99U, 0xA5);
    ReadCheck(99U, 0xA5);

    assert(BtPdsDelete(3U) == 0);
    assert(BtPdsRead(3U, nullptr, 0) == -ENOENT);
    assert(BtPdsInit(&g_Impl) == 0);
    assert(BtPdsRead(3U, nullptr, 0) == -ENOENT);
    for (uint32_t id = 1; id <= 6; id++)
    {
        if (id != 3U)
        {
            ReadCheck(id, expected[id - 1U]);
        }
    }

    puts("BtPds stage2 test passed");
    return 0;
}
