// Cortex-M7 cache operations are checked by the UART register test.
#include "test_core.h"
typedef struct { uint32_t CCR; } TestScb_t;
extern TestScb_t g_TestScb;
#define SCB (&g_TestScb)
#define SCB_CCR_DC_Msk (1U << 16)
#ifdef __cplusplus
extern "C" {
#endif
void TestCache(int Op, void *p, int Size);
#ifdef __cplusplus
}
#endif
#define SCB_CleanDCache_by_Addr(p, n) TestCache(0, p, n)
#define SCB_InvalidateDCache_by_Addr(p, n) TestCache(1, p, n)
#define SCB_CleanInvalidateDCache_by_Addr(p, n) TestCache(2, p, n)
