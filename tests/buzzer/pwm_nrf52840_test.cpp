#include "nrf.h"
#include "coredev/pwm.h"
#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)
int main()
{
 PwmDev_t dev = {};
 PwmCfg_t cfg = {0, 5000, PWM_MODE_EDGE, false, 6, nullptr};
 PwmChanCfg_t pins[] = {{0,PWM_POL_HIGH,0,26},{1,PWM_POL_LOW,1,2}};
 CHECK(PWMInit(&dev,&cfg));
 for(int i=0;i<4;i++) CHECK(NRF_PWM0->PSEL.OUT[i] & 0x80000000UL);
 CHECK(PWMOpenChannel(&dev,pins,2));
 CHECK(NRF_P0->OUTCLR == (1UL<<26));
 CHECK(NRF_P1->OUTSET == (1UL<<2));
 CHECK(NRF_P0->PIN_CNF[26] & GPIO_PIN_CNF_DIR_Msk);
 CHECK(NRF_P1->PIN_CNF[2] & GPIO_PIN_CNF_DIR_Msk);
 CHECK(PWMSetDutyCycle(&dev,0,50));
 CHECK(PWMStart(&dev,0));
 CHECK(NRF_PWM0->ENABLE == 1 && NRF_PWM0->PSEL.OUT[0] == 26);
 PWMStop(&dev);
 CHECK(NRF_PWM0->ENABLE == 0);
 for(int i=0;i<4;i++) CHECK(NRF_PWM0->PSEL.OUT[i] & 0x80000000UL);
 CHECK(NRF_P0->OUTCLR == (1UL<<26) && NRF_P1->OUTSET == (1UL<<2));
 CHECK(PWMStart(&dev,0)); // Restart retains pin assignments.
 CHECK(NRF_PWM0->PSEL.OUT[0] == 26 && NRF_PWM0->PSEL.OUT[1] == 34);
 PWMCloseChannel(&dev,1);
 PWMStop(&dev);
 CHECK(PWMStart(&dev,0));
 CHECK(NRF_PWM0->PSEL.OUT[1] & 0x80000000UL);
 PWMDisable(&dev);
 CHECK(NRF_PWM0->ENABLE == 0 && (NRF_PWM0->PSEL.OUT[0] & 0x80000000UL));
 PWMStop(&dev); // Repeated stop must be harmless.
 return 0;
}
