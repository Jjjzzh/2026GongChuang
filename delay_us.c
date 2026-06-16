#include "delay_us.h"

/*
 * STM32F407  SysTick 时钟源:
 *   HAL_Init() → HAL_InitTick() → SysTick_Config()
 *   设置 SysTick->CTRL CLKSOURCE=1, 即使用 HCLK (处理器时钟)
 *   HCLK = 168MHz → SysTick 递减速率 = 168MHz
 *   因此 CPU_FREQUENCY_MHZ = 168
 */
#define CPU_FREQUENCY_MHZ    168

void delay_us(__IO uint32_t delay)
{
    int last, curr, val;
    int temp;

    while (delay != 0)
    {
        temp = delay > 900 ? 900 : delay;
        last = SysTick->VAL;
        curr = last - CPU_FREQUENCY_MHZ * temp;
        if (curr >= 0)
        {
            do
            {
                val = SysTick->VAL;
            }
            while ((val < last) && (val >= curr));
        }
        else
        {
            curr += CPU_FREQUENCY_MHZ * 1000;
            do
            {
                val = SysTick->VAL;
            }
            while ((val <= last) || (val > curr));
        }
        delay -= temp;
    }
}
