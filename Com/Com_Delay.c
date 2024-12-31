#include "Com_Delay.h"

void for_delay_ms(uint32_t ms) {
    uint32_t Delay = ms * 72000 / 9; /* 72M时钟频率 */
    do {
        __NOP(); /* 空指令（NOP）来占用 CPU 时间 */
    } while (Delay--);
}
