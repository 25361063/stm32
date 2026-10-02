#include <stdint.h>

extern int main(void);

void Reset_Handler(void);
void Default_Handler(void);

void NMI_Handler(void)        __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void)  __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void)  __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void)        __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void)    __attribute__((weak, alias("Default_Handler")));

#define STACK_TOP 0x20005000u   /* top of 20KB SRAM */

/* 链接器生成的区域符号（对应 scatter.sct 里的 RW_IRAM1 区域）：
 * 上电必须做两件事，否则带初值的变量读到的是垃圾值、未初始化变量不归零：
 *   1. 把 Flash 里保存的 RW 数据初值拷贝到 RAM
 *   2. 把 ZI（零初始化）段清零 */
extern uint32_t g_rw_load  __asm("Load$$RW_IRAM1$$Base");
extern uint32_t g_rw_start __asm("Image$$RW_IRAM1$$RW$$Base");
extern uint32_t g_rw_end   __asm("Image$$RW_IRAM1$$RW$$Limit");
extern uint32_t g_zi_start __asm("Image$$RW_IRAM1$$ZI$$Base");
extern uint32_t g_zi_end   __asm("Image$$RW_IRAM1$$ZI$$Limit");

__attribute__((used, section("RESET")))
const uint32_t g_vectors[] = {
    STACK_TOP,
    (uint32_t)Reset_Handler,
    (uint32_t)NMI_Handler,
    (uint32_t)HardFault_Handler,
    (uint32_t)MemManage_Handler,
    (uint32_t)BusFault_Handler,
    (uint32_t)UsageFault_Handler,
    0, 0, 0, 0,
    (uint32_t)SVC_Handler,
    (uint32_t)DebugMon_Handler,
    0,
    (uint32_t)PendSV_Handler,
    (uint32_t)SysTick_Handler,
};

void Reset_Handler(void)
{
    uint32_t *src = &g_rw_load;
    uint32_t *dst;

    /* 1. RW 数据：从 Flash 拷贝初值到 RAM */
    for (dst = &g_rw_start; dst < &g_rw_end; ) {
        *dst++ = *src++;
    }
    /* 2. ZI 段清零 */
    for (dst = &g_zi_start; dst < &g_zi_end; dst++) {
        *dst = 0u;
    }

    main();
    while (1) { }
}

void Default_Handler(void)
{
    while (1) { }
}