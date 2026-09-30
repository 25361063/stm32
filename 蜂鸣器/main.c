#include <stdint.h>

/* ================= 寄存器定义（STM32F103，寄存器级操作） ================= */
#define RCC_APB2ENR  (*(volatile uint32_t *)0x40021018u)
#define RCC_APB1ENR  (*(volatile uint32_t *)0x4002101Cu)
#define GPIOA_CRL    (*(volatile uint32_t *)0x40010800u)

#define TIM3_CR1     (*(volatile uint32_t *)0x40000400u)
#define TIM3_CCMR1   (*(volatile uint32_t *)0x40000418u)
#define TIM3_CCER    (*(volatile uint32_t *)0x40000420u)
#define TIM3_PSC     (*(volatile uint32_t *)0x40000428u)
#define TIM3_ARR     (*(volatile uint32_t *)0x4000042Cu)
#define TIM3_CCR1    (*(volatile uint32_t *)0x40000434u)

/* ================= 参数 ================= */
#define BUZZER_FREQ_HZ  2000u                     /* 驱动频率 2kHz：无源蜂鸣器在共振点附近最响 */
#define TIMER_CNT_HZ    1000000u                  /* 定时器计数频率 = 8MHz HSI / (PSC+1) */
#define TIMER_ARR       (TIMER_CNT_HZ / BUZZER_FREQ_HZ - 1u)

static void delay(volatile uint32_t n)
{
    while (n--) { }                               /* 粗延时，仅用于控制响/停节奏 */
}

int main(void)
{
    /* 1. 开时钟：GPIOA 在 APB2，TIM3 在 APB1 */
    RCC_APB2ENR |= (1u << 2);
    RCC_APB1ENR |= (1u << 1);

    /* 2. PA6 设为复用推挽输出 50MHz，交给 TIM3_CH1 驱动 */
    GPIOA_CRL = (GPIOA_CRL & ~(0xFu << 24)) | (0xBu << 24);

    /* 3. TIM3 输出 PWM 方波：计数 1MHz，周期 2kHz，占空比 50% */
    TIM3_PSC   = 7;                               /* 8MHz / (7+1) = 1MHz */
    TIM3_ARR   = TIMER_ARR;                       /* 1MHz / 2kHz - 1 = 499 */
    TIM3_CCMR1 = (6u << 4);                       /* CH1 = PWM 模式 1 */
    TIM3_CCER  = 1u;                              /* 使能 CH1 输出 */
    TIM3_CCR1  = 0;                               /* 占空比 0%：输出低电平，静音 */
    TIM3_CR1   = 1u;                              /* 启动计数器 */

    /* 4. 循环：响约 0.2 秒 -> 停约 0.2 秒（方便确认是程序在控制） */
    while (1) {
        TIM3_CCR1 = (TIMER_ARR + 1u) / 2u;        /* 50% 占空比 -> 蜂鸣器响 */
        delay(400000u);
        TIM3_CCR1 = 0;                            /* 低电平 -> 停 */
        delay(400000u);
    }
}