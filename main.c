#include <stdint.h>

#define RCC_APB2ENR  (*(volatile uint32_t *)0x40021018u)
#define GPIOA_CRL    (*(volatile uint32_t *)0x40010800u)
#define GPIOA_BSRR   (*(volatile uint32_t *)0x40010810u)

static void delay(volatile uint32_t n)
{
    while (n--) { }
}

int main(void)
{
    RCC_APB2ENR |= (1u << 2);   /* GPIOA clock enable */

    /* PA5: push-pull output, 2 MHz -> CRL[23:20] = 0b0010 */
    GPIOA_CRL = (GPIOA_CRL & ~(0xFu << 20)) | (0x2u << 20);

    while (1) {
        GPIOA_BSRR = (1u << 5);   /* PA5 = 1, LED on  */
        delay(300000);
        GPIOA_BSRR = (1u << 21);  /* PA5 = 0, LED off */
        delay(300000);
    }
}
