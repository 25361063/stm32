#include <stdint.h>

/* ================= 寄存器定义（STM32F103，寄存器级操作） ================= */
#define RCC_APB2ENR  (*(volatile uint32_t *)0x40021018u)
#define RCC_APB1ENR  (*(volatile uint32_t *)0x4002101Cu)
#define GPIOA_CRL    (*(volatile uint32_t *)0x40010800u)

/* 旋律声部：TIM3_CH1 -> PA6 */
#define TIM3_CR1     (*(volatile uint32_t *)0x40000400u)
#define TIM3_CCMR1   (*(volatile uint32_t *)0x40000418u)
#define TIM3_CCER    (*(volatile uint32_t *)0x40000420u)
#define TIM3_CNT     (*(volatile uint32_t *)0x40000424u)
#define TIM3_PSC     (*(volatile uint32_t *)0x40000428u)
#define TIM3_ARR     (*(volatile uint32_t *)0x4000042Cu)
#define TIM3_CCR1    (*(volatile uint32_t *)0x40000434u)

/* 低音声部：TIM2_CH1 -> PA0 */
#define TIM2_CR1     (*(volatile uint32_t *)0x40000000u)
#define TIM2_CCMR1   (*(volatile uint32_t *)0x40000018u)
#define TIM2_CCER    (*(volatile uint32_t *)0x40000020u)
#define TIM2_CNT     (*(volatile uint32_t *)0x40000024u)
#define TIM2_PSC     (*(volatile uint32_t *)0x40000028u)
#define TIM2_ARR     (*(volatile uint32_t *)0x4000002Cu)
#define TIM2_CCR1    (*(volatile uint32_t *)0x40000034u)

#define SYST_CSR     (*(volatile uint32_t *)0xE000E010u)
#define SYST_RVR     (*(volatile uint32_t *)0xE000E014u)
#define SYST_CVR     (*(volatile uint32_t *)0xE000E018u)

/* ================= 参数 ================= */
#define TIMER_CNT_HZ       1000000u   /* 定时器计数频率：8MHz HSI / (PSC+1) */
#define UNIT_MS            250u       /* 时间单位 = 八分音符(半拍)；改小=整体加快 */

#define MELODY_PITCH_PERCENT  300u    /* 旋律音高倍数(%)，越大越响（共振点约 2kHz） */
#define BASS_PITCH_PERCENT    600u    /* 低音部原始音区太低，倍率取大些 */

#define LEN(x)  (sizeof(x) / sizeof((x)[0]))

/* ================= 音符频率（原调，Hz） ================= */
/* 低音声部用 */
#define N_D3   147u
#define N_A2   110u
#define N_B2   123u
#define N_Fs2   92u
#define N_G2    98u

/* 中低音区 */
#define N_E3   165u
#define N_Fs3  185u
#define N_G3   196u
#define N_A3   220u
#define N_B3   247u
#define N_Cs4  277u
#define N_D4   294u
#define N_E4   330u
#define N_Fs4  370u
#define N_G4   392u

/* 中高音区 */
#define N_A4   440u
#define N_B4   494u
#define N_Cs5  554u
#define N_D5   587u
#define N_E5   659u
#define N_Fs5  740u
#define N_G5   784u

/* ================= 数据结构 ================= */
typedef struct {
    uint16_t freq;    /* 原调频率，0 = 休止 */
    uint8_t  units;   /* 时长，单位 = 八分音符（UNIT_MS 毫秒） */
} note_t;

typedef struct {
    const note_t *notes;
    uint8_t       count;
} phrase_t;

/* ============================================================
   卡农固定低音：D-A-B-F#-G-D-G-A，每音 4 拍（8 个八分音符）
   8 段变奏都建立在它之上，每段 8 小节 = 64 个八分音符
   ============================================================ */
static const uint16_t bass[8] = {
    N_D3, N_A2, N_B2, N_Fs2, N_G2, N_D3, N_G2, N_A2,
};

/* ---------- 第 1 段：主题（四分音符，经典下行） ---------- */
static const note_t ph_theme[] = {
    /* D : F#5 E5  D5  C#5  */
    {N_Fs5,2},{N_E5,2},{N_D5,2},{N_Cs5,2},
    /* A : B4  A4  B4  C#5  */
    {N_B4,2},{N_A4,2},{N_B4,2},{N_Cs5,2},
    /* Bm: D5  C#5 B4  A4   */
    {N_D5,2},{N_Cs5,2},{N_B4,2},{N_A4,2},
    /* F#m: G4 F#4 G4 E4    */
    {N_G4,2},{N_Fs4,2},{N_G4,2},{N_E4,2},
    /* G : D5  E5  F#5 G5   */
    {N_D5,2},{N_E5,2},{N_Fs5,2},{N_G5,2},
    /* D : F#5 E5  D5  C#5  */
    {N_Fs5,2},{N_E5,2},{N_D5,2},{N_Cs5,2},
    /* G : B4  C#5 D5  E5   */
    {N_B4,2},{N_Cs5,2},{N_D5,2},{N_E5,2},
    /* A : D5  C#5 B4  A4   */
    {N_D5,2},{N_Cs5,2},{N_B4,2},{N_A4,2},
};

/* ---------- 第 2 段：主题 · 低八度（音色更低沉） ---------- */
static const note_t ph_theme_low[] = {
    {N_Fs4,2},{N_E4,2},{N_D4,2},{N_Cs4,2},
    {N_B3,2},{N_A3,2},{N_B3,2},{N_Cs4,2},
    {N_D4,2},{N_Cs4,2},{N_B3,2},{N_A3,2},
    {N_G3,2},{N_Fs3,2},{N_G3,2},{N_E3,2},
    {N_D4,2},{N_E4,2},{N_Fs4,2},{N_G4,2},
    {N_Fs4,2},{N_E4,2},{N_D4,2},{N_Cs4,2},
    {N_B3,2},{N_Cs4,2},{N_D4,2},{N_E4,2},
    {N_D4,2},{N_Cs4,2},{N_B3,2},{N_A3,2},
};

/* ---------- 第 3 段：上行分解和弦（八分音符琶音） ---------- */
static const note_t ph_arp_up[] = {
    /* D  : D4 F#4 A4 D5 A4 F#4 D4 F#4 */
    {N_D4,1},{N_Fs4,1},{N_A4,1},{N_D5,1},{N_A4,1},{N_Fs4,1},{N_D4,1},{N_Fs4,1},
    /* A  : A3 C#4 E4 A4 E4 C#4 A3 C#4 */
    {N_A3,1},{N_Cs4,1},{N_E4,1},{N_A4,1},{N_E4,1},{N_Cs4,1},{N_A3,1},{N_Cs4,1},
    /* Bm : B3 D4 F#4 B4 F#4 D4 B3 D4  */
    {N_B3,1},{N_D4,1},{N_Fs4,1},{N_B4,1},{N_Fs4,1},{N_D4,1},{N_B3,1},{N_D4,1},
    /* F#m: F#3 A3 C#4 F#4 C#4 A3 F#3 A3 */
    {N_Fs3,1},{N_A3,1},{N_Cs4,1},{N_Fs4,1},{N_Cs4,1},{N_A3,1},{N_Fs3,1},{N_A3,1},
    /* G  : G3 B3 D4 G4 D4 B3 G3 B3   */
    {N_G3,1},{N_B3,1},{N_D4,1},{N_G4,1},{N_D4,1},{N_B3,1},{N_G3,1},{N_B3,1},
    /* D  : D4 F#4 A4 D5 A4 F#4 D4 F#4 */
    {N_D4,1},{N_Fs4,1},{N_A4,1},{N_D5,1},{N_A4,1},{N_Fs4,1},{N_D4,1},{N_Fs4,1},
    /* G  : G3 B3 D4 G4 D4 B3 G3 B3   */
    {N_G3,1},{N_B3,1},{N_D4,1},{N_G4,1},{N_D4,1},{N_B3,1},{N_G3,1},{N_B3,1},
    /* A  : A3 C#4 E4 A4 E4 C#4 A3 C#4 */
    {N_A3,1},{N_Cs4,1},{N_E4,1},{N_A4,1},{N_E4,1},{N_Cs4,1},{N_A3,1},{N_Cs4,1},
};

/* ---------- 第 4 段：下行音阶（八分音符级进） ---------- */
static const note_t ph_scale_down[] = {
    /* D  : D5  C#5 B4  A4  G4  F#4 E4  D4  */
    {N_D5,1},{N_Cs5,1},{N_B4,1},{N_A4,1},{N_G4,1},{N_Fs4,1},{N_E4,1},{N_D4,1},
    /* A  : A4  G4  F#4 E4  D4  C#4 B3  A3  */
    {N_A4,1},{N_G4,1},{N_Fs4,1},{N_E4,1},{N_D4,1},{N_Cs4,1},{N_B3,1},{N_A3,1},
    /* Bm : B4  A4  G4  F#4 E4  D4  C#4 B3  */
    {N_B4,1},{N_A4,1},{N_G4,1},{N_Fs4,1},{N_E4,1},{N_D4,1},{N_Cs4,1},{N_B3,1},
    /* F#m: F#4 E4  D4  C#4 B3  A3  G3  F#3 */
    {N_Fs4,1},{N_E4,1},{N_D4,1},{N_Cs4,1},{N_B3,1},{N_A3,1},{N_G3,1},{N_Fs3,1},
    /* G  : G4  F#4 E4  D4  C#4 B3  A3  G3  */
    {N_G4,1},{N_Fs4,1},{N_E4,1},{N_D4,1},{N_Cs4,1},{N_B3,1},{N_A3,1},{N_G3,1},
    /* D  : D5  C#5 B4  A4  G4  F#4 E4  D4  */
    {N_D5,1},{N_Cs5,1},{N_B4,1},{N_A4,1},{N_G4,1},{N_Fs4,1},{N_E4,1},{N_D4,1},
    /* G  : G4  F#4 E4  D4  C#4 B3  A3  G3  */
    {N_G4,1},{N_Fs4,1},{N_E4,1},{N_D4,1},{N_Cs4,1},{N_B3,1},{N_A3,1},{N_G3,1},
    /* A  : A4  G4  F#4 E4  D4  C#4 B3  A3  */
    {N_A4,1},{N_G4,1},{N_Fs4,1},{N_E4,1},{N_D4,1},{N_Cs4,1},{N_B3,1},{N_A3,1},
};

/* ---------- 第 5 段：长音和声（二分音符，根音 + 五音） ---------- */
static const note_t ph_long[] = {
    {N_D4,4},{N_A4,4},        /* D  */
    {N_A3,4},{N_E4,4},        /* A  */
    {N_B3,4},{N_Fs4,4},       /* Bm */
    {N_Fs3,4},{N_Cs4,4},      /* F#m*/
    {N_G3,4},{N_D4,4},        /* G  */
    {N_D4,4},{N_A4,4},        /* D  */
    {N_G3,4},{N_D4,4},        /* G  */
    {N_A3,4},{N_E4,4},        /* A  */
};

/* ---------- 第 7 段：上行音阶（八分音符级进） ---------- */
static const note_t ph_scale_up[] = {
    /* D  : D4  E4  F#4 G4  A4  B4  C#5 D5  */
    {N_D4,1},{N_E4,1},{N_Fs4,1},{N_G4,1},{N_A4,1},{N_B4,1},{N_Cs5,1},{N_D5,1},
    /* A  : A3  B3  C#4 D4  E4  F#4 G4  A4  */
    {N_A3,1},{N_B3,1},{N_Cs4,1},{N_D4,1},{N_E4,1},{N_Fs4,1},{N_G4,1},{N_A4,1},
    /* Bm : B3  C#4 D4  E4  F#4 G4  A4  B4  */
    {N_B3,1},{N_Cs4,1},{N_D4,1},{N_E4,1},{N_Fs4,1},{N_G4,1},{N_A4,1},{N_B4,1},
    /* F#m: F#3 G3  A3  B3  C#4 D4  E4  F#4 */
    {N_Fs3,1},{N_G3,1},{N_A3,1},{N_B3,1},{N_Cs4,1},{N_D4,1},{N_E4,1},{N_Fs4,1},
    /* G  : G3  A3  B3  C#4 D4  E4  F#4 G4  */
    {N_G3,1},{N_A3,1},{N_B3,1},{N_Cs4,1},{N_D4,1},{N_E4,1},{N_Fs4,1},{N_G4,1},
    /* D  : D4  E4  F#4 G4  A4  B4  C#5 D5  */
    {N_D4,1},{N_E4,1},{N_Fs4,1},{N_G4,1},{N_A4,1},{N_B4,1},{N_Cs5,1},{N_D5,1},
    /* G  : G3  A3  B3  C#4 D4  E4  F#4 G4  */
    {N_G3,1},{N_A3,1},{N_B3,1},{N_Cs4,1},{N_D4,1},{N_E4,1},{N_Fs4,1},{N_G4,1},
    /* A  : A3  B3  C#4 D4  E4  F#4 G4  A4  */
    {N_A3,1},{N_B3,1},{N_Cs4,1},{N_D4,1},{N_E4,1},{N_Fs4,1},{N_G4,1},{N_A4,1},
};

/* ---------- 第 8 段：主题回归 + 收束在主音 ---------- */
static const note_t ph_final[] = {
    {N_Fs5,2},{N_E5,2},{N_D5,2},{N_Cs5,2},
    {N_B4,2},{N_A4,2},{N_B4,2},{N_Cs5,2},
    {N_D5,2},{N_Cs5,2},{N_B4,2},{N_A4,2},
    {N_G4,2},{N_Fs4,2},{N_G4,2},{N_E4,2},
    {N_D5,2},{N_E5,2},{N_Fs5,2},{N_G5,2},
    {N_Fs5,2},{N_E5,2},{N_D5,2},{N_Cs5,2},
    {N_B4,2},{N_Cs5,2},{N_D5,2},{N_E5,2},
    {N_D5,2},{N_Cs5,2},{N_D5,4},     /* 停在主音 D5 上收束 */
};

/* ---------- 全曲：8 段变奏，每段 64 个八分音符 ---------- */
static const phrase_t canon[] = {
    { ph_theme,      LEN(ph_theme)      },   /* 1 主题           */
    { ph_theme_low,  LEN(ph_theme_low)  },   /* 2 主题·低八度     */
    { ph_arp_up,     LEN(ph_arp_up)     },   /* 3 上行分解和弦    */
    { ph_scale_down, LEN(ph_scale_down) },   /* 4 下行音阶        */
    { ph_long,       LEN(ph_long)       },   /* 5 长音和声        */
    { ph_theme,      LEN(ph_theme)      },   /* 6 主题再现        */
    { ph_scale_up,   LEN(ph_scale_up)   },   /* 7 上行音阶        */
    { ph_final,      LEN(ph_final)      },   /* 8 主题回归+收束   */
};

/* ================= 毫秒延时（SysTick，8MHz 内核时钟） ================= */
static void delay_ms(uint32_t ms)
{
    SYST_RVR = 8000u - 1u;
    SYST_CVR = 0u;
    SYST_CSR = 5u;
    while (ms--) {
        while ((SYST_CSR & (1u << 16)) == 0u) { }
    }
    SYST_CSR = 0u;
}

/* ================= 设置旋律声部音高（0 = 静音） ================= */
static void melody_tone(uint16_t base_freq)
{
    if (base_freq == 0u) {
        TIM3_CCR1 = 0u;
        return;
    }
    uint32_t freq = (uint32_t)base_freq * MELODY_PITCH_PERCENT / 100u;
    uint32_t arr  = (TIMER_CNT_HZ / freq) - 1u;

    TIM3_ARR  = arr;
    TIM3_CNT  = 0u;
    TIM3_CCR1 = (arr + 1u) / 2u;
}

/* ================= 设置低音声部音高（0 = 静音） ================= */
static void bass_tone(uint16_t base_freq)
{
    if (base_freq == 0u) {
        TIM2_CCR1 = 0u;
        return;
    }
    uint32_t freq = (uint32_t)base_freq * BASS_PITCH_PERCENT / 100u;
    uint32_t arr  = (TIMER_CNT_HZ / freq) - 1u;

    TIM2_ARR  = arr;
    TIM2_CNT  = 0u;
    TIM2_CCR1 = (arr + 1u) / 2u;
}

int main(void)
{
    /* 1. 开时钟：GPIOA 在 APB2；TIM2、TIM3 在 APB1 */
    RCC_APB2ENR |= (1u << 2);
    RCC_APB1ENR |= (1u << 0) | (1u << 1);

    /* 2. PA6 / PA0 设为复用推挽输出 50MHz */
    GPIOA_CRL = (GPIOA_CRL & ~(0xFu << 24)) | (0xBu << 24);   /* PA6 = TIM3_CH1 */
    GPIOA_CRL = (GPIOA_CRL & ~0xFu)         | 0xBu;           /* PA0 = TIM2_CH1 */

    /* 3. 两个定时器：1MHz 计数、PWM 模式 1，先静音 */
    TIM3_PSC   = 7;
    TIM3_ARR   = 999u;
    TIM3_CCMR1 = (6u << 4);
    TIM3_CCER  = 1u;
    TIM3_CCR1  = 0u;
    TIM3_CR1   = 1u;

    TIM2_PSC   = 7;
    TIM2_ARR   = 999u;
    TIM2_CCMR1 = (6u << 4);
    TIM2_CCER  = 1u;
    TIM2_CCR1  = 0u;
    TIM2_CR1   = 1u;

    /* 4. 循环播放完整卡农：8 段变奏
          旋律按乐句表推进，低音每 8 个八分音符（4 拍）换一次音 */
    while (1) {
        uint32_t unit = 0u;                      /* 全曲时间计数（单位：八分音符） */

        for (uint32_t p = 0u; p < LEN(canon); p++) {
            const phrase_t *ph = &canon[p];

            for (uint32_t n = 0u; n < ph->count; n++) {
                melody_tone(ph->notes[n].freq);

                for (uint32_t u = 0u; u < ph->notes[n].units; u++) {
                    if ((unit % 8u) == 0u) {     /* 每 4 拍换低音 */
                        bass_tone(bass[(unit / 8u) % LEN(bass)]);
                    }
                    delay_ms(UNIT_MS);
                    unit++;
                }
            }
        }

        /* 全曲结束，静音歇一下再从头循环 */
        melody_tone(0u);
        bass_tone(0u);
        delay_ms(1500u);
    }
}