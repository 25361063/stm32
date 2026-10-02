#include <stdint.h>

/* ============================================================================
 * 项目：蓝牙模块调试（串口蓝牙模块 + OLED 状态显示）
 * 硬件：STM32F103C8 + 串口蓝牙模块（HC-05 / HC-06 等）+ SSD1306 OLED（128x64）
 *
 * 接线（断电接线）：
 *   蓝牙 VDD -> 板子 3.3V（模块若注明支持 5V/自带稳压也可接 5V，拿不准用 3.3V）
 *   蓝牙 GND -> 板子 GND
 *   蓝牙 TXD -> 板子 PA10（USART1_RX）
 *   蓝牙 RXD -> 板子 PA9 （USART1_TX）
 *   ★ TX/RX 必须交叉：模块的"发"接 MCU 的"收"，模块的"收"接 MCU 的"发"
 *   蓝牙 LED -> 悬空（状态指示输出脚，不接不影响通信）
 *   OLED 保持原接线不变：GND->GND，VCC->3.3V，SCL->PB6，SDA->PB7
 *
 * 本程序做什么：
 *   1. USART1，115200 8N1（本蓝牙模块实测默认波特率）
 *   2. 上电发 "STM32 BT ready"，之后每 2 秒发一行心跳 "STM32 BT OK n"
 *   3. 收到什么就回显什么（手机发 "hello" 返回 "hello"）
 *   4. OLED 中文状态界面（16x16 点阵中文 + 8x16 ASCII，字库见 font_data.h）：
 *        第一行：蓝牙 115200           （串口参数）
 *        第二行：接收 12 发送 3         （收到字节数 / 已发心跳数）
 *        第三行：活动 * / 活动 -        （近 2 秒是否有数据到达）
 *        第四行：最近 hello...          （最近收到的 11 个可打印字符）
 *   5. 每收到一个字节翻转一次 PA5（LED 还接着的话能看到接收活动）
 *
 * 调试方法：
 *   安卓手机装"蓝牙串口助手"类 App，配对码一般 1234（有的 0000）。
 *   连接后：手机能看到心跳、屏幕 HB 计数递增；发送文字能原样返回，
 *   屏幕 RX 计数增加、LAST 显示发送的内容。
 *   注意：iPhone 不支持经典蓝牙 SPP，调试 HC-05/06 请用安卓；
 *   iOS 需要 BLE 模块（如 HM-10）才可以。
 *
 * 实现说明：
 *   寄存器级操作，无 HAL 库。串口接收为轮询方式，主循环非阻塞；
 *   为了不丢串口数据，OLED 只在"接收空闲 300ms 后"才刷新
 *   （连续大流量接收期间屏幕会暂停刷新，属正常取舍）。
 * ============================================================================ */

/* ================= 寄存器定义（STM32F103，寄存器级操作） ================= */
#define RCC_APB2ENR  (*(volatile uint32_t *)0x40021018u)
#define GPIOA_CRL    (*(volatile uint32_t *)0x40010800u)
#define GPIOA_CRH    (*(volatile uint32_t *)0x40010804u)
#define GPIOA_IDR    (*(volatile uint32_t *)0x40010808u)
#define GPIOA_ODR    (*(volatile uint32_t *)0x4001080Cu)
#define GPIOA_BSRR   (*(volatile uint32_t *)0x40010810u)
#define GPIOB_CRL    (*(volatile uint32_t *)0x40010C00u)
#define GPIOB_IDR    (*(volatile uint32_t *)0x40010C08u)
#define GPIOB_ODR    (*(volatile uint32_t *)0x40010C0Cu)

#define USART1_SR    (*(volatile uint32_t *)0x40013800u)
#define USART1_DR    (*(volatile uint32_t *)0x40013804u)
#define USART1_BRR   (*(volatile uint32_t *)0x40013808u)
#define USART1_CR1   (*(volatile uint32_t *)0x4001380Cu)

#define SYST_CSR     (*(volatile uint32_t *)0xE000E010u)
#define SYST_RVR     (*(volatile uint32_t *)0xE000E014u)
#define SYST_CVR     (*(volatile uint32_t *)0xE000E018u)

/* ================= 参数 ================= */
#define HEARTBEAT_MS     2000u   /* 心跳间隔(ms) */
#define LAST_CHARS_N     11u     /* 最近收到的字符窗口长度（11 个 ASCII 字符 = 88px） */

/* 串口波特率 = 115200（实测本模块的默认速率；8MHz/(16x115200) = 4.3403 -> BRR 0x45） */

#define SCL_PIN          6u      /* OLED：PB6 = SCL */
#define SDA_PIN          7u      /* OLED：PB7 = SDA */
#define OLED_ADDR8_A     0x78u   /* 0x3C << 1，常见默认地址 */
#define OLED_ADDR8_B     0x7Au   /* 0x3D << 1，少数模块 */
#define I2C_DELAY_LOOPS  6u      /* 软件 I2C 半周期延时，改小 = 更快 */

#define FB_SIZE          (128u * 8u)   /* 1024 字节显存 */

/* ================= 全局状态 ================= */
static uint8_t g_addr8 = OLED_ADDR8_A;   /* 当前使用的 I2C 从机地址 */
static uint8_t g_fb[FB_SIZE];            /* 显存：1 字节 = 1 列 8 像素 */
static uint32_t g_scl_pin = SCL_PIN;     /* 软件 I2C 引脚角色 */
static uint32_t g_sda_pin = SDA_PIN;

uint32_t g_uart_rx = 0u;                 /* 累计收到的字节数（调试器可读） */
uint32_t g_uart_hb = 0u;                 /* 累计发送的心跳数 */
uint32_t g_last_rx_ms = 0u;              /* 最近一次收到数据的时刻(ms) */
char     g_last_chars[LAST_CHARS_N + 1u];/* 最近收到的可打印字符窗口 */

/* ============ 字库：16px 点阵（含中文），由 gen_font.ps1 生成 ============ */
#include "font_data.h"

/* ================= 毫秒计时（SysTick 轮询，非中断） ================= */
static void systick_init(void)
{
    SYST_RVR = 8000u - 1u;   /* 8MHz -> 1ms */
    SYST_CVR = 0u;
    SYST_CSR = 5u;           /* ENABLE=1，CLKSOURCE=内核时钟 */
}

/* 在主循环里反复调用；每次读 CSR 顺便清 COUNTFLAG，累加毫秒数 */
static uint32_t systick_ms(void)
{
    static uint32_t ms = 0u;

    if ((SYST_CSR & (1u << 16)) != 0u) ms++;
    return ms;
}

/* 轮询式毫秒延时（自检等场合用，正常流程不使用） */
static void delay_ms(uint32_t ms)
{
    uint32_t t0 = systick_ms();
    while ((uint32_t)(systick_ms() - t0) < ms) { }
}

/* ================= 串口（USART1，9600 8N1） ================= */
static void usart1_init(void)
{
    RCC_APB2ENR |= (1u << 14);   /* USART1EN */

    /* PA9 = USART1_TX：复用推挽输出；PA10 = USART1_RX：浮空输入 */
    GPIOA_CRH = (GPIOA_CRH & ~0xFF0u) | (0xBu << 4) | (0x4u << 8);

    /* 波特率 115200：8MHz / (16 x 115200) = 4.3403 -> 尾数 4、小数 5 -> 0x45 */
    USART1_BRR = 0x45u;

    /* 8 位数据、无校验、1 停止位，使能 USART + 发送 + 接收 */
    USART1_CR1 = (1u << 13) | (1u << 3) | (1u << 2);   /* UE | TE | RE */
}

static void usart1_send_byte(uint8_t b)
{
    while ((USART1_SR & (1u << 7)) == 0u) { }   /* 等 TXE */
    USART1_DR = b;
}

static void usart1_send_str(const char *s)
{
    while (*s) usart1_send_byte((uint8_t)*s++);
}

static void usart1_send_u32(uint32_t v)
{
    char tmp[10];
    uint8_t n = 0u;

    if (v == 0u) { usart1_send_byte('0'); return; }
    while (v) { tmp[n++] = (char)('0' + (v % 10u)); v /= 10u; }
    while (n) usart1_send_byte((uint8_t)tmp[--n]);
}

/* 尝试接收一个字节：收到返回 1，无数据返回 0，上溢时只清标志不算数据 */
static uint32_t usart1_try_recv(uint8_t *b)
{
    uint32_t sr = USART1_SR;

    if ((sr & ((1u << 5) | (1u << 3))) == 0u) return 0u;   /* RXNE / ORE 都没有 */
    *b = (uint8_t)USART1_DR;                               /* 读 DR 清标志 */
    return (sr & (1u << 5)) ? 1u : 0u;
}

/* ================= 软件 I2C（PB6=SCL，PB7=SDA，开漏输出） ================= */
static void i2c_delay(void)
{
    for (volatile uint32_t i = 0; i < I2C_DELAY_LOOPS; i++) { }
}

static void scl_set(uint32_t level)
{
    if (level) GPIOB_ODR |= (1u << g_scl_pin);
    else       GPIOB_ODR &= ~(1u << g_scl_pin);
    i2c_delay();
}

static void sda_set(uint32_t level)
{
    if (level) GPIOB_ODR |= (1u << g_sda_pin);
    else       GPIOB_ODR &= ~(1u << g_sda_pin);
    i2c_delay();
}

static uint32_t sda_get(void)
{
    return (GPIOB_IDR >> g_sda_pin) & 1u;   /* 开漏输出模式下仍可读回引脚电平 */
}

static void i2c_start(void)
{
    sda_set(1); scl_set(1);   /* 总线空闲：双高 */
    sda_set(0);               /* SCL 高时 SDA 下降 = 起始条件 */
    scl_set(0);
}

static void i2c_stop(void)
{
    sda_set(0); scl_set(1);   /* 先抬 SCL */
    sda_set(1);               /* SCL 高时 SDA 上升 = 停止条件 */
}

static uint32_t i2c_write_byte(uint8_t b)
{
    uint32_t ack;

    for (int i = 7; i >= 0; i--) {
        sda_set((b >> i) & 1u);
        scl_set(1);
        scl_set(0);
    }
    sda_set(1);               /* 释放 SDA，读从机 ACK */
    scl_set(1);
    ack = (sda_get() == 0u);
    scl_set(0);
    return ack;
}

/* ================= SSD1306 驱动 ================= */
static void oled_cmd(uint8_t c)
{
    i2c_start();
    i2c_write_byte(g_addr8);
    i2c_write_byte(0x00u);    /* 控制字节：命令 */
    i2c_write_byte(c);
    i2c_stop();
}

static void oled_data(const uint8_t *buf, uint16_t len)
{
    i2c_start();
    i2c_write_byte(g_addr8);
    i2c_write_byte(0x40u);    /* 控制字节：数据 */
    while (len--) i2c_write_byte(*buf++);
    i2c_stop();
}

static uint32_t oled_probe(uint8_t addr8)
{
    uint32_t ack;

    i2c_start();
    ack = i2c_write_byte(addr8);
    i2c_stop();
    return ack;
}

static void oled_init(void)
{
    delay_ms(100);            /* 等模块上电稳定 */

    /* 自动探测地址：0x3C 无应答则用 0x3D */
    if (!oled_probe(OLED_ADDR8_A)) {
        g_addr8 = OLED_ADDR8_B;
    }

    oled_cmd(0xAEu);                   /* 关显示 */
    oled_cmd(0xD5u); oled_cmd(0x80u);  /* 时钟分频 */
    oled_cmd(0xA8u); oled_cmd(0x3Fu);  /* 多路复用 = 64 行 */
    oled_cmd(0xD3u); oled_cmd(0x00u);  /* 显示偏移 0 */
    oled_cmd(0x40u);                   /* 起始行 0 */
    oled_cmd(0x8Du); oled_cmd(0x14u);  /* 电荷泵开 */
    oled_cmd(0x20u); oled_cmd(0x00u);  /* 水平寻址模式 */
    oled_cmd(0xA1u);                   /* 列 127 映射到 SEG0 */
    oled_cmd(0xC8u);                   /* 行扫描反向 */
    oled_cmd(0xDAu); oled_cmd(0x12u);  /* COM 引脚配置 */
    oled_cmd(0x81u); oled_cmd(0xCFu);  /* 对比度 */
    oled_cmd(0xD9u); oled_cmd(0xF1u);  /* 预充电周期 */
    oled_cmd(0xDBu); oled_cmd(0x40u);  /* VCOMH 电压 */
    oled_cmd(0xA4u);                   /* 跟随显存显示 */
    oled_cmd(0xA6u);                   /* 正常显示（非反色） */
    oled_cmd(0x2Eu);                   /* 关闭滚屏 */
    oled_cmd(0xAFu);                   /* 开显示 */
}

/* 把 1024 字节显存整屏刷到屏幕（约 200ms，调用时机会影响串口接收，见主循环） */
static void oled_flush(void)
{
    oled_cmd(0x21u); oled_cmd(0x00u); oled_cmd(0x7Fu);  /* 列 0~127 */
    oled_cmd(0x22u); oled_cmd(0x00u); oled_cmd(0x07u);  /* 页 0~7 */
    oled_data(g_fb, FB_SIZE);
}

/* ================= 显存绘制 ================= */
static void fb_clear(void)
{
    for (uint32_t i = 0; i < FB_SIZE; i++) g_fb[i] = 0u;
}

static void fb_fill(uint8_t v)
{
    for (uint32_t i = 0; i < FB_SIZE; i++) g_fb[i] = v;
}

/* 点亮一个像素（x 0~127，y 0~63），超出屏幕自动忽略 */
static void fb_pixel(uint8_t x, uint8_t y)
{
    if (x > 127u || y > 63u) return;
    g_fb[(uint16_t)(y >> 3) * 128u + x] |= (uint8_t)(1u << (y & 7u));
}

/* 画一个 8x16 的 ASCII 字符（行优先，每行 1 字节，MSB 在左） */
static void fb_ascii16(uint8_t x, uint8_t y, uint8_t ch)
{
    const uint8_t *g;

    if (ch < 0x20u || ch > 0x7Eu) ch = ' ';
    g = font_ascii16[ch - 0x20u];
    for (uint8_t r = 0u; r < 16u; r++) {
        for (uint8_t c = 0u; c < 8u; c++) {
            if (g[r] & (0x80u >> c)) fb_pixel((uint8_t)(x + c), (uint8_t)(y + r));
        }
    }
}

/* 画一个 16x16 的中文字形（行优先，每行 2 字节，左 8 位在前） */
static void fb_cn16(uint8_t x, uint8_t y, const uint8_t *g)
{
    for (uint8_t r = 0u; r < 16u; r++) {
        uint16_t bits = (uint16_t)(((uint16_t)g[r * 2u] << 8) | g[r * 2u + 1u]);
        for (uint8_t c = 0u; c < 16u; c++) {
            if (bits & (0x8000u >> c)) fb_pixel((uint8_t)(x + c), (uint8_t)(y + r));
        }
    }
}

/* 画一行文本（中英混排，UTF-8 编码）：中文 16px 宽，ASCII 8px 宽，右边超出自动裁掉 */
static void fb_text16(uint8_t x, uint8_t y, const char *s)
{
    while (*s != 0) {
        if (x >= 128u) break;                     /* 一行画满即停 */
        if ((uint8_t)*s >= 0xE0u) {               /* 三字节 UTF-8 = 中文 */
            for (uint8_t i = 0u; i < 10u; i++) {
                if ((uint8_t)s[0] == (uint8_t)cn_utf8[i][0] &&
                    (uint8_t)s[1] == (uint8_t)cn_utf8[i][1] &&
                    (uint8_t)s[2] == (uint8_t)cn_utf8[i][2]) {
                    fb_cn16(x, y, font_cn16[i]);
                    break;
                }
            }
            x += 16u;
            s += 3;
        } else {
            fb_ascii16(x, y, (uint8_t)*s);
            x += 8u;
            s++;
        }
    }
}

/* 把字符串追加到 buf（调用方保证容量足够），返回新的长度 */
static uint8_t str_put(char *buf, uint8_t n, const char *s)
{
    while (*s) buf[n++] = *s++;
    return n;
}

/* 把无符号数右对齐追加到 buf（不足 min_w 位用空格补齐），返回新的长度 */
static uint8_t u32_put(char *buf, uint8_t n, uint32_t v, uint8_t min_w)
{
    char tmp[10];
    uint8_t len = 0u;

    if (v == 0u) tmp[len++] = '0';
    while (v) { tmp[len++] = (char)('0' + (v % 10u)); v /= 10u; }
    while (len < min_w) tmp[len++] = ' ';
    while (len) buf[n++] = tmp[--len];
    return n;
}

/* ================= 主程序 ================= */
int main(void)
{
    char line[48];
    uint8_t m;
    uint32_t led = 0u;
    uint32_t last_hb = 0u;
    uint32_t dirty = 1u;        /* 屏幕内容需要刷新 */
    uint32_t last_act = 2u;     /* 上次显示的 ACT 值（2 = 未初始化，首轮强制刷新） */

    /* 0. 显式初始化关键状态（双保险，不依赖 .data 段拷贝） */
    g_addr8 = OLED_ADDR8_A;
    g_scl_pin = SCL_PIN;
    g_sda_pin = SDA_PIN;
    g_uart_rx = g_uart_hb = g_last_rx_ms = 0u;
    for (uint8_t i = 0u; i < LAST_CHARS_N; i++) g_last_chars[i] = ' ';
    g_last_chars[LAST_CHARS_N] = 0;

    /* 1. 时钟与引脚 */
    RCC_APB2ENR |= (1u << 2) | (1u << 3);                   /* IOPAEN | IOPBEN */

    usart1_init();                                          /* PA9/PA10 */

    GPIOA_CRL = (GPIOA_CRL & ~(0xFu << 20)) | (0x2u << 20); /* PA5 推挽输出（LED 指示） */

    GPIOB_ODR |= (1u << SCL_PIN) | (1u << SDA_PIN);         /* 先释放 I2C 总线为高 */
    GPIOB_CRL = (GPIOB_CRL & ~0xFF000000u) | 0x77000000u;   /* PB6/PB7：开漏输出 50MHz */

    systick_init();

    /* 2. OLED 初始化 + 全屏点亮自检 */
    oled_init();
    fb_fill(0xFFu);
    oled_flush();
    delay_ms(300);
    fb_clear();
    oled_flush();

    usart1_send_str("STM32 BT ready\r\n");

    /* 3. 非阻塞主循环 */
    for (;;) {
        uint8_t b;
        uint32_t now = systick_ms();

        /* 3.1 串口接收：回显 + 更新状态 */
        if (usart1_try_recv(&b)) {
            usart1_send_byte(b);                 /* 原样回显 */
            g_uart_rx++;
            g_last_rx_ms = now;
            if (b >= 0x20u && b <= 0x7Eu) {      /* 可打印字符进入显示窗口 */
                for (uint8_t i = 0u; i + 1u < LAST_CHARS_N; i++) {
                    g_last_chars[i] = g_last_chars[i + 1u];
                }
                g_last_chars[LAST_CHARS_N - 1u] = (char)b;
            }
            led ^= 1u;                           /* PA5 翻转：接收活动指示 */
            GPIOA_BSRR = led ? (1u << 5u) : (1u << (5u + 16u));
            dirty = 1u;
        }

        /* 3.2 心跳（每 2 秒一行） */
        if ((uint32_t)(now - last_hb) >= HEARTBEAT_MS) {
            last_hb = now;
            g_uart_hb++;
            usart1_send_str("STM32 BT OK ");
            usart1_send_u32(g_uart_hb);
            usart1_send_str("\r\n");
            dirty = 1u;
        }

        /* 3.3 ACT 状态变化时也标记刷新 */
        {
            uint32_t act = ((uint32_t)(now - g_last_rx_ms) < 2000u) ? 1u : 0u;
            if (act != last_act) {
                last_act = act;
                dirty = 1u;
            }
        }

        /* 3.4 刷屏：只在接收空闲 300ms 后刷新，避免刷屏期间丢串口数据 */
        if (dirty && (uint32_t)(now - g_last_rx_ms) >= 300u) {
            uint32_t act = ((uint32_t)(now - g_last_rx_ms) < 2000u) ? 1u : 0u;

            /* 先清屏再画：否则变化的内容（数字、指示符）会和旧内容叠加成糊块 */
            fb_clear();

            fb_text16(0u, 0u, "蓝牙 115200");

            m = str_put(line, 0u, "接收 ");
            m = u32_put(line, m, g_uart_rx, 1u);
            m = str_put(line, m, " 发送 ");
            m = u32_put(line, m, g_uart_hb, 1u);
            line[m] = 0;
            fb_text16(0u, 16u, line);

            m = str_put(line, 0u, "活动 ");
            m = str_put(line, m, act ? "*" : "-");
            line[m] = 0;
            fb_text16(0u, 32u, line);

            m = str_put(line, 0u, "最近 ");
            m = str_put(line, m, g_last_chars);
            line[m] = 0;
            fb_text16(0u, 48u, line);

            oled_flush();
            dirty = 0u;
        }
    }
}