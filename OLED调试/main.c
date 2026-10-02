#include <stdint.h>

/* ============================================================================
 * 项目：OLED + DHT11 温湿度显示
 * 硬件：STM32F103C8 + 4 线 I2C OLED 模块（SSD1306 128x64）+ DHT11 裸传感器
 *
 * 接线（断电接线）：
 *   OLED  GND     -> 板子 GND
 *   OLED  VCC     -> 板子 3.3V
 *   OLED  SCL     -> 板子 PB6
 *   OLED  SDA     -> 板子 PB7
 *   DHT11 VCC(1)  -> 板子 3.3V
 *   DHT11 DATA(2) -> 板子 PA1，DATA 与 3.3V 之间接 10k 上拉电阻
 *   DHT11 NC(3)   -> 悬空
 *   DHT11 GND(4)  -> 板子 GND
 *
 * 本程序做什么：
 *   1. 软件模拟 I2C 驱动 SSD1306（PB6/PB7，开漏输出 + 模块自带上拉）
 *   2. 软件模拟单总线读 DHT11（PA1 + 10k 上拉），TIM4 做 1us 计时基准
 *   3. 上电自检：OLED 全屏点亮 0.4s
 *   4. 每 2 秒读一次 DHT11：
 *        - 第一行：当前温度/湿度
 *        - 第二行：成功/失败统计 + 最近一次错误码
 *        - 下方 48 像素高区域：温度折线图（128 个采样点，约 4 分钟窗口，量程自动缩放）
 *
 * 备注：寄存器级操作，无 HAL 库；读 DHT11 的每个等待都有超时保护，不会死等。
 * ============================================================================ */

/* ================= 寄存器定义（STM32F103，寄存器级操作） ================= */
#define RCC_APB2ENR  (*(volatile uint32_t *)0x40021018u)
#define RCC_APB1ENR  (*(volatile uint32_t *)0x4002101Cu)
#define GPIOA_CRL    (*(volatile uint32_t *)0x40010800u)
#define GPIOA_IDR    (*(volatile uint32_t *)0x40010808u)
#define GPIOA_ODR    (*(volatile uint32_t *)0x4001080Cu)
#define GPIOB_CRL    (*(volatile uint32_t *)0x40010C00u)
#define GPIOB_IDR    (*(volatile uint32_t *)0x40010C08u)
#define GPIOB_ODR    (*(volatile uint32_t *)0x40010C0Cu)

#define TIM4_CR1     (*(volatile uint32_t *)0x40000800u)
#define TIM4_CNT     (*(volatile uint32_t *)0x40000824u)
#define TIM4_PSC     (*(volatile uint32_t *)0x40000828u)
#define TIM4_ARR     (*(volatile uint32_t *)0x4000082Cu)

#define SYST_CSR     (*(volatile uint32_t *)0xE000E010u)
#define SYST_RVR     (*(volatile uint32_t *)0xE000E014u)
#define SYST_CVR     (*(volatile uint32_t *)0xE000E018u)

/* ================= 参数 ================= */
#define SCL_PIN          6u      /* PB6 = SCL */
#define SDA_PIN          7u      /* PB7 = SDA */

#define OLED_ADDR8_A     0x78u   /* 0x3C << 1，常见默认地址 */
#define OLED_ADDR8_B     0x7Au   /* 0x3D << 1，少数模块 */

#define I2C_DELAY_LOOPS  6u      /* 软件 I2C 半周期延时，改小 = 更快 */
#define DHT_PERIOD_MS    2000u   /* DHT11 采样周期(ms)，最小 1000 */
#define DHT_PIN          1u      /* DHT11 DATA 接在 PA1 */

#define FB_SIZE          (128u * 8u)   /* 1024 字节显存 */

static uint8_t g_addr8 = OLED_ADDR8_A;  /* 当前使用的 8 位从机地址 */
static uint8_t g_fb[FB_SIZE];           /* 显存：1 字节 = 1 列 8 像素，bit0 在上 */

static uint32_t g_scl_pin = SCL_PIN;    /* 软件 I2C 引脚角色（调试时临时对调用） */
static uint32_t g_sda_pin = SDA_PIN;

/* ---------- 调试观测变量（上电自检结果，可用调试器直接读 RAM 查看） ---------- */
volatile uint32_t g_dbg_idle = 0u;      /* 引脚就绪后总线闲电平：bit1=SCL，bit0=SDA */
volatile uint32_t g_dbg_ack3c = 0u;     /* 正常接线探测 0x3C 是否应答 */
volatile uint32_t g_dbg_ack3d = 0u;     /* 正常接线探测 0x3D 是否应答 */
volatile uint32_t g_dbg_ack3c_sw = 0u;  /* SCL/SDA 接反时探测 0x3C 是否应答 */
volatile uint32_t g_dbg_ack3d_sw = 0u;  /* SCL/SDA 接反时探测 0x3D 是否应答 */
volatile uint32_t g_dbg_nak = 0u;       /* 刷新时收到的 NAK 次数（0 = 每个字节都被应答） */
volatile uint32_t g_dbg_stage = 0u;     /* 运行阶段：1=引脚就绪 2=初始化完 3=自检完 4=主循环 */

/* ---------- DHT11 读取结果（全局变量，可用调试器直接读） ---------- */
uint32_t g_dht_ok = 0u;        /* 累计读取成功次数 */
uint32_t g_dht_err = 0u;       /* 累计读取失败次数 */
uint32_t g_dht_last_err = 0u;  /* 最近一次错误码：0=成功，1~6 见 dht_read 注释 */
uint32_t g_dht_temp = 0u;      /* 最近一次温度（摄氏度） */
uint32_t g_dht_humi = 0u;      /* 最近一次湿度（%RH） */

/* ---------- 温度历史（供折线图使用） ---------- */
uint8_t  g_temp_hist[128];     /* 最近 128 个成功采样的温度，最右边为最新 */
uint32_t g_temp_cnt = 0u;      /* 已记录样本数（最多 128） */

/* ============ 5x7 字库（ASCII 0x20 ~ 0x5A，每字符 5 列，bit0 为最上行） ============ */
static const uint8_t font5x7[59][5] = {
    {0x00,0x00,0x00,0x00,0x00}, /* 0x20 空格 */
    {0x00,0x00,0x5F,0x00,0x00}, /* ! */
    {0x00,0x07,0x00,0x07,0x00}, /* " */
    {0x14,0x7F,0x14,0x7F,0x14}, /* # */
    {0x24,0x2A,0x7F,0x2A,0x12}, /* $ */
    {0x23,0x13,0x08,0x64,0x62}, /* % */
    {0x36,0x49,0x55,0x22,0x50}, /* & */
    {0x00,0x05,0x03,0x00,0x00}, /* ' */
    {0x00,0x1C,0x22,0x41,0x00}, /* ( */
    {0x00,0x41,0x22,0x1C,0x00}, /* ) */
    {0x14,0x08,0x3E,0x08,0x14}, /* * */
    {0x08,0x08,0x3E,0x08,0x08}, /* + */
    {0x00,0x50,0x30,0x00,0x00}, /* , */
    {0x08,0x08,0x08,0x08,0x08}, /* - */
    {0x00,0x60,0x60,0x00,0x00}, /* . */
    {0x20,0x10,0x08,0x04,0x02}, /* / */
    {0x3E,0x51,0x49,0x45,0x3E}, /* 0 */
    {0x00,0x42,0x7F,0x40,0x00}, /* 1 */
    {0x42,0x61,0x51,0x49,0x46}, /* 2 */
    {0x21,0x41,0x45,0x4B,0x31}, /* 3 */
    {0x18,0x14,0x12,0x7F,0x10}, /* 4 */
    {0x27,0x45,0x45,0x45,0x39}, /* 5 */
    {0x3C,0x4A,0x49,0x49,0x30}, /* 6 */
    {0x01,0x71,0x09,0x05,0x03}, /* 7 */
    {0x36,0x49,0x49,0x49,0x36}, /* 8 */
    {0x06,0x49,0x49,0x29,0x1E}, /* 9 */
    {0x00,0x36,0x36,0x00,0x00}, /* : */
    {0x00,0x56,0x36,0x00,0x00}, /* ; */
    {0x08,0x14,0x22,0x41,0x00}, /* < */
    {0x14,0x14,0x14,0x14,0x14}, /* = */
    {0x00,0x41,0x22,0x14,0x08}, /* > */
    {0x02,0x01,0x51,0x09,0x06}, /* ? */
    {0x32,0x49,0x79,0x41,0x3E}, /* @ */
    {0x7E,0x11,0x11,0x11,0x7E}, /* A */
    {0x7F,0x49,0x49,0x49,0x36}, /* B */
    {0x3E,0x41,0x41,0x41,0x22}, /* C */
    {0x7F,0x41,0x41,0x22,0x1C}, /* D */
    {0x7F,0x49,0x49,0x49,0x41}, /* E */
    {0x7F,0x09,0x09,0x09,0x01}, /* F */
    {0x3E,0x41,0x49,0x49,0x7A}, /* G */
    {0x7F,0x08,0x08,0x08,0x7F}, /* H */
    {0x00,0x41,0x7F,0x41,0x00}, /* I */
    {0x20,0x40,0x41,0x3F,0x01}, /* J */
    {0x7F,0x08,0x14,0x22,0x41}, /* K */
    {0x7F,0x40,0x40,0x40,0x40}, /* L */
    {0x7F,0x02,0x0C,0x02,0x7F}, /* M */
    {0x7F,0x04,0x08,0x10,0x7F}, /* N */
    {0x3E,0x41,0x41,0x41,0x3E}, /* O */
    {0x7F,0x09,0x09,0x09,0x06}, /* P */
    {0x3E,0x41,0x51,0x21,0x5E}, /* Q */
    {0x7F,0x09,0x19,0x29,0x46}, /* R */
    {0x46,0x49,0x49,0x49,0x31}, /* S */
    {0x01,0x01,0x7F,0x01,0x01}, /* T */
    {0x3F,0x40,0x40,0x40,0x3F}, /* U */
    {0x1F,0x20,0x40,0x20,0x1F}, /* V */
    {0x3F,0x40,0x38,0x40,0x3F}, /* W */
    {0x63,0x14,0x08,0x14,0x63}, /* X */
    {0x07,0x08,0x70,0x08,0x07}, /* Y */
    {0x61,0x51,0x49,0x45,0x43}, /* Z */
};

/* ================= 基础延时（SysTick 1ms） ================= */
static void delay_ms(uint32_t ms)
{
    SYST_RVR = 8000u - 1u;   /* 8MHz 内核时钟 -> 1ms */
    SYST_CSR = 5u;           /* ENABLE=1，CLKSOURCE=内核时钟 */
    while (ms--) {
        SYST_CVR = 0u;
        while ((SYST_CSR & (1u << 16)) == 0u) { }   /* 等 COUNTFLAG */
    }
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
    ack = (sda_get() == 0u);  /* ACK = 从机把 SDA 拉低 */
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
    if (!i2c_write_byte(g_addr8)) g_dbg_nak++;
    if (!i2c_write_byte(0x40u)) g_dbg_nak++;   /* 控制字节：数据 */
    while (len--) {
        if (!i2c_write_byte(*buf++)) g_dbg_nak++;
    }
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
    uint32_t t;

    delay_ms(100);            /* 等模块上电稳定 */

    /* 探测 I2C 地址（正常接线） */
    g_dbg_ack3c = oled_probe(OLED_ADDR8_A);
    g_dbg_ack3d = oled_probe(OLED_ADDR8_B);

    /* 探测 SCL/SDA 是否接反：角色对调后再试一次 */
    t = g_scl_pin; g_scl_pin = g_sda_pin; g_sda_pin = t;
    g_dbg_ack3c_sw = oled_probe(OLED_ADDR8_A);
    g_dbg_ack3d_sw = oled_probe(OLED_ADDR8_B);
    t = g_scl_pin; g_scl_pin = g_sda_pin; g_sda_pin = t;   /* 恢复 */

    if (!g_dbg_ack3c && g_dbg_ack3d) {   /* 0x3C 无应答则用 0x3D */
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

/* 把 1024 字节显存整屏刷到屏幕 */
static void oled_flush(void)
{
    oled_cmd(0x21u); oled_cmd(0x00u); oled_cmd(0x7Fu);  /* 列 0~127 */
    oled_cmd(0x22u); oled_cmd(0x00u); oled_cmd(0x07u);  /* 页 0~7 */
    oled_data(g_fb, FB_SIZE);
}

/* ================= DHT11 驱动（单总线，DATA = PA1） =================
 * 时序：主机拉低 >=18ms -> 释放 -> 传感器应答(低 80us + 高 80us)
 *       -> 40 位数据：每位 = 低 ~50us + 高 27us(数据0) / 70us(数据1)
 * TIM4 配成 1us 计数（8MHz / 8），用于测量脉冲宽度，16 位回绕不影响 */

static void dht_us_delay(uint32_t us)
{
    uint16_t start = (uint16_t)TIM4_CNT;
    while ((uint16_t)(TIM4_CNT - start) < (uint16_t)us) { }
}

static void dht_pin_output_low(void)
{
    /* PA1 推挽输出，并拉低（起始信号） */
    GPIOA_CRL = (GPIOA_CRL & ~(0xFu << (DHT_PIN * 4u))) | (0x2u << (DHT_PIN * 4u));
    GPIOA_ODR &= ~(1u << DHT_PIN);
}

static void dht_pin_input(void)
{
    /* PA1 浮空输入（空闲电平由外部 10k 上拉维持为高） */
    GPIOA_CRL = (GPIOA_CRL & ~(0xFu << (DHT_PIN * 4u))) | (0x4u << (DHT_PIN * 4u));
}

static uint32_t dht_pin_get(void)
{
    return (GPIOA_IDR >> DHT_PIN) & 1u;
}

/* 等引脚变为指定电平；超时(us)返回 1，正常返回 0 */
static uint32_t dht_wait(uint32_t level, uint32_t timeout_us)
{
    uint16_t start = (uint16_t)TIM4_CNT;
    while (dht_pin_get() != level) {
        if ((uint16_t)(TIM4_CNT - start) > (uint16_t)timeout_us) return 1u;
    }
    return 0u;
}

/* 读一次 DHT11；成功返回 0，失败返回错误码：
 * 1=无应答(等低超时) 2=应答高电平异常 3=数据位起点超时
 * 4=位低电平超时 5=位高电平异常 6=校验和不匹配 */
static uint32_t dht_read(uint8_t *temp, uint8_t *humi)
{
    uint8_t d[5] = { 0u, 0u, 0u, 0u, 0u };
    uint16_t t0, dt;

    /* 1. 起始信号：拉低 20ms（>=18ms），再释放 30us */
    dht_pin_output_low();
    delay_ms(20u);
    dht_pin_input();
    dht_us_delay(30u);

    /* 2. 传感器应答：低 ~80us，再高 ~80us */
    if (dht_wait(0u, 300u)) return 1u;   /* 等应答低电平 */
    if (dht_wait(1u, 200u)) return 2u;   /* 等应答高电平 */

    /* 3. 读 40 位：测每个位的高电平宽度，>40us 判为 1 */
    for (uint32_t i = 0u; i < 40u; i++) {
        if (dht_wait(0u, 200u)) return 3u;   /* 等本位的 50us 低电平开始 */
        if (dht_wait(1u, 200u)) return 4u;   /* 等低电平结束、高电平开始 */
        t0 = (uint16_t)TIM4_CNT;

        if (i == 39u) {
            /* 最后一位后面总线直接回空闲（一直为高），不能等下一位的低电平，
             * 因此限时 120us 内测量高电平宽度即可区分 */
            while (dht_pin_get() == 1u && (uint16_t)(TIM4_CNT - t0) < 120u) { }
            dt = (uint16_t)(TIM4_CNT - t0);
        } else {
            if (dht_wait(0u, 200u)) return 5u;   /* 等这一位高电平结束 */
            dt = (uint16_t)(TIM4_CNT - t0);
        }

        d[i >> 3] = (uint8_t)(d[i >> 3] << 1);
        if (dt > 40u) d[i >> 3] |= 1u;
    }

    /* 4. 校验：前 4 字节之和 == 第 5 字节 */
    if ((uint8_t)(d[0] + d[1] + d[2] + d[3]) != d[4]) return 6u;

    *humi = d[0];   /* 湿度整数部分 */
    *temp = d[2];   /* 温度整数部分 */
    return 0u;
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

static const uint8_t *font_glyph(char ch)
{
    if (ch < 0x20 || ch > 'Z') ch = ' ';   /* 字库外字符按空格处理 */
    return font5x7[(uint8_t)ch - 0x20u];
}

/* 在指定页(0~7)、列 x 处画一个字符（6 列宽：5 列字模 + 1 列间距） */
static void fb_char(uint8_t x, uint8_t page, char ch)
{
    const uint8_t *g = font_glyph(ch);
    uint16_t base = (uint16_t)page * 128u + x;

    for (uint8_t i = 0; i < 5u; i++) g_fb[base + i] = g[i];
    g_fb[base + 5u] = 0x00u;
}

static void fb_str(uint8_t x, uint8_t page, const char *s)
{
    while (*s) {
        fb_char(x, page, *s++);
        x += 6u;
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

/* 点亮一个像素（x 0~127，y 0~63） */
static void fb_pixel(uint8_t x, uint8_t y)
{
    g_fb[(uint16_t)(y >> 3) * 128u + x] |= (uint8_t)(1u << (y & 7u));
}

/* 温度折线图：区域 y = 16~63（页 2~7），横轴 128 列对应 128 个采样点
 * 纵轴自动量程：取窗口内最小/最大值，跨度不足 4 度时按 4 度处理，
 * 避免平稳时噪声被放大成大幅波动 */
static void draw_chart(void)
{
    uint8_t n = (uint8_t)g_temp_cnt;
    uint8_t lo = 255u, hi = 0u;
    uint8_t prev_y = 0u, first = 1u;

    /* 先清空图表区域（页 2~7） */
    for (uint16_t i = 2u * 128u; i < FB_SIZE; i++) g_fb[i] = 0u;
    if (n == 0u) return;

    /* 求窗口内最小、最大温度 */
    for (uint8_t i = 0u; i < n; i++) {
        uint8_t v = g_temp_hist[128u - n + i];
        if (v < lo) lo = v;
        if (v > hi) hi = v;
    }
    if ((uint8_t)(hi - lo) < 4u) {
        uint8_t mid = (uint8_t)((hi + lo) / 2u);
        lo = (mid >= 2u) ? (uint8_t)(mid - 2u) : 0u;
        hi = (uint8_t)(lo + 4u);
    }

    /* 逐点画折线：与上一个采样点的纵向区间连起来，形成连续曲线 */
    for (uint8_t i = 0u; i < n; i++) {
        uint8_t v = g_temp_hist[128u - n + i];
        uint8_t x = (uint8_t)(128u - n + i);
        uint8_t y = (uint8_t)(63u - (uint32_t)(v - lo) * 47u / (uint32_t)(hi - lo));

        if (first) {
            fb_pixel(x, y);
            first = 0u;
        } else {
            uint8_t a = (prev_y < y) ? prev_y : y;
            uint8_t b = (prev_y > y) ? prev_y : y;
            for (uint8_t yy = a; yy <= b; yy++) fb_pixel(x, yy);
        }
        prev_y = y;
    }
}

/* ================= 主程序 ================= */
int main(void)
{
    char buf[32];
    uint8_t m;

    /* 0. 关键状态显式初始化（双保险，不依赖 .data 段拷贝） */
    g_addr8 = OLED_ADDR8_A;
    g_scl_pin = SCL_PIN;
    g_sda_pin = SDA_PIN;
    g_dbg_idle = g_dbg_ack3c = g_dbg_ack3d = 0u;
    g_dbg_ack3c_sw = g_dbg_ack3d_sw = g_dbg_nak = g_dbg_stage = 0u;
    g_dht_ok = g_dht_err = g_dht_last_err = 0u;
    g_dht_temp = g_dht_humi = 0u;
    g_temp_cnt = 0u;

    /* 1. 开时钟、配引脚：GPIOA/GPIOB（APB2），TIM4（APB1，给 DHT11 计时） */
    RCC_APB2ENR |= (1u << 2) | (1u << 3);                   /* IOPAEN | IOPBEN */
    RCC_APB1ENR |= (1u << 2);                               /* TIM4EN */
    TIM4_PSC = 7u;                                          /* 8MHz / 8 = 1MHz -> 1us */
    TIM4_ARR = 0xFFFFu;
    TIM4_CR1 = 1u;                                          /* 启动 TIM4 */
    GPIOB_ODR |= (1u << SCL_PIN) | (1u << SDA_PIN);         /* 先释放 I2C 总线为高 */
    GPIOB_CRL = (GPIOB_CRL & ~0xFF000000u) | 0x77000000u;   /* PB6/PB7：开漏输出 50MHz */
    dht_pin_input();                                        /* PA1 空闲为输入（外部上拉） */
    g_dbg_stage = 1u;

    /* 记录总线闲电平（正常情况下 SCL、SDA 都应为 1） */
    delay_ms(10);
    g_dbg_idle = (((GPIOB_IDR >> SCL_PIN) & 1u) << 1) | ((GPIOB_IDR >> SDA_PIN) & 1u);

    /* 2. 初始化屏幕 + 上电自检：全屏点亮 0.4s 再清屏 */
    oled_init();
    g_dbg_stage = 2u;
    fb_fill(0xFFu);
    oled_flush();
    delay_ms(400);
    fb_clear();
    oled_flush();
    g_dbg_stage = 3u;

    /* 3. 首屏骨架（等 DHT11 上电稳定的 1 秒内先显示） */
    fb_str(0u, 0u, "T=--C H=--%");
    fb_str(0u, 1u, "OK  0 ERR  0 L0");
    oled_flush();

    /* 4. 等 DHT11 上电稳定（>=1s），之后每 2 秒读一次并刷新屏幕 */
    delay_ms(1000u);
    g_dbg_stage = 4u;
    for (;;) {
        uint8_t t = 0u, h = 0u;
        uint32_t err = dht_read(&t, &h);

        if (err == 0u) {
            g_dht_ok++;
            g_dht_temp = t;
            g_dht_humi = h;

            /* 记录温度历史：整体左移一格，新样本放最右边 */
            if (g_temp_cnt < 128u) g_temp_cnt++;
            for (uint8_t i = 0u; i + 1u < 128u; i++) {
                g_temp_hist[i] = g_temp_hist[i + 1u];
            }
            g_temp_hist[127] = t;
        } else {
            g_dht_err++;
        }
        g_dht_last_err = err;

        /* 第一行：当前温度 / 湿度 */
        m = str_put(buf, 0u, "T=");
        m = u32_put(buf, m, g_dht_temp, 2u);
        m = str_put(buf, m, "C H=");
        m = u32_put(buf, m, g_dht_humi, 2u);
        m = str_put(buf, m, "%");
        buf[m] = 0;
        fb_str(0u, 0u, buf);

        /* 第二行：成功 / 失败次数 + 最近一次错误码（L0 = 最近一次成功） */
        m = str_put(buf, 0u, "OK");
        m = u32_put(buf, m, g_dht_ok, 3u);
        m = str_put(buf, m, " ERR");
        m = u32_put(buf, m, g_dht_err, 3u);
        m = str_put(buf, m, " L");
        m = u32_put(buf, m, g_dht_last_err, 1u);
        buf[m] = 0;
        fb_str(0u, 1u, buf);

        /* 页 2~7：温度折线图 */
        draw_chart();

        oled_flush();
        delay_ms(DHT_PERIOD_MS);
    }
}