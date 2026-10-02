# STM32F103C8 学习项目集

蓝色板（STM32F103C8）上的寄存器级裸机练习项目，不依赖 HAL/标准外设库，直接用寄存器操作。
工具链：Keil MDK-ARM（Arm Compiler 6.24）命令行编译，OpenOCD + ST-Link 烧录。

## 目录

| 目录 | 内容 |
|---|---|
| `点亮led/` | PA5 上外接 LED 闪烁 |
| `蜂鸣器/` | 双声部（TIM3_CH1 + TIM2_CH1）无源蜂鸣器演奏《卡农》8 段变奏 |
| `OLED调试/` | SSD1306 OLED 显示 DHT11 温湿度 + 温度折线图 |

## 烧录器接线（ST-Link → 开发板）

| ST-Link | 开发板 |
|---|---|
| SWDIO | SWDIO |
| SWCLK | SWCLK |
| GND | GND |
| 3.3V | 3.3V |

BOOT0 跳线帽保持接 GND（从 Flash 启动），无需改动。

---

## 点亮led

| 器件 | 接到 |
|---|---|
| LED 长脚（正极，+） | 经 220Ω ~ 1kΩ 限流电阻到 **PA5** |
| LED 短脚（负极，-） | GND |

PA5 配置为推挽输出，输出高电平点亮、低电平熄灭。

---

## 蜂鸣器（双声部卡农）

两路蜂鸣器使用完全相同的驱动电路，各用一颗 S8050 三极管做低边开关：

| 器件 | 接到 |
|---|---|
| **PA6**（旋律声部，TIM3_CH1） | 1kΩ 电阻 → S8050 基极（B） |
| **PA0**（低音声部，TIM2_CH1） | 1kΩ 电阻 → 第二颗 S8050 基极（B） |
| S8050 发射极（E） | GND |
| S8050 集电极（C） | 无源蜂鸣器负极（-） |
| 无源蜂鸣器正极（+） | 3.3V |

注意事项：

- S8050 的 E / C 脚接反会导致蜂鸣器不响或声音明显变小，注意封装上的丝印方向。
- 使用**无源**蜂鸣器，靠 2 ~ 4kHz 方波发声（代码里已用 PWM 产生）；有源蜂鸣器接上只会听到固定音调。
- 基极必须串 1kΩ 电阻，不要直接接到 IO。

---

## OLED调试（OLED + DHT11 温湿度 + 折线图）

### 接线

| 器件 | 引脚 | 接到开发板 |
|---|---|---|
| OLED（四线 I2C） | GND | GND |
| OLED | VCC | 3.3V |
| OLED | SCL | **PB6** |
| OLED | SDA | **PB7** |
| DHT11 | VCC（第 1 脚） | 3.3V |
| DHT11 | DATA（第 2 脚） | **PA1**，同时经 10kΩ 电阻上拉到 3.3V |
| DHT11 | NC（第 3 脚） | 悬空，不接 |
| DHT11 | GND（第 4 脚） | GND |

DHT11 方向识别：**网格面（能看到小孔阵列的一面）朝向自己、引脚朝下，从左到右依次是 VCC、DATA、NC、GND**。接反可能损坏传感器，接线前先核对。

其他要点：

- OLED 模块自带 4.7kΩ I2C 上拉电阻，无需外接；DHT11 是裸传感器，DATA 线必须外接 10kΩ（4.7k ~ 10k 均可）上拉到 3.3V。
- 屏幕用 3.3V 供电，与 MCU 逻辑电平一致。

### 屏幕显示内容

```
T=30C H=77%          当前温度 / 湿度
OK  6 ERR  0 L0      读取成功次数 / 失败次数 / 最近一次错误码（L0=成功，L1~L6=各类超时或校验错误）
[ 温度折线图 ]        128 个采样点（每 2 秒一个，约 4 分钟窗口），量程自动缩放
```

### 实现说明

- OLED：PB6/PB7 上软件模拟 I2C（开漏输出），驱动 SSD1306 128x64；开机自动探测 0x3C / 0x3D 地址。
- DHT11：PA1 单总线，TIM4 提供 1µs 计时基准测量脉冲宽度；所有等待都有超时保护，传感器掉线不会卡死程序。
- 折线图：128 字节温度历史数组，新样本从右侧进入，纵轴按窗口内最小/最大值自动量程（最小跨度 4°C）。

---

## 编译方法（命令行）

各工程的 `Objects/` 目录下执行：

```powershell
# 编译（以 OLED调试 为例，armclang 位于 Keil 安装目录）
armclang --target=arm-arm-none-eabi -mcpu=cortex-m3 -O2 -c main.c -o Objects\main.o
armclang --target=arm-arm-none-eabi -mcpu=cortex-m3 -O2 -c startup.c -o Objects\startup.o
armlink --cpu=Cortex-M3 --scatter scatter.sct Objects\main.o Objects\startup.o -o Objects\oled.axf
fromelf --bin --output Objects\oled.bin Objects\oled.axf
```

烧录（OpenOCD）：

```powershell
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "program Objects/oled.bin 0x08000000 verify reset exit"
```

> 说明：本机用户名含撇号（`'`），Keil uVision 图形界面编译会因路径解析报错，因此统一用命令行工具链编译。
> 各工程也有对应的 Keil 工程文件（`.uvprojx`），在无特殊字符路径的电脑上可直接用 uVision 打开。

## 备注

- `startup.c` 中包含 RW 数据拷贝（Flash → RAM）与 ZI 段清零，这是带初值全局变量能正常工作的前提。
- 每个工程均为独立目录，包含 `main.c`、`startup.c`、`scatter.sct` 和 Keil 工程文件。