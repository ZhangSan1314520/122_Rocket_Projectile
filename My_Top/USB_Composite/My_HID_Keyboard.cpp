/**
  * @file    My_HID_Keyboard.c
  * @brief   AL94 USB Composite HID Keyboard 封装层实现
  * @note    使用 AL94 库原始 HID_KEYBOARD 描述符格式（含 Report ID）
  *
  * 描述符格式回顾：
  *   byte0: Report ID = 0x01（AL94 描述符中 0x85,0x01 定义）
  *   byte1: Modifier（Ctrl/Shift/Alt/Win 组合键位图）
  *   byte2: Reserved（保留，固定为 0x00）
  *   byte3: Keycode1（HID Usage ID）
  *   byte4: Keycode2
  *   byte5: Keycode3
  *   byte6: Keycode4
  *   byte7: Keycode5
  *
  * 数据长度：8 字节
  *
  * 重要：
  *   - 发送前需确保 USB 设备已枚举完成（dev_state == USBD_STATE_CONFIGURED）
  *   - 每个按键必须发"按下+释放"，否则主机会认为该键一直被按住
  *   - 按键之间需保持至少 10ms 间隔，给主机轮询时间
  */

#include "My_HID_Keyboard.hpp"
#include "usbd_hid_keyboard.h"
#include "FreeRTOS.h"
#include "task.h"

/* USB 设备句柄（定义在 AL94 的 usb_device.c 中） */
extern USBD_HandleTypeDef hUsbDevice;

/* ========================================================================= */
/* ASCII 字符到 HID 键码映射表（US 键盘布局）                                */
/* ========================================================================= */
/* 每个条目 = {keycode, modifier}
 *   keycode:  HID Usage Page 0x07 的 Usage ID
 *   modifier: 0x00 = 不需要 Shift，0x02 = 需要左 Shift
 *
 * 索引 = ASCII 码值（0x00~0x7F），未映射的字符填 {0x00, 0x00} */
static const struct {
    uint8_t keycode;    /* HID Usage ID */
    uint8_t modifier;   /* 0x00 或 HID_MOD_LSHIFT */
} ascii_to_hid[128] = {
    /* 0x00~0x1F: 控制字符（未映射） */
    {0x00,0x00}, {0x00,0x00}, {0x00,0x00}, {0x00,0x00},
    {0x00,0x00}, {0x00,0x00}, {0x00,0x00}, {0x00,0x00},
    {0x00,0x00}, {0x00,0x00}, {0x00,0x00}, {0x00,0x00},
    {0x00,0x00}, {0x00,0x00}, {0x00,0x00}, {0x00,0x00},
    {0x00,0x00}, {0x00,0x00}, {0x00,0x00}, {0x00,0x00},
    {0x00,0x00}, {0x00,0x00}, {0x00,0x00}, {0x00,0x00},
    {0x00,0x00}, {0x00,0x00}, {0x00,0x00}, {0x00,0x00},
    {0x00,0x00}, {0x00,0x00}, {0x00,0x00}, {0x00,0x00},

    /* 0x20 Space */
    {HID_KEY_SPACE,  HID_MOD_NONE},

    /* 0x21 '!' 需要 Shift + '1' */
    {HID_KEY_1,      HID_MOD_LSHIFT},

    /* 0x22 '"' 需要 Shift + '\'' */
    {HID_KEY_QUOTE,  HID_MOD_LSHIFT},

    /* 0x23 '#' 需要 Shift + '3' */
    {HID_KEY_3,      HID_MOD_LSHIFT},

    /* 0x24 '$' 需要 Shift + '4' */
    {HID_KEY_4,      HID_MOD_LSHIFT},

    /* 0x25 '%' 需要 Shift + '5' */
    {HID_KEY_5,      HID_MOD_LSHIFT},

    /* 0x26 '&' 需要 Shift + '7' */
    {HID_KEY_7,      HID_MOD_LSHIFT},

    /* 0x27 '\'' 单引号，不需要 Shift */
    {HID_KEY_QUOTE,  HID_MOD_NONE},

    /* 0x28 '(' 需要 Shift + '9' */
    {HID_KEY_9,      HID_MOD_LSHIFT},

    /* 0x29 ')' 需要 Shift + '0' */
    {HID_KEY_0,      HID_MOD_LSHIFT},

    /* 0x2A '*' 需要 Shift + '8' */
    {HID_KEY_8,      HID_MOD_LSHIFT},

    /* 0x2B '+' 需要 Shift + '=' */
    {HID_KEY_EQUAL,  HID_MOD_LSHIFT},

    /* 0x2C ',' 逗号，不需要 Shift */
    {HID_KEY_COMMA,  HID_MOD_NONE},

    /* 0x2D '-' 减号，不需要 Shift */
    {HID_KEY_MINUS,  HID_MOD_NONE},

    /* 0x2E '.' 句点，不需要 Shift */
    {HID_KEY_PERIOD, HID_MOD_NONE},

    /* 0x2F '/' 斜杠，不需要 Shift */
    {HID_KEY_SLASH,  HID_MOD_NONE},

    /* 0x30~0x39 '0'~'9'，不需要 Shift */
    {HID_KEY_0, HID_MOD_NONE}, {HID_KEY_1, HID_MOD_NONE},
    {HID_KEY_2, HID_MOD_NONE}, {HID_KEY_3, HID_MOD_NONE},
    {HID_KEY_4, HID_MOD_NONE}, {HID_KEY_5, HID_MOD_NONE},
    {HID_KEY_6, HID_MOD_NONE}, {HID_KEY_7, HID_MOD_NONE},
    {HID_KEY_8, HID_MOD_NONE}, {HID_KEY_9, HID_MOD_NONE},

    /* 0x3A ':' 需要 Shift + ';' */
    {HID_KEY_SEMICOLON, HID_MOD_LSHIFT},

    /* 0x3B ';' 分号，不需要 Shift */
    {HID_KEY_SEMICOLON, HID_MOD_NONE},

    /* 0x3C '<' 需要 Shift + ',' */
    {HID_KEY_COMMA,  HID_MOD_LSHIFT},

    /* 0x3D '=' 等号，不需要 Shift */
    {HID_KEY_EQUAL,  HID_MOD_NONE},

    /* 0x3E '>' 需要 Shift + '.' */
    {HID_KEY_PERIOD, HID_MOD_LSHIFT},

    /* 0x3F '?' 需要 Shift + '/' */
    {HID_KEY_SLASH,  HID_MOD_LSHIFT},

    /* 0x40 '@' 需要 Shift + '2' */
    {HID_KEY_2,      HID_MOD_LSHIFT},

    /* 0x41~0x5A 'A'~'Z'，需要 Shift + 对应小写字母 */
    {HID_KEY_A, HID_MOD_LSHIFT}, {HID_KEY_B, HID_MOD_LSHIFT},
    {HID_KEY_C, HID_MOD_LSHIFT}, {HID_KEY_D, HID_MOD_LSHIFT},
    {HID_KEY_E, HID_MOD_LSHIFT}, {HID_KEY_F, HID_MOD_LSHIFT},
    {HID_KEY_G, HID_MOD_LSHIFT}, {HID_KEY_H, HID_MOD_LSHIFT},
    {HID_KEY_I, HID_MOD_LSHIFT}, {HID_KEY_J, HID_MOD_LSHIFT},
    {HID_KEY_K, HID_MOD_LSHIFT}, {HID_KEY_L, HID_MOD_LSHIFT},
    {HID_KEY_M, HID_MOD_LSHIFT}, {HID_KEY_N, HID_MOD_LSHIFT},
    {HID_KEY_O, HID_MOD_LSHIFT}, {HID_KEY_P, HID_MOD_LSHIFT},
    {HID_KEY_Q, HID_MOD_LSHIFT}, {HID_KEY_R, HID_MOD_LSHIFT},
    {HID_KEY_S, HID_MOD_LSHIFT}, {HID_KEY_T, HID_MOD_LSHIFT},
    {HID_KEY_U, HID_MOD_LSHIFT}, {HID_KEY_V, HID_MOD_LSHIFT},
    {HID_KEY_W, HID_MOD_LSHIFT}, {HID_KEY_X, HID_MOD_LSHIFT},
    {HID_KEY_Y, HID_MOD_LSHIFT}, {HID_KEY_Z, HID_MOD_LSHIFT},

    /* 0x5B '[' 左方括号，不需要 Shift */
    {HID_KEY_LBRACKET, HID_MOD_NONE},

    /* 0x5C '\' 反斜杠，不需要 Shift */
    {HID_KEY_BACKSLASH, HID_MOD_NONE},

    /* 0x5D ']' 右方括号，不需要 Shift */
    {HID_KEY_RBRACKET, HID_MOD_NONE},

    /* 0x5E '^' 需要 Shift + '6' */
    {HID_KEY_6, HID_MOD_LSHIFT},

    /* 0x5F '_' 下划线，需要 Shift + '-' */
    {HID_KEY_MINUS, HID_MOD_LSHIFT},

    /* 0x60 '`' 反引号，不需要 Shift */
    {HID_KEY_GRAVE, HID_MOD_NONE},

    /* 0x61~0x7A 'a'~'z'，不需要 Shift */
    {HID_KEY_A, HID_MOD_NONE}, {HID_KEY_B, HID_MOD_NONE},
    {HID_KEY_C, HID_MOD_NONE}, {HID_KEY_D, HID_MOD_NONE},
    {HID_KEY_E, HID_MOD_NONE}, {HID_KEY_F, HID_MOD_NONE},
    {HID_KEY_G, HID_MOD_NONE}, {HID_KEY_H, HID_MOD_NONE},
    {HID_KEY_I, HID_MOD_NONE}, {HID_KEY_J, HID_MOD_NONE},
    {HID_KEY_K, HID_MOD_NONE}, {HID_KEY_L, HID_MOD_NONE},
    {HID_KEY_M, HID_MOD_NONE}, {HID_KEY_N, HID_MOD_NONE},
    {HID_KEY_O, HID_MOD_NONE}, {HID_KEY_P, HID_MOD_NONE},
    {HID_KEY_Q, HID_MOD_NONE}, {HID_KEY_R, HID_MOD_NONE},
    {HID_KEY_S, HID_MOD_NONE}, {HID_KEY_T, HID_MOD_NONE},
    {HID_KEY_U, HID_MOD_NONE}, {HID_KEY_V, HID_MOD_NONE},
    {HID_KEY_W, HID_MOD_NONE}, {HID_KEY_X, HID_MOD_NONE},
    {HID_KEY_Y, HID_MOD_NONE}, {HID_KEY_Z, HID_MOD_NONE},

    /* 0x7B '{' 需要 Shift + '[' */
    {HID_KEY_LBRACKET, HID_MOD_LSHIFT},

    /* 0x7C '|' 需要 Shift + '\' */
    {HID_KEY_BACKSLASH, HID_MOD_LSHIFT},

    /* 0x7D '}' 需要 Shift + ']' */
    {HID_KEY_RBRACKET, HID_MOD_LSHIFT},

    /* 0x7E '~' 需要 Shift + '`' */
    {HID_KEY_GRAVE, HID_MOD_LSHIFT},

    /* 0x7F DEL（未映射） */
    {0x00, 0x00}
};

/* ========================================================================= */
/* 内部辅助函数                                                              */
/* ========================================================================= */

/**
  * @brief  发送一个 HID 键盘报告（8 字节，含 Report ID）
  * @param  modifier: 修饰键位图
  * @param  key1~5:   按键码（HID Usage ID）
  * @note   此函数只发报告，不处理延时或释放，由上层调用者控制
  */
static void HID_KB_SendReport(uint8_t modifier,
                              uint8_t k1, uint8_t k2, uint8_t k3,
                              uint8_t k4, uint8_t k5)
{
    /* 8 字节报告，符合 AL94 HID_KEYBOARD 描述符格式：
     * byte0: Report ID = 0x01（必须，因为描述符里有 0x85,0x01）
     * byte1: Modifier
     * byte2: Reserved = 0x00
     * byte3~7: Keycode1~5 */
    uint8_t report[8] = {
        0x01,       /* Report ID = 1 */
        modifier,   /* Ctrl/Shift/Alt/Win */
        0x00,       /* Reserved */
        k1, k2, k3, k4, k5
    };

    USBD_HID_Keybaord_SendReport(&hUsbDevice, report, 8);
}

/**
  * @brief  发送全零释放报告（所有按键释放）
  */
static void HID_KB_SendRelease(void)
{
    uint8_t report[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    USBD_HID_Keybaord_SendReport(&hUsbDevice, report, 8);
}

/* ========================================================================= */
/* 对外接口函数                                                              */
/* ========================================================================= */

/**
  * @brief  发送单个按键（按下+自动释放）
  * @param  modifier: 修饰键位图（见 HID_MOD_* 宏）
  * @param  keycode:  HID Usage ID（见 HID_KEY_* 宏）
  *
  * 流程：按下 → 延时 10ms → 释放 → 延时 10ms
  * 10ms 是给主机 HID 轮询间隔，确保不会漏掉按键
  */
void HID_KB_SendKey(uint8_t modifier, uint8_t keycode)
{
    /* 1. 发送按下报告 */
    HID_KB_SendReport(modifier, keycode, 0x00, 0x00, 0x00, 0x00);

    /* 2. 等待主机轮询（至少一个 HID 轮询间隔，通常 10ms） */
    vTaskDelay(pdMS_TO_TICKS(10));

    /* 3. 发送释放报告（全零，否则主机认为按键一直被按住） */
    HID_KB_SendRelease();

    /* 4. 再等待一个间隔，避免两次发送太密集 */
    vTaskDelay(pdMS_TO_TICKS(10));
}

/**
  * @brief  发送一个 ASCII 字符（自动处理大小写和符号）
  * @param  c: 要发送的 ASCII 字符（0x00~0x7F）
  *
  * 说明：
  *   - 支持标准 US 键盘布局的全部可打印 ASCII 字符
  *   - 大写字母自动加 Shift
  *   - 符号（如 !@#$%）自动加 Shift
  *   - 非 ASCII（>= 0x80）或控制字符直接忽略
  *   - 换行符 '\n' 映射为 Enter 键
  */
void HID_KB_SendChar(char c)
{
    uint8_t idx = (uint8_t)c;

    /* 非 ASCII，直接返回 */
    if (idx >= 0x80) {
        return;
    }

    /* 换行符映射为 Enter */
    if (c == '\n' || c == '\r') {
        HID_KB_SendSpecialKey(HID_KEY_ENTER);
        return;
    }

    /* 查找映射表 */
    uint8_t keycode  = ascii_to_hid[idx].keycode;
    uint8_t modifier = ascii_to_hid[idx].modifier;

    /* 未映射（如 DEL 0x7F）直接忽略 */
    if (keycode == 0x00) {
        return;
    }

    HID_KB_SendKey(modifier, keycode);
}

/**
  * @brief  发送一个 ASCII 字符串（逐字符发送）
  * @param  str: 要发送的字符串（以 '\0' 结尾）
  *
  * 性能：每个字符约 20ms（按下+释放+延时）
  * 发送 "Hello" 约需 100ms
  */
void HID_KB_SendString(const char *str)
{
    if (str == NULL) {
        return;
    }

    while (*str != '\0') {
        HID_KB_SendChar(*str++);
    }
}

/**
  * @brief  发送特殊功能键（Enter/Esc/方向键等）
  * @param  keycode:  HID Usage ID（见 HID_KEY_* 宏）
  *
  * 等同于 HID_KB_SendKey(HID_MOD_NONE, keycode)
  */
void HID_KB_SendSpecialKey(uint8_t keycode)
{
    HID_KB_SendKey(HID_MOD_NONE, keycode);
}

/**
  * @brief  发送组合键（如 Ctrl+C、Alt+F4）
  * @param  modifier: 修饰键位图（见 HID_MOD_* 宏）
  * @param  keycode:  主键 Usage ID（见 HID_KEY_* 宏）
  *
  * 示例：
  *   HID_KB_SendCombo(HID_MOD_LCTRL, HID_KEY_C);   // Ctrl + C
  *   HID_KB_SendCombo(HID_MOD_LALT,  HID_KEY_F4);  // Alt + F4
  *   HID_KB_SendCombo(HID_MOD_LCTRL | HID_MOD_LSHIFT, HID_KEY_ESC); // Ctrl+Shift+Esc
  */
void HID_KB_SendCombo(uint8_t modifier, uint8_t keycode)
{
    HID_KB_SendKey(modifier, keycode);
}
