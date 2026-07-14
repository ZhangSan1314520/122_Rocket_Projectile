/**
  * @file    My_HID_Keyboard.h
  * @brief   AL94 USB Composite HID Keyboard 封装层
  * @note    使用 AL94 库原始 HID_KEYBOARD 描述符格式（含 Report ID）
  *
  * 描述符格式：
  *   byte0: Report ID = 0x01（AL94 描述符中 0x85,0x01 定义）
  *   byte1: Modifier（Ctrl/Shift/Alt/Win 组合键位图）
  *   byte2: Reserved（保留，固定为 0x00）
  *   byte3~7: Keycode1~5（最多 5 个同时按键的 HID Usage ID）
  *
  * 数据长度：8 字节
  */
#pragma once


#ifndef MY_HID_KEYBOARD_H
#define MY_HID_KEYBOARD_H

#include <stdint.h>


/* ========================================================================= */
/* 特殊按键码（HID Usage Page 0x07）                                         */
/* ========================================================================= */
#define HID_KEY_NONE          0x00  /* 无按键 / 释放所有键 */
#define HID_KEY_A             0x04  /* 字母 A */
#define HID_KEY_B             0x05  /* 字母 B */
#define HID_KEY_C             0x06  /* 字母 C */
#define HID_KEY_D             0x07  /* 字母 D */
#define HID_KEY_E             0x08  /* 字母 E */
#define HID_KEY_F             0x09  /* 字母 F */
#define HID_KEY_G             0x0A  /* 字母 G */
#define HID_KEY_H             0x0B  /* 字母 H */
#define HID_KEY_I             0x0C  /* 字母 I */
#define HID_KEY_J             0x0D  /* 字母 J */
#define HID_KEY_K             0x0E  /* 字母 K */
#define HID_KEY_L             0x0F  /* 字母 L */
#define HID_KEY_M             0x10  /* 字母 M */
#define HID_KEY_N             0x11  /* 字母 N */
#define HID_KEY_O             0x12  /* 字母 O */
#define HID_KEY_P             0x13  /* 字母 P */
#define HID_KEY_Q             0x14  /* 字母 Q */
#define HID_KEY_R             0x15  /* 字母 R */
#define HID_KEY_S             0x16  /* 字母 S */
#define HID_KEY_T             0x17  /* 字母 T */
#define HID_KEY_U             0x18  /* 字母 U */
#define HID_KEY_V             0x19  /* 字母 V */
#define HID_KEY_W             0x1A  /* 字母 W */
#define HID_KEY_X             0x1B  /* 字母 X */
#define HID_KEY_Y             0x1C  /* 字母 Y */
#define HID_KEY_Z             0x1D  /* 字母 Z */
#define HID_KEY_1             0x1E  /* 主键盘 1  !（Shift+1） */
#define HID_KEY_2             0x1F  /* 主键盘 2  @（Shift+2） */
#define HID_KEY_3             0x20  /* 主键盘 3  #（Shift+3） */
#define HID_KEY_4             0x21  /* 主键盘 4  $（Shift+4） */
#define HID_KEY_5             0x22  /* 主键盘 5  %（Shift+5） */
#define HID_KEY_6             0x23  /* 主键盘 6  ^（Shift+6） */
#define HID_KEY_7             0x24  /* 主键盘 7  &（Shift+7） */
#define HID_KEY_8             0x25  /* 主键盘 8  *（Shift+8） */
#define HID_KEY_9             0x26  /* 主键盘 9  (（Shift+9） */
#define HID_KEY_0             0x27  /* 主键盘 0  )（Shift+0） */
#define HID_KEY_ENTER         0x28  /* 回车键（Return/Enter） */
#define HID_KEY_ESC           0x29  /* Escape 键 */
#define HID_KEY_BACKSPACE     0x2A  /* 退格键（Backspace） */
#define HID_KEY_TAB           0x2B  /* Tab 键 */
#define HID_KEY_SPACE         0x2C  /* 空格键（Spacebar） */
#define HID_KEY_MINUS         0x2D  /* 减号 -  下划线 _（Shift+-） */
#define HID_KEY_EQUAL         0x2E  /* 等号 =  加号 +（Shift+=） */
#define HID_KEY_LBRACKET      0x2F  /* 左方括号 [  左花括号 {（Shift+[） */
#define HID_KEY_RBRACKET      0x30  /* 右方括号 ]  右花括号 }（Shift+]） */
#define HID_KEY_BACKSLASH     0x31  /* 反斜杠 \  竖线 |（Shift+\） */
#define HID_KEY_SEMICOLON     0x33  /* 分号 ;  冒号 :（Shift+;） */
#define HID_KEY_QUOTE         0x34  /* 单引号 '  双引号 "（Shift+'） */
#define HID_KEY_GRAVE         0x35  /* 反引号 `  波浪号 ~（Shift+`） */
#define HID_KEY_COMMA         0x36  /* 逗号 ,  左尖括号 <（Shift+,） */
#define HID_KEY_PERIOD        0x37  /* 句号 .  右尖括号 >（Shift+.） */
#define HID_KEY_SLASH         0x38  /* 斜杠 /  问号 ?（Shift+/） */
#define HID_KEY_CAPSLOCK      0x39  /* 大小写锁定（Caps Lock） */
#define HID_KEY_F1            0x3A  /* 功能键 F1 */
#define HID_KEY_F2            0x3B  /* 功能键 F2 */
#define HID_KEY_F3            0x3C  /* 功能键 F3 */
#define HID_KEY_F4            0x3D  /* 功能键 F4 */
#define HID_KEY_F5            0x3E  /* 功能键 F5 */
#define HID_KEY_F6            0x3F  /* 功能键 F6 */
#define HID_KEY_F7            0x40  /* 功能键 F7 */
#define HID_KEY_F8            0x41  /* 功能键 F8 */
#define HID_KEY_F9            0x42  /* 功能键 F9 */
#define HID_KEY_F10           0x43  /* 功能键 F10 */
#define HID_KEY_F11           0x44  /* 功能键 F11 */
#define HID_KEY_F12           0x45  /* 功能键 F12 */
#define HID_KEY_LEFT          0x50  /* 方向键 ←（Left Arrow） */
#define HID_KEY_RIGHT         0x4F  /* 方向键 →（Right Arrow） */
#define HID_KEY_UP            0x52  /* 方向键 ↑（Up Arrow） */
#define HID_KEY_DOWN          0x51  /* 方向键 ↓（Down Arrow） */

/* ========================================================================= */
/* 修饰键位图（byte1 / Modifier）                                            */
/* ========================================================================= */
#define HID_MOD_NONE          0x00
#define HID_MOD_LCTRL         0x01  /* bit0: 左 Ctrl  */
#define HID_MOD_LSHIFT        0x02  /* bit1: 左 Shift  */
#define HID_MOD_LALT          0x04  /* bit2: 左 Alt    */
#define HID_MOD_LGUI          0x08  /* bit3: 左 Win    */
#define HID_MOD_RCTRL         0x10  /* bit4: 右 Ctrl  */
#define HID_MOD_RSHIFT        0x20  /* bit5: 右 Shift  */
#define HID_MOD_RALT          0x40  /* bit6: 右 Alt    */
#define HID_MOD_RGUI          0x80  /* bit7: 右 Win    */

/* ========================================================================= */
/* 函数声明                                                                  */
/* ========================================================================= */

/**
  * @brief  发送单个 HID 按键报告（含按下+释放）
  * @param  modifier: 修饰键位图（Ctrl/Shift/Alt/Win）
  * @param  keycode:  HID Usage ID（见 HID_KEY_* 宏）
  * @note   内部包含 10ms 延时，确保主机能识别
  */
void HID_KB_SendKey(uint8_t modifier, uint8_t keycode);

/**
  * @brief  发送一个 ASCII 字符（自动处理大小写和符号）
  * @param  c: 要发送的 ASCII 字符（0x00~0x7F）
  * @note   只支持标准 US 键盘布局的 ASCII 字符
  *         非 ASCII（如中文）会直接忽略
  */
void HID_KB_SendChar(char c);

/**
  * @brief  发送一个 ASCII 字符串（逐字符发送）
  * @param  str: 要发送的字符串（以 '\0' 结尾）
  * @note   每个字符包含按下+释放，约 20ms/字符
  */
void HID_KB_SendString(const char *str);

/**
  * @brief  发送特殊功能键（Enter/Esc/方向键等）
  * @param  keycode:  HID Usage ID（见 HID_KEY_* 宏）
  */
void HID_KB_SendSpecialKey(uint8_t keycode);

/**
  * @brief  发送组合键（如 Ctrl+C、Alt+F4）
  * @param  modifier: 修饰键位图
  * @param  keycode:  主键 Usage ID
  * @example HID_KB_SendCombo(HID_MOD_LCTRL, HID_KEY_C);   // Ctrl+C
  * @example HID_KB_SendCombo(HID_MOD_LALT,  HID_KEY_F4);  // Alt+F4
  */
void HID_KB_SendCombo(uint8_t modifier, uint8_t keycode);


#endif /* MY_HID_KEYBOARD_H */
