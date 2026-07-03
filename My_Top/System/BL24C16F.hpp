#pragma once

#include "bsp.hpp"

// ============================================================
// EEPROM 芯片配置（按实际型号修改下面 4 个宏）
// ============================================================
// BL24C16F:  PAGE=16, SIZE=2048,  ADDR_SIZE=8BIT
// 24C32:    PAGE=32, SIZE=4096,  ADDR_SIZE=16BIT
// 24C64:    PAGE=32, SIZE=8192,  ADDR_SIZE=16BIT
// 24C256:   PAGE=64, SIZE=32768, ADDR_SIZE=16BIT
// ============================================================
#define EEPROM_DEV_ADDR       0x50                       // 7位 I2C 器件地址
#define EEPROM_PAGE_SIZE      32                         // 页大小（字节）
#define EEPROM_SIZE           4096                       // 总容量（字节）
#define EEPROM_ADDR_SIZE      I2C_MEMADD_SIZE_16BIT      // 内存地址位数
#define EEPROM_WRITE_CYCLE_MS 5                          // 内部写入时间


class BL24C16F
{
public:
    BL24C16F(I2C_HandleTypeDef *hi2c)
    {
        _hi2c = hi2c;
    }

    // ============================================================
    // 初始化：探测 EEPROM 是否在线
    // 返回 true=正常  false=未响应
    // ============================================================
    static bool init(void);

    // ============================================================
    // 读取一个字节
    // @addr: EEPROM 内部地址
    // @return: 读取到的字节值，失败返回 0xFF
    // ============================================================
    static uint8_t read_byte(uint16_t addr);

    // ============================================================
    // 写入一个字节
    // @addr: EEPROM 内部地址
    // @data: 要写入的字节
    // @return: true=成功  false=失败
    // ============================================================
    static bool write_byte(uint16_t addr, uint8_t data);

    // ============================================================
    // 连续读取多个字节
    // @addr: 起始地址
    // @buf:  存放读取数据的缓冲区
    // @len:  读取长度
    // @return: true=成功  false=失败
    // ============================================================
    static bool read(uint16_t addr, uint8_t *buf, uint16_t len);

    // ============================================================
    // 连续写入多个字节（自动处理页边界）
    // @addr: 起始地址
    // @buf:  待写入数据
    // @len:  写入长度
    // @return: true=成功  false=失败
    // ============================================================
    static bool write(uint16_t addr, const uint8_t *buf, uint16_t len);

private:
    // 等待内部写入完成（ACK 轮询，超时后返回）
    static void _wait_write_complete(void);

    static I2C_HandleTypeDef *_hi2c;
};
