#include "XBLW24C64.hpp"
#include "HAL_System.hpp"
I2C_HandleTypeDef* XBLW24C64::_hi2c = nullptr;

// ============================================================
// 初始化：探测 EEPROM 是否在线
// ============================================================
bool XBLW24C64::init(void)
{
    if (HAL_I2C_IsDeviceReady(_hi2c,
                              EEPROM_DEV_ADDR << 1,   // 8位写地址
                              3,                       // 重试次数
                              100) == HAL_OK)          // 超时 ms
    {
        return true;
    }
    return false;
}


// ============================================================
// 读取一个字节
// ============================================================
uint8_t XBLW24C64::read_byte(uint16_t addr)
{
    uint8_t data = 0xFF;

    if (HAL_I2C_Mem_Read(_hi2c, EEPROM_DEV_ADDR << 1, addr,
                         EEPROM_ADDR_SIZE,
                         &data, 1, 100) != HAL_OK)
    {
        return 0xFF;
    }
    return data;
}


// ============================================================
// 写入一个字节
// ============================================================
bool XBLW24C64::write_byte(uint16_t addr, uint8_t data)
{
    if (HAL_I2C_Mem_Write(_hi2c, EEPROM_DEV_ADDR << 1, addr,
                           EEPROM_ADDR_SIZE,
                           &data, 1, 100) != HAL_OK)
    {
        return false;
    }
    _wait_write_complete();
    return true;
}


// ============================================================
// 连续读取多个字节（16位地址芯片支持跨页连续读）
// ============================================================
bool XBLW24C64::read(uint16_t addr, uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0)
        return false;
    if (addr + len > EEPROM_SIZE)
        return false;

    if (HAL_I2C_Mem_Read(_hi2c, EEPROM_DEV_ADDR << 1, addr,
                         EEPROM_ADDR_SIZE,
                         buf, len, 100) != HAL_OK)
    {
        return false;
    }
    return true;
}


// ============================================================
// 连续写入多个字节（自动处理页边界）
// ============================================================
bool XBLW24C64::write(uint16_t addr, const uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0)
        return false;
    if (addr + len > EEPROM_SIZE)
        return false;

    while (len > 0)
    {
        // 计算当前页内还能写多少字节
        uint8_t page_remain = EEPROM_PAGE_SIZE - (addr % EEPROM_PAGE_SIZE);
        uint16_t chunk = (len < page_remain) ? len : page_remain;//等价于min(len, page_remain);

        if (HAL_I2C_Mem_Write(_hi2c, EEPROM_DEV_ADDR << 1, addr,
                               EEPROM_ADDR_SIZE,
                               (uint8_t *)buf, chunk, 100) != HAL_OK)
        {
            return false;
        }

        // 等待内部写入完成
        _wait_write_complete();

        addr += chunk;
        buf  += chunk;
        len  -= chunk;
    }
    return true;
}


// ============================================================
// 等待 EEPROM 内部写入完成（ACK 轮询）
// ============================================================
void XBLW24C64::_wait_write_complete(void)
{
    uint32_t time_old = HAL_System::get_tick_ms();
    while (HAL_I2C_IsDeviceReady(_hi2c,
                                  EEPROM_DEV_ADDR << 1,
                                  1,     // 每次1次尝试
                                  1) != HAL_OK)   // 1ms 超时
    {
        if( (HAL_System::get_tick_ms()-time_old) >= EEPROM_WRITE_CYCLE_MS)
        {
            return;
        }
    }
}



