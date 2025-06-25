#ifndef MCP23S17_H
#define MCP23S17_H

/**
 * @file MCP23S17.h
 * @author https://github.com/xfp23
 * @brief MCP23S17 IO扩展驱动
 * @note 此驱动不会帮你初始化任何硬件外设，请自行初始化硬件外设。
 *       此驱动直接使用分组(字节)模式，不支持顺序模式
 * @version 0.1
 * @date 2025-06-23
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "main.h"
#include "stdint.h"

#ifdef __cplusplus
extern "C"
{
#endif // ! __cplusplus

#define MCP23S17_CHECKPTR(ptr)     \
do                             \
{                              \
if ((ptr) == NULL)         \
{                          \
    return MCP23S17_ERROR; \
}                          \
} while (0)


/**
 * @brief 生成写命令
 * 
 */
#define MCP23S17_MAKE_WRITE_COMMAND(A0, A1, A2) \
((MCP23S17_Command_t){                      \
.rw = 0,                                \
.address0 = (A0),                       \
.address1 = (A1),                       \
.address2 = (A2),                       \
.fixedBits = 0x04})


/**
 * @brief 生成读命令
 * 
 */
#define MCP23S17_MAKE_READ_COMMAND(A0, A1, A2) \
((MCP23S17_Command_t){                     \
.rw = 1,                               \
.address0 = (A0),                      \
.address1 = (A1),                      \
.address2 = (A2),                      \
.fixedBits = 0x04})


/**
 * @brief 检查错误
 * 
 */
#define MCP23S17_CHECK_ERR(ret)         \
do                                  \
{                                   \
    MCP23S17_State_t state = (ret); \
    if (state != MCP23S17_OK)       \
        return state;               \
} while (0)

typedef enum
{
    MCP23S17_IODIRA = 0x00,
    MCP23S17_IODIRB = 0x01,
    // MCP23S17_IPOLA = 0x02,
    // MCP23S17_IPOLB = 0x03,
    // MCP23S17_GPINTENA = 0x04,
    // MCP23S17_GPINTENB = 0x05,
    MCP23S17_DEFVALA,
    MCP23S17_DEFVALB,
    MCP23S17_INTCONA,
    MCP23S17_INTCONB,
    MCP23S17_IOCON,
    MCP23S17_GPPUA = 0X0C,
    MCP23S17_GPPUB = 0X0D,
    MCP23S17_INTFA,
    MCP23S17_INTFB,
    MCP23S17_INTCAPA,
    MCP23S17_INTCAPB,
    MCP23S17_OLATA = 0x14,
    MCP23S17_OLATB = 0x15,
} MCP23S17_Register_t; // 寄存器

typedef enum
{
    MCP23S17_OK,
    MCP23S17_ERROR,
    MCP23S17_SPIERR,
} MCP23S17_State_t; // 操作状态

typedef enum
{
    MCP23S17_GPIOA = 0x12,
    MCP23S17_GPIOB = 0x13,
} MCP23S17_GPIOPort_t; // gpio端口


typedef enum
{
    MCP23S17_OUTPUT,
    MCP23S17_INPUT,
} MCP23S17_DIR_t;

typedef enum
{
    MCP23S17_POL_NORMAL = 0, // 正极性，不翻转输入gpio
    MCP23S17_POL_INVERT = 1, // 负极性,翻转gpio
} MCP23S17_Polarity_t;

typedef enum
{
    MCP23S17_IPOLA = 0x02,
    MCP23S17_IPOLB = 0x03,
}MCP23S17_IPOLx_t;

typedef enum
{
    MCP23S17_GPINTENA = 0x04,
    MCP23S17_GPINTENB = 0x05,
}MCP23S17_Interruptx_t;

typedef enum
{
    MCP23S17_INTERRUPT_DISABLE = 0,  // 禁用中断
    MCP23S17_INTERRUPT_ENABLE  = 1,  // 启用中断
} MCP23S17_InterruptEnable_t;


/**
 * @brief | 配置                  | 含义                          | 行为                                         |
| ------------------- | --------------------------- | ------------------------------------------ |
| INTCON=1 且 DEFVAL=0 | 输入引脚只要为 **1（高电平）** 时触发中断    | 相当于“只要引脚为高电平时中断有效”，直到引脚返回 0 时中断清除          |
| INTCON=1 且 DEFVAL=1 | 输入引脚只要为 **0（低电平）** 时触发中断    | 相当于“只要引脚为低电平时中断有效”，直到引脚返回 1 时中断清除          |
| INTCON=0（无视 DEFVAL） | 引脚只要**电平发生变化**（上升沿或下降沿）触发中断 | 相当于边沿触发：只触发一次，然后必须读 `INTCAP` 或 `GPIO` 清除中断 |
 */
typedef struct 
{

}MCP23S17_Interrupt_conf_t;

typedef enum
{
    MCP23S17_GPIO_NUM_0 = (1 << 0),
    MCP23S17_GPIO_NUM_1 = (1 << 1),
    MCP23S17_GPIO_NUM_2 = (1 << 2),
    MCP23S17_GPIO_NUM_3 = (1 << 3),
    MCP23S17_GPIO_NUM_4 = (1 << 4),
    MCP23S17_GPIO_NUM_5 = (1 << 5),
    MCP23S17_GPIO_NUM_6 = (1 << 6),
    MCP23S17_GPIO_NUM_7 = (1 << 7),
} MCP23S17_GPIO_NUM_t; // 含掩码


typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} MCP23S17_gpio_t;

typedef union
{
    uint8_t raw; ///< Raw 8-bit command byte
    struct
    {
        uint8_t rw : 1;        ///< Read=1, Write=0
        uint8_t address0 : 1;  ///< Hardware address A0
        uint8_t address1 : 1;  ///< Hardware address A1
        uint8_t address2 : 1;  ///< Hardware address A2
        uint8_t fixedBits : 4; ///< Fixed upper bits (usually 0b0100)
    };
} MCP23S17_Command_t;

// typedef union
// {
//     uint8_t flag;
//     struct
//     {

//     }bit;
// }MCP23S17_flag_t;

typedef struct
{
    MCP23S17_gpio_t CS;     // spi片选
    SPI_HandleTypeDef *spi; // 硬件spi
    int Timeout;            // spi超时时间
} MCP23S17_conf_t;

typedef struct
{
    MCP23S17_gpio_t CS;     // 片选
    SPI_HandleTypeDef *spi; // 硬件spi
    int Timeout;
} MCP23S17_Hardware_t;

typedef struct
{
    MCP23S17_Hardware_t Hardware; // 硬件配置

} MCP23S17_obj;

typedef MCP23S17_obj *MCP23S17_Handle; // 句柄

#ifdef __cplusplus
}
#endif // !__cplusplus

#endif // !MCP23S17_H