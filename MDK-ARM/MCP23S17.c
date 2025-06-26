#include "MCP23S17.h"

MCP23S17_Command_t cmdWrite = MCP23S17_MAKE_WRITE_COMMAND(0, 0, 0);
MCP23S17_Command_t cmdRead = MCP23S17_MAKE_READ_COMMAND(0, 0, 0);

/**
 * @brief 写字节
 *
 * @param handle
 * @param buffer
 * @param size
 * @return MCP23S17_State_t
 */
static MCP23S17_State_t MCP23S17_Writebyte(MCP23S17_Handle handle, uint8_t *buffer, size_t size)
{
    MCP23S17_CHECKPTR(handle);
    MCP23S17_CHECKPTR(buffer);

    HAL_GPIO_WritePin(handle->Hardware.CS.port, handle->Hardware.CS.pin, GPIO_PIN_RESET);

    if (HAL_SPI_Transmit(handle->Hardware.spi, buffer, size, handle->Hardware.Timeout) != HAL_OK)
    {

        HAL_GPIO_WritePin(handle->Hardware.CS.port, handle->Hardware.CS.pin, GPIO_PIN_SET);
        return MCP23S17_SPIERR;
    }

    HAL_GPIO_WritePin(handle->Hardware.CS.port, handle->Hardware.CS.pin, GPIO_PIN_SET);
    return MCP23S17_OK;
}

/**
 * @brief 读字节
 *
 * @param handle
 * @param buffer
 * @param size
 * @return MCP23S17_State_t
 */
static MCP23S17_State_t MCP23S17_Readbyte(MCP23S17_Handle handle, uint8_t *buffer, size_t size)
{
    MCP23S17_CHECKPTR(handle);
    MCP23S17_CHECKPTR(buffer);

    HAL_GPIO_WritePin(handle->Hardware.CS.port, handle->Hardware.CS.pin, GPIO_PIN_RESET);

    if (HAL_SPI_Receive(handle->Hardware.spi, buffer, size, handle->Hardware.Timeout) != HAL_OK)
    {

        HAL_GPIO_WritePin(handle->Hardware.CS.port, handle->Hardware.CS.pin, GPIO_PIN_SET);
        return MCP23S17_SPIERR;
    }

    HAL_GPIO_WritePin(handle->Hardware.CS.port, handle->Hardware.CS.pin, GPIO_PIN_SET);
    return MCP23S17_OK;
}

/**
 * @brief 写寄存器
 *
 * @param handle
 * @param reg
 * @param buffer
 * @return MCP23S17_State_t
 */
static MCP23S17_State_t MCP23S17_WriteReg(MCP23S17_Handle handle, MCP23S17_Register_t reg, uint8_t buffer)
{
    MCP23S17_CHECKPTR(handle);
    uint8_t data[3] = {cmdWrite.raw, reg, buffer};
    return MCP23S17_Writebyte(handle, data, 3);
}

/**
 * @brief 读寄存器
 *
 * @param handle
 * @param reg
 * @param buffer
 * @return MCP23S17_State_t
 */
static MCP23S17_State_t MCP23S17_ReadReg(MCP23S17_Handle handle, uint8_t reg, uint8_t *buffer)
{
    MCP23S17_CHECKPTR(handle);
    uint8_t data[3] = {cmdRead.raw, (uint8_t)reg, 0x00};

    MCP23S17_CHECK_ERR(MCP23S17_Writebyte(handle, data, 2));

    MCP23S17_CHECK_ERR(MCP23S17_Readbyte(handle, data, data[2]));

    *buffer = data[2];
    return MCP23S17_OK;
}

/**
 * @brief 设置引脚方向
 *
 * @param handle
 * @param port
 * @param gpio
 * @param dir
 * @return MCP23S17_State_t
 */
MCP23S17_State_t MCP23S17_SetPinsDirection(MCP23S17_Handle handle, MCP23S17_GPIOPort_t port, MCP23S17_GPIO_NUM_t gpio, MCP23S17_DIR_t dir)
{
    MCP23S17_CHECKPTR(handle);

    uint8_t buffer = 0;

    MCP23S17_CHECK_ERR(MCP23S17_ReadReg(handle, port, &buffer));

    // out 0 input 1

    // buffer 已经是读出来的原有寄存器值
    if (dir == MCP23S17_OUTPUT)
    {
        buffer &= ~gpio;
    }
    else if (dir == MCP23S17_INPUT)
    {
        buffer |= gpio;
    }

    MCP23S17_CHECK_ERR(MCP23S17_WriteReg(handle, port, buffer));

    return MCP23S17_OK;
}

/**
 * @brief 设置极性翻转
 *
 * @param handle
 * @param ipolx
 * @param gpio
 * @param polarity
 * @return MCP23S17_State_t
 */
MCP23S17_State_t MCP23S17_SetPinsPolarity(MCP23S17_Handle handle, MCP23S17_IPOLx_t ipolx, MCP23S17_GPIO_NUM_t gpio, MCP23S17_Polarity_t polarity)
{
    MCP23S17_CHECKPTR(handle);

    uint8_t buffer = 0;

    MCP23S17_CHECK_ERR(MCP23S17_ReadReg(handle, ipolx, &buffer));

    if (polarity == MCP23S17_POL_NORMAL)
    {
        buffer &= ~gpio;
    }
    else if (polarity == MCP23S17_POL_INVERT)
    {
        buffer |= gpio;
    }

    MCP23S17_CHECK_ERR(MCP23S17_WriteReg(handle, ipolx, buffer));

    return MCP23S17_OK;
}

/**
 * @brief 
 * 
 * @param handle 
 * @param conf 
 * @return MCP23S17_State_t 
 */
MCP23S17_State_t MCP23S17_SetPinsInterrupt(MCP23S17_Handle handle, MCP23S17_Interrupt_conf_t *conf)
{
    MCP23S17_CHECKPTR(handle);

    uint8_t gpintenVal = 0;
    uint8_t defvalVal = 0;
    uint8_t intconVal = 0;
    uint8_t pullupVal = 0;
    uint8_t defPort, intPort, pullPort;

    // 1. 启用/禁用中断
    MCP23S17_CHECK_ERR(MCP23S17_ReadReg(handle, conf->port, &gpintenVal));
    if (conf->inter == MCP23S17_INTERRUPT_ENABLE) {
        gpintenVal |= conf->gpio;
    } else {
        gpintenVal &= ~conf->gpio;
    }
    MCP23S17_CHECK_ERR(MCP23S17_WriteReg(handle, conf->port, gpintenVal));

    // 2. 确定对应 DEFVAL / INTCON / GPPU 寄存器
    if (conf->port == MCP23S17_GPINTENA) {
        defPort  = MCP23S17_DEFVALA;
        intPort  = MCP23S17_INTCONA;
        pullPort = MCP23S17_GPPUA;
    } else {
        defPort  = MCP23S17_DEFVALB;
        intPort  = MCP23S17_INTCONB;
        pullPort = MCP23S17_GPPUB;
    }

    // 3. 读 DEFVAL 和 INTCON 当前值
    MCP23S17_CHECK_ERR(MCP23S17_ReadReg(handle, defPort, &defvalVal));
    MCP23S17_CHECK_ERR(MCP23S17_ReadReg(handle, intPort, &intconVal));

    // 4. 根据中断模式更新 DEFVAL 和 INTCON
    switch (conf->mode) {
    case MCP23S17_INT_TRIGGER_EDGE: // 电平变化触发
        defvalVal &= ~conf->gpio;
        intconVal &= ~conf->gpio;
        break;

    case MCP23S17_INT_TRIGGER_HIGH: // 高电平触发
        defvalVal &= ~conf->gpio;  // DEFVAL = 0
        intconVal |= conf->gpio;   // 启用比较模式
        break;

    case MCP23S17_INT_TRIGGER_LOW:  // 低电平触发
        defvalVal |= conf->gpio;    // DEFVAL = 1
        intconVal |= conf->gpio;    // 启用比较模式
        break;

    default:
        break;
    }

    // 5. 写回 DEFVAL 和 INTCON
    MCP23S17_CHECK_ERR(MCP23S17_WriteReg(handle, defPort, defvalVal));
    MCP23S17_CHECK_ERR(MCP23S17_WriteReg(handle, intPort, intconVal));

    // 6. 配置内部上拉
    MCP23S17_CHECK_ERR(MCP23S17_ReadReg(handle, pullPort, &pullupVal));
    if (conf->pull == MCP23S17_PULLUP_ENABLE) {
        pullupVal |= conf->gpio;
    } else {
        pullupVal &= ~conf->gpio;
    }
    MCP23S17_CHECK_ERR(MCP23S17_WriteReg(handle, pullPort, pullupVal));

    return MCP23S17_OK;
}


MCP23S17_State_t  MCP23S17_InterruptA_Callback(MCP23S17_Handle *handle)
{
    MCP23S17_CHECKPTR(handle);
    MCP23S17_CHECKPTR(*handle);

    if(!(*handle)->flag.bit.INTA_Tigger)
    {
   (*handle)->flag.bit.INTA_Tigger = 1;
    }

    return MCP23S17_OK;
}

MCP23S17_State_t  MCP23S17_InterruptB_Callback(MCP23S17_Handle *handle)
{
    MCP23S17_CHECKPTR(handle);
    MCP23S17_CHECKPTR(*handle);

    if(!(*handle)->flag.bit.INTB_Tigger)
    {
   (*handle)->flag.bit.INTB_Tigger = 1;
    }

    return MCP23S17_OK;
}

MCP23S17_State_t MCP23S17_LoopHandler(MCP23S17_Handle *handle)
{
    MCP23S17_CHECKPTR(handle);
    MCP23S17_CHECKPTR(*handle);

    uint8_t gpio_mask = 0;



    if ((*handle)->flag.bit.INTA_Tigger == 1)
    {
        (*handle)->flag.bit.INTA_Tigger = 0;

            MCP23S17_CHECK_ERR(MCP23S17_ReadReg(*handle,MCP23S17_INTFA,&gpio_mask)); 
        if ((*handle)->intrA != NULL)
        {
            (*handle)->intrA(gpio_mask);
        }
    }


    if ((*handle)->flag.bit.INTB_Tigger == 1)
    {
        (*handle)->flag.bit.INTB_Tigger = 0;
        if ((*handle)->intrB != NULL)
        {
            (*handle)->intrB(gpio_mask);
        }
    }

    return MCP23S17_OK;
}


MCP23S17_State_t MCP23S17_RegisterINTA_Callback(MCP23S17_Handle *handle,void (*cb)(uint8_t gpio_mask))
{
    MCP23S17_CHECKPTR(handle);
    MCP23S17_CHECKPTR(*handle);

    (*handle)->intrA = cb;

    return MCP23S17_OK;
}

MCP23S17_State_t MCP23S17_RegisterINTB_Callback(MCP23S17_Handle *handle,MCP23S17_INTCallback_t cb)
{
    MCP23S17_CHECKPTR(handle);
    MCP23S17_CHECKPTR(*handle);

    (*handle)->intrB = cb;
    
    return MCP23S17_OK;
}

