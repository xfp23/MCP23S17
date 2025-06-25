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
    uint8_t data[3] = {cmdWrite.raw,reg, buffer};
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
static MCP23S17_State_t MCP23S17_ReadReg(MCP23S17_Handle handle, MCP23S17_Register_t reg, uint8_t *buffer)
{
    MCP23S17_CHECKPTR(handle);
    uint8_t data[3] = {cmdRead.raw,(uint8_t)reg, 0x00};

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
MCP23S17_State_t MCP23S17_SetPinsPolarity(MCP23S17_Handle handle,MCP23S17_IPOLx_t ipolx,MCP23S17_GPIO_NUM_t gpio,MCP23S17_Polarity_t polarity)
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
 * @param interruptx 
 * @param gpio 
 * @param en 
 * @return MCP23S17_State_t 
 */
MCP23S17_State_t MCP23S17_SetPinsInterrupt(MCP23S17_Handle handle,MCP23S17_Interruptx_t interruptx,MCP23S17_GPIO_NUM_t gpio,MCP23S17_InterruptEnable_t en)
{
    MCP23S17_CHECKPTR(handle);

    uint8_t buffer = 0;


    MCP23S17_CHECK_ERR(MCP23S17_ReadReg(handle, interruptx, &buffer));


    if (en == MCP23S17_INTERRUPT_DISABLE)
    {
        buffer &= ~gpio;
    }
    else if (en == MCP23S17_INTERRUPT_ENABLE)
    {
        buffer |= gpio;
    }

    MCP23S17_CHECK_ERR(MCP23S17_WriteReg(handle, interruptx, buffer));

    return MCP23S17_OK;
}


