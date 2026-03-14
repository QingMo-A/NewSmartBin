#include "sccb.h"

#define SCCB_SCL_HIGH() HAL_GPIO_WritePin(SCCB_SCL_GPIO_Port, SCCB_SCL_Pin, GPIO_PIN_SET)
#define SCCB_SCL_LOW()  HAL_GPIO_WritePin(SCCB_SCL_GPIO_Port, SCCB_SCL_Pin, GPIO_PIN_RESET)
#define SCCB_SDA_HIGH() HAL_GPIO_WritePin(SCCB_SDA_GPIO_Port, SCCB_SDA_Pin, GPIO_PIN_SET)
#define SCCB_SDA_LOW()  HAL_GPIO_WritePin(SCCB_SDA_GPIO_Port, SCCB_SDA_Pin, GPIO_PIN_RESET)
#define SCCB_SDA_READ() HAL_GPIO_ReadPin(SCCB_SDA_GPIO_Port, SCCB_SDA_Pin)

static void SCCB_Delay(void)
{
  for (volatile uint32_t i = 0; i < 40U; ++i)
  {
    __NOP();
  }
}

static void SCCB_SendByte(uint8_t data)
{
  for (uint8_t bit = 0U; bit < 8U; ++bit)
  {
    SCCB_SCL_LOW();
    if ((data & 0x80U) != 0U)
    {
      SCCB_SDA_HIGH();
    }
    else
    {
      SCCB_SDA_LOW();
    }
    SCCB_Delay();
    SCCB_SCL_HIGH();
    SCCB_Delay();
    data <<= 1;
  }
  SCCB_SCL_LOW();
}

static uint8_t SCCB_ReceiveByte(void)
{
  uint8_t data = 0U;

  SCCB_SDA_HIGH();
  for (uint8_t bit = 0U; bit < 8U; ++bit)
  {
    data <<= 1;
    SCCB_SCL_LOW();
    SCCB_Delay();
    SCCB_SCL_HIGH();
    SCCB_Delay();
    if (SCCB_SDA_READ() == GPIO_PIN_SET)
    {
      data |= 0x01U;
    }
  }

  SCCB_SCL_LOW();
  return data;
}

static HAL_StatusTypeDef SCCB_Start(void)
{
  SCCB_SDA_HIGH();
  SCCB_SCL_HIGH();
  SCCB_Delay();

  if (SCCB_SDA_READ() == GPIO_PIN_RESET)
  {
    return HAL_BUSY;
  }

  SCCB_SDA_LOW();
  SCCB_Delay();
  SCCB_SCL_LOW();
  SCCB_Delay();
  return HAL_OK;
}

static void SCCB_Stop(void)
{
  SCCB_SCL_LOW();
  SCCB_SDA_LOW();
  SCCB_Delay();
  SCCB_SCL_HIGH();
  SCCB_Delay();
  SCCB_SDA_HIGH();
  SCCB_Delay();
}

static HAL_StatusTypeDef SCCB_WaitAck(void)
{
  SCCB_SCL_LOW();
  SCCB_SDA_HIGH();
  SCCB_Delay();
  SCCB_SCL_HIGH();
  SCCB_Delay();

  if (SCCB_SDA_READ() == GPIO_PIN_SET)
  {
    SCCB_SCL_LOW();
    return HAL_ERROR;
  }

  SCCB_SCL_LOW();
  return HAL_OK;
}

static void SCCB_NoAck(void)
{
  SCCB_SCL_LOW();
  SCCB_SDA_HIGH();
  SCCB_Delay();
  SCCB_SCL_HIGH();
  SCCB_Delay();
  SCCB_SCL_LOW();
}

void SCCB_Init(void)
{
  SCCB_SCL_HIGH();
  SCCB_SDA_HIGH();
  SCCB_Stop();
}

HAL_StatusTypeDef SCCB_WriteReg(uint8_t reg, uint8_t value)
{
  if (SCCB_Start() != HAL_OK)
  {
    return HAL_BUSY;
  }

  SCCB_SendByte(SCCB_ADDR_OV7725_WRITE);
  if (SCCB_WaitAck() != HAL_OK)
  {
    SCCB_Stop();
    return HAL_ERROR;
  }

  SCCB_SendByte(reg);
  if (SCCB_WaitAck() != HAL_OK)
  {
    SCCB_Stop();
    return HAL_ERROR;
  }

  SCCB_SendByte(value);
  if (SCCB_WaitAck() != HAL_OK)
  {
    SCCB_Stop();
    return HAL_ERROR;
  }

  SCCB_Stop();
  return HAL_OK;
}

HAL_StatusTypeDef SCCB_ReadReg(uint8_t reg, uint8_t *value)
{
  if (value == NULL)
  {
    return HAL_ERROR;
  }

  if (SCCB_Start() != HAL_OK)
  {
    return HAL_BUSY;
  }

  SCCB_SendByte(SCCB_ADDR_OV7725_WRITE);
  if (SCCB_WaitAck() != HAL_OK)
  {
    SCCB_Stop();
    return HAL_ERROR;
  }

  SCCB_SendByte(reg);
  if (SCCB_WaitAck() != HAL_OK)
  {
    SCCB_Stop();
    return HAL_ERROR;
  }

  SCCB_Stop();

  if (SCCB_Start() != HAL_OK)
  {
    return HAL_BUSY;
  }

  SCCB_SendByte(SCCB_ADDR_OV7725_READ);
  if (SCCB_WaitAck() != HAL_OK)
  {
    SCCB_Stop();
    return HAL_ERROR;
  }

  *value = SCCB_ReceiveByte();
  SCCB_NoAck();
  SCCB_Stop();

  return HAL_OK;
}
