#define SYSTEM_CLOCK_FREQ (168000000) // 168MHz
typedef enum
{
    I2C_OK = 0,
    I2C_ERR_TIMEOUT,
    I2C_ERR_NO_ACK,
} i2c_status_t;


i2c_status_t i2c_master_transmit(uint32_t i2c_periph, uint32_t address, uint8_t* pData, uint16_t Size, uint32_t timeout)
{
    uint64_t timeout_counter = ((SYSTEM_CLOCK_FREQ/1000)*timeout);
    /* 等待I2C总线空闲 */
    while(i2c_flag_get(i2c_periph, I2C_FLAG_I2CBSY))
    {
        if((timeout_counter--) == 0) 
        {
            i2c_stop_on_bus();
            return I2C_ERR_TIMEOUT;
        }
    }
     /* 发送起始条件 */
    i2c_start_on_bus(i2c_periph);
    /* 等待起始条件发送完成 */
    while(!i2c_flag_get(i2c_periph, I2C_FLAG_SBSEND))
    {
        if((timeout_counter--) == 0) 
        {
            i2c_stop_on_bus();
            return I2C_ERR_TIMEOUT;
        }
    }
    /* 发送从机地址（写模式） */
    i2c_master_addressing(i2c_periph, address, I2C_TRANSMITTER);
    /* 等待地址发送完毕 */
    while(!i2c_flag_get(i2c_periph, I2C_FLAG_ADDSEND))
    {
        if(i2c_flag_get(i2c_periph,I2C_FLAG_AERR)) 
        {
            i2c_flag_clear(i2c_periph,I2C_FLAG_AERR);
            i2c_stop_on_bus();
            return I2C_ERR_NO_ACK;
        }
        if((timeout_counter--) == 0) 
        {
            i2c_stop_on_bus();
            return I2C_ERR_TIMEOUT;
        }
    }
    /* 清除地址发送完成标志 */
    i2c_flag_clear(i2c_periph, I2C_FLAG_ADDSEND);
    /* 发送数据 */
    for(uint16_t i = 0; i < Size; i++) 
    {
        /* 等待缓冲区可写入 */
        while(!i2c_flag_get(i2c_periph, I2C_FLAG_TBE))
        {
            if((timeout_counter--) == 0) 
            {
                i2c_stop_on_bus();
                return I2C_ERR_TIMEOUT;
            }
        }
        i2c_data_transmit( i2c_periph, *(pData+i) );
    }
    /* 等待数据发送完成 */
    while(!i2c_flag_get(i2c_periph, I2C_FLAG_BTC))
    {
        if((timeout_counter--) == 0) 
        {
            i2c_stop_on_bus();
            return I2C_ERR_TIMEOUT;
        }
    }
    /* 发送停止条件 */
    i2c_stop_on_bus(i2c_periph);
    /* 等待停止完成 */
    while(i2c_flag_get(i2c_periph, I2C_FLAG_I2CBSY))
    {
        if((timeout_counter--) == 0) 
        {
            i2c_stop_on_bus();
            return I2C_ERR_TIMEOUT;
        }
    }
    return I2C_OK;
}

#define MAX_DELAY      0xFFFFFFFFU
void OLED_WriteCommand(uint8_t cmd)
{
	uint8_t sendBuffer[2];
	sendBuffer[0]=0x00;
	sendBuffer[1]=cmd;
    i2c_master_transmit(I2Cx, OLED_ADDRESS, sendBuffer, 2, MAX_DELAY);
}

void OLED_WriteData(uint8_t dat)
{
	uint8_t sendBuffer[2];
	sendBuffer[0]=0x40;
	sendBuffer[1]=dat;
	i2c_master_transmit(I2Cx, OLED_ADDRESS, sendBuffer, 2, MAX_DELAY);
}

HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)