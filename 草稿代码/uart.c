#include <stdint.h>
void gpio_config()
{
    rcu_periph_clock_disable(RCU_GPIOx);
    gpio_mode_set(GPIOx, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_x);//tx
    gpio_mode_set(GPIOx, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_x);//rx
    gpio_af_set(GPIOx, GPIO_AF_7, GPIO_PIN_x);//usart
    gpio_af_set(GPIOx, GPIO_AF_7, GPIO_PIN_x);//usart
}

void usart_config()
{
    rcu_periph_clock_disable(RCU_USARTx);
    usart_baudrate_set(USARTx, 115200);//设置波特率
    usart_parity_config(USARTx, USART_PM_NONE);//不校验
    usart_word_length_set(USARTx, USART_WL_8BIT);//8bit
    usart_stop_bit_set(USARTx, USART_STB_1BIT);//1bit停止位
    usart_transmit_config(USARTx, USART_TRANSMIT_ENABLE);//启用发送
    usart_receive_config(USARTx, USART_RECEIVE_ENABLE);//启用接收
    usart_data_first_config(USARTx, USART_MSBF_LSB);//先发LSB
    usart_hardware_flow_rts_config(USARTx, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USARTx, USART_CTS_DISABLE);//不启用硬件流控制
    nvic_irq_enable(USARTx_IRQn, 0, 0);//打开串口中断
    usart_interrupt_enable(USARTx, USART_INT_RBNE);//启用串口接收中断
    usart_enable(USARTx);//打开串口
}

typedef enum
{
    USART_OK = 0,
    USART_ERR_TIMEOUT,
} usart_status_t;

usart_status_t uart_transmit(uint32_t usart_periph, uint8_t* pData, uint16_t Size, uint32_t timeout)
{
    uint64_t timeout_counter = ((SYSTEM_CLOCK_FREQ/1000)*timeout);
    for(uint16_t i=0;i<Size;i++)
    {
        while(usart_flag_get(usart_periph, USART_FLAG_TBE) == RESET);
        {
            if((timeout_counter--) == 0) 
            {
                return USART_ERR_TIMEOUT;
            }
        }
        usart_data_transmit(uint32_t usart_periph, (uint16_t) *(pData+i));
    }
    return USART_OK;
}