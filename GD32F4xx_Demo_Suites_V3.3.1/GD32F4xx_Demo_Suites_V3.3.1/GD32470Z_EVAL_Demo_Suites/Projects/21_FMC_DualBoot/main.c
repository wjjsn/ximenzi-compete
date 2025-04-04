/*!
    \file    main.c
    \brief   FMC dualboot

    \version 2024-12-20, V3.3.1, demo for GD32F4xx
*/

/*
    Copyright (c) 2024, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32f4xx.h"
#include "gd32f470z_eval.h"
#include "gd32f4xx_it.h"
#include <stdio.h>

volatile uint8_t flag = 0;

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
#ifndef BB_ENABLE
    nvic_vector_table_set(NVIC_VECTTAB_FLASH, 0x000000);
    /* configure user key */
    gd_eval_key_init(KEY_TAMPER, KEY_MODE_EXTI);
    /* configure EVAL_COM0 */
    gd_eval_com_init(EVAL_COM0);
    /* configure LED1 */
    gd_eval_led_init(LED1);

    printf("\r\n This application starts from bank0, and turn on the LED1 \r\n");
    gd_eval_led_on(LED1);
#else
    nvic_vector_table_set(NVIC_VECTTAB_FLASH, 0x100000);
    /* configure user key */
    gd_eval_key_init(KEY_TAMPER, KEY_MODE_EXTI);
    /* configure EVAL_COM0 */
    gd_eval_com_init(EVAL_COM0);
    /* configure LED2 */
    gd_eval_led_init(LED2);

    printf("\r\n This application starts from bank1, and turn on the LED2 \r\n");
    gd_eval_led_on(LED2);
#endif /* BB_ENABLE */

    while(1) {
        /* wait for user key is pressed */
        if(flag) {
            flag = 0;
            /* unlock the option byte operation */
            ob_unlock();
            /* clear pending flags */
            fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_OPERR | FMC_FLAG_WPERR | FMC_FLAG_PGMERR | FMC_FLAG_PGSERR);
            /* configure BB bit */
            if(RESET == (FMC_OBCTL0 & FMC_OBCTL0_BB)) {
                ob_boot_mode_config(OB_BB_ENABLE);
                printf("\r\n Set BB bit and then restart from bank1\r\n");
            } else {
                ob_boot_mode_config(OB_BB_DISABLE);
                printf("\r\n Reset BB bit and then restart from bank0\r\n");
            }
            /* send option byte change command */
            ob_start();
            /* lock the option byte operation */
            ob_lock();

            /* generate a systemreset to reload option byte */
            NVIC_SystemReset();
        }
    }
}

/*!
    \brief      retarget the C library printf function to the USART
    \param[in]  none
    \param[out] none
    \retval     none
*/
int fputc(int ch, FILE *f)
{
    while(usart_flag_get(USART0, USART_FLAG_TC) == RESET);
    usart_data_transmit(USART0, (uint8_t) ch);
    while(usart_flag_get(USART0, USART_FLAG_TC) == RESET);
    return ch;
}
