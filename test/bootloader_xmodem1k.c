/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_flexspi.h"
#include "fsl_lpuart.h"

#include "pin_mux.h"
#include "clock_config.h"

#include "flexspi_nor.h"
#include "xmodem1k.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define BOOT_APP_START_ADDR_BASE    0x60008000    //32KB for bootloader
/*******************************************************************************
 * Prototypes
 ******************************************************************************/


/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile uint32_t boot_program_addr_point = BOOT_APP_START_ADDR_BASE;

volatile uint32_t g_systick_time_count = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/

/**
 * \brief SYSTICK 中断处理函数
 * \return 无
 * \note SYSTICK中断处理属于内核中断，故无需调用
 *       am_int_connect() 和 am_int_enable() 函数。
 */
void SysTick_Handler(void)
{
    g_systick_time_count++;
}


uint32_t sysTimerGet(void)
{
    return g_systick_time_count;
}

void sysTimerClr(uint32_t i)
{
    /* 设置当前的值为0 */
    g_systick_time_count = 0;
}


uint8_t uartsendnoblock (uint8_t *p_ch)
{
    if ((LPUART1->STAT & LPUART_STAT_TDRE_MASK)){
        LPUART1->DATA = (uint32_t)(*p_ch);
        return 1;
    } else {
        return 0;
    }
}

uint8_t uartreadnoblock (uint8_t *p_ch)
{
    uint32_t statusFlag;
#if defined(FSL_FEATURE_LPUART_HAS_7BIT_DATA_SUPPORT) && FSL_FEATURE_LPUART_HAS_7BIT_DATA_SUPPORT
    uint32_t ctrl = LPUART1->CTRL;
    bool isSevenDataBits =
        ((ctrl & LPUART_CTRL_M7_MASK) || ((!(ctrl & LPUART_CTRL_M_MASK)) && (ctrl & LPUART_CTRL_PE_MASK)));
#endif

#if defined(FSL_FEATURE_LPUART_HAS_FIFO) && FSL_FEATURE_LPUART_HAS_FIFO
    if (0 == ((LPUART1->WATER & LPUART_WATER_RXCOUNT_MASK) >> LPUART_WATER_RXCOUNT_SHIFT))
#else
    if (!(LPUART1->STAT & LPUART_STAT_RDRF_MASK))
#endif
    {
        statusFlag = LPUART_GetStatusFlags(LPUART1);

        if (statusFlag & kLPUART_RxOverrunFlag)
        {
            LPUART_ClearStatusFlags(LPUART1, kLPUART_RxOverrunFlag);
            return 0;
        }

        if (statusFlag & kLPUART_NoiseErrorFlag)
        {
            LPUART_ClearStatusFlags(LPUART1, kLPUART_NoiseErrorFlag);
            return 0;
        }

        if (statusFlag & kLPUART_FramingErrorFlag)
        {
            LPUART_ClearStatusFlags(LPUART1, kLPUART_FramingErrorFlag);
            return 0;
        }

        if (statusFlag & kLPUART_ParityErrorFlag)
        {
            LPUART_ClearStatusFlags(LPUART1, kLPUART_ParityErrorFlag);
            return 0;
        }
        
        return 0;
    }
#if defined(FSL_FEATURE_LPUART_HAS_7BIT_DATA_SUPPORT) && FSL_FEATURE_LPUART_HAS_7BIT_DATA_SUPPORT
    if (isSevenDataBits)
    {
        *p_ch = (LPUART1->DATA & 0x7F);
    }
    else
    {
        *p_ch = LPUART1->DATA;
    }
#else
    *p_ch = (uint8_t)(LPUART1->DATA);
#endif

    return 1;   
}


xmodem_fun_handle_t gp_xmodem_fun ={
    uartreadnoblock,
    uartsendnoblock,
    sysTimerClr,
    sysTimerGet,
    NULL,
    
};

uint8_t boot_data_program (uint8_t *ptr, uint32_t len)
{
    status_t status;
    int i = 0;
    
    if ((boot_program_addr_point % FLASH_SECTOR_SIZE) == 0) {
        /* Erase sectors. */
        status = flexspi_nor_flash_erase_sector(FLEXSPI, (boot_program_addr_point - EXAMPLE_FLEXSPI_AMBA_BASE) );
        if (status != kStatus_Success)
        {
            return 0;
        }
    }
    
    if (len == 128) {
        status =
            flexspi_nor_flash_page_program(FLEXSPI, (boot_program_addr_point - EXAMPLE_FLEXSPI_AMBA_BASE), (void *)ptr, len);
        if (status != kStatus_Success)
        {
            return 0;
        }
        boot_program_addr_point += 128;
    } else if (len == 1024) {
        for (i = 0; i < (1024 / 256); i++) {
            status =
                flexspi_nor_flash_page_program(FLEXSPI, (boot_program_addr_point - EXAMPLE_FLEXSPI_AMBA_BASE), (void *)ptr, 256);
            if (status != kStatus_Success)
            {
                return 0;
            }
            boot_program_addr_point += 256;
            ptr += 256;
        }
    } else {
        return 0;
    }

    return 1;
}

/*********************************************************************************************************
** Function name:       vControlSwitch
** Descriptions:        控制转换到用户代码程序
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
#if defined(__CC_ARM)

static __asm void vControlSwitch (void)
{
    ldr   r0, = BOOT_APP_START_ADDR_BASE
    ldr   r0, [r0]
    mov   sp, r0
    
    ldr   r0, = BOOT_APP_START_ADDR_BASE +4
    ldr   r0, [r0]
    bx    r0
}

#else
  #error "You must overload this funcation!"
#endif

/*********************************************************************************************************
** Function name:       vSceneRenew
** Descriptions:        具体控制器的Bootloader运行结束后初始化用户代码环境
**                      关闭SysTick，将向量表重定向到用户程序ROM的起始地址
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void vSceneRenew (void)
{
    SysTick->LOAD  = 0;                                                 /* reset the load register      */
    SysTick->VAL   = 0;                                                 /* reset the Counter Value      */
    SysTick->CTRL  = 0;                                                 /* disable SysTick Timer        */
    
    SCB->VTOR  = BOOT_APP_START_ADDR_BASE;                                   
}

/*!
 * @brief Main function
 */
int main(void)
{
    status_t status;
    uint8_t vendorID = 0;
    
    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t led_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};

    /* Board pin, clock, debug console init */
    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    /* Print a note to terminal. */
    PRINTF("\r\nFlexspi Nor Bootloader Xmodem1K Example.\r\n");

    PRINTF("\r\nFlexspi Nor init.\r\n");
    flexspi_nor_init();    

    /* Get vendor ID. */
    status = flexspi_nor_get_vendor_id(FLEXSPI, &vendorID);
    if (status != kStatus_Success)
    {
        return status;
    }
    PRINTF("\r\nVendor ID: 0x%x\r\n", vendorID);

    
    /* Init output LED GPIO. */
    GPIO_PinInit(BOARD_USER_LED_GPIO, BOARD_USER_LED_GPIO_PIN, &led_config);
    SysTick_Config(SystemCoreClock/1000);
    PRINTF("\r\nStart xmodem1k client. \r\n");
    
#if TEST_APP_JUMP
    PRINTF("\r\n Firmware Download Complete,Jump App. \r\n");
    LPUART_Deinit(LPUART1);
    vSceneRenew();
    vControlSwitch();
#endif
    while (1)
    {
        boot_program_addr_point = BOOT_APP_START_ADDR_BASE;
        /* Check whether occur interupt and toggle LED */
        if (xmodem1k_client(boot_data_program,&gp_xmodem_fun,100,1000) == 0) {
            PRINTF("\r\n Firmware Download Complete,Jump App. \r\n");
            PRINTF("------------------------------------------\r\n");
            LPUART_Deinit(LPUART1);
            vSceneRenew();
            vControlSwitch();
        }
    }
}
