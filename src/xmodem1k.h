/****************************************Copyright (c)****************************************************
**                            Guangzhou ZLGMCU Development Co., LTD
**
**                                 http://www.zlgmcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:           XMODEM1K.h
** Last modified Date:  2012-9-10
** Last Version:        V1.0
** Descriptions:        The XMODEM1K Communication Protocol
**
**--------------------------------------------------------------------------------------------------------
** Created by:          CaoHua
** Created date:        2012-9-10
** Version:             V1.00
** Descriptions:        整理应用程序
**
**--------------------------------------------------------------------------------------------------------
** Modified by:         
** Modified date:       
** Version:             
** Descriptions:        
**
**--------------------------------------------------------------------------------------------------------
** Modified by:        
** Modified date:      
** Version:            
** Descriptions:       
**
** Rechecked by:
*********************************************************************************************************/
#ifndef __XMODEM1K_H
#define __XMODEM1K_H

#include "board.h"
#include "fsl_debug_console.h"

#ifdef  __cplusplus
    extern "C" {
#endif

typedef struct xmodem_fun_handle{
    uint8_t   (*pfn_uart_recv_byte) (uint8_t *data);
    uint8_t   (*pfn_uart_send_byte) (uint8_t *data);
    void      (*pfn_time_set) (uint32_t val);
    uint32_t  (*pfn_time_get) (void);
    uint16_t  (*pfn_crc_verify) (uint8_t *puData, uint8_t len, uint16_t seed_val);
}xmodem_fun_handle_t;


typedef uint8_t (*pfn_data_handle_t)(uint8_t *ptr, uint32_t len); 

/*********************************************************************************************************
 头文件
*********************************************************************************************************/
#include <stdint.h>

typedef uint8_t  (*pFunPKTHAND)(uint8_t  *pu8Data, uint16_t  u16Len);
/*********************************************************************************************************
** Function name:       u8Xmodem1kClient
**
** Descriptions:        Xmodem1k协议传输客户端程序，该客户端程序用于Xmodem1k协议接收方使用，使用时需要提供
**                      接收数据包处理函数、CRC校验函数、通信设备收字节函数、通信设备发字节函数以及通信设备
**                      清空接收缓冲区函数。
**                      传输开始时，该客户端会每隔 10*u16ShortDly (ms)向Xmodem1k协议发送方发送C字符，通知
**                      发送方已经准备好接收，如果 u8LongDly (s)后发送方仍然没有开始发送，则退出传输客户端
**                      程序且返回成功(0)。
**
** input parameters:    ptFun：         Xmodem1k协议传输所需函数结构体指针
**                      u16ShortDly：   轮询发送C字符的时间间隔
**                      u8LongDly:      等待传输开始超时时限
** output parameters:   无
** Returned value:      传输结果，0：成功，1：失败
*********************************************************************************************************/
extern uint8_t xmodem1k_client (pfn_data_handle_t    pfn_data_handle,
                                xmodem_fun_handle_t *p_fun, 
                                uint32_t             uShortDly, 
                                uint32_t             uLongDly);

#ifdef  __cplusplus
    }
#endif

#endif                                                                  /* end __XMODEM1K_H             */
/*********************************************************************************************************
  End Of File
*********************************************************************************************************/
