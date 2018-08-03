/*********************************************Copyright (c)***********************************************
**                                Guangzhou ZLG MCU Technology Co., Ltd.
**
**                                        http://www.zlgmcu.com
**
**      广州周立功单片机科技有限公司所提供的所有服务内容旨在协助客户加速产品的研发进度，在服务过程中所提供
**  的任何程序、文档、测试结果、方案、支持等资料和信息，都仅供参考，客户有权不使用或自行参考修改，本公司不
**  提供任何的完整性、可靠性等保证，若在客户使用过程中因任何原因造成的特别的、偶然的或间接的损失，本公司不
**  承担任何责任。
**                                                                          ——广州周立功单片机科技有限公司
**
**--------------File Info---------------------------------------------------------------------------------
** File name:           XMODEM1K.c
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
/*********************************************************************************************************
 头文件
*********************************************************************************************************/
#include "xmodem1k.h"

/*********************************************************************************************************
 宏定义
*********************************************************************************************************/
#define SOH                        0x01      /**< \brief Xmodem协议起始      */
#define STX                        0x02      /**< \brief Xmodem-1k协议起始   */
#define EOT                        0x04      /**< \brief 结束                */
#define ACK                        0x06      /**< \brief 正常响应            */
#define NAK                        0x15      /**< \brief 非正常响应          */
#define CAN                        0x18      /**< \brief 严重错误，结束      */
#define POLL                       0x43      /**< \brief 轮询字符            */

#define STAT_IDLE_DATA              0        /**< \brief 轮询读数            */
#define STAT_IDLE_C                 1        /**< \brief 轮询发C             */
#define STAT_CONNECT                2        /**< \brief 数据连接            */
#define STAT_RECEIVE                3        /**< \brief 数据接收            */
#define STAT_HANDLE                 4        /**< \brief 数据处理            */
#define STAT_ACK                    5        /**< \brief 正常响应状态        */
#define STAT_NAK                    6        /**< \brief 非正常响应状态      */
#define STAT_CAN                    7        /**< \brief 强制结束状态        */
#define STAT_END                    8        /**< \brief 数据传输结束状态    */

#define LONGPKT_LEN                 1024     /**< \brief 1k协议，数据段长度  */
#define SHORTPKT_LEN                128      /**< \brief Xmodem，数据段长度  */
#define PKT_HEAD_LEN                3        /**< \brief 数据包头信息长度    */

#define PKT_TIMEOUT_MS              500      /**< \brief 包间隔超时时限      */
#define CHAR_TIMEOUT_MS             100      /**< \brief 包内字符间超时时限  */

/*********************************************************************************************************
 结构体变量定义
*********************************************************************************************************/
/*  使用无名结构体 */
#if defined ( __CC_ARM )
#pragma anon_unions
#endif

typedef union {
    struct {
        uint8_t u8ShortBuff[SHORTPKT_LEN];  /**< \brief 标准包数据存放buffer */
        uint8_t u8ShortCRC[2];              /**< \brief 标准包CRC值.大端存储 */
    };
    struct {
        uint8_t u8LongBuff[LONGPKT_LEN];    /**< \brief 扩展包数据存放buffer */
        uint8_t u8LongCRC[2];               /**< \brief 扩展包CRC值          */
    };
} __uXMODEM_DATA, *__puXMODEM_DATA;

typedef struct {
    uint8_t u8Ctrl;                         /**< \brief 开始控制字符         */
    uint8_t u8Index;                        /**< \brief 包序号               */
    uint8_t u8Patch;                        /**< \brief 包序号补码           */
} __tXMODEM_HEAD, *__ptXMODEM_HEAD;

/* 结束使用无名结构体 */
#if defined ( __CC_ARM )
#pragma no_anon_unions
#endif

__uXMODEM_DATA  uData;                      /* 数据结构变量                  */
__puXMODEM_DATA puData = &uData;
__tXMODEM_HEAD  tHead;                      /* 包头结构变量                  */
__ptXMODEM_HEAD ptHead = &tHead;

/*********************************************************************************************************
  全局变量定义
*********************************************************************************************************/

uint16_t __crc16_verify(uint8_t *p_data, uint16_t len, uint16_t seed)
{
    uint16_t temp    = 0;
    uint16_t crc_val = 0;
    int      i       = 0;
    int      j       = 0;

    for (i = 0; i < len ; i++) {
        for (j = 0; j < 8 ; j++) {
            temp = ((p_data[i] << j) & 0x80) ^ ((crc_val & 0x8000) >> 8);
            crc_val <<= 1;

            if (temp != 0) {
                crc_val ^= 0x1021;
            }
        }
    }
    return crc_val;
}


/*********************************************************************************************************
** Function name:       u8Xmodem1kClient
** Descriptions:        Xmodem1k协议传输客户端程序
** input parameters:    ptFun：         Xmodem1k协议传输所需函数结构体指针
**                      u16ShortDly：   轮询发送C字符的时间间隔
**                      u8LongDly:      等待传输开始超时时限
** output parameters:   无
** Returned value:      传输结果，0：成功，1：失败
*********************************************************************************************************/
uint8_t xmodem1k_client (pfn_data_handle_t    pfn_data_handle,
                         xmodem_fun_handle_t *p_fun, 
                         uint32_t             uShortDly, 
                         uint32_t             uLongDly)
{
    uint32_t u32ByteCnt   = 0;                                          /* 位计数器                     */
    uint8_t  u8TimeoutCnt = 0;                                          /* 超时次数                     */
    uint8_t  u8DataerrCnt = 0;                                          /* 数据错误次数                 */
    uint8_t  u8PktIndex   = 1;                                          /* 包序号期望值                 */

    uint8_t  u8STATE = STAT_IDLE_C;                                     /* 状态变量                     */
    uint8_t  u8Data;                                                    /* 存放接收数据及发送命令       */
    uint16_t u16PktLen;                                                 /* 包中有效数据的长度           */
    uint8_t  u8Message;
    
    uint32_t temp = 0;
    
    p_fun->pfn_time_set(0);
    PRINTF("mcu ready to recv data...\r\n");
    while (1) {
        
        switch (u8STATE) {

        case STAT_IDLE_C:                                               /* 轮询发C状态                  */
            if (p_fun->pfn_time_get() >= uLongDly) {
                u8STATE = STAT_END;                                     /* 等待开始超时，跳到结束状态   */
            } else {
                u8Data = POLL;
                do {
                    u8Message = p_fun->pfn_uart_send_byte(&u8Data);
                } while (u8Message != 1);
                p_fun->pfn_time_set(0);
                u8STATE = STAT_IDLE_DATA;                               /* 跳到轮询读数状态             */
            }
            break;

        case STAT_IDLE_DATA:                                            /* 轮询读数状态                 */
            if (p_fun->pfn_uart_recv_byte(&u8Data) == 1) {
                u8STATE = STAT_CONNECT;                                 /* 接收到数据，跳到数据连接状态 */
                p_fun->pfn_time_set(0);
            } else {
                if (p_fun->pfn_time_get() >= uShortDly) {
                    u8STATE = STAT_IDLE_C;                              /* 轮询读数超时，跳回轮询发C    */
                }
            }
            break;

        case STAT_CONNECT:                                              /* 数据连接状态                 */
            
            if ((u8Data == SOH) || (u8Data == STX)) {
                u16PktLen = (u8Data == SOH)? SHORTPKT_LEN : LONGPKT_LEN;
                ((uint8_t *)ptHead)[u32ByteCnt] = u8Data;
                u32ByteCnt++;
                u8STATE = STAT_RECEIVE;                                 /* 连接成功，跳到数据接收状态   */
            } else {
                u8STATE = STAT_IDLE_C;                                  /* 起始控制字符错，跳回轮询发C  */
            }
            break;

        case STAT_RECEIVE:                                              /* 数据接收状态                 */
            if (p_fun->pfn_uart_recv_byte(&u8Data) == 1) {              /* 收到数据                     */
                if (u32ByteCnt < PKT_HEAD_LEN) {
                    ((uint8_t *)ptHead)[u32ByteCnt] = u8Data;           /* 控制字符、序号、序号补码     */
                    if (ptHead->u8Ctrl == EOT) {
                        u8STATE = STAT_ACK;
                        break;
                    }
                    /**
                     * Modified by Damon: Xmodem1K lost pack will be Xmodem pack.
                     * */
                    if ((ptHead->u8Ctrl == SOH) || (ptHead->u8Ctrl == STX)) {
                        u16PktLen = (ptHead->u8Ctrl == SOH)? SHORTPKT_LEN : LONGPKT_LEN;
                    }
                    /* End */
                } else {
                    ((uint8_t *)puData)[u32ByteCnt - 3] = u8Data;       /* 数据段部分（数据、CRC值）    */
                }
                u32ByteCnt++;
                if (u32ByteCnt >= u16PktLen + PKT_HEAD_LEN + 2) {
                    u8STATE = STAT_HANDLE;                              /* 包接收结束，跳到数据处理状态 */
                }
                u8TimeoutCnt = 0;
                p_fun->pfn_time_set(0);
            } else {                                                    /* 未收到数据，判断超时         */
                /*
                 * 包间隔最大为10s，字符间隔最大为1s，
                 * 根据包内部和包之间的不同选择不同的超时间隔
                 */
                if (p_fun->pfn_time_get() >= ((u32ByteCnt == 0) ? PKT_TIMEOUT_MS : CHAR_TIMEOUT_MS)) {
                    p_fun->pfn_time_set(0);
                    u8TimeoutCnt++;
                    u8STATE = STAT_NAK;
                }
            }
            break;    

        case STAT_HANDLE:                                               /* 数据处理状态                 */
        {
            uint16_t u16CRCTemp;
            if (ptHead->u8Ctrl != ((u16PktLen == SHORTPKT_LEN) ? SOH : STX)) {
                                                                        /* 检查控制字符是否一致         */
                u8DataerrCnt++;
                u8STATE = STAT_NAK;
                break;
            }
            if (ptHead->u8Index + ptHead->u8Patch != 0xFF) {            /* 检查序号、序号补码是否完整   */
                u8DataerrCnt++;
                u8STATE = STAT_NAK;
                break;
            }
            if ((ptHead->u8Index) == (u8PktIndex - 1)) {                /* 检查序号是否为上一包序号     */
                u8STATE = STAT_ACK;
                break;
            }
            if (ptHead->u8Index != u8PktIndex) {                        /* 检查序号是否为期望的包序号   */
                u8DataerrCnt++;
                u8STATE = STAT_NAK;
                break;
            }
            u16CRCTemp = ((uint16_t)(*((uint8_t *)puData + u16PktLen)) << 8) |
                                    *((uint8_t *)puData + u16PktLen + 1);

            if ((p_fun->pfn_crc_verify) == NULL) {
                temp = __crc16_verify((uint8_t *)puData, u16PktLen, 0);
                if ((__crc16_verify((uint8_t *)puData, u16PktLen, 0)) != u16CRCTemp) {
                    u8DataerrCnt++;
                    u8STATE = STAT_NAK;                                     /* CRC检查                      */
                    break;
                }
            } else {
                if ((p_fun->pfn_crc_verify((uint8_t *)puData, u16PktLen, 0)) != u16CRCTemp) {
                    u8DataerrCnt++;
                    u8STATE = STAT_NAK;                                     /* CRC检查                      */
                    break;
                }
            }

            if (pfn_data_handle(puData->u8LongBuff,u16PktLen)) {       
                u8PktIndex++;
                u8STATE = STAT_ACK;                                     /* 数据处理                     */
                break;
            }
            u8DataerrCnt++;
            u8STATE = STAT_NAK;
            break;
        }

        case STAT_ACK:                                                  /* 正常响应状态（ACK）          */
            p_fun->pfn_time_set(0);
            u8Data = ACK;
            do {
                    u8Message = p_fun->pfn_uart_send_byte(&u8Data);
                } while (u8Message != 1);
            if (ptHead->u8Ctrl == EOT) {                                /* 结束控制符时进入ACK状态情况  */
                u8STATE = STAT_END;                                     /* 发送后跳到结束状态           */
                break;
            }
            u8DataerrCnt = 0;
            u32ByteCnt = 0;
            u8STATE = STAT_RECEIVE;                                     /* 正常响应发送ACK后跳到数据接收*/
            break;
            
        case STAT_NAK:                                                  /* 非正常响应状态（NAK）        */
            if ((u8DataerrCnt >= 10) || (u8TimeoutCnt >= 10)) {
                u8STATE = STAT_CAN;
                break;
            }
            p_fun->pfn_time_set(0);
            u8Data = NAK;
            do {
                    u8Message = p_fun->pfn_uart_send_byte(&u8Data);
                } while (u8Message != 1);
            u32ByteCnt = 0;
            u8STATE = STAT_RECEIVE;
            break;
        
        case STAT_CAN:                                                  /* 强制结束状态（CAN）          */
            p_fun->pfn_time_set(0);
            u8Data = CAN;
            do {
                    u8Message = p_fun->pfn_uart_send_byte(&u8Data);
                } while (u8Message != 1);
            return 1;
            
        case STAT_END:                                                  /* 传输结束状态（CAN）          */
            return 0;

        default:
            break;
        }
    }
}


/*********************************************************************************************************
  End Of File
*********************************************************************************************************/
