/*
 * Copyright 2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
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

#ifndef _FLEXSPI_NOR_H_
#define _FLEXSPI_NOR_H_
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"

#include "fsl_flexspi.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief The board name */


#define NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD              0
#define NOR_CMD_LUT_SEQ_IDX_READSTATUSREG               1
#define NOR_CMD_LUT_SEQ_IDX_WRITESTATUSREG              2
#define NOR_CMD_LUT_SEQ_IDX_WRITEENABLE                 3
#define NOR_CMD_LUT_SEQ_IDX_READID                      4
#define NOR_CMD_LUT_SEQ_IDX_ERASESECTOR                 5
#define NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE          9

#define FLASH_PAGE_SIZE     256
#define FLASH_SECTOR_SIZE   0x1000 /* 4K */
#define EXAMPLE_FLEXSPI_AMBA_BASE FlexSPI_AMBA_BASE

#define FLASH_BUSY_STATUS_POL       1
#define FLASH_BUSY_STATUS_OFFSET    0

#define EXAMPLE_SECTOR 10

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
 * API
 ******************************************************************************/
extern status_t flexspi_nor_write_enable(FLEXSPI_Type *base, uint32_t baseAddr);

extern status_t flexspi_nor_wait_bus_busy(FLEXSPI_Type *base);

extern status_t flexspi_nor_enable_quad_mode(FLEXSPI_Type *base);

extern status_t flexspi_nor_flash_erase_sector(FLEXSPI_Type *base, uint32_t address);

extern status_t flexspi_nor_flash_page_program(FLEXSPI_Type *base, uint32_t dstAddr, const uint32_t *src, uint32_t len);

extern status_t flexspi_nor_get_vendor_id(FLEXSPI_Type *base, uint8_t *vendorId);

extern void flexspi_nor_init (void);


#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _FLEXSPI_NOR_H_ */
