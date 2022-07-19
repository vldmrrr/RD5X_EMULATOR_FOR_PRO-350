/*******************************************************************************
 System Interrupts File

  Company:
    Microchip Technology Inc.

  File Name:
    interrupt.c

  Summary:
    Interrupt vectors mapping

  Description:
    This file maps all the interrupt vectors to their corresponding
    implementations. If a particular module interrupt is used, then its ISR
    definition can be found in corresponding PLIB source file. If a module
    interrupt is not used, then its ISR implementation is mapped to dummy
    handler.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2018 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "configuration.h"
#include "interrupts.h"
#include "definitions.h"

#pragma GCC optimize ("O0")
// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************


void CORE_TIMER_InterruptHandler( void );
void UART_1_InterruptHandler( void );
void CHANGE_NOTICE_InterruptHandler( void );
void DRV_USBFS_USB_Handler( void );



/* All the handlers are defined here.  Each will call its PLIB-specific function. */
void __ISR(_CORE_TIMER_VECTOR, ipl1SOFT) CORE_TIMER_Handler (void)
{
    CORE_TIMER_InterruptHandler();
}

void __ISR(_UART_1_VECTOR, ipl1SOFT) UART_1_Handler (void)
{
    UART_1_InterruptHandler();
}

void __ISR(_CHANGE_NOTICE_VECTOR, ipl7SRS) CHANGE_NOTICE_Handler (void)
//void __attribute__((vector(_CHANGE_NOTICE_VECTOR))) __attribute__((naked)) CHANGE_NOTICE_Handler (void)
{
	uint16_t portg = PORTG;
    CTI_STATE cti_state = CTI_STATE_IDLE;

    if (!CTI_SS) {
        uint16_t addr = portg>>12;
        if (addr > REG_ADDR_RANGE)
            goto exit;
        if (addr != REG_STATUS_ADDR && (*reg_R_STAT & STATUS_BUSY_MASK))
            goto exit;
        cti_state = (CTI_WR)?CTI_STATE_WR:CTI_STATE_RD;
        if (cti_state == CTI_STATE_RD) {
            PORTD = ~r_regs[addr];
#ifdef TRACE_CTI
            if (log_pipe_len < (LOG_PIPE_LEN-1))
                log_pipe[(log_pipe_head+log_pipe_len++)%LOG_PIPE_LEN] = (addr<<16) | r_regs[addr];
            else
                log_pipe[(log_pipe_head+log_pipe_len)%LOG_PIPE_LEN] = 0xffffffff;
#endif
            if (addr == REG_DATA && DRQ_PENDING) {
                DRQ_CLEAR();
                if (sector_buffer_idx==255) {
                    OPEND_SET();
                    LED0G_Set();
                } else {
                    *reg_R_DATA = sector_buffer[++sector_buffer_idx];
                    DRQ_SET();
                }
            } else
            if (OPEND_PENDING && addr == REG_STA2_ADDR)
                OPEND_CLEAR();
            else
            if (addr == REG_STATUS_ADDR) {
                if ((*reg_W_CMD&0xff) == CMD_RESTORE) {
                    OPEND_SET();
                    // DRQ_CLEAR();
                    *reg_W_CMD &= 0xff00;
                }
            }
            cti_state = CTI_STATE_RD_DONE;
            do {
                portg = PORTG;
            } while (CTI_DS);
            TRISDCLR = 0xffff;
        } else {  // cti_state == CTI_STATE_WR
            do {
                portg = PORTG;
            } while (CTI_DS);
            uint16_t data =  ~PORTD;
            if (addr!=0) {
                uint16_t wmsk = regs_wmsk[addr];
                r_regs[addr] = w_regs[addr] = ((r_regs[addr]) & ~(wmsk)) | (data & (wmsk));
            }
#ifdef TRACE_CTI
            if (log_pipe_len < (LOG_PIPE_LEN-1))
                log_pipe[(log_pipe_head+log_pipe_len++)%LOG_PIPE_LEN] = 0x80000000 | (addr<<16) | data;
            else
                log_pipe[(log_pipe_head+log_pipe_len)%LOG_PIPE_LEN] = 0xffffffff;
#endif
            if (DRQ_PENDING && addr == REG_STATUS_ADDR)
                DRQ_CLEAR();
            else
            if (addr == REG_STA2_ADDR) {
                ERR_CLEAR();
                DRQ_CLEAR();
                OPEND_CLEAR();
                if ((data&0xff) == CMD_RESTORE) {
                    BUSY_SET();
                }
            } else
            if (addr == REG_DATA && DRQ_PENDING) {
                sector_buffer[sector_buffer_idx] = *reg_W_DATA;
                DRQ_CLEAR();
                if (sector_buffer_idx==255) {
                    BUSY_SET();
                } else {
                    ++sector_buffer_idx;
                    DRQ_SET();
                }
            }
            cti_state = CTI_STATE_WR_DONE;
        }
        RPLY_Clear();
        int i=0;
        do {
            i++;
            portg = PORTG;            
        } while (!CTI_DS && i<1);
        if (cti_state == CTI_STATE_RD_DONE) {
            TRISDSET = 0xffff;
            RPLY_Set();
            cti_state = CTI_STATE_IDLE;
        } else 
        if (cti_state == CTI_STATE_WR_DONE) {
            RPLY_Set();
            cti_state = CTI_STATE_IDLE;
            if (RESET_PENDING) {
                appData.state = APP_STATE_RESET;
                *reg_W_INIT &= ~STATUS_RESET_MASK;
            }
        }
    } else
    if (!CTI_INIT) {
        appData.state = APP_STATE_RESET;
        TRISDSET = 0xffff;
    }

exit:
    IFS1CLR = _IFS1_CNIF_MASK;
    //CHANGE_NOTICE_InterruptHandler();
}

void __ISR(_USB_1_VECTOR, ipl4SOFT) USB_1_Handler (void)
{
    DRV_USBFS_USB_Handler();
}





/*******************************************************************************
 End of File
*/
