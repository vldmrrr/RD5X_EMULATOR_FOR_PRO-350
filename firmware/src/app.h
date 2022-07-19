/*******************************************************************************
 MPLAB Harmony Application Header File
 
 Company:
 Microchip Technology Inc.
 
 File Name:
 app.h
 
 Summary:
 This header file provides prototypes and definitions for the application.
 
 Description:
 This header file provides function prototypes and data type definitions for
 the application.  Some of these are required by the system (such as the
 "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
 internally by the application (such as the "APP_STATES" definition).  Both
 are defined here for convenience.
 *******************************************************************************/

#ifndef _APP_H
#define _APP_H
//#define TRACE_CTI
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "configuration.h"
#include "definitions.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {
    
#endif
    // DOM-IGNORE-END

#define IMAGE_CFG "/rdimage.cfg"
    typedef enum
    {
        APP_STATE_INIT=0,
        APP_STATE_RESET,
        APP_STATE_WORK,
                
    } APP_STATES;
    
    typedef enum {
        CTI_STATE_IDLE = 0,
        CTI_STATE_RD,
        CTI_STATE_WR,
        CTI_STATE_RD_DONE,
        CTI_STATE_WR_DONE,
    } CTI_STATE;
    
    // *****************************************************************************
    /* Application Data
     
     Summary:
     Holds application data
     
     Description:
     This structure holds the application's data.
     
     Remarks:
     Application strings and buffers are be defined outside this structure.
     */
    
    typedef struct
    {
        /* The application's current state */
        APP_STATES state;
        bool deviceIsConnected;
        bool imageTested;
        uintptr_t fileHandle;
        int heads;
        int cylinders;
        int sectors;
    } APP_DATA;
extern APP_DATA appData;


#define START_REG_ADDR 0174000
#define REG_ADDR_RANGE 010
#define REG_ERR_PRECOMP_ADDR 2
#define REG_DATA 4
#define REG_STA2_ADDR 7
#define REG_STATUS_ADDR 8
#define STATUS_OPEND_MASK 1
#define OPEND_PENDING (*reg_R_STAT & STATUS_OPEND_MASK)
#define OPEND_CLEAR() (*reg_R_STAT&= ~STATUS_OPEND_MASK,IRQA_Set())
#define OPEND_SET() (*reg_R_STAT = (*reg_R_STAT&~STATUS_BUSY_MASK)|STATUS_OPEND_MASK,IRQA_Clear())
#define STATUS_RESET_MASK (1<<3)
#define RESET_PENDING (*reg_W_INIT & STATUS_RESET_MASK)
#define STATUS_DRQ_MASK (1<<7)
#define DRQ_PENDING (*reg_R_STAT & STATUS_DRQ_MASK)
#define DRQ_SET() (*reg_R_STAT |= STATUS_DRQ_MASK,*reg_R_STA2 |= STA2_DRQ,IRQB_Clear())
#define DRQ_CLEAR() (*reg_R_STAT &= ~STATUS_DRQ_MASK,*reg_R_STA2 &= ~STA2_DRQ, IRQB_Set())
#define STATUS_BUSY_MASK (1<<15)
#define BUSY_SET() (*reg_R_STAT |= STATUS_BUSY_MASK)
#define BUSY_CLEAR() (*reg_R_STAT &= ~STATUS_BUSY_MASK)
    extern uint16_t r_regs[9];
    extern uint16_t regs_wmsk[9];
    extern uint16_t* const reg_R_ID;
    extern uint16_t* const reg_R_ERR;
    extern uint16_t* const reg_R_REV_SEC_ID;
    extern uint16_t* const reg_R_DATA;
    extern uint16_t* const reg_R_CYL_ID;
    extern uint16_t* const reg_R_HEAD_ID;
    extern uint16_t* const reg_R_STA2;
    extern uint16_t* const reg_R_STAT;
    extern uint16_t w_regs[9];
    extern uint16_t* const reg_W_PCOMP;
    extern uint16_t* const reg_W_REV_SEC_ID;
    extern uint16_t* const reg_W_DATA;
    extern uint16_t* const reg_W_CYL_ID;
    extern uint16_t* const reg_W_HEAD_ID;
    extern uint16_t* const reg_W_CMD;
    extern uint16_t* const reg_W_INIT;
    
    extern uint16_t sector_buffer[256];
    extern uint8_t sector_buffer_idx;
    
#define LOG_PIPE_LEN 256
    extern uint32_t log_pipe[LOG_PIPE_LEN];
    extern volatile uint8_t log_pipe_head;
    extern volatile uint8_t log_pipe_len;

#define ERR_DM (1<<8)
#define ERR_TR0 (1<<9)
#define ERR_ILL_CMD (1<<10)
#define ERR_ID (1<<12)
#define ERR_CRC_ID (1<<13)
#define ERR_CRC_DATA (1<<14)

#define CMD_RESTORE 0x10
#define CMD_READ 0x20
#define CMD_WRITE 0x30
#define CMD_FORMAT 0x50

#define STA2_ERR (1<<8)
#define STA2_DRQ (1<<11)
#define STA2_SEEK_DONE (1<<12)
#define STA2_WR_FLT (1<<13)
#define STA2_READY (1<<14)

#define ERR_SET(e) (*reg_R_STA2 |= STA2_ERR, *reg_R_ERR |= e)
#define ERR_CLEAR() (*reg_R_STA2 &= ~STA2_ERR, *reg_R_ERR &= 0xff)
#define DR_READY_SET() (*reg_R_STA2 |= STA2_READY| STA2_SEEK_DONE)
#define DR_READY_CLEAR() (*reg_R_STA2 &= ~STA2_READY)

    void APP_Initialize ( void );
    
    
    void APP_Tasks( void );
    
    //DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif /* _APP_H */

/*******************************************************************************
 End of File
 */

