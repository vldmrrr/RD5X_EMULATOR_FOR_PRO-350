/*******************************************************************************
 MPLAB Harmony Application Source File
 
 Company:
 Microchip Technology Inc.
 
 File Name:
 app.c
 
 Summary:
 This file contains the source code for the MPLAB Harmony application.
 
 Description:
 This file contains the source code for the MPLAB Harmony application.  It
 implements the logic of the application's state machine and it may call
 API routines of other MPLAB Harmony modules in the system, such as drivers,
 system services, and middleware.  However, it does not call any of the
 system interfaces (such as the "Initialize" and "Tasks" functions) of any of
 the modules in the system or make any assumptions about when those functions
 are called.  That is the responsibility of the configuration-specific system
 files.
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data
 
 Summary:
 Holds application data
 
 Description:
 This structure holds the application's data.
 
 Remarks:
 This structure should be initialized by the APP_Initialize function.
 
 Application strings and buffers are be defined outside this structure.
 */

APP_DATA appData = {
    .state = APP_STATE_RESET,
    .fileHandle = 0,
};

uint16_t r_regs[9] = {0x0101,0,0,0,0,0,0,0,0};
uint16_t regs_wmsk[9] = {0,0,0000377,0177437,0xffff,0001777,0000007,0000377,0};
uint16_t* const reg_R_ID = &r_regs[0];
uint16_t* const reg_R_ERR = &r_regs[2];
uint16_t* const reg_R_REV_SEC_ID = &r_regs[3];
uint16_t* const reg_R_DATA = &r_regs[4];
uint16_t* const reg_R_CYL_ID = &r_regs[5];
uint16_t* const reg_R_HEAD_ID = &r_regs[6];
uint16_t* const reg_R_STA2 = &r_regs[7];
uint16_t* const reg_R_STAT = &r_regs[8];
uint16_t w_regs[9] = {0,0,0,0,0,0,0,0,0};
uint16_t* const reg_W_PCOMP = &w_regs[2];
uint16_t* const reg_W_REV_SEC_ID = &w_regs[3];
uint16_t* const reg_W_DATA = &w_regs[4];
uint16_t* const reg_W_CYL_ID = &w_regs[5];
uint16_t* const reg_W_HEAD_ID = &w_regs[6];
uint16_t* const reg_W_CMD = &w_regs[7];
uint16_t* const reg_W_INIT = &w_regs[8];


uint16_t portg;
uint16_t prev_portg;
uint16_t portf;

uint16_t sector_buffer[256];
uint8_t sector_buffer_idx;

uint32_t log_pipe[LOG_PIPE_LEN];
uint8_t volatile log_pipe_head;
uint8_t volatile log_pipe_len;

bool printTrace = false;

USB_HOST_EVENT_RESPONSE APP_USBHostEventHandler (USB_HOST_EVENT event, void * eventData, uintptr_t context)
{
    switch (event)
    {
    case USB_HOST_EVENT_DEVICE_UNSUPPORTED:
        break;
    default:
        break;
        
    }
    
    return(USB_HOST_EVENT_RESPONSE_NONE);
}

void APP_SYSFSEventHandler(SYS_FS_EVENT event, void *eventData, uintptr_t context)
{
    switch (event)
    {
    case SYS_FS_EVENT_MOUNT:
        appData.deviceIsConnected = true;
        appData.imageTested = false;
        LED1W_Clear();
        break;
        
    case SYS_FS_EVENT_UNMOUNT:
//__builtin_disable_interrupts();
//SYSKEY = 0x00000000; //write invalid key to force lock
//SYSKEY = 0xAA996655; //write key1 to SYSKEY
//SYSKEY = 0x556699AA; //write key2 to SYSKEY
///* set SWRST bit to arm reset */
//RSWRSTSET = 1;
///* read RSWRST register to trigger reset */
//unsigned int dummy;
//dummy = RSWRST;
///* prevent any unwanted code execution until reset occurs*/
//for(;;){dummy++;}

        appData.deviceIsConnected = false;
        if (appData.fileHandle) {
            FATFS_close(appData.fileHandle);
            appData.fileHandle = 0;
        }
        DR_READY_CLEAR();
        LED1W_Set();
        LED3Y_Set();
        break;
        
    default:
        break;
    }
}

/* Known hard drive geometries */

#define NUMGEOM	12	/* number of defined geometries */

const int	pro_rd_geom[NUMGEOM][3] = {{4,  615, 16},	/* RD31 21M */
	                                   {4,  615, 17},
	                                   {6,  820, 16},	/* RD32 43M */
	                                   {6,  820, 17},
	                                   {4,  153, 16},	/* RD50 5M */
	                                   {4,  153, 17},
	                                   {4,  306, 16},	/* RD51 10M */
	                                   {4,  306, 17},
	                                   {8,  512, 16},	/* RD52 36M */
	                                   {8,  512, 17},
	                                   {8, 1024, 16},	/* RD53 71M */
	                                   {8, 1024, 17}};

char fname[512];

void checkDisk() {
    if (!appData.deviceIsConnected || appData.imageTested) 
        return;
    
    appData.imageTested=true;
    
    SYS_FS_HANDLE cfgf = SYS_FS_FileOpen(IMAGE_CFG, (SYS_FS_FILE_OPEN_READ));
    if(appData.fileHandle == SYS_FS_HANDLE_INVALID)
        return;
    size_t sz = SYS_FS_FileRead( cfgf, (void *) fname, sizeof(fname) );
    SYS_FS_FileClose(cfgf);
    if (sz == -1 || sz>=sizeof(fname))
        return;
    fname[sz]=0;
    SYS_FS_FSTAT stat;
    if (SYS_FS_FileStat(fname, &stat) != SYS_FS_RES_SUCCESS)
        return;
    appData.heads = 0;
    for (int i=0; i<NUMGEOM; i++)
        if (stat.fsize == 512*pro_rd_geom[i][0]*pro_rd_geom[i][1]*pro_rd_geom[i][2]) {
            appData.heads = pro_rd_geom[i][0];
            appData.cylinders = pro_rd_geom[i][1];
            appData.sectors = pro_rd_geom[i][2];
            break;
        }
    if (!appData.heads)
        return;
    int res = FATFS_open((uintptr_t)&appData.fileHandle, fname, (SYS_FS_FILE_OPEN_APPEND_PLUS));
    if(res) {
        appData.fileHandle = 0;
        return;
    }
    SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\n\r Mounted image %d heads/ %d cylinders/%d sectors",
            appData.heads,appData.cylinders,appData.sectors);
    LED3Y_Clear();
    DR_READY_SET();
}


#ifdef TRACE_CTI
void dumpTrace() {
    while (log_pipe_len) {
        uint32_t log = log_pipe[log_pipe_head];
        log_pipe_head = (log_pipe_head+1)%LOG_PIPE_LEN;
        log_pipe_len--;
        char op = (log&0x80000000)?'W':'R';
        uint16_t data = log&0xffff;
        uint8_t addr = (log>>16)&0xf;
        if (printTrace)
            SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\n\r%c %d:%04x",op,addr,data);
        int rpt = 0;
        while (log_pipe_len) {
            if (log == log_pipe[log_pipe_head]) {
                rpt++;
                log_pipe_head = (log_pipe_head+1)%LOG_PIPE_LEN;
                log_pipe_len--;
            } else
                break;
        }
        if (printTrace && rpt)
            SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, " x%d",rpt+1);
    }
}
#endif


int LBA(int c, int h, int s) {
    return s + h*appData.sectors + c*appData.sectors*appData.heads;
}

void StartRead() {
    //BUSY_SET();
    ERR_CLEAR();
    
    if (!appData.fileHandle) {
        SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\n\rErr: read no disk");
        ERR_SET(ERR_DM);
        OPEND_SET();
        BUSY_CLEAR();
        return;
    }
    LED0G_Clear();
    uint32_t offset = 512 * LBA(*reg_W_CYL_ID&0x03ff,*reg_W_HEAD_ID & 7, *reg_W_REV_SEC_ID & 0xf);
    int res = FATFS_lseek(appData.fileHandle, offset);
    if (res) {
        SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\n\rErr: seek to offs %x",offset);
        ERR_SET(ERR_ID);
        OPEND_SET();
        LED0G_Set();
        BUSY_CLEAR();
        return;
    }
    if (*reg_W_CMD&0xff) {
        LED0G_Set();
        return;
    }
    uint32_t br;
    res = FATFS_read(appData.fileHandle,(void*)sector_buffer,512,&br);
    if (res || br<512) {
        SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\n\rErr: read at offs %x",offset);
        ERR_SET(ERR_DM);
        OPEND_SET();
        LED0G_Set();
        BUSY_CLEAR();
        return;
    }
    if (*reg_W_CMD&0xff) {
        LED0G_Set();
        return;
    }
    sector_buffer_idx = 0;
    *reg_R_DATA = sector_buffer[0];
//    if (offset == 0x94c00) {
//        printTrace=true;
//        log_pipe_head=log_pipe_len=0;
//        dumpTrace();
//    }
    BUSY_CLEAR();
    DRQ_SET();
    SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\n\rStart read at 0x%x",offset);
}

void FinishWrite() {
    LED2R_Set();
    if ((*reg_R_STA2 & 0xff) == CMD_FORMAT) {
        appData.state = APP_STATE_WORK;
        OPEND_SET();
        return;
    }
    if (!appData.fileHandle) {
        ERR_SET(ERR_DM);
        OPEND_SET();
        return;
    }
    int offset = 512 * LBA(*reg_W_CYL_ID&0x03ff,*reg_W_HEAD_ID & 7, *reg_W_REV_SEC_ID & 0xf);
    int res = FATFS_lseek(appData.fileHandle, offset);
    if (res) {
        ERR_SET(ERR_ID);
        OPEND_SET();
        return;
    }
    uint32_t bw;
    res = FATFS_write(appData.fileHandle,(void*)sector_buffer,512,&bw);
    if (res || bw<512) {
        ERR_SET(ERR_DM);
        OPEND_SET();
        return;
    }
    appData.state = APP_STATE_WORK;
    OPEND_SET();
}

void APP_Reset(bool warm) {
    printTrace = false;
    if (warm)
        SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\n\rWarm reset");
    GPIO_PinInterruptDisable(CN9_PIN|CN10_PIN);
    SYS_INT_SourceDisable(INT_SOURCE_CHANGE_NOTICE);
    log_pipe_head=log_pipe_len=0;
    LED0G_Set();LED2R_Set();
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    TRISDSET = 0xffff;
    portg = prev_portg = PORTG;
    *reg_R_ID = 0x0101;
    *reg_R_REV_SEC_ID = *reg_R_CYL_ID = *reg_R_HEAD_ID = 0;
    *reg_W_REV_SEC_ID = *reg_W_CYL_ID = *reg_W_HEAD_ID = 0;
    *reg_W_PCOMP = 128;
    *reg_R_ERR = *reg_W_INIT = 0;
    *reg_R_STAT = STATUS_OPEND_MASK;
    RPLY_Set();
    IRQA_Set();
    IRQB_Set();
    if (!warm) {
        *reg_R_STA2 = 0; 
        LED1W_Set();
        LED3Y_Set();
        DR_READY_CLEAR();
        /* Set the event handler and enable the bus */
        SYS_FS_EventHandlerSet((void *)APP_SYSFSEventHandler, (uintptr_t)NULL);
        USB_HOST_EventHandlerSet(APP_USBHostEventHandler, 0);
        USB_HOST_BusEnable(USB_HOST_BUS_ALL);
    } else {
        *reg_R_STA2 &= 0xf000;
    }
//    if (appData.fileHandle) {
//        SYS_FS_FileClose(appData.fileHandle);
//        appData.fileHandle = 0;
//    }
    GPIO_PinInterruptEnable(CN9_PIN|CN10_PIN);
    SYS_INT_SourceEnable(INT_SOURCE_CHANGE_NOTICE);    
    if (warm)
        SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\n\rDone");
}

void APP_Initialize ( void )
{
    appData.state = APP_STATE_RESET;
    appData.fileHandle = 0;
    APP_Reset(false);
}


void APP_Tasks ( void )
{
#ifdef TRACE_CTI
    dumpTrace();
#endif
    if (*reg_W_CMD&0xff && appData.state != APP_STATE_WORK) {
        appData.state = APP_STATE_WORK;
    }
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
    case APP_STATE_RESET:
        APP_Reset(true);
    case APP_STATE_INIT:
    {
        bool appInitialized = true;
        
        
        if (appInitialized)
        {
            
            appData.state = APP_STATE_WORK;
        }
        break;
    }
    case APP_STATE_WORK:
        if (!appData.fileHandle)
            checkDisk();
        uint16_t cmd = *reg_W_CMD&0xff;
        if (cmd) {
            *reg_W_CMD &= 0xff00;
            *reg_R_ERR &= 0xff;
            *reg_R_STA2 &= ~STA2_ERR;
            switch (cmd) {
            case CMD_RESTORE:
                appData.state = APP_STATE_WORK;
                if (!appData.fileHandle) {
                    ERR_SET(ERR_TR0);
                    SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\n\rRestore no disk");
                }
                OPEND_SET();
                BUSY_CLEAR();
                break;
            case CMD_READ:
                StartRead();
                break;
            case CMD_WRITE:
            case CMD_FORMAT:
                ERR_CLEAR();
                sector_buffer_idx=0;
                LED2R_Clear();
                DRQ_SET();
                break;
            default:
                ERR_SET(ERR_ILL_CMD);
                break;
            }
        }
        if (*reg_R_STAT & STATUS_BUSY_MASK) {
            uint16_t ccmd = *reg_R_STA2&0xff;
            if ((ccmd == CMD_WRITE || ccmd == CMD_FORMAT) && sector_buffer_idx>=255)
                FinishWrite();
        }
        break;        
        /* The default state should never be executed. */
    default:
    {
        SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\n\rInvalid app state %x",appData.state);
        /* TODO: Handle error in application's state machine. */
        break;
    }
    }
}


/*******************************************************************************
 End of File
 */
