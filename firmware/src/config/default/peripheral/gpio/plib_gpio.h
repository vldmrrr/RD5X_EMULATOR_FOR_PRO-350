/*******************************************************************************
  GPIO PLIB

  Company:
    Microchip Technology Inc.

  File Name:
    plib_gpio.h

  Summary:
    GPIO PLIB Header File

  Description:
    This library provides an interface to control and interact with Parallel
    Input/Output controller (GPIO) module.

*******************************************************************************/

/*******************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
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

#ifndef PLIB_GPIO_H
#define PLIB_GPIO_H

#include <device.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Data types and constants
// *****************************************************************************
// *****************************************************************************


/*** Macros for A4 pin ***/
#define A4_Set()               (LATGSET = (1<<15))
#define A4_Clear()             (LATGCLR = (1<<15))
#define A4_Toggle()            (LATGINV= (1<<15))
#define A4_OutputEnable()      (TRISGCLR = (1<<15))
#define A4_InputEnable()       (TRISGSET = (1<<15))
#define A4_Get()               ((PORTG >> 15) & 0x1)
#define A4_PIN                  GPIO_PIN_RG15

/*** Macros for IRQA pin ***/
#define IRQA_Set()               (LATESET = (1<<5))
#define IRQA_Clear()             (LATECLR = (1<<5))
#define IRQA_Toggle()            (LATEINV= (1<<5))
#define IRQA_OutputEnable()      (TRISECLR = (1<<5))
#define IRQA_InputEnable()       (TRISESET = (1<<5))
#define IRQA_Get()               ((PORTE >> 5) & 0x1)
#define IRQA_PIN                  GPIO_PIN_RE5

/*** Macros for INIT pin ***/
#define INIT_Get()               ((PORTG >> 7) & 0x1)
#define INIT_PIN                  GPIO_PIN_RG7
#define INIT_InterruptEnable()   (CNENGSET = (1<<7))
#define INIT_InterruptDisable()  (CNENGCLR = (1<<7))
#define INIT_PIN_MASK (1<<7)
#define CTI_INIT (portg & INIT_PIN_MASK)
#define PREV_CTI_INIT (prev_portg & INIT_PIN_MASK)

/*** Macros for AS pin ***/
#define SS_Get()               ((PORTG >> 8) & 0x1)
#define SS_PIN                  GPIO_PIN_RG8
#define SS_PIN_MASK (1<<8)
#define CTI_SS (portg & SS_PIN_MASK)
#define PREV_CTI_SS (prev_portg & SS_PIN_MASK)

/*** Macros for DS pin ***/
#define DS_Set()               (LATGSET = (1<<9))
#define DS_Clear()             (LATGCLR = (1<<9))
#define DS_Toggle()            (LATGINV= (1<<9))
#define DS_OutputEnable()      (TRISGCLR = (1<<9))
#define DS_InputEnable()       (TRISGSET = (1<<9))
#define DS_Get()               ((PORTG >> 9) & 0x1)
#define DS_PIN                  GPIO_PIN_RG9
#define DS_PIN_MASK (1<<9)
#define CTI_DS (portg & DS_PIN_MASK)
#define PREV_CTI_DS (prev_portg & DS_PIN_MASK)

/*** Macros for GPIO_RD14 pin ***/
#define GPIO_RD14_Set()               (LATDSET = (1<<14))
#define GPIO_RD14_Clear()             (LATDCLR = (1<<14))
#define GPIO_RD14_Toggle()            (LATDINV= (1<<14))
#define GPIO_RD14_OutputEnable()      (TRISDCLR = (1<<14))
#define GPIO_RD14_InputEnable()       (TRISDSET = (1<<14))
#define GPIO_RD14_Get()               ((PORTD >> 14) & 0x1)
#define GPIO_RD14_PIN                  GPIO_PIN_RD14

/*** Macros for GPIO_RD15 pin ***/
#define GPIO_RD15_Set()               (LATDSET = (1<<15))
#define GPIO_RD15_Clear()             (LATDCLR = (1<<15))
#define GPIO_RD15_Toggle()            (LATDINV= (1<<15))
#define GPIO_RD15_OutputEnable()      (TRISDCLR = (1<<15))
#define GPIO_RD15_InputEnable()       (TRISDSET = (1<<15))
#define GPIO_RD15_Get()               ((PORTD >> 15) & 0x1)
#define GPIO_RD15_PIN                  GPIO_PIN_RD15

/*** Macros for GPIO_RD8 pin ***/
#define GPIO_RD8_Set()               (LATDSET = (1<<8))
#define GPIO_RD8_Clear()             (LATDCLR = (1<<8))
#define GPIO_RD8_Toggle()            (LATDINV= (1<<8))
#define GPIO_RD8_OutputEnable()      (TRISDCLR = (1<<8))
#define GPIO_RD8_InputEnable()       (TRISDSET = (1<<8))
#define GPIO_RD8_Get()               ((PORTD >> 8) & 0x1)
#define GPIO_RD8_PIN                  GPIO_PIN_RD8

/*** Macros for GPIO_RD9 pin ***/
#define GPIO_RD9_Set()               (LATDSET = (1<<9))
#define GPIO_RD9_Clear()             (LATDCLR = (1<<9))
#define GPIO_RD9_Toggle()            (LATDINV= (1<<9))
#define GPIO_RD9_OutputEnable()      (TRISDCLR = (1<<9))
#define GPIO_RD9_InputEnable()       (TRISDSET = (1<<9))
#define GPIO_RD9_Get()               ((PORTD >> 9) & 0x1)
#define GPIO_RD9_PIN                  GPIO_PIN_RD9

/*** Macros for GPIO_RD10 pin ***/
#define GPIO_RD10_Set()               (LATDSET = (1<<10))
#define GPIO_RD10_Clear()             (LATDCLR = (1<<10))
#define GPIO_RD10_Toggle()            (LATDINV= (1<<10))
#define GPIO_RD10_OutputEnable()      (TRISDCLR = (1<<10))
#define GPIO_RD10_InputEnable()       (TRISDSET = (1<<10))
#define GPIO_RD10_Get()               ((PORTD >> 10) & 0x1)
#define GPIO_RD10_PIN                  GPIO_PIN_RD10

/*** Macros for GPIO_RD11 pin ***/
#define GPIO_RD11_Set()               (LATDSET = (1<<11))
#define GPIO_RD11_Clear()             (LATDCLR = (1<<11))
#define GPIO_RD11_Toggle()            (LATDINV= (1<<11))
#define GPIO_RD11_OutputEnable()      (TRISDCLR = (1<<11))
#define GPIO_RD11_InputEnable()       (TRISDSET = (1<<11))
#define GPIO_RD11_Get()               ((PORTD >> 11) & 0x1)
#define GPIO_RD11_PIN                  GPIO_PIN_RD11

/*** Macros for GPIO_RD0 pin ***/
#define GPIO_RD0_Set()               (LATDSET = (1<<0))
#define GPIO_RD0_Clear()             (LATDCLR = (1<<0))
#define GPIO_RD0_Toggle()            (LATDINV= (1<<0))
#define GPIO_RD0_OutputEnable()      (TRISDCLR = (1<<0))
#define GPIO_RD0_InputEnable()       (TRISDSET = (1<<0))
#define GPIO_RD0_Get()               ((PORTD >> 0) & 0x1)
#define GPIO_RD0_PIN                  GPIO_PIN_RD0

/*** Macros for GPIO_RD1 pin ***/
#define GPIO_RD1_Set()               (LATDSET = (1<<1))
#define GPIO_RD1_Clear()             (LATDCLR = (1<<1))
#define GPIO_RD1_Toggle()            (LATDINV= (1<<1))
#define GPIO_RD1_OutputEnable()      (TRISDCLR = (1<<1))
#define GPIO_RD1_InputEnable()       (TRISDSET = (1<<1))
#define GPIO_RD1_Get()               ((PORTD >> 1) & 0x1)
#define GPIO_RD1_PIN                  GPIO_PIN_RD1

/*** Macros for GPIO_RD2 pin ***/
#define GPIO_RD2_Set()               (LATDSET = (1<<2))
#define GPIO_RD2_Clear()             (LATDCLR = (1<<2))
#define GPIO_RD2_Toggle()            (LATDINV= (1<<2))
#define GPIO_RD2_OutputEnable()      (TRISDCLR = (1<<2))
#define GPIO_RD2_InputEnable()       (TRISDSET = (1<<2))
#define GPIO_RD2_Get()               ((PORTD >> 2) & 0x1)
#define GPIO_RD2_PIN                  GPIO_PIN_RD2

/*** Macros for GPIO_RD3 pin ***/
#define GPIO_RD3_Set()               (LATDSET = (1<<3))
#define GPIO_RD3_Clear()             (LATDCLR = (1<<3))
#define GPIO_RD3_Toggle()            (LATDINV= (1<<3))
#define GPIO_RD3_OutputEnable()      (TRISDCLR = (1<<3))
#define GPIO_RD3_InputEnable()       (TRISDSET = (1<<3))
#define GPIO_RD3_Get()               ((PORTD >> 3) & 0x1)
#define GPIO_RD3_PIN                  GPIO_PIN_RD3

/*** Macros for GPIO_RD12 pin ***/
#define GPIO_RD12_Set()               (LATDSET = (1<<12))
#define GPIO_RD12_Clear()             (LATDCLR = (1<<12))
#define GPIO_RD12_Toggle()            (LATDINV= (1<<12))
#define GPIO_RD12_OutputEnable()      (TRISDCLR = (1<<12))
#define GPIO_RD12_InputEnable()       (TRISDSET = (1<<12))
#define GPIO_RD12_Get()               ((PORTD >> 12) & 0x1)
#define GPIO_RD12_PIN                  GPIO_PIN_RD12

/*** Macros for GPIO_RD13 pin ***/
#define GPIO_RD13_Set()               (LATDSET = (1<<13))
#define GPIO_RD13_Clear()             (LATDCLR = (1<<13))
#define GPIO_RD13_Toggle()            (LATDINV= (1<<13))
#define GPIO_RD13_OutputEnable()      (TRISDCLR = (1<<13))
#define GPIO_RD13_InputEnable()       (TRISDSET = (1<<13))
#define GPIO_RD13_Get()               ((PORTD >> 13) & 0x1)
#define GPIO_RD13_PIN                  GPIO_PIN_RD13

/*** Macros for GPIO_RD4 pin ***/
#define GPIO_RD4_Set()               (LATDSET = (1<<4))
#define GPIO_RD4_Clear()             (LATDCLR = (1<<4))
#define GPIO_RD4_Toggle()            (LATDINV= (1<<4))
#define GPIO_RD4_OutputEnable()      (TRISDCLR = (1<<4))
#define GPIO_RD4_InputEnable()       (TRISDSET = (1<<4))
#define GPIO_RD4_Get()               ((PORTD >> 4) & 0x1)
#define GPIO_RD4_PIN                  GPIO_PIN_RD4

/*** Macros for GPIO_RD5 pin ***/
#define GPIO_RD5_Set()               (LATDSET = (1<<5))
#define GPIO_RD5_Clear()             (LATDCLR = (1<<5))
#define GPIO_RD5_Toggle()            (LATDINV= (1<<5))
#define GPIO_RD5_OutputEnable()      (TRISDCLR = (1<<5))
#define GPIO_RD5_InputEnable()       (TRISDSET = (1<<5))
#define GPIO_RD5_Get()               ((PORTD >> 5) & 0x1)
#define GPIO_RD5_PIN                  GPIO_PIN_RD5

/*** Macros for GPIO_RD6 pin ***/
#define GPIO_RD6_Set()               (LATDSET = (1<<6))
#define GPIO_RD6_Clear()             (LATDCLR = (1<<6))
#define GPIO_RD6_Toggle()            (LATDINV= (1<<6))
#define GPIO_RD6_OutputEnable()      (TRISDCLR = (1<<6))
#define GPIO_RD6_InputEnable()       (TRISDSET = (1<<6))
#define GPIO_RD6_Get()               ((PORTD >> 6) & 0x1)
#define GPIO_RD6_PIN                  GPIO_PIN_RD6

/*** Macros for GPIO_RD7 pin ***/
#define GPIO_RD7_Set()               (LATDSET = (1<<7))
#define GPIO_RD7_Clear()             (LATDCLR = (1<<7))
#define GPIO_RD7_Toggle()            (LATDINV= (1<<7))
#define GPIO_RD7_OutputEnable()      (TRISDCLR = (1<<7))
#define GPIO_RD7_InputEnable()       (TRISDSET = (1<<7))
#define GPIO_RD7_Get()               ((PORTD >> 7) & 0x1)
#define GPIO_RD7_PIN                  GPIO_PIN_RD7

/*** Macros for RPLY pin ***/
#define RPLY_Set()               (LATFSET = (1<<0))
#define RPLY_Clear()             (LATFCLR = (1<<0))
#define RPLY_Toggle()            (LATFINV= (1<<0))
#define RPLY_OutputEnable()      (TRISFCLR = (1<<0))
#define RPLY_InputEnable()       (TRISFSET = (1<<0))
#define RPLY_Get()               ((PORTF >> 0) & 0x1)
#define RPLY_PIN                  GPIO_PIN_RF0

/*** Macros for WR pin ***/
#define WR_Set()               (LATGSET = (1<<1))
#define WR_Clear()             (LATGCLR = (1<<1))
#define WR_Toggle()            (LATGINV= (1<<1))
#define WR_OutputEnable()      (TRISGCLR = (1<<1))
#define WR_InputEnable()       (TRISGSET = (1<<1))
#define WR_Get()               ((PORTG >> 1) & 0x1)
#define WR_PIN                  GPIO_PIN_RG1
#define WR_PIN_MASK (1<<1)
#define CTI_WR (!(portg & WR_PIN_MASK))

/*** Macros for SDEN pin ***/
#define SDEN_Set()               (LATGSET = (1<<0))
#define SDEN_Clear()             (LATGCLR = (1<<0))
#define SDEN_Toggle()            (LATGINV= (1<<0))
#define SDEN_OutputEnable()      (TRISGCLR = (1<<0))
#define SDEN_InputEnable()       (TRISGSET = (1<<0))
#define SDEN_Get()               ((PORTG >> 0) & 0x1)
#define SDEN_PIN                  GPIO_PIN_RG0
#define SDEN_PIN_MASK (1<<0)
#define CTI_SDEN (portg & SDEN_PIN_MASK)
#define PREV_CTI_SDEN (prev_portg & SDEN_PIN_MASK)

/*** Macros for LED3Y pin ***/
#define LED3Y_Set()               (LATESET = (1<<0))
#define LED3Y_Clear()             (LATECLR = (1<<0))
#define LED3Y_Toggle()            (LATEINV= (1<<0))
#define LED3Y_OutputEnable()      (TRISECLR = (1<<0))
#define LED3Y_InputEnable()       (TRISESET = (1<<0))
#define LED3Y_Get()               ((PORTE >> 0) & 0x1)
#define LED3Y_PIN                  GPIO_PIN_RE0

/*** Macros for LED2R pin ***/
#define LED2R_Set()               (LATESET = (1<<1))
#define LED2R_Clear()             (LATECLR = (1<<1))
#define LED2R_Toggle()            (LATEINV= (1<<1))
#define LED2R_OutputEnable()      (TRISECLR = (1<<1))
#define LED2R_InputEnable()       (TRISESET = (1<<1))
#define LED2R_Get()               ((PORTE >> 1) & 0x1)
#define LED2R_PIN                  GPIO_PIN_RE1

/*** Macros for A3 pin ***/
#define A3_Set()               (LATGSET = (1<<14))
#define A3_Clear()             (LATGCLR = (1<<14))
#define A3_Toggle()            (LATGINV= (1<<14))
#define A3_OutputEnable()      (TRISGCLR = (1<<14))
#define A3_InputEnable()       (TRISGSET = (1<<14))
#define A3_Get()               ((PORTG >> 14) & 0x1)
#define A3_PIN                  GPIO_PIN_RG14

/*** Macros for A1 pin ***/
#define A1_Set()               (LATGSET = (1<<12))
#define A1_Clear()             (LATGCLR = (1<<12))
#define A1_Toggle()            (LATGINV= (1<<12))
#define A1_OutputEnable()      (TRISGCLR = (1<<12))
#define A1_InputEnable()       (TRISGSET = (1<<12))
#define A1_Get()               ((PORTG >> 12) & 0x1)
#define A1_PIN                  GPIO_PIN_RG12

/*** Macros for A2 pin ***/
#define A2_Set()               (LATGSET = (1<<13))
#define A2_Clear()             (LATGCLR = (1<<13))
#define A2_Toggle()            (LATGINV= (1<<13))
#define A2_OutputEnable()      (TRISGCLR = (1<<13))
#define A2_InputEnable()       (TRISGSET = (1<<13))
#define A2_Get()               ((PORTG >> 13) & 0x1)
#define A2_PIN                  GPIO_PIN_RG13

/*** Macros for LED1W pin ***/
#define LED1W_Set()               (LATESET = (1<<2))
#define LED1W_Clear()             (LATECLR = (1<<2))
#define LED1W_Toggle()            (LATEINV= (1<<2))
#define LED1W_OutputEnable()      (TRISECLR = (1<<2))
#define LED1W_InputEnable()       (TRISESET = (1<<2))
#define LED1W_Get()               ((PORTE >> 2) & 0x1)
#define LED1W_PIN                  GPIO_PIN_RE2

/*** Macros for LED0G pin ***/
#define LED0G_Set()               (LATESET = (1<<3))
#define LED0G_Clear()             (LATECLR = (1<<3))
#define LED0G_Toggle()            (LATEINV= (1<<3))
#define LED0G_OutputEnable()      (TRISECLR = (1<<3))
#define LED0G_InputEnable()       (TRISESET = (1<<3))
#define LED0G_Get()               ((PORTE >> 3) & 0x1)
#define LED0G_PIN                  GPIO_PIN_RE3

/*** Macros for IRQB pin ***/
#define IRQB_Set()               (LATESET = (1<<4))
#define IRQB_Clear()             (LATECLR = (1<<4))
#define IRQB_Toggle()            (LATEINV= (1<<4))
#define IRQB_OutputEnable()      (TRISECLR = (1<<4))
#define IRQB_InputEnable()       (TRISESET = (1<<4))
#define IRQB_Get()               ((PORTE >> 4) & 0x1)
#define IRQB_PIN                  GPIO_PIN_RE4


// *****************************************************************************
/* GPIO Port

  Summary:
    Identifies the available GPIO Ports.

  Description:
    This enumeration identifies the available GPIO Ports.

  Remarks:
    The caller should not rely on the specific numbers assigned to any of
    these values as they may change from one processor to the next.

    Not all ports are available on all devices.  Refer to the specific
    device data sheet to determine which ports are supported.
*/

typedef enum
{
    GPIO_PORT_A = 0,
    GPIO_PORT_B = 1,
    GPIO_PORT_C = 2,
    GPIO_PORT_D = 3,
    GPIO_PORT_E = 4,
    GPIO_PORT_F = 5,
    GPIO_PORT_G = 6,
} GPIO_PORT;

// *****************************************************************************
/* GPIO Port Pins

  Summary:
    Identifies the available GPIO port pins.

  Description:
    This enumeration identifies the available GPIO port pins.

  Remarks:
    The caller should not rely on the specific numbers assigned to any of
    these values as they may change from one processor to the next.

    Not all pins are available on all devices.  Refer to the specific
    device data sheet to determine which pins are supported.
*/

typedef enum
{
    GPIO_PIN_RA0 = 0,
    GPIO_PIN_RA1 = 1,
    GPIO_PIN_RA2 = 2,
    GPIO_PIN_RA3 = 3,
    GPIO_PIN_RA4 = 4,
    GPIO_PIN_RA5 = 5,
    GPIO_PIN_RA6 = 6,
    GPIO_PIN_RA7 = 7,
    GPIO_PIN_RA9 = 9,
    GPIO_PIN_RA10 = 10,
    GPIO_PIN_RA14 = 14,
    GPIO_PIN_RA15 = 15,
    GPIO_PIN_RB0 = 16,
    GPIO_PIN_RB1 = 17,
    GPIO_PIN_RB2 = 18,
    GPIO_PIN_RB3 = 19,
    GPIO_PIN_RB4 = 20,
    GPIO_PIN_RB5 = 21,
    GPIO_PIN_RB6 = 22,
    GPIO_PIN_RB7 = 23,
    GPIO_PIN_RB8 = 24,
    GPIO_PIN_RB9 = 25,
    GPIO_PIN_RB10 = 26,
    GPIO_PIN_RB11 = 27,
    GPIO_PIN_RB12 = 28,
    GPIO_PIN_RB13 = 29,
    GPIO_PIN_RB14 = 30,
    GPIO_PIN_RB15 = 31,
    GPIO_PIN_RC1 = 33,
    GPIO_PIN_RC2 = 34,
    GPIO_PIN_RC3 = 35,
    GPIO_PIN_RC4 = 36,
    GPIO_PIN_RC12 = 44,
    GPIO_PIN_RC13 = 45,
    GPIO_PIN_RC14 = 46,
    GPIO_PIN_RC15 = 47,
    GPIO_PIN_RD0 = 48,
    GPIO_PIN_RD1 = 49,
    GPIO_PIN_RD2 = 50,
    GPIO_PIN_RD3 = 51,
    GPIO_PIN_RD4 = 52,
    GPIO_PIN_RD5 = 53,
    GPIO_PIN_RD6 = 54,
    GPIO_PIN_RD7 = 55,
    GPIO_PIN_RD8 = 56,
    GPIO_PIN_RD9 = 57,
    GPIO_PIN_RD10 = 58,
    GPIO_PIN_RD11 = 59,
    GPIO_PIN_RD12 = 60,
    GPIO_PIN_RD13 = 61,
    GPIO_PIN_RD14 = 62,
    GPIO_PIN_RD15 = 63,
    GPIO_PIN_RE0 = 64,
    GPIO_PIN_RE1 = 65,
    GPIO_PIN_RE2 = 66,
    GPIO_PIN_RE3 = 67,
    GPIO_PIN_RE4 = 68,
    GPIO_PIN_RE5 = 69,
    GPIO_PIN_RE6 = 70,
    GPIO_PIN_RE7 = 71,
    GPIO_PIN_RE8 = 72,
    GPIO_PIN_RE9 = 73,
    GPIO_PIN_RF0 = 80,
    GPIO_PIN_RF1 = 81,
    GPIO_PIN_RF2 = 82,
    GPIO_PIN_RF3 = 83,
    GPIO_PIN_RF4 = 84,
    GPIO_PIN_RF5 = 85,
    GPIO_PIN_RF8 = 88,
    GPIO_PIN_RF12 = 92,
    GPIO_PIN_RF13 = 93,
    GPIO_PIN_RG0 = 96,
    GPIO_PIN_RG1 = 97,
    GPIO_PIN_RG2 = 98,
    GPIO_PIN_RG3 = 99,
    GPIO_PIN_RG6 = 102,
    GPIO_PIN_RG7 = 103,
    GPIO_PIN_RG8 = 104,
    GPIO_PIN_RG9 = 105,
    GPIO_PIN_RG12 = 108,
    GPIO_PIN_RG13 = 109,
    GPIO_PIN_RG14 = 110,
    GPIO_PIN_RG15 = 111,

    /* This element should not be used in any of the GPIO APIs.
       It will be used by other modules or application to denote that none of the GPIO Pin is used */
    GPIO_PIN_NONE = -1

} GPIO_PIN;

typedef enum
{
  CN0_PIN = 1 << 0,
  CN1_PIN = 1 << 1,
  CN2_PIN = 1 << 2,
  CN3_PIN = 1 << 3,
  CN4_PIN = 1 << 4,
  CN5_PIN = 1 << 5,
  CN6_PIN = 1 << 6,
  CN7_PIN = 1 << 7,
  CN8_PIN = 1 << 8,
  CN9_PIN = 1 << 9,
  CN10_PIN = 1 << 10,
  CN11_PIN = 1 << 11,
  CN12_PIN = 1 << 12,
  CN13_PIN = 1 << 13,
  CN14_PIN = 1 << 14,
  CN15_PIN = 1 << 15,
  CN16_PIN = 1 << 16,
  CN17_PIN = 1 << 17,
  CN18_PIN = 1 << 18,
  CN19_PIN = 1 << 19,
  CN20_PIN = 1 << 20,
  CN21_PIN = 1 << 21,
}CN_PIN;

typedef  void (*GPIO_PIN_CALLBACK) ( CN_PIN cnPin, uintptr_t context);

void GPIO_Initialize(void);

// *****************************************************************************
// *****************************************************************************
// Section: GPIO Functions which operates on multiple pins of a port
// *****************************************************************************
// *****************************************************************************

uint32_t GPIO_PortRead(GPIO_PORT port);

void GPIO_PortWrite(GPIO_PORT port, uint32_t mask, uint32_t value);

uint32_t GPIO_PortLatchRead ( GPIO_PORT port );

void GPIO_PortSet(GPIO_PORT port, uint32_t mask);

void GPIO_PortClear(GPIO_PORT port, uint32_t mask);

void GPIO_PortToggle(GPIO_PORT port, uint32_t mask);

void GPIO_PortInputEnable(GPIO_PORT port, uint32_t mask);

void GPIO_PortOutputEnable(GPIO_PORT port, uint32_t mask);

void GPIO_PinInterruptEnable(CN_PIN cnPin);

void GPIO_PinInterruptDisable(CN_PIN cnPin);

// *****************************************************************************
// *****************************************************************************
// Section: Local Data types and Prototypes
// *****************************************************************************
// *****************************************************************************

typedef struct {

    /* CN Pin number */
    CN_PIN                  cnPin;

    /* Corresponding GPIO pin name */
    GPIO_PIN                gpioPin;

    /* previous port pin value, need to be stored to check if it has changed later */
    bool                    prevPinValue;

    /* Callback for event on target pin*/
    GPIO_PIN_CALLBACK       callback;

    /* Callback Context */
    uintptr_t               context;

} GPIO_PIN_CALLBACK_OBJ;

// *****************************************************************************
// *****************************************************************************
// Section: GPIO Functions which operates on one pin at a time
// *****************************************************************************
// *****************************************************************************

static inline void GPIO_PinWrite(GPIO_PIN pin, bool value)
{
    GPIO_PortWrite((GPIO_PORT)(pin>>4), (uint32_t)(0x1) << (pin & 0xF), (uint32_t)(value) << (pin & 0xF));
}

static inline bool GPIO_PinRead(GPIO_PIN pin)
{
    return (bool)(((GPIO_PortRead((GPIO_PORT)(pin>>4))) >> (pin & 0xF)) & 0x1);
}

static inline bool GPIO_PinLatchRead(GPIO_PIN pin)
{
    return (bool)((GPIO_PortLatchRead((GPIO_PORT)(pin>>4)) >> (pin & 0xF)) & 0x1);
}

static inline void GPIO_PinToggle(GPIO_PIN pin)
{
    GPIO_PortToggle((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}

static inline void GPIO_PinSet(GPIO_PIN pin)
{
    GPIO_PortSet((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}

static inline void GPIO_PinClear(GPIO_PIN pin)
{
    GPIO_PortClear((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}

static inline void GPIO_PinInputEnable(GPIO_PIN pin)
{
    GPIO_PortInputEnable((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}

static inline void GPIO_PinOutputEnable(GPIO_PIN pin)
{
    GPIO_PortOutputEnable((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}

bool GPIO_PinInterruptCallbackRegister(
    CN_PIN cnPin,
    const   GPIO_PIN_CALLBACK callBack,
    uintptr_t context
);

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif
// DOM-IGNORE-END
#endif // PLIB_GPIO_H
