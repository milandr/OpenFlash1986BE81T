/*********************************************************************
*               (c) SEGGER Microcontroller GmbH & Co. KG             *
*                        The Embedded Experts                        *
*                           www.segger.com                           *
**********************************************************************
----------------------------------------------------------------------
File    : FlashPrg.c
Purpose : Implementation of RAMCode template
--------  END-OF-HEADER  ---------------------------------------------
*/
#include "FlashOS.h"
#include "cm4ikmcu.h"

extern unsigned char getecc(uint32_t data, uint32_t adr);
/*********************************************************************
*
*       Defines
*
**********************************************************************
*/
#define PAGE_SIZE_SHIFT (2)      // The smallest program unit (one page) is 8 byte in size
#define HSEn_ON (1<<27)
#define HSEn_BYP (1<<28)
#define HSE_FILTER_EN (1<<29)
#define HSEn_RDY (1<<20)
#define MAX_CLK_SEL_HSE0 2

/*********************************************************************
*
*       Types
*
**********************************************************************
*/

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/
//
// We use this dummy variable to make sure that the PrgData
// section is present in the output elf-file as this section
// is mandatory in current versions of the J-Link DLL 
//
static volatile int _Dummy;

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/

/*********************************************************************
*
*       _FeedWatchdog
*
*  Function description
*    Feeds the watchdog. Needs to be called during RAMCode execution
*    in case of an watchdog is active.
*/
static void _FeedWatchdog(void) {
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/

/*********************************************************************
*
*       Init
*
*  Function description
*    Handles the initialization of the flash module.
*
*  Parameters
*    Addr: Flash base address
*    Freq: Clock frequency in Hz
*    Func: Caller type (e.g.: 1 - Erase, 2 - Program, 3 - Verify)
*
*  Return value 
*    0 O.K.
*    1 Error
*/
int Init(U32 Addr, U32 Freq, U32 Func) {
  (void)Addr;
  (void)Freq;
  (void)Func;
  //
  // Init code
  //

  OTP_CNTR->KEY          = 0x8555aaa1;
  
  return 0;
}

/*********************************************************************
*
*       UnInit
*
*  Function description
*    Handles the de-initialization of the flash module.
*
*  Parameters
*    Func: Caller type (e.g.: 1 - Erase, 2 - Program, 3 - Verify)
*
*  Return value 
*    0 O.K.
*    1 Error
*/
int UnInit(U32 Func) {
  (void)Func;
  //
  // Uninit code
  //

  OTP_CNTR->KEY = 0;
  
  return 0;
}

/*
* Program 32-bit word through the OTP_CTRL register mode
*
*/
int ProgramWord (uint32_t adr, uint32_t wrd) 
{
  uint8_t ecc;
                
  ecc = getecc(adr, wrd);
                
  OTP_CNTR->ADR=adr;
  OTP_CNTR->WDATA=wrd;

  OTP_CNTR->CNTR=(ecc<<16) | 1<<5 | 1<<4 | 0x1;
  OTP_CNTR->CNTR|=(0x3e<<8);

  OTP_CNTR->CNTR&=~(1<<5 | 1<<4);
                
  return (0); 
}

/*********************************************************************
*
*       EraseSector
*
*  Function description
*    Erases one flash sector.
*
*  Parameters
*    Addr: Address of the sector to be erased
*
*  Return value 
*    0 O.K.
*    1 Error
*/
int EraseSector(U32 SectorAddr) {
  volatile uint32_t * pSrc;
  uint32_t * pDest;
  U8 AccessWidth;
  U32 Status;
  U32 NumWords;
  int r;

  r           = -1;
  pDest       = (uint32_t *) SectorAddr;
  //
  // RAMCode is able to program multiple pages
  //
  NumWords    = 0x00000200 >> 2;
  //
  // Program page-wise
  //
  _FeedWatchdog();
  if (NumWords) {
    r = 0;
    do {
        ProgramWord((uint32_t) pDest, -1);
        pDest++;
    } while (--NumWords);
  }
  return r;
}

/*********************************************************************
*
*       ProgramPage
*
*  Function description
*    Programs one flash page.
*
*  Parameters
*    DestAddr: Destination address
*    NumBytes: Number of bytes to be programmed (always a multiple of program page size, defined in FlashDev.c)
*    pSrcBuff: Point to the source buffer
*
*  Return value 
*    0 O.K.
*    1 Error
*/
int ProgramPage(U32 DestAddr, U32 NumBytes, U8 *pSrcBuff) {
  volatile uint32_t * pSrc;
  uint32_t * pDest;
  U8 AccessWidth;
  U32 Status;
  U32 NumWords;
  int r;

  r           = -1;
  pSrc        = (volatile uint32_t *) pSrcBuff;
  pDest       = (uint32_t *) DestAddr;
  //
  // RAMCode is able to program multiple pages
  //
  NumWords    = NumBytes >> 2;
  //
  // Program page-wise
  //
  _FeedWatchdog();
  if (NumWords) {
    r = 0;
    do {
        ProgramWord((uint32_t) pDest, *pSrc);
        pDest++;
        pSrc++;
    } while (--NumWords);
  }
  return r;
}
