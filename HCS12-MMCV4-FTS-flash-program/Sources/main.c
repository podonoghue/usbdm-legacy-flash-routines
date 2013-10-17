//!
//! Flash programming code (HCS12-MMCV4-FTS-flash-program)
//! Flash programming code (HCS12-MMCV4-FTS_2-flash-program)
//!
//! Families
//!   'Standard' HCS12 devices e.g.
//!   Cxx, Exx, GCxx, Qxx, 
//!
//! CPU
//!   CPU12
//!
//! Paged memory management (PPAGE=0x30)
//!   S12MMCVx
//!
//! CPU Memory accesses are limited to 64K
//!

//!   S12FTS16KV1    -  16K Flash  1x16K block,  512 byte sector
//!   S12FTS32KV1    -  32K Flash  1x32K block,  512 byte sector
//!   S12FTS64K      -  64K Flash  1x64K block,  512 byte sector
//!   S12FTS64KV1    -  64K Flash  1x64K block,  512 byte sector
//!   S12FTS64KV4    -  64K Flash  1x64K block, 1024 byte sector
//!   S12FTS96KV1    -  96K Flash  1x96K block, 1024 byte sector
//!   S12FTS128K1V1  - 128K Flash 1x128K block, 1024 byte sector
//!   S12FTS128KV1   - 128K Flash  2x64K block,  512 byte sector
//!   S12FTS128KV2   - 128K Flash  2x64K block,  512 byte sector
//!   S12FTS256KV2   - 256K Flash  4x64K block,  512 byte sector
//!   S12FTS256KV3   - 256K Flash  4x64K block,  512 byte sector
//!   S12FTS256K2V1  - 256K Flash 2x128K block, 1024 byte sector - Requires shift modification (fcnfgValue)
//!   S12FTS512K4V1  - 512K Flash 4x128K block, 1024 byte sector - Requires shift modification (fcnfgValue)
//!
//!   S12EETS2KV1  -   1K EEPROM   4 byte sectors
//!   S12EETS2KV1  -   2K EEPROM   4 byte sectors
//!   S12EETS4KV2  -   4K EEPROM   4 byte sectors
//!
           
//! This code accepts paged addresses only
//!
#pragma CODE_SEG code

typedef unsigned long  uint32_t;
typedef unsigned int   uint16_t;
typedef unsigned char  uint8_t;

#ifndef NULL
#define NULL ((void*)0)
#endif

//#define DEBUG

//==========================================================================================================
// Target defines

#define CLOCK_CALIB (339279872) // 1 second count for a 4MHz bus speed HCS12 chip

// PPAGE register
#define PPAGE  (*(uint8_t*)0x0030)

#ifdef DEBUG
#define FLASH_REG_BASE              ((FlashController*) 0x0100)  // For testing
#define COPCTL_ADDRESS              ((uint8_t *) 0x03C)
// This is the smallest unit of Flash that can be erased
#define FLASH_SECTOR_SIZE   (512) // 512 byte (1/2 K) block size (used for stride in erase)
#define EEPROM_SECTOR_SIZE  (4)   // 4 bytes
#else
#define COPCTL_ADDRESS              ((uint8_t *) NULL)
#endif

typedef struct {
   volatile uint8_t  fclkdiv;
   volatile uint8_t  fsec;
   volatile uint8_t  ftstmod;
   volatile uint8_t  fcnfg;
   volatile uint8_t  fprot;
   volatile uint8_t  fstat;
   volatile uint8_t  fcmd;
   volatile uint8_t  reserved;
   volatile uint16_t faddr;
   volatile uint16_t fdata;
} FlashController;

#define FSTAT_OFFSET 5

#define fclkdiv_FDIVLD  (1<<7)

#define FSTAT_CBEIF     (1<<7)  //!< Command buffer empty
#define FSTAT_CCIF      (1<<6)  //!< Command complete
#define FSTAT_PVIOL     (1<<5)  //!< Protection violation
#define FSTAT_ACCERR    (1<<4)  //!< Access error
#define FSTAT_BLANK     (1<<2)  //!< Blank check OK
#define FSTAT_FAIL      (1<<1)  //!< Flash Operation failed (blank only?)
#define FSTAT_DONE      (1<<0)  //!< Flash operation complete

#define FTSTMOD_WRALL   (1<<4)

#define CFMCLKD_DIVLD   (1<<7)
#define CFMCLKD_PRDIV8  (1<<6)
#define CFMCLKD_FDIV    (0x3F)

// Flash commands
#define FCMD_PAGE_ERASE_VERIFY    (0x05)
#define FCMD_WORD_PROGRAM         (0x20)
#define FCMD_PAGE_ERASE           (0x40)
#define FCMD_MASS_ERASE           (0x41)

//==========================================================================================================
// Operation masks
//
//  The following combinations (amongst others) are sensible:
//  DO_MASS_ERASE|DO_UNLOCK_FLASH - erase all flash and program default unsecured value
//  DO_ERASE_RANGE|DO_UNLOCK_FLASH - erase security area only and program default unsecured value
//  DO_LOCK_FLASH - program default secured value, assuming security area has been previously erased
//  DO_ERASE_RANGE|DO_LOCK_FLASH - erase security area and program default secured value
//  DO_PROGRAM_RANGE|DO_VERIFY_RANGE program & verify range assuming previously erased
//  DO_ERASE_RANGE|DO_BLANK_CHECK_RANGE|DO_PROGRAM_RANGE|DO_VERIFY_RANGE do all steps
//
#define DO_INIT_FLASH         (1<<0) // Do initialisation of flash 
#define DO_MASS_ERASE         (1<<1) // Mass erase flash array
#define DO_ERASE_RANGE        (1<<2) // Erase range (including option region)
#define DO_BLANK_CHECK_RANGE  (1<<3) // Blank check region
#define DO_PROGRAM_RANGE      (1<<4) // Program range (including option region)
#define DO_VERIFY_RANGE       (1<<5) // Verify range
#define DO_UNLOCK_FLASH       (1<<6) // Unlock flash with default security options  (+mass erase if needed)
#define DO_LOCK_FLASH         (1<<7) // Lock flash with default security options
#define DO_TIMING_LOOP        (1<<8) // Do timing loop
// 9 - 14 reserved
#define IS_COMPLETE           (1<<15)

// Capability masks
#define CAP_MASS_ERASE      (1<<1)
#define CAP_ERASE_RANGE     (1<<2)
#define CAP_BLANK_CHECK     (1<<3)
#define CAP_PROGRAM_RANGE   (1<<4)
#define CAP_VERIFY_RANGE    (1<<5)
#define CAP_UNLOCK_FLASH    (1<<6)
#define CAP_TIMING          (1<<7)
//
#define CAP_ALIGN_OFFS      (13)
#define CAP_ALIGN_MASK      (3<<CAP_ALIGN_OFFS)
#define CAP_ALIGN_BYTE      (0<<CAP_ALIGN_OFFS)
#define CAP_ALIGN_2BYTE     (1<<CAP_ALIGN_OFFS)
#define CAP_ALIGN_4BYTE     (2<<CAP_ALIGN_OFFS)
#define CAP_ALIGN_8BYTE     (3<<CAP_ALIGN_OFFS)

#define CAP_RELOCATABLE     (1<<15)

// These error numbers are just for debugging
typedef enum {
     FLASH_ERR_OK                = (0),
     FLASH_ERR_LOCKED            = (1),  // Flash is still locked
     FLASH_ERR_ILLEGAL_PARAMS    = (2),  // Parameters illegal
     FLASH_ERR_PROG_FAILED       = (3),  // STM - Programming operation failed - general
     FLASH_ERR_PROG_WPROT        = (4),  // STM - Programming operation failed - write protected
     FLASH_ERR_VERIFY_FAILED     = (5),  // Verify failed
     FLASH_ERR_ERASE_FAILED      = (6),  // Erase or Blank Check failed
     FLASH_ERR_TRAP              = (7),  // Program trapped (illegal instruction/location etc.)
     FLASH_ERR_PROG_ACCERR       = (8),  // Kinetis/CFVx - Programming operation failed - ACCERR
     FLASH_ERR_PROG_FPVIOL       = (9),  // Kinetis/CFVx - Programming operation failed - FPVIOL
     FLASH_ERR_PROG_MGSTAT0      = (10), // Kinetis - Programming operation failed - MGSTAT0
     FLASH_ERR_CLKDIV            = (11), // CFVx - Clock divider not set
     FLASH_ERR_ILLEGAL_SECURITY  = (12)  // Kinetis - Illegal value for security location
} FlashDriverError_t;

typedef void (*EntryPoint_t)(void);

// Describes a block to be programmed & result
typedef struct {
   uint16_t         flags;             //  0: Controls actions of routine
   uint16_t         errorCode;         //  2: Error code from action
   FlashController *controller;        //  4: Ptr to flash controller
   uint16_t         frequency;         //  6: Target frequency (kHz)
   uint16_t         sectorSize;        //  8: Size of Flash memory sectors (smallest erasable block)
   uint8_t          res;               // 10:
   uint8_t          page;              // 11: Page number
   uint16_t         address;           // 12: Memory address being accessed
   uint16_t         size;              // 14: Size of memory range being accessed
   const uint16_t  *data;              // 16: Ptr to data to program
} FlashData_t;

// Timing information
typedef struct {
   uint16_t         flags;             // 0: Controls actions of routine
   uint16_t         errorCode;         // 2: Error code from action
   FlashController *controller;        // 4: Ptr to flash controller
   uint32_t         timingCount;       // 6: Timing count
} TimingData_t;

#define FLAGS_OFFSET        0
#define ERROR_CODE_OFFSET   2
#define CONTROLLER_OFFSET   4
#define TIMING_COUNT_OFFSET 6
 
//! Describe the flash programming code
//!
typedef struct {
   uint16_t         loadAddress;       //  0: Address where to load this image
   EntryPoint_t     entry;             //  2: Ptr to entry routine
   uint16_t         capabilities;      //  4: Capabilities of routine
   uint8_t         *copctlAddress;     //  6: Address of SOPT register
   uint32_t         clockFactor;       //  8: Calibration factor for speed detemination
   FlashData_t     *flashData;         // 12: Ptr to information about operation
} FlashProgramHeader_t;

#define FLASH_DATA_OFFSET 12

void asm_entry(void);
const FlashProgramHeader_t gFlashProgramHeader;

// Used by programmer to locate flashProgramHeader 
const FlashProgramHeader_t *const headerPtr = &gFlashProgramHeader;

//! Flash programming information table
//!
const FlashProgramHeader_t gFlashProgramHeader = {
     /* loadAddress    */ (uint16_t)&headerPtr,  // default load address of image
     /* entry          */ asm_entry,             // entry point for code
     /* capabilities   */ CAP_RELOCATABLE|CAP_BLANK_CHECK|CAP_ERASE_RANGE|CAP_MASS_ERASE|CAP_PROGRAM_RANGE|CAP_VERIFY_RANGE|CAP_ALIGN_2BYTE,
     /* copctlAddress  */ COPCTL_ADDRESS,
     /* clockFactor    */ CLOCK_CALIB,
     /* flashData      */ NULL,
};

#pragma CODE_SEG code
void  setErrorCode(uint8_t errorCode);
void  initFlash(void);
void  massEraseFlash(void);
void  programRange(void);
void  verifyRange(void);
void  eraseRange(void);
void  blankCheckRange(void);
void  doTiming();
void  entry(void);
void  testApp(void);
void  asm_testApp(void);

//! Set error code to return to BDM
//!
//! @note Doesn't return
//!
#pragma NO_ENTRY
#pragma NO_EXIT
#pragma NO_RETURN
#pragma MESSAGE DISABLE C1404
#pragma MESSAGE DISABLE C5703
void setErrorCode(uint8_t errorCode) {
   // ENTRY : B = errorCode
   asm {
      ldx   gFlashProgramHeader:FLASH_DATA_OFFSET,PCR   // flashData
      stab  ERROR_CODE_OFFSET+1,x
      bset  FLAGS_OFFSET,x,#(IS_COMPLETE&0xFF)
      bgnd
   }
}
#pragma MESSAGE DEFAULT C1404
#pragma MESSAGE DEFAULT C5703


//! Does any initialisation required before accessing the Flash
//!
void initFlash(void) {
   FlashController *controller;

   FlashData_t     *flashData;  
   asm {
      ldx   gFlashProgramHeader:FLASH_DATA_OFFSET,PCR   // flashData
      stx   flashData
   }
   if ((flashData->flags&DO_INIT_FLASH) == 0)
      return;

   controller = flashData->controller;

#ifdef DEBUG
   controller->fclkdiv = 0x0F;
#endif
   
   // Check if clock divider correctly set
   if ((controller->fclkdiv & CFMCLKD_DIVLD) == 0) {
      setErrorCode(FLASH_ERR_CLKDIV);
   }
   controller->fprot = 0xFF;  // Unprotect flash
   
   flashData->flags &= ~DO_INIT_FLASH;
}

//! Launch & wait for Flash command to complete
//!
//! @param completeMask - FSTAT mask to wait for
//!
void doFlashCommand(uint8_t completeMask) {
   uint8_t *pFstat;
   uint8_t fstat;
   asm {
      ldx   gFlashProgramHeader:FLASH_DATA_OFFSET,PCR   // flashData
      ldx   CONTROLLER_OFFSET,x                          // controller
      leax  FSTAT_OFFSET,x                               // &controller.fstat
      stx   pFstat
   }
   // Launch command
   *pFstat = FSTAT_CBEIF;
   asm {
      nop // allow FSTAT_CCIF to set
      nop
   }
   // Wait for command complete
   do {
      fstat = *pFstat;
   } while ((fstat&completeMask) == 0);
   if ((fstat & FSTAT_ACCERR) != 0)
      setErrorCode(FLASH_ERR_PROG_ACCERR);
   if ((fstat & FSTAT_PVIOL) != 0)
      setErrorCode(FLASH_ERR_PROG_FPVIOL);
}

//! Erase entire flash array
//!
void massEraseFlash(void) {
   FlashController *controller;
   
   FlashData_t     *flashData;  
   asm {
      ldx   gFlashProgramHeader:FLASH_DATA_OFFSET,PCR   // flashData
      stx   flashData
   }
   if ((flashData->flags&DO_MASS_ERASE) == 0)
      return;

   controller = flashData->controller;

   // Clear any existing errors
   controller->fstat   = FSTAT_ACCERR|FSTAT_PVIOL;
   controller->ftstmod = 0;
   controller->fstat   = FSTAT_FAIL;
   // Apply to all flash blocks (<> pages!)
   controller->ftstmod = FTSTMOD_WRALL;
   // Buffer address & dummy data
   controller->faddr   = flashData->address;
   controller->fdata   = 0xFFFF;

   // Write command
   controller->fcmd = FCMD_MASS_ERASE;
   
   doFlashCommand(FSTAT_CCIF|FSTAT_ACCERR|FSTAT_PVIOL);

   flashData->flags &= ~DO_MASS_ERASE;
}

//! Program a range of flash from buffer
//!
void programRange(void) {
   FlashController *controller;
   uint16_t         address;
   const uint16_t  *data;
   uint16_t         numWords;
   
   FlashData_t     *flashData;  
   asm {
      ldx   gFlashProgramHeader:FLASH_DATA_OFFSET,PCR   // flashData
      stx   flashData
   }
   if ((flashData->flags&DO_PROGRAM_RANGE) == 0)
      return;
   
   controller = flashData->controller;
   address    = flashData->address;
   data       = flashData->data;
   numWords   = flashData->size/2;

   if ((address & 0x1) != 0) {
      setErrorCode(FLASH_ERR_ILLEGAL_PARAMS);
   }
   // Clear any existing errors
   controller->fstat   = FSTAT_ACCERR|FSTAT_PVIOL;
   controller->ftstmod = 0;
   controller->fstat   = FSTAT_FAIL;
   
   // Program words
   while (numWords-- > 0) {
      // Write data to flash address
      *(uint16_t*)(address) = *data++;

      // Set command
      controller->fcmd = FCMD_WORD_PROGRAM;

      if (numWords == 0) {
         // Last command
         doFlashCommand(FSTAT_CCIF|FSTAT_ACCERR|FSTAT_PVIOL);
      }
      else {
         doFlashCommand(FSTAT_CBEIF|FSTAT_ACCERR|FSTAT_PVIOL);
      }
      address += 2;
   }
   flashData->flags &= ~DO_PROGRAM_RANGE;
}

//! Verify a range of flash against buffer
//!
void verifyRange(void) {
   uint16_t       *address;
   const uint16_t *data;
   uint16_t        numWords;

   FlashData_t    *flashData;  
   asm {
      ldx   gFlashProgramHeader:FLASH_DATA_OFFSET,PCR   // flashData
      stx   flashData
   }
   if ((flashData->flags&DO_VERIFY_RANGE) == 0)
      return;
   
   address   = (uint16_t *)flashData->address;
   
   data      = flashData->data;
   numWords  = flashData->size/2;

   if (((uint16_t)address & 0x01) != 0) {
      setErrorCode(FLASH_ERR_ILLEGAL_PARAMS);
   }
   // Verify words
   while (numWords-- > 0) {
      if (*address++ != *data++)
         setErrorCode(FLASH_ERR_VERIFY_FAILED);
   }
   flashData->flags &= ~DO_VERIFY_RANGE;
}

//! Erase a range of flash
//!
void eraseRange(void) {
   FlashController *controller;
   uint16_t address;
   uint16_t endAddress;
   uint16_t sectorSize;
   uint16_t pageMask;
   
   FlashData_t     *flashData;  
   asm {
      ldx   gFlashProgramHeader:FLASH_DATA_OFFSET,PCR   // flashData
      stx   flashData
   }
   if ((flashData->flags&DO_ERASE_RANGE) == 0) {
      return;
   }
   controller  = flashData->controller;
   address     = flashData->address;
   endAddress  = address + (flashData->size-1); // Inclusive
   sectorSize  = flashData->sectorSize;
   pageMask    = ~(sectorSize-1U);
   
   // Check for empty range
   if (flashData->size == 0) {
      flashData->flags &= ~DO_ERASE_RANGE;
      return;
   }
   // Round start address to start of block (inclusive)
   address &= pageMask;
   
   // Round end address to end of block (exclusive)
   endAddress += sectorSize;
   endAddress &= pageMask;

   // Clear any existing errors
   controller->fstat   = FSTAT_ACCERR|FSTAT_PVIOL;
   controller->ftstmod = 0;
   controller->fstat   = FSTAT_FAIL;

   // Erase each sector
   while (address != endAddress) {
      // Write dummy data to flash address
      *(uint16_t*)(address) = 0xFFFF;
      
      // Set command
      controller->fcmd = FCMD_PAGE_ERASE;

      doFlashCommand(FSTAT_CCIF|FSTAT_ACCERR|FSTAT_PVIOL);

      // Advance to start of next sector
      address += sectorSize;
   }
   flashData->flags &= ~DO_ERASE_RANGE;
}

//! Check that a range of flash is blank (=0xFFFF)
//!
void blankCheckRange(void) {
   uint16_t address;
   uint16_t numWords;

   FlashData_t     *flashData;  
   asm {
      ldx   gFlashProgramHeader:FLASH_DATA_OFFSET,PCR   // flashData
      stx   flashData
   }
   if ((flashData->flags&DO_BLANK_CHECK_RANGE) == 0) {
      return;
   }
   address  = flashData->address;
   numWords = (flashData->size+1)/2;
   
   if ((address & 0x01) != 0)
      setErrorCode(FLASH_ERR_ILLEGAL_PARAMS);

   while (numWords>0) {
      if (*(uint16_t*)address != 0xFFFF)
         setErrorCode(FLASH_ERR_ERASE_FAILED);
      numWords--;
      address += 2;
   }
   flashData->flags &= ~DO_BLANK_CHECK_RANGE;
}

//!  Assembly counting loop
//!
void doTiming(void) {
   asm {
      ldx   gFlashProgramHeader:FLASH_DATA_OFFSET,PCR   // flashData
      brclr FLAGS_OFFSET,X,#(DO_TIMING_LOOP>>8),done     // flashData->flags&DO_TIMING_LOOP ?
      
      ldy   #0
      sty   TIMING_COUNT_OFFSET+2,x
      clra
//! ==== Any changes require CLOCK_CALIB to be altered ====
   loop:
      sty   TIMING_COUNT_OFFSET,x
   loop2:
      dbne  A,loop2
      iny
      bra   loop
//!========================================================
   done:
   }
   return;
}

//! Some stack space
#define STACK_SIZE 60
volatile const uint8_t stackSpace[STACK_SIZE];

//! Main C entry point
//!
void entry(void) {
   const FlashProgramHeader_t *theFlashProgramHeader;
   FlashData_t                *flashData;  
   uint8_t                    fcnfgValue;  
   asm {
      // theFlashProgramHeader = &flashProgramHeader;
      leax  gFlashProgramHeader,PCR
      stx   theFlashProgramHeader
   }
   // Set up access to data for this operation
   flashData = theFlashProgramHeader->flashData;  

   // Disable COP
   *(theFlashProgramHeader->copctlAddress) = 0x00;
   
   // Indicate not complete
   flashData->flags &= ~IS_COMPLETE;

   // No errors so far
   flashData->errorCode = FLASH_ERR_OK;
   
   // Set PPAGE
   PPAGE = flashData->page;
   
   // Set FCNFG
   fcnfgValue = 0x00;  
   if (((flashData->address>>8)>=0x80) && ((flashData->address>>8)<0xC0)) {
#ifdef FTS_2  // 128K Blocks
      fcnfgValue = ~(flashData->page>>3)&0x0F;
#else         // 64K Blocks or only a single block
      fcnfgValue = ~(flashData->page>>2)&0x0F;
#endif
   }
   flashData->controller->fcnfg = fcnfgValue;

   // Uses Paged addresses
   initFlash();
   doTiming();
   massEraseFlash();
   eraseRange();
   blankCheckRange();
   programRange();
   verifyRange();

   // Indicate completed
   flashData->flags |= IS_COMPLETE; 
}

#pragma MESSAGE DISABLE C12053
//! Low level entry point
//! 
#pragma NO_ENTRY
#pragma NO_EXIT
void asm_entry(void) {
   asm {
     
      // Disable interrupts
      sei
      
#ifndef DEBUG
      // Setup the stack before we attempt anything else
      leas   stackSpace:STACK_SIZE-1,PCR
      pula   // SP should point off end of stack space
#endif
      
      jsr     entry
      
#ifndef DEBUG
      bgnd
#else
      rts
#endif
   }
}
#pragma MESSAGE DEFAULT C12053

#ifndef DEBUG
#pragma NO_ENTRY
#pragma NO_EXIT
void asm_testApp(void) {
   asm {
      rts
      nop
   } 
}
#else
#define TEST 5
#if TEST == 1
// Unpaged Flash address
static const uint8_t buffer[] = {0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xAA,0xBB,0xFF,0xFF,0xFF,0xFF,
                                 0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xAA,0xBB,0xFF,0xFF,0xFF,0xFF}; 

static const FlashData_t flashdataA = {
   /* flags      */ DO_INIT_FLASH|DO_ERASE_RANGE|DO_BLANK_CHECK_RANGE|DO_PROGRAM_RANGE|DO_VERIFY_RANGE,
   /* errorCode  */ 0xAA55,
   /* controller */ FLASH_REG_BASE,
   /* frequency  */ 8000, /* nominally 8MHz */
   /* sectorSize */ FLASH_SECTOR_SIZE,
   /* res        */ 0,
   /* page       */ 0x00,
   /* address    */ 0xC000,
   /* size       */ sizeof(buffer),
   /* data       */ (uint16_t *)buffer,
};
#elif TEST == 2
// Paged Flash address
static const uint8_t buffer[] = {0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xAA,0xBB,0xFF,0xFF,0xFF,0xFF,
                                 0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xAA,0xBB,0xFF,0xFF,0xFF,0xFF}; 

static const FlashData_t flashdataA = {
   /* flags      */ DO_INIT_FLASH|DO_ERASE_RANGE|DO_BLANK_CHECK_RANGE|DO_PROGRAM_RANGE|DO_VERIFY_RANGE,
   /* errorCode  */ 0xAA55,
   /* controller */ FLASH_REG_BASE,
   /* frequency  */ 8000, /* nominally 8MHz */
   /* sectorSize */ FLASH_SECTOR_SIZE,
   /* res        */ 0,
   /* page       */ 0x31,
   /* address    */ 0x8000,
   /* size       */ sizeof(buffer),
   /* data       */ (uint16_t *)buffer,
};
#elif TEST == 3
// Paged EEPROM (or DFlash) address
static const uint8_t buffer[] = {0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xAA,0xBB,0xFF,0xFF,0xFF,0xFF,
                                 0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xAA,0xBB,0xFF,0xFF,0xFF,0xFF}; 

static const FlashData_t flashdataA = {
   /* flags      */ DO_INIT_FLASH|DO_ERASE_RANGE|DO_BLANK_CHECK_RANGE|DO_PROGRAM_RANGE|DO_VERIFY_RANGE,
   /* errorCode  */ 0xAA55,
   /* controller */ FLASH_REG_BASE,
   /* frequency  */ 8000, /* nominally 8MHz */
   /* sectorSize */ FLASH_SECTOR_SIZE,
   /* res        */ 0,
   /* page       */ 0xFF,
   /* address    */ 0x0800,
   /* size       */ sizeof(buffer),
   /* data       */ (uint16_t *)buffer,
};
#elif TEST == 4
// Fixed EEPROM (or DFlash) address
static const uint8_t buffer[] = {0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xAA,0xBB,0xFF,0xFF,0xFF,0xFF,
                                 0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xAA,0xBB,0xFF,0xFF,0xFF,0xFF}; 

static const FlashData_t flashdataA = {
   /* flags      */ DO_INIT_FLASH|DO_ERASE_RANGE|DO_BLANK_CHECK_RANGE|DO_PROGRAM_RANGE|DO_VERIFY_RANGE,
   /* errorCode  */ 0xAA55,
   /* controller */ FLASH_REG_BASE,
   /* frequency  */ 8000, /* nominally 8MHz */
   /* sectorSize */ FLASH_SECTOR_SIZE,
   /* res        */ 0,
   /* page       */ 0x00,
   /* address    */ 0x0C00,
   /* size       */ sizeof(buffer),
   /* data       */ (uint16_t *)buffer,
};
#elif TEST == 5
// Programming general flash
static const uint8_t buffer[] = {0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xAA,0xBB,0xCC,0xDD,0xEE,0xFF,
                                 0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}; 
static const FlashData_t flashdataA = {
   /* flags      */ DO_INIT_FLASH|DO_ERASE_RANGE|DO_BLANK_CHECK_RANGE|DO_PROGRAM_RANGE|DO_VERIFY_RANGE,
   /* errorCode  */ 0xAA55,
   /* controller */ FLASH_REG_BASE,
   /* frequency  */ 4000, /* nominally 8MHz */
   /* sectorSize */ FLASH_SECTOR_SIZE,
   /* res        */ 0,
   /* page       */ 0x31,
   /* address    */ 0x8000,
   /* size       */ sizeof(buffer),
   /* data       */ (uint16_t *)buffer,
};
#define DO_B
static const FlashData_t flashdataB = {
   /* flags      */ DO_INIT_FLASH|DO_ERASE_RANGE|DO_BLANK_CHECK_RANGE,
   /* errorCode  */ 0xAA55,
   /* controller */ FLASH_REG_BASE,
   /* frequency  */ 4000, /* nominally 8MHz */
   /* sectorSize */ FLASH_SECTOR_SIZE,
   /* res        */ 0,
   /* page       */ 0x31,
   /* address    */ 0x8000,
   /* size       */ 0x4000,
   /* data       */ (uint16_t*)0x1304,
};
#elif TEST == 6
// Programming general flash
static const uint8_t buffer[] = {0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xAA,0xBB,0xCC,0xDD,0xEE,0xFF,
                                 0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}; 
static const FlashData_t flashdataA = {
   /* flags      */ DO_INIT_FLASH|DO_ERASE_RANGE|DO_BLANK_CHECK_RANGE|DO_PROGRAM_RANGE|DO_VERIFY_RANGE,
   /* errorCode  */ 0xAA55,
   /* controller */ FLASH_REG_BASE,
   /* frequency  */ 4000, /* nominally 8MHz */
   /* sectorSize */ FLASH_SECTOR_SIZE,
   /* res        */ 0,
   /* page       */ 0x31,
   /* address    */ 0xC000,
   /* size       */ sizeof(buffer),
   /* data       */ (uint16_t *)buffer,
};
#define DO_B
static const FlashData_t flashdataB = {
   /* flags      */ DO_INIT_FLASH|DO_ERASE_RANGE|DO_BLANK_CHECK_RANGE|DO_PROGRAM_RANGE|DO_VERIFY_RANGE,
   /* errorCode  */ 0xAA55,
   /* controller */ FLASH_REG_BASE,
   /* frequency  */ 4000, /* nominally 8MHz */
   /* sectorSize */ FLASH_SECTOR_SIZE,
   /* res        */ 0,
   /* page       */ 0xF8,
   /* address    */ 0xC200,
   /* size       */ sizeof(buffer),
   /* data       */ (uint16_t *)buffer,
};
#define DO_C
static const FlashData_t flashdataC = {
   /* flags      */ DO_INIT_FLASH|DO_ERASE_RANGE|DO_BLANK_CHECK_RANGE|DO_PROGRAM_RANGE|DO_VERIFY_RANGE,
   /* errorCode  */ 0xAA55,
   /* controller */ FLASH_REG_BASE,
   /* frequency  */ 4000, /* nominally 8MHz */
   /* sectorSize */ FLASH_SECTOR_SIZE,
   /* res        */ 0,
   /* page       */ 0xF8,
   /* address    */ 0xC440,
   /* size       */ sizeof(buffer),
   /* data       */ (uint16_t *)buffer,
};
#define DO_D
static const FlashData_t flashdataD = {
   /* flags      */ DO_INIT_FLASH|DO_ERASE_RANGE|DO_BLANK_CHECK_RANGE|DO_PROGRAM_RANGE|DO_VERIFY_RANGE,
   /* errorCode  */ 0xAA55,
   /* controller */ FLASH_REG_BASE,
   /* frequency  */ 4000, /* nominally 8MHz */
   /* sectorSize */ FLASH_SECTOR_SIZE,
   /* res        */ 0,
   /* page       */ 0xF8,
   /* address    */ 0xFF00,
   /* size       */ sizeof(buffer),
   /* data       */ (uint16_t *)buffer,
};
#endif

#pragma MESSAGE DISABLE C12056
#pragma NO_RETURN
//! Dummy test program for debugging
void testApp(void) {
   FlashProgramHeader_t *fph = (FlashProgramHeader_t*) &gFlashProgramHeader;
   
   // Disable COP
   *(gFlashProgramHeader.copctlAddress) = 0;
   
   fph->flashData = (FlashData_t *)&flashdataA;
   fph->entry();
#ifdef DO_B
   fph->flashData = (FlashData_t *)&flashdataB;
   fph->entry();
#endif
#ifdef DO_C
   fph->flashData = (FlashData_t *)&flashdataC;
   fph->entry();
#endif
#ifdef DO_D
   fph->flashData = (FlashData_t *)&flashdataD;
   fph->entry();
#endif
#ifdef DO_E
   fph->flashData = (FlashData_t *)&flashdataE;
   fph->entry();
#endif
   asm {
      bgnd
   }
}
#pragma MESSAGE DEFAULT C12056

//!
//!
#pragma NO_ENTRY
#pragma NO_EXIT
#pragma MESSAGE DISABLE C12053
void asm_testApp(void) {
asm {
  
   // Disable interrupts
   sei
   
   // Setup the stack before we attempt anything else
   leas   stackSpace:STACK_SIZE-1,PCR
   
   leax   stackSpace,PCR
   ldaa   #STACK_SIZE
   ldab   #0xFF
clrLoop:
   stab   1,x+
   dbne   a,clrLoop
   
   jmp     testApp,pcr
}
}
#pragma MESSAGE DEFAULT C12053
#endif
