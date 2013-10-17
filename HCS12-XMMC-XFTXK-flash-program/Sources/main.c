//!
//! Flash programming code (HCS12-XMMC-XFTXK-flash-program)
//!
//! Families  (MC9S12XD)
//!   MC9S12XA
//!   MC9S12XB
//!   MC9S12XD
//!   MC9S12XDG
//!   MC9S12XDP
//!   MC9S12XDQ
//!   MC9S12XDT
//!
//! CPU
//!   CPU12X
//!
//! Global memory management (GPAGE=0x0010)
//!   S12XMMCV2
//!   S12XMMCV3
//!
//! Global flash programming (EETX & FTX via global memory writes)
//! Paged addresses are mapped to Global for programming
//! CPU Memory accesses use GPAGE and CPU12X-extensions
//!
//!   S12XEETX2KV1   -   2K EEPROM (512x4B sectors)
//!   S12XEETX4KV2   -   4K EEPROM (1024x4B sectors)
//!   S12XFTX512K4V2 - 512K Flash  (4x128KB blocks, each 128x1024B sectors)
//!   S12XFTX256K2V1 - 256K Flash  (2x128KB blocks, each 128x1024B sectors)
//!   S12XFTX128K1V1 - 128K Flash  (1x128KB blocks, each 128x1024B sectors)
//!
//! This code accepts Global addresses (bit 31=1) or paged addresses
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

#define GPAGE 0x10

#ifdef DEBUG
#define FLASH_REG_BASE              ((FlashController*) 0x0100)  // For testing
#define EEPROM_REG_BASE             ((FlashController*) 0x0110)  // For testing
#define COPCTL_ADDRESS              ((uint8_t *) 0x03C)
// This is the smallest unit of Flash that can be erased
#define FLASH_SECTOR_SIZE   (1024) // Flash 1024 byte size (used for stride in erase)
#define EEPROM_SECTOR_SIZE  (4)    // EEPROM 4 bytes
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

#define FSTAT_CBEIF    (1<<7)  //!< Command buffer empty
#define FSTAT_CCIF     (1<<6)  //!< Command complete
#define FSTAT_PVIOL    (1<<5)  //!< Protection violation
#define FSTAT_ACCERR   (1<<4)  //!< Access error
#define FSTAT_BLANK    (1<<2)  //!< Blank check OK
#define FSTAT_FAIL     (1<<1)  //!< Flash Operation failed (blank only?)
#define FSTAT_DONE     (1<<0)  //!< Flash operation complete

#define FTSTMOD_WRALL  (1<<4)

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
   uint32_t         address;           // 10: Linear memory address being accessed
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
     /* clockFactor    */ 0,
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

#pragma NO_ENTRY
#pragma NO_EXIT
#pragma NO_RETURN
#pragma MESSAGE DISABLE C1404
#pragma MESSAGE DISABLE C5703
//! Reads a value from a global address (using PPAGE)
//!
//! @param gAddress - (X:D) 32-bit global address
//!
//! @return (D) 16-bit value read from global address
//!
uint16_t readGlobalWord(uint32_t gAddress) {
   // X:D = gAaddress
   asm {
      xgdx         // Now X=offset, B=GPAGE, A=XX
      stab GPAGE
      gldd 0,x
      rts
   }
}
#pragma MESSAGE DEFAULT C5703
#pragma MESSAGE DEFAULT C1404

#pragma NO_ENTRY
#pragma NO_EXIT
#pragma NO_RETURN
#pragma MESSAGE DISABLE C1404
#pragma MESSAGE DISABLE C5703
void writeLinearWord(uint16_t data, uint32_t address) {
   // D:X = address
   asm {
      xgdx         // Now X=offset, B=GPAGE, A=XX
      stab GPAGE
      ldd  2,sp    // data
      gstd 0,x
      rts
   }
}
#pragma MESSAGE DEFAULT C5703
#pragma MESSAGE DEFAULT C1404

#pragma NO_ENTRY
#pragma NO_EXIT
#pragma NO_RETURN
#pragma MESSAGE DISABLE C1404
#pragma MESSAGE DISABLE C5703
//! Translates a local/global address to global
//!
//! @param address - (D) ptr to 32-bit local/global address
//!
//! @note bit#31 indicates a global address needing no translation
void makeLinearAddress(uint32_t *address) {
   // D = address
   asm {
      tfr d,x   
      // X = address
      // 0,x     - MSB indicates already global address
      // 1,x     - page number
      // 2,x 3,x - virtual address/page offset
      //
      brset 0,x,#0x80,done          // Already global - done
      
      ldd   1,x                     // A = PPAGE, B = addr[15:8]
      
      cmpb  #0x40                   // Try 0x0000-0x3FFF
      blo   tryDFlash               // Yes - try D-flash
                                    
      ldaa  #0xFD                   // Try 0x4000-0x7FFF, equivalent page=0xFD
      cmpb  #0x80
      blo   checkUnPaged            // Yes - check unpaged

      ldaa  1,x                     // Try 0x8000-0xBFFF, page is dynamic
      cmpb  #0xC0
      blo   doPFlash                // Yes - paged P-flash

      ldaa  #0xFF                   // Must be 0xC000-0xFFFF, equivalent page=0xFF
                                    // Check unpaged

   checkUnPaged:
      // 0x4000-0x7FFF & 0xC000-0xFFFF should have page# = 0
      tst   1,x
      bne   error
            
   doPFlash:
      // P-Flash
      // A=PPage #, B=Address[15:8]
      // G-address[23:16] <= 01:PPage[7:2]
      // G-address[15:08] <= PPage[0:1]:Address[13:08]
      //
      lslb
      lslb
      lsrd
      lsrd         // B = PPage[0:1]:Address[13:08]
      oraa  #0x40  // A = 01:Page[7:2]
      bra   setGAddress
      
   tryDFlash:

      // A=PPage #, B=Address[15:8]
      cmpb  #0x08                  // Check if 0x0800-0x0CFF
      blo   error                  // <0x0800? Yes  - error
      cmpb  #0x0C
      blo   doDFlash               // <0x0C00? Yes  - mapped DFlash
      cmpb  #0x10
      bhs   error                  // >=0x1000  - error
      tsta                         // Page # must be 0 for non-paged 0x0C00-0x0FFF
      bne   error                   
      ldaa  #0xFF                  // Fixed page = page #0xFF

   doDFlash:
      // D-Flash
      // A=EPage #, B=Address[15:8]
      // G-address[23:16] <= 000100:EPage[7:6]         (in 1,x)
      // G-address[15:08] <= EPage[5:0]:Address[09:08] (in 2,x)
      //
      andb  #0x03 // Save Address[09:08]
      pshb
      tfr   a,b
      clra
      lsld
      lsld
      oraa  #0x10    // G-address[23:16] = 000100:EPage[7:6]
      orab  1,sp+    // G-address[15:08] = Epage[5:0]:Address[09:08]
      
   setGAddress:
      std   1,x      // Update G-address[23:08]
   done:
      clr   0,x      // Clear global indicator      
      rts
      
   error:
      ldab  #FLASH_ERR_ILLEGAL_PARAMS
      jmp   setErrorCode,pcr
   }

}
#pragma MESSAGE DEFAULT C5703
#pragma MESSAGE DEFAULT C1404

//! Does any initialisation required before accessing the Flash
//!
void initFlash(void) {
   FlashController *controller;

   FlashData_t     *flashData;  
   asm {
      ldx   gFlashProgramHeader:FLASH_DATA_OFFSET,PCR   // flashData
      stx   flashData
   }
   if ((flashData->flags&DO_INIT_FLASH) == 0) {
      return;
   }
   controller = flashData->controller;

#ifdef DEBUG
   controller->fclkdiv = 0x0F;
#endif
   
   // Check if clock divider correctly set
   if ((controller->fclkdiv & CFMCLKD_DIVLD) == 0) {
      setErrorCode(FLASH_ERR_CLKDIV);
   }
   controller->fprot  = 0xFF;  // Unprotect flash/EEPROM
   
   flashData->flags &= ~DO_INIT_FLASH;
   return;
}

//! Launch & wait for Flash command to complete
//!
void doFlashCommand(void) {
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
      nop // allow FSTAT_CBEIF to clear
      nop
      nop
      nop
   }
   // Wait for command complete
   do {
      fstat = *pFstat;
   } while ((fstat&(FSTAT_CCIF|FSTAT_ACCERR|FSTAT_PVIOL)) == 0);
   if ((fstat & FSTAT_ACCERR ) != 0)
      setErrorCode(FLASH_ERR_PROG_ACCERR);
   if ((fstat & FSTAT_PVIOL ) != 0)
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
   // Buffer dummy address & data
   controller->faddr   = 0x0000;
   controller->fdata   = 0xFFFF;
   
   // Write command
   controller->fcmd = FCMD_MASS_ERASE;
   
   doFlashCommand();

   flashData->flags &= ~DO_MASS_ERASE;
}

//! Program a range of flash from buffer
//!
void programRange(void) {
   FlashController *controller;
   uint32_t         address;
   const uint16_t  *data;
   uint16_t         numWords;
   uint8_t          fstat;
   
   FlashData_t     *flashData;  
   asm {
      ldd   gFlashProgramHeader:FLASH_DATA_OFFSET,PCR   // flashData
      std   flashData
   }
   if ((flashData->flags&DO_PROGRAM_RANGE) == 0)
      return;
   
   controller = flashData->controller;
   address    = flashData->address;
   data       = flashData->data;
   numWords   = flashData->size/2;

   if ((address & 0x1) != 0)
      setErrorCode(FLASH_ERR_ILLEGAL_PARAMS);

   // Clear any existing errors
   controller->fstat   = FSTAT_ACCERR|FSTAT_PVIOL;
   controller->ftstmod = 0;
   controller->fstat   = FSTAT_FAIL;
   //controller->fcnfg   = 0;

   // Program words
   while (numWords-- > 0) {
      // Write data to flash global address
      writeLinearWord(*data, address);

      // Set command
      controller->fcmd = FCMD_WORD_PROGRAM;

      // Launch command
      controller->fstat = FSTAT_CBEIF;
      asm {
         nop // allow FSTAT_CBEIF to clear
         nop
         nop
         nop
      }
      // Wait for buffer empty or error
      do {
         fstat = controller->fstat;
      } while ((fstat&(FSTAT_CBEIF|FSTAT_ACCERR|FSTAT_PVIOL)) == 0);
      if ((fstat & (FSTAT_ACCERR|FSTAT_PVIOL)) != 0)
         break;
      address += 2;
      data++;
   }
   // Wait for last command complete
   do {
      fstat = controller->fstat;
   } while ((fstat&(FSTAT_CCIF|FSTAT_ACCERR|FSTAT_PVIOL)) == 0);
   if ((fstat & FSTAT_ACCERR) != 0)
      setErrorCode(FLASH_ERR_PROG_ACCERR);
   if ((fstat & FSTAT_PVIOL) != 0)
      setErrorCode(FLASH_ERR_PROG_FPVIOL);

   flashData->flags &= ~DO_PROGRAM_RANGE;
}

//! Verify a range of flash against buffer
//!
void verifyRange(void) {
   uint32_t        address;
   const uint16_t *data;
   uint16_t        numWords;

   FlashData_t    *flashData;  
   asm {
      ldd   gFlashProgramHeader:FLASH_DATA_OFFSET,PCR   // flashData
      std   flashData
   }
   if ((flashData->flags&DO_VERIFY_RANGE) == 0)
      return;
   
   address   = flashData->address;
   data      = flashData->data;
   numWords  = flashData->size/2;

   if (((uint8_t)address & 0x01) != 0) {
      setErrorCode(FLASH_ERR_ILLEGAL_PARAMS);
   }
   // Verify words
   while (numWords-- > 0) {
      if (readGlobalWord(address) != *data++)
         setErrorCode(FLASH_ERR_VERIFY_FAILED);
      address += 2;
   }
   flashData->flags &= ~DO_VERIFY_RANGE;
}

//! Erase a range of flash
//!
void eraseRange(void) {
   FlashController *controller;
   uint32_t address;
   uint32_t endAddress;
   uint16_t sectorSize;
   uint32_t pageMask;
   
   FlashData_t     *flashData;  
   asm {
      ldd   gFlashProgramHeader:FLASH_DATA_OFFSET,PCR   // flashData
      std   flashData
   }
   if ((flashData->flags&DO_ERASE_RANGE) == 0) {
      return;
   }
   controller  = flashData->controller;
   address     = flashData->address;
   endAddress  = address + (flashData->size-1); // Inclusive
   sectorSize  = flashData->sectorSize;
   pageMask    = 0xFFFF0000UL|~(sectorSize-1U);
   
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
   //controller->fcnfg   = 0;

   // Erase each sector
   while (address != endAddress) {
      // Write dummy data to flash global address
      writeLinearWord(0xFFFF, address);
      
      // Write command
      controller->fcmd = FCMD_PAGE_ERASE;

      doFlashCommand();

      // Advance to start of next sector
      address += sectorSize;
   }
   flashData->flags &= ~DO_ERASE_RANGE;
}

//! Check that a range of flash is blank (=0xFFFF)
//!
void blankCheckRange(void) {
   uint32_t address;
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
      if (readGlobalWord(address) != 0xFFFF)
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
#define STACK_SIZE 40
volatile const uint8_t stackSpace[STACK_SIZE];

//! Main C entry point
//!
void entry(void) {
   const FlashProgramHeader_t *theFlashProgramHeader;
   FlashData_t                *flashData;  
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
   
   // Make address linear if needed
   makeLinearAddress(&flashData->address);
   
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
      
      jsr     entry,PCR
      
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
   } 
}
#else
#define TEST 4
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
   /* address    */ 0x0000C000,
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
   /* address    */ 0xF88000,
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
   /* controller */ EEPROM_REG_BASE,
   /* frequency  */ 8000, /* nominally 8MHz */
   /* sectorSize */ EEPROM_SECTOR_SIZE,
   /* address    */ 0xFC0800,
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
   /* controller */ EEPROM_REG_BASE,
   /* frequency  */ 8000, /* nominally 8MHz */
   /* sectorSize */ EEPROM_SECTOR_SIZE,
   /* address    */ 0x000C00,
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
   /* sectorSize */ EEPROM_SECTOR_SIZE,
   /* address    */ 0xFF0800,
   /* size       */ sizeof(buffer),
   /* data       */ (uint16_t *)buffer,
};
#define DO_B
static const FlashData_t flashdataB = {
   /* flags      */ DO_INIT_FLASH|DO_ERASE_RANGE|DO_BLANK_CHECK_RANGE|DO_PROGRAM_RANGE|DO_VERIFY_RANGE,
   /* errorCode  */ 0xAA55,
   /* controller */ FLASH_REG_BASE,
   /* frequency  */ 4000, /* nominally 8MHz */
   /* sectorSize */ PFLASH_SECTOR_SIZE,
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
   /* sectorSize */ PFLASH_SECTOR_SIZE,
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
   /* sectorSize */ PFLASH_SECTOR_SIZE,
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
   pula   // SP should point off end of stack space
   
   jmp     testApp,pcr
}
}
#pragma MESSAGE DEFAULT C12053
#endif
