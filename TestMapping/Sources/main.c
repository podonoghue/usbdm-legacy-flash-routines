#define FLASH_ERR_ILLEGAL_PARAMS 1

typedef unsigned long  uint32_t;
typedef unsigned int   uint16_t;
typedef unsigned char  uint8_t;

#define S12HY 1
#define S12X  2
#define S12XS 3

#define TARGET S12HY

uint32_t gAddress;
uint32_t mappedPAddress;
uint32_t mappedGAddress;

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
      bgnd
   }
}
#pragma MESSAGE DEFAULT C1404
#pragma MESSAGE DEFAULT C5703

#if TARGET == S12HY
#pragma NO_ENTRY
#pragma NO_EXIT
#pragma NO_RETURN
#pragma MESSAGE DISABLE C1404
#pragma MESSAGE DISABLE C5703
uint16_t readLinearWord(uint32_t gAddress) {
   // X:D = gAaddress
   asm {
      psha         // Save GAddress[15:8]
      anda #0x3F
      oraa #0x80
      xgdx         // Now X=LAddress[15:0], B=GAddress[17:16], A=XX
      tfr  b,a
      pulb
      lsld
      lsld         // A=ppage[3:0]
      clr  mappedPAddress:0
      staa mappedPAddress:1
      stx  mappedPAddress:2
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
//! Translates a local address to global
//!
//! @param address - 24-bit paged address
//!
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
      
      ldd   1,x                     // A = PPage, B = addr[15:8]
      
      cmpb  #0x40                   // Try 0x0000-0x3FFF
      blo   tryDFlash               // Yes - try D-flash
                                    
      ldaa  #0x0D                   // Try 0x4000-0x7FFF, equivalent page=0x0D
      cmpb  #0x80
      blo   checkUnPaged            // Yes - check unpaged

      ldaa  1,x                     // Try 0x8000-0xBFFF, page is dynamic
      cmpb  #0xC0
      blo   doPFlash                // Yes - paged P-flash

      ldaa  #0x0F                   // Must be 0xC000-0xFFFF, equivalent page=0x0F
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
                   // A = Page[7:2]
      bra   setGAddress
      
   tryDFlash:

      // A=PPage #, B=Address[15:8]
      tsta                         // Page # must be 0 for non-paged 0x0000-0x3FFF
      bne   error                   
      cmpb  #0x04                  // <0x0400
      blo   error                  // No  - error (Registers)
      ldaa  #0x0C                  // PPage for unpaged P-Flash  0x1400-
      cmpb  #0x14                  // >=0x1400
      bhs   doPFlash               // No  - error

   doDFlash:
      // D-Flash
      // A=0, B=Address[15:8]
      // G-address[23:16] <= 00000000            (in 1,x)
      // G-address[15:08] <= 0x40+Address[15:08] (in 2,x)
      //
      clra        // G-Address[23:16]
      addb  #0x40 // G-Address[15:08]
      
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


uint32_t pagedTestAddresses[] = {
   0x000400, 0x0013FF, 0x001123,   // D-flash          (page #0x00)
   0x001400, 0x0014FF, 0x001523,   // P-flash unmapped (page #0x0C)
   0x004000, 0x007FFF, 0x006123,   // P-flash unmapped (page #0x0D) 
   0x008000, 0x00BFFF, 0x00A123,   // P-flash mapped   (page #0x00)
   0x0E8000, 0x0EBFFF, 0x0EA123,   // P-flash mapped   (page #0x0E)
   0x00C000, 0x00FFFF, 0x00E123,   // P-flash unmapped (page #0x0F) 
   0x00FF00,                       // P-flash unmapped (page #0x0F) 
};

uint32_t globalTestAddresses[] = {
   0x004400, 0x0053FF, 0x005123,   // D-flash          (page #0x00)
   0x031400, 0x0314FF, 0x031523,   // P-flash unmapped (page #0x0C)
   0x034000, 0x037FFF, 0x036123,   // P-flash unmapped (page #0x0D) 
   0x000000, 0x003FFF, 0x002123,   // P-flash mapped   (page #0x00)
   0x038000, 0x03BFFF, 0x03A123,   // P-flash mapped   (page #0x0E)
   0x03C000, 0x03FFFF, 0x03E123,   // P-flash unmapped (page #0x0F) 
   0x03FF00,                       // P-flash unmapped (page #0x0F) 
};
#endif

#if TARGET == S12X
#pragma NO_ENTRY
#pragma NO_EXIT
#pragma NO_RETURN
#pragma MESSAGE DISABLE C1404
#pragma MESSAGE DISABLE C5703
//! Translates a local address to global
//!
//! @param address - 24-bit paged address
//!
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
      
      ldd   1,x                     // A = ppage, B = addr[15:8]
      
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
      // A=ppage #, B=Address[15:8]
      // G-address[23:16] <= 01:ppage[7:2]
      // G-address[15:08] <= ppage[0:1]:Address[13:08]
      //
      lslb
      lslb
      lsrd
      lsrd         // B = ppage[0:1]:Address[13:08]
      oraa  #0x40  // A = 01:Page[7:2]
      bra   setGAddress
      
   tryDFlash:

      // A=ppage #, B=Address[15:8]
      cmpb  #0x08                  // Check if 0x0800-0x0CFF
      blo   error                  // No  - error
      cmpb  #0x10
      bhi   error                  // No  - error
      cmpb  #0x0C                  // Check if fixed page
      blo   doDFlash               // No - skip
      tsta                         // Page # must be 0 for non-paged
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
      oraa  #0x10  // G-address[23:16] = 000100:EPage[7:6]
      orab  1,sp+  // G-address[15:08] = Epage[5:0]:Address[09:08]
      
   setGAddress:
      std   1,x    // Update G-address[23:08]
   done:
      clr   0,x    // Clear global indicator      
      rts
      
   error:
      ldab  #FLASH_ERR_ILLEGAL_PARAMS
      jmp   setErrorCode,pcr
   }

}
#pragma MESSAGE DEFAULT C5703
#pragma MESSAGE DEFAULT C1404


uint32_t unMapped[] = {
   0xFC0800, 0xFC0BFF, 0xFC0923,   // EEPROM mapped   (page 0xFC) 
   0xFD0800, 0xFD0BFF, 0xFD0923,   // EEPROM mapped   (page 0xFD) 
   0x000C00, 0x000FFF, 0x000D23,   // EEPROM unmapped (page 0xFF)
    
   0x004000, 0x007FFF, 0x006123,   // P-flash unmapped (page 0xFD) 
   0x008000, 0x00BFFF, 0x00A123,   // P-flash mapped   (page 0x00)
   0xE08000, 0xE0BFFF, 0xE0A123,   // P-flash mapped   (page 0xE0)
   0xFF8000, 0xFFBFFF, 0xFFA123,   // P-flash mapped   (page 0xFF
   0x00C000, 0x00FFFF, 0x00E123,   // P-flash unmapped (page 0xFF) 
};

uint32_t mapped[] = {
   0x13F000, 0x13F3FF, 0x13F123,   // EEPROM mapped   (page 0xFC) 
   0x13F400, 0x13F7FF, 0x13F523,   // EEPROM mapped   (page 0xFD) 
   0x13FC00, 0x13FFFF, 0x13FD23,   // EEPROM unmapped (page 0xFF)
    
   0x7F4000, 0x7F7FFF, 0x7F6123,   // P-flash unmapped (page 0xFD) 
   0x400000, 0x403FFF, 0x402123,   // P-flash mapped   (page 0x00)
   0x780000, 0x783FFF, 0x782123,   // P-flash mapped   (page 0xE0)
   0x7FC000, 0x7FFFFF, 0x7FE123,   // P-flash mapped   (page 0xFF
   0x7FC000, 0x7FFFFF, 0x7FE123,   // P-flash unmapped (page 0xFF) 
};
#endif

#if TARGET == S12XS
#pragma NO_ENTRY
#pragma NO_EXIT
#pragma NO_RETURN
#pragma MESSAGE DISABLE C1404
#pragma MESSAGE DISABLE C5703
//! Translates a local address to global
//!
//! @param address - 24-bit paged address
//!
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
      
      ldd   1,x                     // A = ppage, B = addr[15:8]
      
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
      // A=ppage #, B=Address[15:8]
      // G-address[23:16] <= 01:ppage[7:2]
      // G-address[15:08] <= ppage[0:1]:Address[13:08]
      //
      lslb
      lslb
      lsrd
      lsrd         // B = ppage[0:1]:Address[13:08]
      oraa  #0x40  // A = 01:Page[7:2]
      bra   setGAddress
      
   tryDFlash:

      // A=ppage #, B=Address[15:8]
      cmpb  #0x08                  // Check if 0x0800-0x0CFF
      blo   error                  // No  - error
      cmpb  #0x0B
      bhi   error                  // No  - error

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
      oraa  #0x10  // G-address[23:16] = 000100:EPage[7:6]
      orab  1,sp+  // G-address[15:08] = Epage[5:0]:Address[09:08]
      
   setGAddress:
      std   1,x    // Update G-address[23:08]
   done:
      clr   0,x    // Clear global indicator      
      rts
      
   error:
      ldab  #FLASH_ERR_ILLEGAL_PARAMS
      jmp   setErrorCode,pcr
   }

}
#pragma MESSAGE DEFAULT C5703
#pragma MESSAGE DEFAULT C1404

uint32_t unMapped[] = {
   0x000800, 0x000BFF, 0x000923,   // D-flash mapped   (page 0x00) 
   0x010800, 0x010BFF, 0x010923,   // D-flash mapped   (page 0x01) 
   0xFC0800, 0xFC0BFF, 0xFC0923,   // D-flash mapped   (page 0xFC) 
   0xFD0800, 0xFD0BFF, 0xFD0923,   // D-flash mapped   (page 0xFD) 
//   0x000C00, 0x000FFF, 0x000D23,   // D-flash unmapped (page 0xFF)
    
   0x004000, 0x007FFF, 0x006123,   // P-flash unmapped (page 0xFD) 
   0x008000, 0x00BFFF, 0x00A123,   // P-flash mapped   (page 0x00)
   0x018000, 0x01BFFF, 0x01A123,   // P-flash mapped   (page 0x01)
   0xE08000, 0xE0BFFF, 0xE0A123,   // P-flash mapped   (page 0xE0)
   0xFF8000, 0xFFBFFF, 0xFFA123,   // P-flash mapped   (page 0xFF
   0x00C000, 0x00FFFF, 0x00E123,   // P-flash unmapped (page 0xFF) 
};

uint32_t mapped[] = {
   0x100000, 0x1003FF, 0x100123,   // D-flash mapped   (page 0x00) 
   0x100400, 0x1007FF, 0x100523,   // D-flash mapped   (page 0x01) 
   0x13F000, 0x13F3FF, 0x13F123,   // D-flash mapped   (page 0xFC) 
   0x13F400, 0x13F7FF, 0x13F523,   // D-flash mapped   (page 0xFD) 
//   0x13FC00, 0x13FFFF, 0x13FD23,   // D-flash unmapped (page 0xFF)
    
   0x7F4000, 0x7F7FFF, 0x7F6123,   // P-flash unmapped (page 0xFD) 
   0x400000, 0x403FFF, 0x402123,   // P-flash mapped   (page 0x00)
   0x404000, 0x407FFF, 0x406123,   // P-flash mapped   (page 0x01)
   0x780000, 0x783FFF, 0x782123,   // P-flash mapped   (page 0xE0)
   0x7FC000, 0x7FFFFF, 0x7FE123,   // P-flash mapped   (page 0xFF
   0x7FC000, 0x7FFFFF, 0x7FE123,   // P-flash unmapped (page 0xFF) 
};
#endif

int index;

void main(void) {
   uint32_t temp;
   for (index=0; index<(sizeof(globalTestAddresses)/sizeof(globalTestAddresses[0])); index++) {
      gAddress = globalTestAddresses[index];
      (void)readLinearWord(gAddress);
      temp = mappedPAddress;
      makeLinearAddress(&temp);
      mappedGAddress = temp;      
      if (gAddress != mappedGAddress) {
         asm("bgnd");  
      }
   }  
   for (index=0; index<(sizeof(globalTestAddresses)/sizeof(globalTestAddresses[0])); index++) {
      makeLinearAddress(&pagedTestAddresses[index]);
      if (pagedTestAddresses[index] != globalTestAddresses[index]) {
         asm("bgnd");  
      }
   }  
   for(;;) {
      asm("bgnd");  
   }
}
     
     