     ; Include derivative-specific definitions
     INCLUDE 'derivative.inc'

     ABSENTRY   Start

PAGEWINDOW  equ  $00C0

;********************************************************************************
; Fast RAM ($0 - $D)
                  org   $0
;            
; Programming parameters (values given are dummies for testing)
;
Destination    equ   $3F11

; Timing constants for Flash Programming 
;          
; Assumes bus freq = ~4 MHz clock => 250 ns, 4cy = 1us
;
; Timing constants for Flash Programming 
;          
; Assumes bus freq = ~4 MHz clock => 250 ns, 4cy = 1us
;
TimeTnvs         dc.b  4   ; PGM/MASS to HVEN setup time   Tnvs >  5us,   12+4*N, 28cy = ~7us
                           ; HVEN hold time for PGM        Tnvh >  5us,    8+4*N, 24cy = ~6us
TimeTpgs         dc.b  6   ; HVEN to program setup time    Tpgs > 10 us,  24+4*N, 48cy = ~12us 
TimeTprog        dc.b  23  ; Byte program time             Tprog(20-40us) 29+4*N, 81cy = ~30.25us
;                                                          Tprog(20-40us) 23+4*N, 75cy = ~28.75us
TimeTnvhl        dc.b  1   ; HVEN hold time for MASS       100 us
TimeTrcv         dc.b  1   ; Recovery time                 1us
TimeTme1         dc.b  9   ; Time for Mass erase          
TimeTme2         dc.b  216 ; Time for Mass erase           N*2.306 ms ~ 500 ms

;********************************************************************************
; Flash programming code in Z-Page RAM
;            
   org   RAMStart

Start:
   mov   #HIGH_6_13(SOPT),PAGESEL       ; disable COP
   mov   #mSOPT_BKGDPE,MAP_ADDR_6(SOPT) 
                 
   if 0
;1.   Apply external VPP.
;2.   Set the MASS bit in the flash memory control register.
   mov   #HIGH_6_13(FLCR),PAGESEL   ; 4 cy
   bset  FLCR_MASS,MAP_ADDR_6(FLCR) ; 5 cy     ; Enable Flash programming
;3.   Write any data to any flash memory, via the high page accessing window $00C0-$00FF.
;     (Prior to the data writing operation, the PAGE register must be configured correctly
;     to map the high page accessing window to the any FLASH locations).
   mov   #HIGH_6_13($3FFF),PAGESEL  ; 4 cy
   mov   #$55,MAP_ADDR_6(PAGESEL)   ; 4 cy     ; Enable Flash programming
;4.   Wait for a time, tnvs.
   lda   TimeTnvs                   ; 3 cy     ; Wait Tnvs 
   dbnza *                          ; 4*N cy      
;5.   Set the HVEN bit.
   mov   #HIGH_6_13(FLCR),PAGESEL   ; 4 cy
   bset  FLCR_HVEN,MAP_ADDR_6(FLCR) ; 5 cy     ; Enable High voltage
                                    ; = 20 + 4*N = 36 cy  = 9 us > 5 us
;6.   Wait for a time Tme.
oLoop:
   mov   #11,TimeTme1               ; 4 cy
iLoop:   
   clra                             ;            1 cy
   dbnza *                          ; 4*256 cy => 1024 cy  => 256 us   
   dbnz  TimeTme1,iLoop             ; N*1024+7 cy => 11*1024+7 cy => 11,271 cy => 2.818/2.254 ms @ 4/5MHz    
   dbnz  TimeTme2,oLoop             ; ~M*(N*1024+7) cy => ~M*9223 cy = ~177*2.818/222*2.254 cy => 500 ms
      
;7.   Clear the MASS bit.
   bclr  FLCR_MASS,MAP_ADDR_6(FLCR) ; 5 cy     ; Disable High voltage
;8.   Wait for a time, tnvhl.
   lda   TimeTnvhl                  ; 3 cy     ; Wait Tnvs 
   dbnza *                          ; 4*N cy      
;9.   Clear the HVEN bit.
   bclr  FLCR_HVEN,MAP_ADDR_6(FLCR) ; 5 cy     ; Enable Flash programming
;10.  After time, trcv, the memory can be accessed in read mode again.
   lda   TimeTrcv                   ; 3 cy     ; Wait Tnvs 
   dbnza *                          ; 4*N cy      
;11.  Remove external VPP.

   else

; Test code - square wave period = ~2*Tme  
;======================================================================
   mov #$FF,PTADD
ooLoop:
   sta  PTAD

;6.   Wait for a time Tme.
oLoop:
   mov   #9,TimeTme1                ; 4 cy
iLoop:   
   clra                             ;            1 cy
   dbnza *                          ; 4*256 cy => 1024 cy  => 256 us   
   dbnz  TimeTme1,iLoop             ; N*1024+7 cy => 11*1024+7 cy => 11,271 cy => 2.818/2.254 ms @ 4/5MHz    
   dbnz  TimeTme2,oLoop             ; ~M*(N*1024+7) cy => ~M*9223 cy = ~177*2.818/222*2.254 cy => 500 ms
   lda  PTAD
   coma
   bra   ooLoop   
;=======================================================================   
   endif

   bgnd                             ; 5+ cy    ; Suspend - BDM looks for this

   org   $3FFD                      ;          ; Dummy reset vector for simulation
   jmp   RAMStart