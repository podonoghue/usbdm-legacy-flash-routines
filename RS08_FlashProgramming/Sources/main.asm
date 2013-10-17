     ; Include derivative-specific definitions
     INCLUDE 'derivative.inc'

     XDEF         Start
     ABSENTRY     Start

;********************************************************************************
; Fast RAM ($0 - $D)
                  org      $0
;            
; Programming parameters (values given are dummies for testing)
;
; This area of memory is re-written for each block written to flash.
; The last byte (ByteCount) is used as a flag to control execution as well as the
; source buffer count.  It is the last byte written by the BDM to start programming.
; The values must lie within a single flash page (64 byte modulo)
;
; Example values are for a write of 6 bytes [$11,$22,$33,$44,$55,$66] => [$3F3A..$3F3F]
;
;Destination       equ      $3F3A
Destination       equ      $3000
FLCRAddress       dc.b     HIGH_6_13(FLCR)                 ; Page # for FLCR Reg
   if RAMEnd < RAMStart+$30
Buffer            dc.b     $11,$22,$33,$44,$55,$66,$77,$88 ; Buffer for data to program (<= 8 bytes)
BufferSize        equ *-Buffer
   else
                  ds.b     8                               ; Skip unused buffer
   endif
DestinationAddr   dc.b     MAP_ADDR_6(Destination)         ; Address in Paging Window for destination
DestinationPage   dc.b     HIGH_6_13(Destination)          ; Page for destination
DelayValue        dc.b     20                              ; Delay value (4MHz=>N=20, 8MHz=>N=40)
SourceAddr        dc.b     Buffer                          ; Address in Zero Page buffer for source
ByteCount         dc.b     BufferSize                      ; # of byte to write <page & buffer size
  
;********************************************************************************
; Flash programming code in Z-Page RAM
;            
      org   RAMStart

Start:
      clr   ByteCount                  ; Set Idle mode
      
Idle:      
      lda   ByteCount
      beq   Idle                       ; Wait for some bytes to program

      lda   FLCRAddress
      sta   PAGESEL                    ; 
      bset  FLCR_PGM,MAP_ADDR_6(FLCR)  ; Enable Flash programming

      lda   DestinationPage            ; Set window to page to program
      sta   PAGESEL
      clr   MAP_ADDR_6(0)              ; Dummy write to Flash block

      bsr   delay30us                  ; Wait Tnvs

      bset  FLCR_HVEN,MAP_ADDR_6(FLCR) ; Enable Flash high voltage

      bsr   delay30us                  ; Wait Tpgs

Loop:                                  ; Copy byte to flash (program)
      lda   DestinationPage            ; 2 cy - Set window to page to program
      sta   PAGESEL                    ; 2 cy 
      lda   SourceAddr                 ; 3 cy
      sta   X                          ; 2 cy    
      lda   D[X]                       ; 3 cy  
      mov   DestinationAddr,X          ; 5 cy  
      sta   D[X]                       ; 2 cy  
                                       
      bsr   delay30us                  ; 3 cy    - Wait Tprog
      
      inc   DestinationAddr            ; 4 cy    - Update ptrs/counter
      inc   SourceAddr
      dbnz  ByteCount,Loop             ; 4 cy    - Finished? - no, loop for next byte
                                       ; 30 cy + delay
      
      bclr  FLCR_PGM,MAP_ADDR_6(FLCR)  ; Disable Flash programming

      bsr   delay30us                  ; Wait Tnvh

      bclr  FLCR_HVEN,MAP_ADDR_6(FLCR) ; Disable High voltage
      
      bra   Start                      ; Back for more

;*********************************************************************
; A short delay (~30us) sufficient to satisfy all the required delays
; The PAGESEL register is set to point at the FLCR page
;
;         Tnvs  > 5us
;         Tpgs  > 10us
;         Tnvh  > 5us
;  40us > Tprog > 20us
;
; The delay may be adjusted through DelayValue 
; The values are chosen to satisy Tprog including loop overhead above.
; The other delays are less critical (minimums only) 
; Assuming bus freq = ~4 MHz clock => 250 ns, 4cy = 1us
; For 30us@4MHz, 4x30 = 120 cy, N = (4x30-30-10)/4 = 20
;
delay30us:
      lda   DelayValue              ;   3 cy
      dbnza *                       ; 4*N cy   
      lda   FLCRAddress             ;   2 cy
      sta   PAGESEL                 ;   2 cy   
      rts                           ;   3 cy
                                    ;      = 10+4*N cy
   
   if RAMEnd > RAMStart+$30 ; Place buffer here if sufficient free RAM
Buffer            dc.b     $11,$22,$33,$44,$55,$66,$77,$88 ; Buffer for data to program (<= 32 bytes)
                  ds.b     RAMEnd-*
                  dc.b     $EE
BufferSize        equ *-Buffer
   endif

;*********************************************************************
; Dummy reset vector for simulation
;
      org   $3FFD                  
      jmp   Idle

      end
