; Note: This file is recreated by the project wizard whenever the MCU is
;       changed and should not be edited by hand

; include derivative specific macros
            INCLUDE 'mc9rs08le4.inc'


; RAM/ROM definitions


; Watchdog feed macro
feed_watchdog: MACRO
            MOV    #HIGH_6_13(SRS),PAGESEL
            STA    MAP_ADDR_6(SRS)  ; feed the watchdog
          ENDM
