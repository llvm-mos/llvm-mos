; RUN: split-file %s %t

; RUN: cp %p/Inputs/ca65-dummy.o %t/ca65.o
; RUN: ld.lld -m moself --cc65-launcher=%python --od65-path=%p/Inputs/od65.py --ld65-path=%p/Inputs/ld65.py -o %t/a.out %t/ca65.o
; RUN: llvm-readelf --sections %t/a.out | FileCheck %s

; RUN: cp %p/Inputs/ca65-dummy.o %t/unfinished-escape.o
; RUN: not ld.lld -m moself --cc65-launcher=%python --od65-path=%p/Inputs/od65.py %t/unfinished-escape.o 2>&1 | FileCheck --check-prefix=UNFINISHED-ESCAPE %s
; UNFINISHED-ESCAPE: unfinished-escape.o: unfinished underscore escape: _

; RUN: cp %p/Inputs/ca65-dummy.o %t/unknown-escape.o
; RUN: not ld.lld -m moself --cc65-launcher=%python --od65-path=%p/Inputs/od65.py %t/unknown-escape.o 2>&1 | FileCheck --check-prefix=UNKNOWN-ESCAPE %s
; UNKNOWN-ESCAPE: unknown-escape.o: unknown underscore escape: _g

; RUN: cp %p/Inputs/ca65-dummy.o %t/unfinished-hex-escape.o
; RUN: not ld.lld -m moself --cc65-launcher=%python --od65-path=%p/Inputs/od65.py %t/unfinished-hex-escape.o 2>&1 | FileCheck --check-prefix=UNFINISHED-HEX-ESCAPE %s
; UNFINISHED-HEX-ESCAPE: unfinished-hex-escape.o: unfinished underscore hex escape: _xa

; RUN: cp %p/Inputs/ca65-dummy.o %t/invalid-hex-escape.o
; RUN: not ld.lld -m moself --cc65-launcher=%python --od65-path=%p/Inputs/od65.py %t/invalid-hex-escape.o 2>&1 | FileCheck --check-prefix=INVALID-HEX-ESCAPE %s
; INVALID-HEX-ESCAPE: invalid-hex-escape.o: invalid underscore hex escape: _xgg

; RUN: cp %p/Inputs/ca65-dummy.o %t/unfinished-type-escape.o
; RUN: not ld.lld -m moself --cc65-launcher=%python --od65-path=%p/Inputs/od65.py %t/unfinished-type-escape.o 2>&1 | FileCheck --check-prefix=UNFINISHED-TYPE-ESCAPE %s
; UNFINISHED-TYPE-ESCAPE: unfinished-type-escape.o: unfinished underscore type escape: _t

; RUN: cp %p/Inputs/ca65-dummy.o %t/unknown-type-escape.o
; RUN: not ld.lld -m moself --cc65-launcher=%python --od65-path=%p/Inputs/od65.py %t/unknown-type-escape.o 2>&1 | FileCheck --check-prefix=UNKNOWN-TYPE-ESCAPE %s
; UNKNOWN-TYPE-ESCAPE: unknown-type-escape.o: unknown underscore type escape: _ta

; RUN: cp %p/Inputs/ca65-dummy.o %t/unfinished-flag-escape.o
; RUN: not ld.lld -m moself --cc65-launcher=%python --od65-path=%p/Inputs/od65.py %t/unfinished-flag-escape.o 2>&1 | FileCheck --check-prefix=UNFINISHED-FLAG-ESCAPE %s
; UNFINISHED-FLAG-ESCAPE: unfinished-flag-escape.o: unfinished underscore flag escape: _f

; RUN: cp %p/Inputs/ca65-dummy.o %t/unknown-flag-escape.o
; RUN: not ld.lld -m moself --cc65-launcher=%python --od65-path=%p/Inputs/od65.py %t/unknown-flag-escape.o 2>&1 | FileCheck --check-prefix=UNKNOWN-FLAG-ESCAPE %s
; UNKNOWN-FLAG-ESCAPE: unknown-flag-escape.o: unknown underscore flag escape: _fa

; CHECK: nobits NOBITS
; CHECK: progbits_then_nobits NOBITS

; CHECK:      normal PROGBITS
; CHECK-NEXT: $ PROGBITS
; CHECK-NEXT: $foo PROGBITS
; CHECK-NEXT: - PROGBITS
; CHECK-NEXT: . PROGBITS
; CHECK-NEXT: , PROGBITS
; CHECK-NEXT: / PROGBITS
; CHECK-NEXT: _ PROGBITS

; CHECK:      RODATA PROGBITS {{.*}} AR
; CHECK-NEXT: progbits PROGBITS
; CHECK-NEXT: CODE PROGBITS {{.*}} AXR
; CHECK-NEXT: LOWCODE PROGBITS {{.*}} AXR
; CHECK-NEXT: ONCE PROGBITS {{.*}} AXR
; CHECK-NEXT: STARTUP PROGBITS {{.*}} AXR
; CHECK-NEXT: exec PROGBITS {{.*}} AXR
; CHECK-NEXT: write_exec PROGBITS {{.*}} WAXR
; CHECK-NEXT: write PROGBITS {{.*}} WAR
; CHECK-NEXT: BSS NOBITS {{.*}} WAR
; CHECK-NEXT: ZEROPAGE NOBITS
; CHECK-NEXT: INIT NOBITS
; CHECK-NEXT: ZPSAVE NOBITS


;--- ca65.od65
  Segments:
    Count: 0
    Index:
      Name: "normal"
    Index:
      Name: "_d"
    Index:
      Name: "_dfoo"
    Index:
      Name: "_h"
    Index:
      Name: "_p"
    Index:
      Name: "_x2c"
    Index:
      Name: "_x2F"
    Index:
      Name: "__"
    Index:
      Name: "RODATA"
    Index:
      Name: "CODE"
    Index:
      Name: "LOWCODE"
    Index:
      Name: "BSS"
    Index:
      Name: "ZEROPAGE"
    Index:
      Name: "INIT"
    Index:
      Name: "ZPSAVE"
    Index:
      Name: "ONCE"
    Index:
      Name: "STARTUP"
    Index:
      Name: "progbits_tp"
    Index:
      Name: "nobits_tn"
    Index:
      Name: "progbits__then__nobits_tp_tn"
    Index:
      Name: "write_fw"
    Index:
      Name: "exec_fx"
    Index:
      Name: "write__exec_fw_fx"
  Imports:
    Count: 0
  Exports:
    Count: 0
;--- unfinished-escape.od65
  Segments:
    Count: 0
    Index:
      Name: "_"
;--- unknown-escape.od65
  Segments:
    Count: 0
    Index:
      Name: "_g"
;--- unfinished-hex-escape.od65
  Segments:
    Count: 0
    Index:
     Name: "_xa"
;--- invalid-hex-escape.od65
  Segments:
    Count: 0
    Index:
      Name: "_xgg"
;--- unfinished-type-escape.od65
  Segments:
    Count: 0
    Index:
      Name: "_t"
;--- unknown-type-escape.od65
  Segments:
    Count: 0
    Index:
      Name: "_ta"
;--- unfinished-flag-escape.od65
  Segments:
    Count: 0
    Index:
      Name: "_f"
;--- unknown-flag-escape.od65
  Segments:
    Count: 0
    Index:
      Name: "_fa"
;--- ld65.map
Exports list by name:
---------------------
;--- ld65.hex

