; REQUIRES: system-linux
; RUN: split-file %s %t
; RUN: cp %p/Inputs/ca65-dummy.o %t/ca65.o
; RUN: llvm-mc -filetype=obj -triple=mos -o %t/main.o %t/main.s
; RUN: not ld.lld --od65-path=%p/Inputs/od65.py --ld65-path=%p/Inputs/ld65.py -o %t/a.out %t/main.o %t/ca65.o 2>&1 | FileCheck %s

; CHECK: error: duplicate symbol: duplicate
; CHECK-NEXT: >>> defined in {{.*}}main.o
; CHECK-NEXT: >>> defined in {{.*}}ca65.o

;--- main.s
.globl duplicate
duplicate:
  .byte 0

;--- ca65.od65
  Segments:
    Count: 0
  Imports:
    Count: 0
  Exports:
    Count: 0
    Index:
      Name: "duplicate"
;--- ld65.map
Exports list by name:
---------------------
;--- ld65.hex
