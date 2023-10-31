; RUN: split-file %s %t
; RUN: cp %p/Inputs/ca65-dummy.o %t/ca65.o
; RUN: llvm-mc -filetype=obj -triple=mos -o %t/main.o %t/main.s
; RUN: ld.lld --cc65-launcher=%python --od65-path=%p/Inputs/od65.py --ld65-path=%p/Inputs/ld65.py --gc-sections -o %t/a.out %t/main.o %t/ca65.o
; RUN: llvm-readelf -x .referenced_by_ca65 %t/a.out | FileCheck %s
; CHECK: .referenced_by_ca65

;--- main.s
.section .referenced_by_ca65,"a",@progbits 
.globl referenced_by_ca65
referenced_by_ca65:
  .byte 0

;--- ca65.od65
  Segments:
    Count: 0
  Imports:
    Count: 0
    Index:
      Name: "referenced_by_ca65"
  Exports:
    Count: 0
;--- ld65.map
Exports list by name:
---------------------
;--- ld65.hex
