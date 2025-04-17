; RUN: split-file %s %t
; RUN: cp %p/Inputs/ca65-dummy.o %t/ca65.o
; RUN: llvm-mc -filetype=obj -triple=mos -o %t/main.o %t/main.s
; RUN: ld.lld --cc65-launcher=%python --od65-path=%p/Inputs/od65.py --ld65-path=%p/Inputs/ld65.py -o %t/a.out -T%t/link.ld %t/main.o %t/ca65.o
; RUN: FileCheck --input-file=%t/ld65.cfg %s

; CHECK:      MEMORY {
; CHECK-NEXT:   far: start = 13452647, size = 0, file = "";
; CHECK-NEXT:   CONTENTS_: start = 0, size = 0;
; CHECK-NEXT: }
; CHECK-NEXT: SEGMENTS {
; CHECK-NEXT:   far: load = CONTENTS_, run = far;
; CHECK-NEXT: }
; CHECK-NEXT: SYMBOLS {
; CHECK-NEXT:   far_sym: type = export, value = 11211316;
; CHECK-NEXT: }

;--- main.s
;--- ca65.od65
  Segments:
    Count: 0
    Index:
      Name: "far"
      Address size: 0x03
  Imports:
    Index:
      Name: "far_sym"
  Exports:
    Count: 0
;--- link.ld
far_sym = 0xab1234;

SECTIONS {
  far 0xcd4567 : { *(far) }
}
;--- ld65.map
Exports list by name:
---------------------
;--- ld65.hex
