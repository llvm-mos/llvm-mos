; RUN: split-file %s %t
; RUN: cp %p/Inputs/ca65-dummy.o %t/ca65.o
; RUN: cp %p/Inputs/ca65-dummy.o %t/other-ca65.o
; RUN: llvm-mc -filetype=obj -triple=mos -o %t/main.o %t/main.s
; RUN: ld.lld --cc65-launcher=%python --od65-path=%p/Inputs/od65.py --ld65-path=%p/Inputs/ld65.py -o %t/a.out -T%t/link.ld %t/main.o %t/ca65.o %t/other-ca65.o
; RUN: FileCheck --input-file=%t/ld65.cfg %s

; CHECK:      MEMORY {
; CHECK-NEXT:   underscore__escape: start = {{.*}}, size = 0, file = "";
; CHECK-NEXT:   CONTENTS_: start = {{.*}}, size = 0;
; CHECK-NEXT: }
; CHECK-NEXT: SEGMENTS {
; CHECK-NEXT:   underscore__escape: load = CONTENTS_, run = underscore__escape;
; CHECK-NEXT: }
; CHECK-NEXT: SYMBOLS {
; CHECK-NEXT:   abs_sym_without_file: type = export, value = 1234;
; CHECK-NEXT:   zp_sym: addrsize = zp, type = export, value = 123;
; CHECK-NEXT: }

;--- main.s
;--- ca65.od65
  Segments:
    Count: 0
    Index:
      Name: "underscore__escape"
  Imports:
    Count: 0
    Index:
      Name: "abs_sym_without_file"
    Index:
      Name: "defined_in_other_ca65"
    Index:
      Name: "zp_sym"
  Exports:
    Count: 0
;--- other-ca65.od65
  Segments:
    Count: 0
  Imports:
    Count: 0
  Exports:
    Count: 0
    Index:
      Name: "defined_in_other_ca65"
;--- link.ld
abs_sym_without_file = 1234;
zp_sym = 123;
;--- ld65.map
Exports list by name:
---------------------
;--- ld65.hex
