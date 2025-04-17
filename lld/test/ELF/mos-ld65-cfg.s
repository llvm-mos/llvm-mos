; RUN: split-file %s %t
; RUN: cp %p/Inputs/ca65-dummy.o %t/ca65.o
; RUN: cp %p/Inputs/ca65-dummy.o %t/other-ca65.o
; RUN: llvm-mc -filetype=obj -triple=mos -o %t/main.o %t/main.s
; RUN: ld.lld --cc65-launcher=%python --od65-path=%p/Inputs/od65.py --ld65-path=%p/Inputs/ld65.py -o %t/a.out -T%t/link.ld %t/main.o %t/ca65.o %t/other-ca65.o
; RUN: FileCheck --input-file=%t/ld65.cfg %s

; CHECK:      MEMORY {
; CHECK-NEXT:   underscore__escape: start = 0, size = 0, file = "";
; CHECK-NEXT:   banked: start = 17767, size = 0, file = "", bank = 205;
; CHECK-NEXT:   CONTENTS_: start = 0, size = 0;
; CHECK-NEXT: }
; CHECK-NEXT: SEGMENTS {
; CHECK-NEXT:   underscore__escape: load = CONTENTS_, run = underscore__escape;
; CHECK-NEXT:   banked: load = CONTENTS_, run = banked;
; CHECK-NEXT: }
; CHECK-NEXT: SYMBOLS {
; CHECK-NEXT:   abs_sym_without_file: type = export, value = 1234;
; CHECK-NEXT:   zp_sym: addrsize = zp, type = export, value = 123;
; CHECK-NEXT:   banked_sym: type = export, value = 4660;
; CHECK-NEXT: }

;--- main.s
;--- ca65.od65
  Segments:
    Count: 0
    Index:
      Name: "underscore__escape"
    Index:
      Name: "banked"
  Imports:
    Count: 0
    Index:
      Name: "abs_sym_without_file"
    Index:
      Name: "defined_in_other_ca65"
    Index:
      Name: "zp_sym"
    Index:
      Name: "banked_sym"
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
banked_sym = 0xab1234;

SECTIONS {
  underscore_escape : { *(underscore_escape) }
  banked 0xcd4567 : { *(banked) }
}
;--- ld65.map
Exports list by name:
---------------------
;--- ld65.hex
