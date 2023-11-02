; RUN: split-file %s %t
; RUN: cp %p/Inputs/ca65-dummy.o %t/ca65.o
; RUN: llvm-mc -filetype=obj -triple=mos -o %t/main.o %t/main.s

; RUN: ld.lld --cc65-launcher=%python --od65-path=%p/Inputs/od65.py --ld65-path=%p/Inputs/ld65.py -o %t/a.out %t/main.o %t/ca65.o
; RUN: llvm-nm %t/a.out | FileCheck %s
; CHECK: 1234 A abs_sym
; CHECK: abcd A ca65_sym

; RUN: cp %t/no-exports-list.map %t/ld65.map
; RUN: not ld.lld -m moself --cc65-launcher=%python --od65-path=%p/Inputs/od65.py --ld65-path=%p/Inputs/ld65.py -o %t/a.out %t/ca65.o 2>&1 | FileCheck --check-prefix=NO-EXPORTS-LIST %s
; NO-EXPORTS-LIST: error: ld65 map file: expected "Exports list by name:"

; RUN: cp %t/bad-symbol-line.map %t/ld65.map
; RUN: not ld.lld -m moself --cc65-launcher=%python --od65-path=%p/Inputs/od65.py --ld65-path=%p/Inputs/ld65.py -o %t/a.out %t/ca65.o 2>&1 | FileCheck --check-prefix=BAD-SYMBOL-LINE %s
; BAD-SYMBOL-LINE: error: ld65 map file: expected symbol values line; found

; RUN: cp %t/bad-address.map %t/ld65.map
; RUN: not ld.lld -m moself --cc65-launcher=%python --od65-path=%p/Inputs/od65.py --ld65-path=%p/Inputs/ld65.py -o %t/a.out %t/ca65.o 2>&1 | FileCheck --check-prefix=BAD-ADDRESS %s
; BAD-ADDRESS: error: ld65 map file: expected hex integer; found: 9999999999999999999999999

; RUN: cp %t/unknown-symbol.map %t/ld65.map
; RUN: not ld.lld -m moself --cc65-launcher=%python --od65-path=%p/Inputs/od65.py --ld65-path=%p/Inputs/ld65.py -o %t/a.out %t/ca65.o 2>&1 | FileCheck --check-prefix=UNKNOWN-SYMBOL %s
; UNKNOWN-SYMBOL: error: ld65 map file: could not find symbol definition: unknown


;--- main.s
.globl abs_sym
abs_sym = 0x1234;

foo:
  .word ca65_sym
;--- ca65.od65
  Segments:
    Count: 0
  Imports:
    Count: 0
    Index:
      Name: "abs_sym"
  Exports:
    Count: 0
    Index:
      Name: "ca65_sym"
;--- ld65.map
Exports list by name:
---------------------
abs_sym 1234
ca65_sym abcd
;--- no-exports-list.map
;--- bad-symbol-line.map
Exports list by name:
---------------------
bad
;--- bad-address.map
Exports list by name:
---------------------
sym 9999999999999999999999999
;--- unknown-symbol.map
Exports list by name:
---------------------
unknown 1234
;--- ld65.hex
