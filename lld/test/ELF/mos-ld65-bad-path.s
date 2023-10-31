; REQUIRES: system-linux
; RUN: split-file %s %t
; RUN: cp %p/Inputs/ca65-dummy.o %t/ca65.o
; RUN: not ld.lld -m moself --od65-path=bad %t/ca65.o 2>&1 | FileCheck --check-prefix=OD65-PATH %s
; RUN: not ld.lld -m moself --od65-path=%p/Inputs/od65.py --ld65-path=bad %t/ca65.o 2>&1 | FileCheck --check-prefix=LD65-PATH %s
; OD65-PATH: could not run od65:
; LD65-PATH: could not run ld65:

;--- ca65.od65
  Segments:
    Count: 0
  Imports:
  Exports:
    Count: 0
;--- ld65.map
Exports list by name:
---------------------
;--- ld65.hex
