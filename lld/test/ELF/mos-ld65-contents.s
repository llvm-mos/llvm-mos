; RUN: split-file %s %t
; RUN: cp %p/Inputs/ca65-dummy.o %t/ca65.o
; RUN: ld.lld -m moself --cc65-launcher=%python --od65-path=%p/Inputs/od65.py --ld65-path=%p/Inputs/ld65.py -o %t/a.out %t/ca65.o
; RUN: llvm-readelf -x two_bytes %t/a.out | FileCheck --check-prefix=TWO-BYTES %s
; RUN: llvm-readelf -x one_byte %t/a.out | FileCheck --check-prefix=ONE-BYTE %s
; TWO-BYTES: 12ab
; ONE-BYTE: cd

;--- ca65.od65
  Segments:
    Count: 0
    Index:
      Name: "two__bytes"
      Size: 2
    Index:
      Name: "one__byte"
      Size: 1
  Imports:
    Count: 0
  Exports:
    Count: 0
;--- ld65.map
Exports list by name:
---------------------
;--- ld65.hex
12abcd
