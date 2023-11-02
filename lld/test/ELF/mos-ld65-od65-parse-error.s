; RUN: split-file %s %t
; RUN: cp %p/Inputs/ca65-dummy.o %t/ca65.o

; RUN: cp %t/no-segments.od65 %t/ca65.od65
; RUN: not ld.lld -m moself --cc65-launcher=%python --od65-path=%p/Inputs/od65.py %t/ca65.o 2>&1 | FileCheck --check-prefix=NO-SEGMENTS %s
; NO-SEGMENTS: ca65.o: could not parse od65 output: expected "  Segments:"

; RUN: cp %t/no-segment-count.od65 %t/ca65.od65
; RUN: not ld.lld -m moself --cc65-launcher=%python --od65-path=%p/Inputs/od65.py %t/ca65.o 2>&1 | FileCheck --check-prefix=NO-SEGMENT-COUNT %s
; NO-SEGMENT-COUNT: ca65.o: could not parse od65 output: expected prefix " Count:"

; RUN: cp %t/bad-quotes.od65 %t/ca65.od65
; RUN: not ld.lld -m moself --cc65-launcher=%python --od65-path=%p/Inputs/od65.py %t/ca65.o 2>&1 | FileCheck --check-prefix=BAD-QUOTES %s
; BAD-QUOTES: ca65.o: could not parse quoted string Name: Bad

; RUN: cp %t/bad-kv.od65 %t/ca65.od65
; RUN: not ld.lld -m moself --cc65-launcher=%python --od65-path=%p/Inputs/od65.py %t/ca65.o 2>&1 | FileCheck --check-prefix=BAD-KV %s
; BAD-KV: ca65.o: could not parse key-value pair: Size

; RUN: cp %t/bad-int.od65 %t/ca65.od65
; RUN: not ld.lld -m moself --cc65-launcher=%python --od65-path=%p/Inputs/od65.py %t/ca65.o 2>&1 | FileCheck --check-prefix=BAD-INT %s
; BAD-INT: ca65.o: could not parse int Size: Bad

;--- no-segments.od65
;--- no-segment-count.od65
  Segments:
;--- no-segment-index.od65
  Segments:
    Count: 0
    Bad
;--- bad-quotes.od65
  Segments:
    Count: 0
    Index:
      Name: Bad
;--- bad-kv.od65
  Segments:
    Count: 0
    Index:
      Size
;--- bad-int.od65
  Segments:
    Count: 0
    Index:
      Size: Bad
