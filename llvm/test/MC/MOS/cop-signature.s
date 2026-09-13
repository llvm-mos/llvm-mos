; RUN: llvm-mc -triple mos -mcpu=mosw65816 -motorola-integers --filetype=obj -o=%t.obj %s
; RUN: llvm-objdump -d --mcpu=mosw65816 %t.obj | FileCheck %s

; cop's signature operand accepts arbitrary expressions, not just literal
; immediates.
sig = $5a
	cop #sig                        ; CHECK: 02 5a cop #$5a

; WDC reserves $80-$ff for internal use and recommends confining user
; signature bytes to $00-$7f; $7f is the top of that range.
	cop #$7f                        ; CHECK: 02 7f cop #$7f
