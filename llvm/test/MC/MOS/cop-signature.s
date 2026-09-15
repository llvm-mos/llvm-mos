; RUN: llvm-mc -triple mos -mcpu=mosw65816 -motorola-integers --filetype=obj -o=%t.obj %s
; RUN: llvm-objdump -d --mcpu=mosw65816 %t.obj | FileCheck %s

; cop's signature operand accepts arbitrary expressions, not just literal
; immediates.
sig = $5a
	cop #sig                        ; CHECK: 02 5a cop #$5a

; WDC reserves $80-$ff for internal use and recommends confining user
; signature bytes to $00-$7f; $7f is the top of that range.
	cop #$7f                        ; CHECK: 02 7f cop #$7f

; The reservation is a usage recommendation, not an encoding restriction:
; the assembler still accepts the full 8-bit range, $80-$ff included.
	cop #$80                        ; CHECK: 02 80 cop #$80
	cop #$ff                        ; CHECK: 02 ff cop #$ff
