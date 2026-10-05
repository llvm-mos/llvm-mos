; RUN: llvm-mc -triple mos -mcpu=mosw65816 -motorola-integers --filetype=obj -o=%t.obj %s
; RUN: llvm-objdump -d --mcpu=mosw65816 %t.obj | FileCheck %s
; RUN: echo "0x02 0x80" | llvm-mc -triple mos -mcpu=mosw65816 -disassemble | llvm-mc -triple mos -mcpu=mosw65816 -show-encoding | FileCheck %s --check-prefix=ROUNDTRIP80
; RUN: echo "0x02 0xff" | llvm-mc -triple mos -mcpu=mosw65816 -disassemble | llvm-mc -triple mos -mcpu=mosw65816 -show-encoding | FileCheck %s --check-prefix=ROUNDTRIPFF

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

; Byte-level round trip on the reserved-range boundary and its top: disassemble
; the raw encoding, reassemble the result, and confirm it re-encodes identically.
; ROUNDTRIP80: cop #128{{.*}}encoding: [0x02,0x80]
; ROUNDTRIPFF: cop #255{{.*}}encoding: [0x02,0xff]
