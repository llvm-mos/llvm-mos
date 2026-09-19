; RUN: llvm-mc -triple mos -show-encoding < %s | FileCheck %s --check-prefix=ENC
; RUN: llvm-mc -triple mos -filetype=obj < %s | llvm-objdump -d - | FileCheck %s --check-prefix=DIS
; RUN: llvm-mc -triple mos -mcpu=mosw65816 -filetype=obj < %s | llvm-objdump -d --mcpu=mosw65816 - | FileCheck %s --check-prefix=DIS
; RUN: echo "0x00 0xea" | llvm-mc -triple mos -disassemble | llvm-mc -triple mos -show-encoding | FileCheck %s --check-prefix=ROUNDTRIP
; RUN: echo "0x00 0xea" | llvm-mc -triple mos -mcpu=mosw65816 -disassemble | llvm-mc -triple mos -mcpu=mosw65816 -show-encoding | FileCheck %s --check-prefix=ROUNDTRIP

; The signature is assembler syntax only: disassembly consumes one byte for
; BRK and decodes the following byte as a separate instruction.

; ENC: encoding: [0x00]
; DIS: 00 brk
brk
; ENC: encoding: [0x00,0xea]
; DIS-NEXT: 00 brk
; DIS-NEXT: ea nop
brk #234
; ENC: encoding: [0xea]
; DIS-NEXT: ea nop
nop

; ROUNDTRIP: brk {{.*}}encoding: [0x00]
; ROUNDTRIP-NEXT: nop {{.*}}encoding: [0xea]
