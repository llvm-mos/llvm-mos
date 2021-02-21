; RUN: llvm-mc -triple mos --filetype=obj -I %S/Inputs -o=%t.obj %s 
; RUN: llvm-objdump --all-headers --print-imm-hex -D %t.obj | FileCheck %s

; An 8-bit immediate value to be used as an address
adrImm8 = 0xea
; A 16-bit immediate value to be used as an address
adrImm16 = 0xeaea

; A section with the special name "zp", which should be placed in 8-bit memory
.section .zp,"",@nobits
adrzp: .ds.b 1

; A section with the special name "zeropage", which should be placed in 8-bit
; memory
.section .zeropage,"",@nobits
adrzeropage: .ds.b 1

; A section with the special name "directpage", which should be placed in 8-bit
; memory
.section .directpage,"",@nobits
adrdirectpage: .ds.b 1

; A section with the ordinary name "lowmemory", but it has the z flag, 
; which hints to the relaxation logic that the addresses in it should be
; referenced as though they will be placed in zero page
.section .lowmemory,"z",@nobits
adrlowmemory: .ds.b 1

; A section without a special name and without the z flag, which will be 
; treated by the relaxation logic as though it will be placed in 16-bit 
; memory, and thus two-byte address references will be generated to it
.section .notzeropage,"",@nobits
adrnotzeropage: .ds.b 1

.text

_start:

  lda adrImm8             ; CHECK: a5 ea
  lda adrImm16            ; CHECK: ad ea ea
  lda _start              ; CHECK: ad 00 00
                          ; CHECK: R_MOS_ADDR16	.text
  lda adrzp               ; CHECK: a5 00
                          ; CHECK: R_MOS_ADDR8	.zp
  lda adrzeropage         ; CHECK: a5 00
                          ; CHECK: R_MOS_ADDR8	.zeropage
  lda adrdirectpage       ; CHECK: a5 00
                          ; CHECK: R_MOS_ADDR8	.directpage
  lda adrnotzeropage      ; CHECK: ad 00 00
                          ; CHECK: R_MOS_ADDR16	.notzeropage
                          