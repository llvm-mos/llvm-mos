; RUN: llvm-mc -triple mos -motorola-integers --filetype=obj -o=%t.obj %s
; RUN: llvm-objdump --all-headers --print-imm-hex -D %t.obj | FileCheck %s

; Test all the 8/16-bit modifiers for the MOS assembler.

val8 = 0x01
val16 = 0x0202
val24 = 0x030303

. = 0x01
addr8:
.ds.b 0

. = 0x0202
addr16:
.ds.b 0
.ds.b 0

. = 0x0303
addr24:
.ds.b 0
.ds.b 0
.ds.b 0

_start:
    .byte addr16@mos16hi        ; CHECK: 00
                                ; CHECK: R_MOS_ADDR16_HI	.text+0x202

    lda mos16lo(addr8)          ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR16_LO	.text+0x1
    lda mos16lo(addr16)         ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR16_LO	.text+0x202
    lda mos16lo(addr24)         ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR16_LO	.text+0x303

    lda <addr8                  ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR8	.text+0x1
    lda <addr16                 ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR8	.text+0x202
    lda <addr24                 ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR8	.text+0x303

    lda #<val8                  ; CHECK: lda #$1
    lda #<val16                 ; CHECK: lda #$2
    lda #<val24                 ; CHECK: lda #$3

    lda mos16hi(addr8)          ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR16_HI	.text+0x1
    lda mos16hi(addr16)         ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR16_HI	.text+0x202
    lda mos16hi(addr24)         ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR16_HI	.text+0x303

    lda !addr8                  ; CHECK: ad 00 00
                                ; CHECK: R_MOS_ADDR16	.text+0x1
    lda !addr16                 ; CHECK: ad 00 00
                                ; CHECK: R_MOS_ADDR16	.text+0x202
    lda !addr24                 ; CHECK: ad 00 00
                                ; CHECK: R_MOS_ADDR16	.text+0x303

    lda #>val8                  ; CHECK: lda #$0
                                ; val8 has no high byte so $0 is correct
    lda #>val16                 ; CHECK: lda #$2
    lda #>val24                 ; CHECK: lda #$3

    lda mos24bank(addr8)        ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR24_BANK	.text+0x1
    lda mos24bank(addr16)       ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR24_BANK	.text+0x202
    lda mos24bank(addr24)       ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR24_BANK	.text+0x303

    lda mos24segment(addr8)     ; CHECK: ad 00 00
                                ; CHECK: R_MOS_ADDR24_SEGMENT	.text+0x1
    lda mos24segment(addr16)    ; CHECK: ad 00 00
                                ; CHECK: R_MOS_ADDR24_SEGMENT	.text+0x202
    lda mos24segment(addr24)    ; CHECK: ad 00 00
                                ; CHECK: R_MOS_ADDR24_SEGMENT	.text+0x303

    lda mos24segmentlo(addr8)   ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR24_SEGMENT_LO	.text+0x1
    lda mos24segmentlo(addr16)  ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR24_SEGMENT_LO	.text+0x202
    lda mos24segmentlo(addr24)  ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR24_SEGMENT_LO	.text+0x303

    lda #^val8                  ; CHECK: lda #$0
                                ; val8 has no high byte so $0 is correct
    lda #^val16                 ; CHECK: lda #$0
                                ; val16 has no high byte so $0 is correct
    lda #^val24                 ; CHECK: lda #$3

    lda mos24segmenthi(addr8)   ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR24_SEGMENT_HI	.text+0x1
    lda mos24segmenthi(addr16)  ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR24_SEGMENT_HI	.text+0x202
    lda mos24segmenthi(addr24)  ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR24_SEGMENT_HI	.text+0x303
