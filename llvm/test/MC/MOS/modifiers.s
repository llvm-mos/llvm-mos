; RUN: llvm-mc -triple mos -motorola-integers --filetype=obj -o=%t.obj %s
; RUN: llvm-objdump --all-headers --print-imm-hex -D %t.obj | FileCheck %s

; Test all the 8/16-bit modifiers for the MOS assembler.

val8 = 0x01
val16 = 0x0203
val24 = 0x040506

. = 0x01
addr8:
.ds.b 0

. = 0x0203
addr16:
.ds.b 0
.ds.b 0

. = 0x0405
addr24:
.ds.b 0
.ds.b 0
.ds.b 0

_start:
    .byte addr16@mos16hi        ; CHECK: 00
                                ; CHECK: R_MOS_ADDR16_HI	.text+0x203

    lda mos16lo(addr8)          ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR16_LO	.text+0x1
    lda mos16lo(addr16)         ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR16_LO	.text+0x203
    lda mos16lo(addr24)         ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR16_LO	.text+0x405

    lda <addr8                  ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR8	.text+0x1
    lda <addr16                 ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR8	.text+0x203
    lda <addr24                 ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR8	.text+0x405

    lda #<val8                  ; CHECK: lda #$1
    lda #<val16                 ; CHECK: lda #$3
    lda #<val24                 ; CHECK: lda #$6

    lda mos16hi(addr8)          ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR16_HI	.text+0x1
    lda mos16hi(addr16)         ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR16_HI	.text+0x203
    lda mos16hi(addr24)         ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR16_HI	.text+0x405

    lda !addr8                  ; CHECK: ad 00 00
                                ; CHECK: R_MOS_ADDR16	.text+0x1
    lda !addr16                 ; CHECK: ad 00 00
                                ; CHECK: R_MOS_ADDR16	.text+0x203
    lda !addr24                 ; CHECK: ad 00 00
                                ; CHECK: R_MOS_ADDR16	.text+0x405

    lda #>val8                  ; CHECK: lda #$0
                                ; val8 has no high byte so $0 is correct
    lda #>val16                 ; CHECK: lda #$2
    lda #>val24                 ; CHECK: lda #$5

    lda mos24bank(addr8)        ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR24_BANK	.text+0x1
    lda mos24bank(addr16)       ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR24_BANK	.text+0x203
    lda mos24bank(addr24)       ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR24_BANK	.text+0x405

    lda mos24segment(addr8)     ; CHECK: ad 00 00
                                ; CHECK: R_MOS_ADDR24_SEGMENT	.text+0x1
    lda mos24segment(addr16)    ; CHECK: ad 00 00
                                ; CHECK: R_MOS_ADDR24_SEGMENT	.text+0x203
    lda mos24segment(addr24)    ; CHECK: ad 00 00
                                ; CHECK: R_MOS_ADDR24_SEGMENT	.text+0x405

    lda mos24segmentlo(addr8)   ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR24_SEGMENT_LO	.text+0x1
    lda mos24segmentlo(addr16)  ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR24_SEGMENT_LO	.text+0x203
    lda mos24segmentlo(addr24)  ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR24_SEGMENT_LO	.text+0x405

    lda #^val8                  ; CHECK: lda #$0
                                ; val8 has no high byte so $0 is correct
    lda #^val16                 ; CHECK: lda #$0
                                ; val16 has no high byte so $0 is correct
    lda #^val24                 ; CHECK: lda #$4

    lda mos24segmenthi(addr8)   ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR24_SEGMENT_HI	.text+0x1
    lda mos24segmenthi(addr16)  ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR24_SEGMENT_HI	.text+0x203
    lda mos24segmenthi(addr24)  ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR24_SEGMENT_HI	.text+0x405

    lda #<1234                  ; CHECK: a9 d2
    lda #<$1234                 ; CHECK: a9 34

    lda #>1234                  ; CHECK: a9 04
    lda #>$1234                 ; CHECK: a9 12

    lda #^1234                  ; CHECK: a9 00
    lda #^$1234                 ; CHECK: a9 00

    lda #1234@mos16lo           ; CHECK: a9 d2
    lda #$1234@mos16lo          ; CHECK: a9 34

    lda #1234@mos16hi           ; CHECK: a9 04
    lda #$1234@mos16hi          ; CHECK: a9 12

    lda #1234@mos24bank         ; CHECK: a9 00
    lda #$1234@mos24bank        ; CHECK: a9 00

.byte $a9, <1234                ; CHECK: a9 d2
.byte $a9, <$1234               ; CHECK: a9 34

.byte $a9, >1234                ; CHECK: a9 04
.byte $a9, >$1234               ; CHECK: a9 12

.byte $a9, ^1234                ; CHECK: a9 00
.byte $a9, ^$1234               ; CHECK: a9 00

.byte $a9, 1234@mos16lo         ; CHECK: a9 d2
.byte $a9, $1234@mos16lo        ; CHECK: a9 34

.byte $a9, 1234@mos16hi         ; CHECK: a9 04
.byte $a9, $1234@mos16hi        ; CHECK: a9 12

.byte $a9, 1234@mos24bank       ; CHECK: a9 00
.byte $a9, $1234@mos24bank      ; CHECK: a9 00

.byte $a9, mos16lo(1234)        ; CHECK: a9 d2
.byte $a9, mos16lo($1234)       ; CHECK: a9 34

.byte $a9, mos16hi(1234)        ; CHECK: a9 04
.byte $a9, mos16hi($1234)       ; CHECK: a9 12

.byte $a9, mos24bank(1234)      ; CHECK: a9 00
.byte $a9, mos24bank($1234)     ; CHECK: a9 00

; Function-style modifiers in directives: mos8
.byte $a9, mos8($1234)          ; CHECK: a9 34

; Function-style modifiers in directives: mos16 (16-bit value)
.2byte mos16($1234)             ; CHECK: 34
                                ; CHECK: 12

; Function-style modifiers in directives: mos24 byte extractors
.byte $a9, mos24segmentlo($040506) ; CHECK: a9 06
.byte $a9, mos24segmenthi($040506) ; CHECK: a9 05

; Function-style modifiers in directives: mos24segment (16-bit value)
.2byte mos24segment($040506)    ; CHECK: 06
                                ; CHECK: 05

; Function-style modifiers in directives: mos13 (13-bit value fits in 16 bits)
.2byte mos13($1FFF)             ; CHECK: ff
                                ; CHECK: 1f
.2byte mos13($3FFF)             ; CHECK: ff
                                ; CHECK: 1f

; Function-style modifiers with symbolic expressions
.byte $a9, mos16lo(val16)       ; CHECK: a9 03
.byte $a9, mos16hi(val16)       ; CHECK: a9 02
.byte $a9, mos24bank(val24)     ; CHECK: a9 04

; Function-style modifiers with arithmetic expressions
.byte $a9, mos16lo($1200 + $34) ; CHECK: a9 34
.byte $a9, mos16hi($1200 + $34) ; CHECK: a9 12
.byte $a9, mos24bank($040000 + $0506) ; CHECK: a9 04