; RUN: llvm-mc -triple mos -mcpu=mosw65816 -motorola-integers --filetype=obj -o=%t.obj %s
; RUN: llvm-objdump --all-headers --print-imm-hex -D %t.obj | FileCheck %s

; Test 65816 addressing modes for the MOS assembler.

. = 0x01
.zeropage addr8
addr8:
.ds.b 0

. = 0x0202
addr16:
.ds.b 0
.ds.b 0

. = 0x030303
addr24:
.ds.b 0
.ds.b 0
.ds.b 0

_start:
    ; Immediate (8-bit)

    lda #addr8                  ; CHECK: a9 00
                                ; CHECK: R_MOS_IMM8		.text+0x1
    lda #addr16                 ; CHECK: a9 00
                                ; CHECK: R_MOS_IMM8		.text+0x202
    lda #addr24                 ; CHECK: a9 00
                                ; CHECK: R_MOS_IMM8		.text+0x30303
    lda #<addr8                 ; CHECK: a9 00
                                ; CHECK: R_MOS_ADDR16_LO		.text+0x1
    lda #<addr16                ; CHECK: a9 00
                                ; CHECK: R_MOS_ADDR16_LO		.text+0x202
    lda #<addr24                ; CHECK: a9 00
                                ; CHECK: R_MOS_ADDR16_LO		.text+0x30303
    lda #>addr8                 ; CHECK: a9 00
                                ; CHECK: R_MOS_ADDR16_HI		.text+0x1
    lda #>addr16                ; CHECK: a9 00
                                ; CHECK: R_MOS_ADDR16_HI		.text+0x202
    lda #>addr24                ; CHECK: a9 00
                                ; CHECK: R_MOS_ADDR16_HI		.text+0x30303
    lda #^addr8                 ; CHECK: a9 00
                                ; CHECK: R_MOS_ADDR24_BANK		.text+0x1
    lda #^addr16                ; CHECK: a9 00
                                ; CHECK: R_MOS_ADDR24_BANK		.text+0x202
    lda #^addr24                ; CHECK: a9 00
                                ; CHECK: R_MOS_ADDR24_BANK		.text+0x30303

    ; TODO: Immediate (16-bit)

    ; Absolute

    lda !addr8                  ; CHECK: ad 00 00
                                ; CHECK: R_MOS_ADDR16		.text+0x1
    lda !addr16                 ; CHECK: ad 00 00
                                ; CHECK: R_MOS_ADDR16		.text+0x202
    ; TODO: lda addr16
    lda !addr24                 ; CHECK: ad 00 00
                                ; CHECK: R_MOS_ADDR16		.text+0x30303

    ; Absolute Long

    lda >addr8                  ; CHECK: af 00 00 00
                                ; CHECK: R_MOS_ADDR24		.text+0x1
    lda >addr16                 ; CHECK: af 00 00 00
                                ; CHECK: R_MOS_ADDR24		.text+0x202
    lda >addr24                 ; CHECK: af 00 00 00
                                ; CHECK: R_MOS_ADDR24		.text+0x30303
    lda addr24                  ; CHECK: af 00 00 00
                                ; CHECK: R_MOS_ADDR24		.text+0x30303

    ; Direct Page

    lda addr8                   ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR8		.text+0x1
    lda <addr8                  ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR8		.text+0x1
    lda <addr16                 ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR8		.text+0x202
    lda <addr24                 ; CHECK: a5 00
                                ; CHECK: R_MOS_ADDR8		.text+0x30303

    ; Direct Indirect Indexed

    lda (addr8), y              ; CHECK: b1 00
                                ; CHECK: R_MOS_ADDR8		.text+0x1
    lda (<addr8), y             ; CHECK: b1 00
                                ; CHECK: R_MOS_ADDR8		.text+0x1
    lda (<addr16), y            ; CHECK: b1 00
                                ; CHECK: R_MOS_ADDR8		.text+0x202
    lda (<addr24), y            ; CHECK: b1 00
                                ; CHECK: R_MOS_ADDR8		.text+0x30303

    ; Direct Indirect Indexed Long

    lda [addr8], y              ; CHECK: b7 00
                                ; CHECK: R_MOS_ADDR8		.text+0x1
    lda [<addr8], y             ; CHECK: b7 00
                                ; CHECK: R_MOS_ADDR8		.text+0x1
    lda [<addr16], y            ; CHECK: b7 00
                                ; CHECK: R_MOS_ADDR8		.text+0x202
    lda [<addr24], y            ; CHECK: b7 00
                                ; CHECK: R_MOS_ADDR8		.text+0x30303

    ; Direct Indexed Indirect

    lda (addr8, x)              ; CHECK: a1 00
                                ; CHECK: R_MOS_ADDR8		.text+0x1
    lda (<addr8, x)             ; CHECK: a1 00
                                ; CHECK: R_MOS_ADDR8		.text+0x1
    lda (<addr16, x)            ; CHECK: a1 00
                                ; CHECK: R_MOS_ADDR8		.text+0x202
    lda (<addr24, x)            ; CHECK: a1 00
                                ; CHECK: R_MOS_ADDR8		.text+0x30303

    ; Direct Indexed by X

    lda addr8, x                ; CHECK: b5 00
                                ; CHECK: R_MOS_ADDR8		.text+0x1
    lda <addr8, x               ; CHECK: b5 00
                                ; CHECK: R_MOS_ADDR8		.text+0x1
    lda <addr16, x              ; CHECK: b5 00
                                ; CHECK: R_MOS_ADDR8		.text+0x202
    lda <addr24, x              ; CHECK: b5 00
                                ; CHECK: R_MOS_ADDR8		.text+0x30303

    ; Direct Indexed by Y

    ldx addr8, y                ; CHECK: b6 00
                                ; CHECK: R_MOS_ADDR8		.text+0x1
    ldx <addr8, y               ; CHECK: b6 00
                                ; CHECK: R_MOS_ADDR8		.text+0x1
    ldx <addr16, y              ; CHECK: b6 00
                                ; CHECK: R_MOS_ADDR8		.text+0x202
    ldx <addr24, y              ; CHECK: b6 00
                                ; CHECK: R_MOS_ADDR8		.text+0x30303

    ; Absolute Indexed by X

    ; lda addr8, x will be lowered to Direct Indexed by X
    lda !addr8, x               ; CHECK: bd 00 00
                                ; CHECK: R_MOS_ADDR16		.text+0x1
    ; TODO: lda addr16, x
    lda !addr16, x              ; CHECK: bd 00 00
                                ; CHECK: R_MOS_ADDR16		.text+0x202
    lda !addr24, x              ; CHECK: bd 00 00
                                ; CHECK: R_MOS_ADDR16		.text+0x30303

    ; Absolute Indexed by Y

    ; lda addr8, y will be lowered to Direct Indexed by Y
    lda !addr8, y               ; CHECK: b9 00 00
                                ; CHECK: R_MOS_ADDR16		.text+0x1
    ; TODO: lda addr16, y
    lda !addr16, y              ; CHECK: b9 00 00
                                ; CHECK: R_MOS_ADDR16		.text+0x202
    lda !addr24, y              ; CHECK: b9 00 00
                                ; CHECK: R_MOS_ADDR16		.text+0x30303

    ; Absolute Long Indexed by X

    lda >addr8, x               ; CHECK: bf 00 00 00
                                ; CHECK: R_MOS_ADDR24		.text+0x1
    lda >addr16, x              ; CHECK: bf 00 00 00
                                ; CHECK: R_MOS_ADDR24		.text+0x202
    lda >addr24, x              ; CHECK: bf 00 00 00
                                ; CHECK: R_MOS_ADDR24		.text+0x30303
    lda addr24, x               ; CHECK: bf 00 00 00
                                ; CHECK: R_MOS_ADDR24		.text+0x30303

    ; Absolute Indirect

    jmp (addr8)                 ; CHECK: 6c 00 00
                                ; CHECK: R_MOS_ADDR16		.text+0x1
    jmp (!addr8)                ; CHECK: 6c 00 00
                                ; CHECK: R_MOS_ADDR16		.text+0x1
    jmp (addr16)                ; CHECK: 6c 00 00
                                ; CHECK: R_MOS_ADDR16		.text+0x202
    jmp (!addr16)               ; CHECK: 6c 00 00
                                ; CHECK: R_MOS_ADDR16		.text+0x202
    jmp (!addr24)               ; CHECK: 6c 00 00
                                ; CHECK: R_MOS_ADDR16		.text+0x30303

    ; Direct Indirect

    lda (addr8)                 ; CHECK: b2 00
                                ; CHECK: R_MOS_ADDR8		.text+0x1
    lda (<addr8)                ; CHECK: b2 00
                                ; CHECK: R_MOS_ADDR8		.text+0x1
    lda (<addr16)               ; CHECK: b2 00
                                ; CHECK: R_MOS_ADDR8		.text+0x202
    lda (<addr24)               ; CHECK: b2 00
                                ; CHECK: R_MOS_ADDR8		.text+0x30303

    ; Direct Indirect Long

    lda [addr8]                 ; CHECK: a7 00
                                ; CHECK: R_MOS_ADDR8		.text+0x1
    lda [<addr8]                ; CHECK: a7 00
                                ; CHECK: R_MOS_ADDR8		.text+0x1
    lda [<addr16]               ; CHECK: a7 00
                                ; CHECK: R_MOS_ADDR8		.text+0x202
    lda [<addr24]               ; CHECK: a7 00
                                ; CHECK: R_MOS_ADDR8		.text+0x30303

    ; Absolute Indexed Indirect

    jmp (addr8, x)              ; CHECK: 7c 00 00
                                ; CHECK: R_MOS_ADDR16		.text+0x1
    jmp (!addr8, x)             ; CHECK: 7c 00 00
                                ; CHECK: R_MOS_ADDR16		.text+0x1
    jmp (addr16, x)             ; CHECK: 7c 00 00
                                ; CHECK: R_MOS_ADDR16		.text+0x202
    jmp (!addr16, x)            ; CHECK: 7c 00 00
                                ; CHECK: R_MOS_ADDR16		.text+0x202
    jmp (!addr24, x)            ; CHECK: 7c 00 00
                                ; CHECK: R_MOS_ADDR16		.text+0x30303
