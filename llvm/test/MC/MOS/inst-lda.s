; RUN: llvm-mc -triple mos -motorola-integers -show-encoding < %s | FileCheck %s

foo:
;	lda foo,y
;	lda foo,x
;	lda foo

	lda #$0b						; CHECK: encoding: [0xa9,0x0b]
    lda ($28,x)                     ; CHECK: encoding: [0xa1,0x28]
	lda ($28),y                     ; CHECK: encoding: [0xb1,0x28]
	lda $40,x                       ; CHECK: encoding: [0xb5,0x40]
	lda #$ff                        ; CHECK: encoding: [0xa9,0xff]
	lda $40                         ; CHECK: encoding: [0xa5,0x40]
	lda $ff                         ; CHECK: encoding: [0xa5,0xff]
	lda ($28,x)                     ; CHECK: encoding: [0xa1,0x28]
	lda ($40,x)                     ; CHECK: encoding: [0xa1,0x40]
	lda $4000,x                     ; CHECK: encoding: [0xbd,0x00,0x40]
	lda $4001,y                     ; CHECK: encoding: [0xb9,0x01,0x40]
	lda #$ff                        ; CHECK: encoding: [0xa9,0xff]
	lda $100                        ; CHECK: encoding: [0xad,0x00,0x01]
	lda $101                        ; CHECK: encoding: [0xad,0x01,0x01]
	lda $ffff                       ; CHECK: encoding: [0xad,0xff,0xff]
	lda #$0                         ; CHECK: encoding: [0xa9,0x00]