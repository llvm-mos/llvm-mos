; RUN: llvm-mc -triple mos -show-encoding < %s | FileCheck %s
	brk                           ; CHECK: encoding: [0x00]
	ora	($12,x)                 ; CHECK: encoding: [0x01,0x12]
	.byte	$02
	.byte	$03
	.byte	$04
	ora	$12                     ; CHECK: encoding: [0x05,0x12]
	asl	$12                     ; CHECK: encoding: [0x06,0x12]
	.byte	$07
	php                           ; CHECK: encoding: [0x08]
	ora	#$12                    ; CHECK: encoding: [0x09,0x12]
	asl                           ; CHECK: encoding: [0x0a]
	.byte	$0b
	.byte	$0c
	ora	$3456                   ; CHECK: encoding: [0x0d,0x56,0x34]
	asl	$3456                   ; CHECK: encoding: [0x0e,0x56,0x34]
	.byte	$0f
	bpl	$12                     ; CHECK: encoding: [0x10,0x12]
	ora	($12),y                 ; CHECK: encoding: [0x11,0x12]
	.byte	$12
	.byte	$13
	.byte	$14
	ora	$12,x                   ; CHECK: encoding: [0x15,0x12]
	asl	$12,x                   ; CHECK: encoding: [0x16,0x12]
	.byte	$17
	clc                           ; CHECK: encoding: [0x18]
	ora	$3456,y                 ; CHECK: encoding: [0x19,0x56,0x34]
	.byte	$1a
	.byte	$1b
	.byte	$1c
	ora	$3456,x                 ; CHECK: encoding: [0x1d,0x56,0x34]
	asl	$3456,x                 ; CHECK: encoding: [0x1e,0x56,0x34]
	.byte	$1f
	jsr	$3456                   ; CHECK: encoding: [0x20,0x56,0x34]
	and	($12,x)                 ; CHECK: encoding: [0x21,0x12]
	.byte	$22
	.byte	$23
	bit	$12                     ; CHECK: encoding: [0x24,0x12]
	and	$12                     ; CHECK: encoding: [0x25,0x12]
	rol	$12                     ; CHECK: encoding: [0x26,0x12]
	.byte	$27
	plp                           ; CHECK: encoding: [0x28]
	and	#$12                    ; CHECK: encoding: [0x29,0x12]
	rol                           ; CHECK: encoding: [0x2a]
	.byte	$2b
	bit	$3456                   ; CHECK: encoding: [0x2c,0x56,0x34]
	and	$3456                   ; CHECK: encoding: [0x2d,0x56,0x34]
	rol	$3456                   ; CHECK: encoding: [0x2e,0x56,0x34]
	.byte	$2f
	bmi	$12                     ; CHECK: encoding: [0x30,0x12]
	and	($12),y                 ; CHECK: encoding: [0x31,0x12]
	.byte	$32
	.byte	$33
	.byte	$34
	and	$12,x                   ; CHECK: encoding: [0x35,0x12]
	rol	$12,x                   ; CHECK: encoding: [0x36,0x12]
	.byte	$37
	sec                           ; CHECK: encoding: [0x38]
	and	$3456,y                 ; CHECK: encoding: [0x39,0x56,0x34]
	.byte	$3a
	.byte	$3b
	.byte	$3c
	and	$3456,x                 ; CHECK: encoding: [0x3d,0x56,0x34]
	rol	$3456,x                 ; CHECK: encoding: [0x3e,0x56,0x34]
	.byte	$3f
	rti                           ; CHECK: encoding: [0x40]
	eor	($12,x)                 ; CHECK: encoding: [0x41,0x12]
	.byte	$42
	.byte	$43
	.byte	$44
	eor	$12                     ; CHECK: encoding: [0x45,0x12]
	lsr	$12                     ; CHECK: encoding: [0x46,0x12]
	.byte	$47
	pha                           ; CHECK: encoding: [0x48]
	eor	#$12                    ; CHECK: encoding: [0x49,0x12]
	lsr                           ; CHECK: encoding: [0x4a]
	.byte	$4b
	jmp	$3456                   ; CHECK: encoding: [0x4c,0x56,0x34]
	eor	$3456                   ; CHECK: encoding: [0x4d,0x56,0x34]
	lsr	$3456                   ; CHECK: encoding: [0x4e,0x56,0x34]
	.byte	$4f
	bvc	$12                     ; CHECK: encoding: [0x50,0x12]
	eor	($12),y                 ; CHECK: encoding: [0x51,0x12]
	.byte	$52
	.byte	$53
	.byte	$54
	eor	$12,x                   ; CHECK: encoding: [0x55,0x12]
	lsr	$12,x                   ; CHECK: encoding: [0x56,0x12]
	.byte	$57
	cli                           ; CHECK: encoding: [0x58]
	eor	$3456,y                 ; CHECK: encoding: [0x59,0x56,0x34]
	.byte	$5a
	.byte	$5b
	.byte	$5c
	eor	$3456,x                 ; CHECK: encoding: [0x5d,0x56,0x34]
	lsr	$3456,x                 ; CHECK: encoding: [0x5e,0x56,0x34]
	.byte	$5f
	rts                           ; CHECK: encoding: [0x60]
	adc	($12,x)                 ; CHECK: encoding: [0x61,0x12]
	.byte	$62
	.byte	$63
	.byte	$64
	adc	$12                     ; CHECK: encoding: [0x65,0x12]
	ror	$12                     ; CHECK: encoding: [0x66,0x12]
	.byte	$67
	pla                           ; CHECK: encoding: [0x68]
	adc	#$12                    ; CHECK: encoding: [0x69,0x12]
	ror                           ; CHECK: encoding: [0x6a]
	.byte	$6b
	jmp	($3456)                 ; CHECK: encoding: [0x6c,0x56,0x34]
	adc	$3456                   ; CHECK: encoding: [0x6d,0x56,0x34]
	ror	$3456                   ; CHECK: encoding: [0x6e,0x56,0x34]
	.byte	$6f
	bvs	$12                     ; CHECK: encoding: [0x70,0x12]
	adc	($12),y                 ; CHECK: encoding: [0x71,0x12]
	.byte	$72
	.byte	$73
	.byte	$74
	adc	$12,x                   ; CHECK: encoding: [0x75,0x12]
	ror	$12,x                   ; CHECK: encoding: [0x76,0x12]
	.byte $77
	sei                           ; CHECK: encoding: [0x78]
	adc	$3456,y                 ; CHECK: encoding: [0x79,0x56,0x34]
	.byte	$7a
	.byte	$7b
	.byte	$7c
	adc	$3456,x                 ; CHECK: encoding: [0x7d,0x56,0x34]
	ror	$3456,x                 ; CHECK: encoding: [0x7e,0x56,0x34]
	.byte	$7f
	.byte	$80
	sta	($12,x)                 ; CHECK: encoding: [0x81,0x12]
	.byte	$82
	.byte	$83
	sty	$12                     ; CHECK: encoding: [0x84,0x12]
	sta	$12                     ; CHECK: encoding: [0x85,0x12]
	stx	$12                     ; CHECK: encoding: [0x86,0x12]
	.byte	$87
	dey                             ; CHECK: encoding: [0x88]
	.byte	$89
	txa                             ; CHECK: encoding: [0x8a]
	.byte	$8b
	sty	$3456                   ; CHECK: encoding: [0x8c,0x56,0x34]
	sta	$3456                   ; CHECK: encoding: [0x8d,0x56,0x34]
	stx	$3456                   ; CHECK: encoding: [0x8e,0x56,0x34]
	.byte	$8f
	bcc	$12                     ; CHECK: encoding: [0x90,0x12]
	sta	($12),y                 ; CHECK: encoding: [0x91,0x12]
	.byte	$92
	.byte	$93
	sty	$12,x                   ; CHECK: encoding: [0x94,0x12]
	sta	$12,x                   ; CHECK: encoding: [0x95,0x12]
	stx	$12,y                   ; CHECK: encoding: [0x96,0x12]
	.byte	$97
	tya                           ; CHECK: encoding: [0x98]
	sta	$3456,y                 ; CHECK: encoding: [0x99,0x56,0x34]
	txs                           ; CHECK: encoding: [0x9a]
	.byte	$9b
	.byte	$9c
	sta	$3456,x                 ; CHECK: encoding: [0x9d,0x56,0x34]
	.byte	$9e
	.byte	$9f
	ldy	#$12                    ; CHECK: encoding: [0xa0,0x12]
	lda	($12,x)                 ; CHECK: encoding: [0xa1,0x12]
	ldx	#$12                    ; CHECK: encoding: [0xa2,0x12]
	.byte $a3
	ldy	$12                     ; CHECK: encoding: [0xa4,0x12]
	lda	$12                     ; CHECK: encoding: [0xa5,0x12]
	ldx	$12                     ; CHECK: encoding: [0xa6,0x12]
	.byte	$a7
	tay                           ; CHECK: encoding: [0xa8]
	lda	#$12                    ; CHECK: encoding: [0xa9,0x12]
	tax                           ; CHECK: encoding: [0xaa]
	.byte	$ab
	ldy	$3456                   ; CHECK: encoding: [0xac,0x56,0x34]
	lda	$3456                   ; CHECK: encoding: [0xad,0x56,0x34]
	ldx	$3456                   ; CHECK: encoding: [0xae,0x56,0x34]
	.byte	$af
	bcs	$12                     ; CHECK: encoding: [0xb0,0x12]
	lda	($12),y                 ; CHECK: encoding: [0xb1,0x12]
	.byte	$b2
	.byte	$b3
	ldy	$12,x                   ; CHECK: encoding: [0xb4,0x12]
	lda	$12,x                   ; CHECK: encoding: [0xb5,0x12]
	ldx	$12,y                   ; CHECK: encoding: [0xbe,0x12,0x00]
	.byte	$bf
	clv                           ; CHECK: encoding: [0xb8]
	lda	$3456,y                 ; CHECK: encoding: [0xb9,0x56,0x34]
	tsx                           ; CHECK: encoding: [0xba]
	.byte	$bb
	ldy	$3456,x                 ; CHECK: encoding: [0xbc,0x56,0x34]
	lda	$3456,x                 ; CHECK: encoding: [0xbd,0x56,0x34]
	ldx	$3456,y                 ; CHECK: encoding: [0xbe,0x56,0x34]
	.byte	$bf
	cpy	#$12                    ; CHECK: encoding: [0xc0,0x12]
	cmp	($12,x)                 ; CHECK: encoding: [0xc1,0x12]
	.byte	$c2
	.byte	$c3
	cpy	$12                     ; CHECK: encoding: [0xc4,0x12]
	cmp	$12                     ; CHECK: encoding: [0xc5,0x12]
	dec	$12                     ; CHECK: encoding: [0xc6,0x12]
	.byte	$c7
	iny                           ; CHECK: encoding: [0xc8]
	cmp	#$12                    ; CHECK: encoding: [0xc9,0x12]
	dex                           ; CHECK: encoding: [0xca]
	.byte	$cb
	cpy	$3456                   ; CHECK: encoding: [0xcc,0x56,0x34]
	cmp	$3456                   ; CHECK: encoding: [0xcd,0x56,0x34]
	dec	$3456                   ; CHECK: encoding: [0xce,0x56,0x34]
	.byte	$cf
	bne	$12                     ; CHECK: encoding: [0xd0,0x12]
	cmp	($12),y                 ; CHECK: encoding: [0xd1,0x12]
	.byte	$d2
	.byte	$d3
	.byte	$d4
	cmp	$12,x                   ; CHECK: encoding: [0xd5,0x12]
	dec	$12,x                   ; CHECK: encoding: [0xd6,0x12]
	.byte	$d7
	cld                           ; CHECK: encoding: [0xd8]
	cmp	$3456,y                 ; CHECK: encoding: [0xd9,0x56,0x34]
	.byte $da
	.byte	$db
	.byte	$dc
	cmp	$3456,x                 ; CHECK: encoding: [0xdd,0x56,0x34]
	dec	$3456,x                 ; CHECK: encoding: [0xde,0x56,0x34]
	.byte	$df
	cpx	#$12                    ; CHECK: encoding: [0xe0,0x12]
	sbc	($12,x)                 ; CHECK: encoding: [0xe1,0x12]
	.byte	$e2
	.byte	$e3
	cpx	$12                     ; CHECK: encoding: [0xe4,0x12]
	sbc	$12                     ; CHECK: encoding: [0xe5,0x12]
	inc	$12                     ; CHECK: encoding: [0xe6,0x12]
	.byte	$e7
	inx                           ; CHECK: encoding: [0xe8]
	sbc	#$12                    ; CHECK: encoding: [0xe9,0x12]
	.byte	$ea
   .byte $eb
	cpx	$3456                   ; CHECK: encoding: [0xec,0x56,0x34]
	sbc	$3456                   ; CHECK: encoding: [0xed,0x56,0x34]
	inc	$3456                   ; CHECK: encoding: [0xee,0x56,0x34]
	.byte	$ef
	beq	$12                     ; CHECK: encoding: [0xf0,0x12]
	sbc	($12),y                 ; CHECK: encoding: [0xf1,0x12]
	.byte	$f2
	.byte	$f3
	.byte	$f4
	sbc	$12,x                   ; CHECK: encoding: [0xf5,0x12]
	inc	$12,x                   ; CHECK: encoding: [0xf6,0x12]
	.byte	$f7
	sed                           ; CHECK: encoding: [0xf8]
	sbc	$3456,y                 ; CHECK: encoding: [0xf9,0x56,0x34]
	.byte	$fa
	.byte $fb
	.byte	$fc
	sbc	$3456,x                 ; CHECK: encoding: [0xfd,0x56,0x34]
	inc	$3456,x                 ; CHECK: encoding: [0xfe,0x56,0x34]
	.byte $ff
