; REQUIRES: mos
; RUN: llvm-mc -triple mos -motorola-integers --filetype=obj -o=%t.obj %s
; RUN: llvm-objdump --all-headers --print-imm-hex -D %t.obj 
; RUN: lld -flavor gnu %t.obj -o %t.elf
; RUN: llvm-readelf --all %t.elf 
; RUN: llvm-objcopy --output-target binary --strip-unneeded %t.elf %t.bin

chrout = $ffd0 + (2 * 2) - 2

_start:
	ldx	#mos16lo(_start)        ; CHECK: encoding: [0xa2'A',0x00]
                                ; CHECK:   fixup A - offset: 0, value: mos16lo(_start), kind: Addr16_Low
	stx	$10                     ; CHECK: encoding: [0x86,0x10]
	lda	#mos16hi(_start)        ; CHECK: encoding: [0xa9'A',0x00]
                                ; CHECK:   fixup A - offset: 0, value: mos16hi(_start), kind: Addr16_High
	sta	$11                     ; CHECK: encoding: [0x85,0x11]
	ldx	#mos16lo(chrout)        ; CHECK: encoding: [0xa2,0xd2]
	stx	$12                     ; CHECK: encoding: [0x86,0x12]
	lda	#mos16hi(chrout)        ; CHECK: encoding: [0xa9,0xff]
	sta	$13                     ; CHECK: encoding: [0x85,0x13]
done:
	rts                         ; CHECK: encoding: [0x60]
