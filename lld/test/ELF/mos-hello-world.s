; REQUIRES: mos
; RUN: llvm-mc -g -triple mos -motorola-integers --filetype=obj -I %S/Inputs -o=%t.obj %s
; RUN: llvm-objdump --all-headers --print-imm-hex -D %t.obj 
; RUN: llvm-readelf --all %t.obj
; RUN: lld -flavor gnu %t.obj -o %t.elf -L %S/Inputs %S/Inputs/mos-c64.ld
; RUN: llvm-readelf --all %t.elf 
; RUN: llvm-objdump --all-headers --print-imm-hex -D %t.elf
; RUN: llvm-dwarfdump --all -v %t.elf
; RUN: llvm-objcopy --output-target binary --strip-unneeded %t.elf %t.bin

.include "mos-c64.inc"

_start:
	ldx	#$0                     ; CHECK: encoding: [0xa2,0x00]
loop:
	lda	hello,x                 ; CHECK: encoding: [0xb5,A]
                                ; CHECK:  fixup A - offset: 1, value: hello, kind: Addr8
	beq	done                    ; CHECK: encoding: [0xf0,A]
                                ; CHECK:  fixup A - offset: 1, value: done, kind: PCRel8
	jsr	$ffd2                   ; CHECK: encoding: [0x20,0xd2,0xff]
	inx                         ; CHECK: encoding: [0xe8]
	bne	loop                    ; CHECK: encoding: [0xd0,A]
                                ; CHECK:  fixup A - offset: 1, value: loop, kind: PCRel8
done:
	jmp exit
	brk
exit:
	rts                         ; CHECK: encoding: [0x60]
hello:  
    .ascii "HELLO, LLVM MOS ASSEMBLER!"
	.byte 0x0d
	.asciz "WELCOME TO 2020!"