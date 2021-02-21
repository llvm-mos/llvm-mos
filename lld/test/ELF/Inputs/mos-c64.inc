; mos-c65.inc

; The BASIC stub asserted at the beginning of all programs.
.ifndef __llvm_mos_no_basic_header
__llvm_mos_basic.load_address:
	.short $0801
__llvm_mos_basic.stub_begin:
	.short __llvm_mos_basic.last_line
	             ; address of next BASIC line
	.short 7773  ; line number: l33t for LLVM, a hint that this program was
	             ; compiled with LLVM
	.byte 0x9e   ; SYS keyword BASIC token
	.asciz "2061" ; the location of the _start symbol, in ASCII
__llvm_mos_basic.last_line:
	.short 0    ; end of basic program
__llvm_mos_basic.stub_end:
.endif ; __llvm_mos_no_basic_header

; Send a character to the output.
chrout = 0xffd2