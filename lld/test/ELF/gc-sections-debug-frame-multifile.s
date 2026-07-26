# REQUIRES: x86

## Multi-file regression test for .debug_frame under --gc-sections. Two
## input files each contribute a CIE at input-offset 0 plus FDEs that
## reference it via a section-relative absolute pointer. Under GC:
##
## - The surviving FDE from each file must reference ITS OWN CIE in the
##   output, not the other file's. The linker keys per-input CIEs by their
##   (input-section, input-offset) pair; keying by input-offset alone would
##   have every FDE at cieOffset=0 point at whichever file's CIE was
##   inserted last.
## - The dead functions from both files are collected and their FDEs
##   dropped, leaving no tombstoned FDEs at pc=0.

# RUN: rm -rf %t && split-file %s %t
# RUN: llvm-mc -filetype=obj -triple=x86_64-unknown-linux %t/a.s -o %t/a.o
# RUN: llvm-mc -filetype=obj -triple=x86_64-unknown-linux %t/b.s -o %t/b.o
# RUN: ld.lld %t/a.o %t/b.o -o %t.out --gc-sections
# RUN: llvm-readobj --symbols %t.out | FileCheck --check-prefix=SYM %s
# RUN: llvm-dwarfdump --debug-frame %t.out | FileCheck --check-prefix=DF %s

# SYM-NOT: dead_a
# SYM-NOT: dead_b

## Exactly two CIEs (one per input file) and two FDEs (one per surviving
## function). Each FDE's cie= must reference the corresponding file's CIE
## by output offset. Using FileCheck capture-and-reuse ([[CIE_A:...]] then
## [[CIE_A]]) proves the two FDEs point at distinct CIEs; if the map-key
## collision were present the two cie= values would collapse to one.

# DF:      .debug_frame contents:
# DF:      [[CIE_A:[0-9a-f]+]] {{[0-9a-f]+}} ffffffff CIE
# DF:      [[CIE_B:[0-9a-f]+]] {{[0-9a-f]+}} ffffffff CIE
# DF:      FDE cie=[[CIE_A]] pc={{[0-9a-f]+}}...
# DF:      FDE cie=[[CIE_B]] pc={{[0-9a-f]+}}...
# DF-NOT:  CIE
# DF-NOT:  FDE

#--- a.s
	.cfi_sections .debug_frame

	.section	.text.dead_a,"ax",@progbits
	.globl	dead_a
dead_a:
	.cfi_startproc
	ret
	.cfi_endproc

	.section	.text.live_a,"ax",@progbits
	.globl	live_a
live_a:
	.cfi_startproc
	ret
	.cfi_endproc

#--- b.s
	.cfi_sections .debug_frame

	.section	.text.dead_b,"ax",@progbits
	.globl	dead_b
dead_b:
	.cfi_startproc
	ret
	.cfi_endproc

	.section	.text.live_b,"ax",@progbits
	.globl	_start
_start:
	.cfi_startproc
	call live_a
	ret
	.cfi_endproc
