# REQUIRES: x86

## Verify that FDEs in .debug_frame for functions garbage-collected by
## --gc-sections are dropped from the output, and that surviving FDEs
## reference the correct PC (not tombstoned to 0). Without the linker
## dropping dead FDEs, the tombstone value written into the CIE-pointer
## and PC-start relocations would leave multiple FDEs claiming to describe
## code at address 0, which is undetectable by consumers because DWARF
## CFI has no codified tombstone-recognition convention.

# RUN: llvm-mc -filetype=obj -triple=x86_64-unknown-linux %s -o %t.o

## With --gc-sections: dead_fn is collected, so only the FDE for _start
## survives, referencing a non-zero PC.
# RUN: ld.lld %t.o -o %t.gc --gc-sections
# RUN: llvm-readobj --symbols %t.gc | FileCheck --check-prefix=GC-SYM %s
# RUN: llvm-dwarfdump --debug-frame %t.gc | FileCheck --check-prefix=GC-DF %s

## Without --gc-sections: both functions live, both FDEs present.
# RUN: ld.lld %t.o -o %t.nogc
# RUN: llvm-readobj --symbols %t.nogc | FileCheck --check-prefix=NOGC-SYM %s
# RUN: llvm-dwarfdump --debug-frame %t.nogc | FileCheck --check-prefix=NOGC-DF %s

# GC-SYM-NOT: dead_fn
# NOGC-SYM: dead_fn

# GC-DF: .debug_frame contents:
# GC-DF: CIE
# GC-DF: FDE cie={{.*}} pc={{[0-9a-f]+}}...
# GC-DF-NOT: FDE

# NOGC-DF: .debug_frame contents:
# NOGC-DF: CIE
# NOGC-DF: FDE cie={{.*}} pc={{[0-9a-f]+}}...
# NOGC-DF: FDE cie={{.*}} pc={{[0-9a-f]+}}...
# NOGC-DF-NOT: FDE

	.cfi_sections .debug_frame

	.section	.text.dead,"ax",@progbits
	.globl	dead_fn
dead_fn:
	.cfi_startproc
	ret
	.cfi_endproc

	.section	.text.live,"ax",@progbits
	.globl	_start
_start:
	.cfi_startproc
	ret
	.cfi_endproc
