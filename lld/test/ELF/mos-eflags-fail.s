# REQUIRES: mos
# RUN: llvm-mc -filetype=obj -triple=mos %s -o %t.o.6502
# RUN: sed -e 's/_start/_start2/' %s | llvm-mc -filetype=obj -triple=mos -mcpu=mosr65c02 -o %t.o.r65c02
# RUN: not ld.lld %t.o.6502 %t.o.r65c02 -o %t 2>&1 | FileCheck %s

# returns with 42 in accumulator
.globl _start
_start:
  lda #42
  rts

; CHECK: error: Input file '{{.*\.o\.r65c02}}' uses bad MOS feature combination from rest of output file.
; CHECK-NEXT: Input file: Flags [ (0x19)
; CHECK-NEXT:   EF_MOS_ARCH_6502 (0x1)
; CHECK-NEXT:   EF_MOS_ARCH_65C02 (0x8)
; CHECK-NEXT:   EF_MOS_ARCH_R65C02 (0x10)
; CHECK-NEXT: ]
; CHECK-NEXT: Output file: Flags [ (0x3)
; CHECK-NEXT:   EF_MOS_ARCH_6502 (0x1)
; CHECK-NEXT:   EF_MOS_ARCH_6502_BCD (0x2)
; CHECK-NEXT: ]
