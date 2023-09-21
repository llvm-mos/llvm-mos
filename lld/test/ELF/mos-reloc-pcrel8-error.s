# REQUIRES: mos
# RUN: llvm-mc -filetype=obj -triple=mos -motorola-integers -mos-force-pcrel-reloc %s -o %t.o
# RUN: not ld.lld %t.o -o %t 2> %t.err
# RUN: FileCheck %s --input-file %t.err

.section .R_MOS_PCREL_8,"ax",@progbits

  bpl in_range
  .fill 127
in_range:
  .fill 126
  bpl in_range

; CHECK: (.R_MOS_PCREL_8+0x102): relocation R_MOS_PCREL_8 out of range: 128 is not in [-128, 127]
  bpl out_of_range
  .fill 128
; CHECK: (.R_MOS_PCREL_8+0x203): relocation R_MOS_PCREL_8 out of range: -129 is not in [-128, 127]
out_of_range:
  .fill 127
  bpl out_of_range
