# REQUIRES: mos
# RUN: llvm-mc -filetype=obj -triple=mos -motorola-integers -mos-force-pcrel-reloc %s -o %t.o
# RUN: not ld.lld --defsym=n1=-1 --defsym=large=1234567890 %t.o --threads=1 -o %t 2> %t.err
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

; CHECK: (.R_MOS_IMM_8+0x1): relocation R_MOS_IMM8 out of range: 1234567890 is not in [-128, 255]
.section .R_MOS_IMM_8,"ax",@progbits
  adc #large

; CHECK: (.R_MOS_ADDR8+0x1): relocation R_MOS_ADDR8 out of range: 722 is not in [0, 255]
.section .R_MOS_ADDR8,"ax",@progbits
  adc mos8(large)

; CHECK: (.R_MOS_ADDR24_SEGMENT_LO+0x1): relocation R_MOS_ADDR24_SEGMENT_LO out of range: 1234567890 is not in [0, 16777215]
.section .R_MOS_ADDR24_SEGMENT_LO,"ax",@progbits
  adc #mos24segmentlo(large)

; CHECK: (.R_MOS_ADDR24_SEGMENT_HI+0x1): relocation R_MOS_ADDR24_SEGMENT_HI out of range: 1234567890 is not in [0, 16777215]
.section .R_MOS_ADDR24_SEGMENT_HI,"ax",@progbits
  adc #mos24segmenthi(large)

; CHECK: (.R_MOS_ADDR24_BANK+0x1): relocation R_MOS_ADDR24_BANK out of range: 1234567890 is not in [0, 16777215]
.section .R_MOS_ADDR24_BANK,"ax",@progbits
  adc #mos24bank(large)
