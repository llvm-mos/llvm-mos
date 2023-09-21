# REQUIRES: mos
# RUN: llvm-mc -filetype=obj -triple=mos -motorola-integers %s -o %t.o
# RUN: ld.lld %t.o --defsym=adr16=12345 -o %t
# RUN: llvm-readelf -x .R_MOS_ADDR_ASCIZ %t | FileCheck %s

.section .R_MOS_ADDR_ASCIZ,"a",@progbits
  .mos_addr_asciz adr16, 5
# CHECK-LABEL: section '.R_MOS_ADDR_ASCIZ':
# CHECK-NEXT: 0x000100b4 31323334 3500 12345.
