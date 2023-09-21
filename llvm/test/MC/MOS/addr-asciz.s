; RUN: llvm-mc -triple mos -motorola-integers --filetype=obj -o=%t.obj %s
; RUN: llvm-readelf --relocs -x .header %t.obj | FileCheck %s

.section .header,"a",@progbits
  .mos_addr_asciz _start, 5
  .mos_addr_asciz 42, 5
  .mos_addr_asciz 99999999, 8
# CHECK: Relocation section '.rela.header' at offset 0x{{[0-9a-f]+}} contains 1 entries:
# CHECK-NEXT:  Offset     Info    Type                Sym. Value  Symbol's Name + Addend
# CHECK-NEXT: 00000000  0000010f R_MOS_ADDR_ASCIZ       00000000   _start + 0
# CHECK-LABEL: section '.header':
# CHECK-NEXT: 0x00000000 30000000 00003432 00000000 39393939 0.....42....9999
# CHECK-NEXT: 0x00000010 39393939 00                         9999.
