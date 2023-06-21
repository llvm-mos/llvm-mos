# REQUIRES: mos
# RUN: llvm-mc -filetype=obj -triple=mos %s -o %t

# RUN: echo "OUTPUT_FORMAT { BYTE(0x11) SHORT(0x2233) LONG(0x44556677) QUAD(0x8899aabbccddeeff)}" > %t.script
# RUN: ld.lld -o %t2.out --script %t.script %t
# RUN: od -t x1 -v %t2.out | FileCheck --check-prefix=BYTES-CHECK %s
# RUN: ls %t2.out.elf | count 1

# BYTES-CHECK:      000000 11 33 22 77 66 55 44 ff ee dd cc bb aa 99 88
# BYTES-CHECK-NEXT: 000017

# RUN: echo "MEMORY {lma : ORIGIN = 0x1000 , LENGTH = 8 vma : ORIGIN = 0x2000, LENGTH = 8} SECTIONS { .mysec : AT (0x1001) { *(.mysec.*) } >vma } OUTPUT_FORMAT {BYTE(0x11) FULL(lma) BYTE(0xff)}" > %t.script
# RUN: ld.lld -o %t2.out --script %t.script %t
# RUN: od -t x1 -v %t2.out | FileCheck --check-prefix=FULL-CHECK %s
# RUN: ls %t2.out.elf | count 1

# FULL-CHECK:      000000 11 00 11 22 33 00 00 00 00 ff
# FULL-CHECK-NEXT: 000012

# RUN: echo "MEMORY {lma : ORIGIN = 0x1000 , LENGTH = 8 vma : ORIGIN = 0x2000, LENGTH = 8} SECTIONS { .mysec : AT (0x1001) { *(.mysec.*) } >vma } OUTPUT_FORMAT {BYTE(0x11) TRIM(lma) BYTE(0xff)}" > %t.script
# RUN: ld.lld -o %t2.out --script %t.script %t
# RUN: od -t x1 -v %t2.out | FileCheck --check-prefix=TRIM-CHECK %s
# RUN: ls %t2.out.elf | count 1

# TRIM-CHECK:      000000 11 00 11 22 33 ff
# TRIM-CHECK-NEXT: 000006

# RUN: echo "MEMORY {lma : ORIGIN = 0x1000, LENGTH = 1 vma : ORIGIN = 0x2000, LENGTH = 8} SECTIONS { .mysec : AT (0x0fff) { *(.mysec.*) } >vma } OUTPUT_FORMAT {BYTE(0x11) FULL(lma) BYTE(0xff)}" > %t.script
# RUN: ld.lld -o %t2.out --script %t.script %t
# RUN: od -t x1 -v %t2.out | FileCheck --check-prefix=TRUNCATE-CHECK %s
# RUN: ls %t2.out.elf | count 1

# TRUNCATE-CHECK:      000000 11 22 ff
# TRUNCATE-CHECK-NEXT: 000003

# RUN: echo "MEMORY {lma : ORIGIN = 0x1000 , LENGTH = 8 vma : ORIGIN = 0x2000, LENGTH = 8} SECTIONS { .mysec : AT (0x1001) { *(.mysec.*) } >vma } OUTPUT_FORMAT {BYTE(0x11) FULL(lma, 1, 5) BYTE(0xff)}" > %t.script
# RUN: ld.lld -o %t2.out --script %t.script %t
# RUN: od -t x1 -v %t2.out | FileCheck --check-prefix=FULL-PART-CHECK %s
# RUN: ls %t2.out.elf | count 1

# FULL-PART-CHECK:      000000 11 11 22 33 00 00 ff
# FULL-PART-CHECK-NEXT: 000007

# RUN: echo "MEMORY {lma : ORIGIN = 0x1000 , LENGTH = 8 vma : ORIGIN = 0x2000, LENGTH = 8} SECTIONS { .mysec : AT (0x1001) { *(.mysec.*) } >vma } OUTPUT_FORMAT {BYTE(0x11) TRIM(lma, 1, 4) BYTE(0xff)}" > %t.script
# RUN: ld.lld -o %t2.out --script %t.script %t
# RUN: od -t x1 -v %t2.out | FileCheck --check-prefix=TRIM-PART-CHECK %s
# RUN: ls %t2.out.elf | count 1

# TRIM-PART-CHECK:      000000 11 11 22 33 ff
# TRIM-PART-CHECK-NEXT: 000005

.section        .mysec.1,"ax"
.byte   0x11

.section        .mysec.2,"ax"
.byte   0x22, 0x33
