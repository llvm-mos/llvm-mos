// RUN: llvm-mc %s -o %t.o -filetype=obj -triple mos -mcpu=mosw65816 -motorola-integers
// RUN: llvm-readobj --symbols %t.o | FileCheck %s
// RUN: llvm-nm %t.o | FileCheck -allow-empty --check-prefix=NM %s

// Test that nm doesn't print the mapping symbols

// CHECK: Name: $ml.0
// NM-NOT: $ml.0

        .section        .foobar,"",%progbits
        lda #$eaea
