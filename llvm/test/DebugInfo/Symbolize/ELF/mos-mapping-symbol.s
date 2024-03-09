# REQUIRES: mos-registered-target
## Ignore MOS mapping symbols (with a prefix of $m or $x).

# RUN: llvm-mc -filetype=obj -triple=mos -mcpu=mosw65816 -motorola-integers %s -o %t

## Verify that mapping symbols are actually present in the object at expected
## addresses.
# RUN: llvm-nm --special-syms %t | FileCheck %s -check-prefix MAPPING

# MAPPING:      00000006 t $mh.2
# MAPPING-NEXT: 00000000 t $ml.0
# MAPPING-NEXT: 00000008 t $xh.3
# MAPPING-NEXT: 00000003 t $xl.1
# MAPPING-NEXT: 00000000 T foo

# RUN: llvm-symbolizer --obj=%t 4 8 | FileCheck %s -check-prefix SYMBOL

# SYMBOL:       foo
# SYMBOL-NEXT:  ??:0:0
# SYMBOL-EMPTY:
# SYMBOL-NEXT:  foo
# SYMBOL-NEXT:  ??:0:0

.globl foo
foo:
  ora #$eaea
  ldx #$eaea
  ora #$ea
  ldx #$ea
