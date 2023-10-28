; RUN: not ld.lld -m moself -r %p/Inputs/ca65-dummy.o 2>&1 | FileCheck %s
; CHECK: error: {{.*}}ca65-dummy.o: cc65 (xo65) object file cannot be relocatably linked
