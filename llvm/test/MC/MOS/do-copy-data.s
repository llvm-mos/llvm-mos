; RUN: llvm-mc -triple mos < %s | FileCheck %s
.data
.fill 1

; CHECK: Declaring this symbol
; CHECK: __do_copy_data
