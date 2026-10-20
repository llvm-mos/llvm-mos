; RUN: llc -mtriple=mos -filetype=obj < %s -o %t.o
; RUN: llvm-dwarfdump --debug-frame --eh-frame %t.o | FileCheck %s

; The CIE describes the MOS frame at function entry. LLDB's MOS ABI decodes
; these same initial instructions from MCAsmInfo for its function-entry unwind
; plan, so this is the only place the rules are written down.
;
;   CFA       = normalized(S + 3)
;   caller PC = byte(normalized(S + 1)) | byte(normalized(S + 2)) << 8, + 1
;   caller S  = normalized(S + 2)
;   RS0       = same value
; where normalized(x) = (x & 0xff) | 0x100.

target triple = "mos"

define void @leaf() {
  ret void
}

; CHECK:      CIE
; CHECK:      Return address column: 5
; CHECK:      DW_CFA_def_cfa_expression: DW_OP_breg4 S+3, DW_OP_const1u 0xff, DW_OP_and, DW_OP_const2u 0x100, DW_OP_or
; CHECK-NEXT: DW_CFA_val_expression: PC DW_OP_breg4 S+1, DW_OP_const1u 0xff, DW_OP_and, DW_OP_const2u 0x100, DW_OP_or, DW_OP_deref_size 0x1, DW_OP_breg4 S+2, DW_OP_const1u 0xff, DW_OP_and, DW_OP_const2u 0x100, DW_OP_or, DW_OP_deref_size 0x1, DW_OP_const1u 0x8, DW_OP_shl, DW_OP_or, DW_OP_plus_uconst 0x1
; CHECK-NEXT: DW_CFA_val_expression: S DW_OP_breg4 S+2, DW_OP_const1u 0xff, DW_OP_and, DW_OP_const2u 0x100, DW_OP_or
; CHECK-NEXT: DW_CFA_same_value: RS0
