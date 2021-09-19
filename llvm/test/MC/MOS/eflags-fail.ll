; RUN: not --crash llc -mtriple=mos -filetype=obj < %s 2>&1 | FileCheck %s

define void @func0() #0 {
entry:
  ret void
}

define void @func1() #1 {
entry:
  ret void
}

; R65C02 and BCD features cannot be mixed
attributes #0 = { "target-cpu"="mos6502" }
attributes #1 = { "target-cpu"="mosr65c02" }

; CHECK: error: Function 'func1' uses bad MOS feature combination from rest of module.
; CHECK-NEXT: Function: Flags [ (0x19)
; CHECK-NEXT:   EF_MOS_ARCH_6502 (0x1)
; CHECK-NEXT:   EF_MOS_ARCH_65C02 (0x8)
; CHECK-NEXT:   EF_MOS_ARCH_R65C02 (0x10)
; CHECK-NEXT: ]
; CHECK-NEXT: Module: Flags [ (0x3)
; CHECK-NEXT:   EF_MOS_ARCH_6502 (0x1)
; CHECK-NEXT:   EF_MOS_ARCH_6502_BCD (0x2)
; CHECK-NEXT: ]
