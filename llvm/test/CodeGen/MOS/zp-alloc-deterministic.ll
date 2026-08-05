; RUN: llc -mtriple=mos -zp-avail=4 -verify-machineinstrs < %s | FileCheck %s

; Zero page allocation must be deterministic.
;
; Every one of these globals is stored exactly once from the same basic block,
; so MOSZeroPageAlloc computes the identical benefit (2 * 1.0 / 1) for all eight
; and only four of them fit in the zero page. The winners are therefore decided
; entirely by the order in which candidates were collected, which must be the
; MachineInstr walk order -- i.e. module order, g0 through g3.
;
; Before the fix, collectCandidates() accumulated benefits in a
; DenseMap<GlobalVariable *, float> and iterated it to build the candidate list;
; that iterates in pointer-hash order, i.e. by heap address, and the subsequent
; stable_sort on benefit leaves ties in exactly that order. The winning set then
; differed on every process execution: twenty runs of this file produced six
; distinct winning sets, none of them the module-order set below (this test's
; expectations failed all twenty pre-fix runs).

target triple = "mos"

@g0 = global i8 undef, align 1
@g1 = global i8 undef, align 1
@g2 = global i8 undef, align 1
@g3 = global i8 undef, align 1
@g4 = global i8 undef, align 1
@g5 = global i8 undef, align 1
@g6 = global i8 undef, align 1
@g7 = global i8 undef, align 1

; The first four globals collected win the zero page and are addressed with
; mos8(); the rest keep absolute addressing.

; CHECK-LABEL: main:
; CHECK:      stx mos8(g0)
; CHECK:      stx mos8(g1)
; CHECK:      stx mos8(g2)
; CHECK:      stx mos8(g3)
; CHECK:      stx g4
; CHECK:      stx g5
; CHECK:      stx g6
; CHECK:      stx g7

; CHECK:      .type g0,@object
; CHECK-NEXT: .section .zp.noinit
; CHECK:      .type g4,@object
; CHECK-NEXT: .section .noinit

define void @main() {
entry:
  store volatile i8 1, ptr @g0, align 1
  store volatile i8 2, ptr @g1, align 1
  store volatile i8 3, ptr @g2, align 1
  store volatile i8 4, ptr @g3, align 1
  store volatile i8 5, ptr @g4, align 1
  store volatile i8 6, ptr @g5, align 1
  store volatile i8 7, ptr @g6, align 1
  store volatile i8 8, ptr @g7, align 1
  ret void
}
