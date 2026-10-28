; RUN: llc -mtriple=mos -O2 -enable-tail-merge=0 < %s \
; RUN:   | FileCheck %s --check-prefix=ASM
; RUN: llc -mtriple=mos -O2 -enable-tail-merge=0 -filetype=obj < %s -o %t.o
; RUN: llvm-dwarfdump --eh-frame %t.o | FileCheck %s --check-prefix=CFI \
; RUN:   --implicit-check-not=DW_CFA_def_cfa: \
; RUN:   --implicit-check-not=DW_CFA_val_offset

; The CFA is always based on the hardware stack; the soft stack is described
; by value expressions for the caller's RS0 and by expressions for the save
; slots of soft-stack callee-saved registers. Each check below ties a CFI rule
; to the code it describes.

declare void @ext()
declare void @use(ptr)
declare i8 @g(i8)

; A soft-stack frame without a frame pointer: the caller's RS0 is RS0 plus the
; frame size from the instruction after the prologue's decrement until the
; epilogue's increment, and the CIE's rule afterwards.
define void @soft_frame(i8 %n) {
; The prologue subtracts 4 from RS0 (adding 0xfffc); the caller's RS0 is RS0+4
; from the next instruction on.
; ASM-LABEL: soft_frame:
; ASM:         adc #252
; ASM-NEXT:    sta __rc0
; ASM-NEXT:    lda __rc1
; ASM-NEXT:    adc #255
; ASM-NEXT:    sta __rc1
; ASM-NEXT:    .cfi_escape 0x16, 0x80, 0x80, 0x0c, 0x05, 0x92, 0x80, 0x80, 0x0c, 0x04
; The epilogue adds 4 back; RS0 then holds the caller's value.
; ASM:         adc #4
; ASM-NEXT:    sta __rc0
; ASM-NEXT:    lda __rc1
; ASM-NEXT:    adc #0
; ASM-NEXT:    sta __rc1
; ASM-NEXT:    .cfi_restore llvm_mos_rs0
; ASM-NEXT:    rts
; CFI:       FDE
; CFI:         DW_CFA_val_expression: RS0 DW_OP_bregx RS0+4
; CFI:         DW_CFA_restore_extended: RS0
  %buf = alloca [4 x i8]
  call void @use(ptr %buf)
  %m = add i8 %n, -1
  call void @soft_frame(i8 %m)
  ret void
}

; A frame pointer and a variable-sized object: once the frame pointer is set,
; RS0 moves in the body, so the caller's RS0 is described relative to RS15.
define void @fp_frame(i16 %n) {
; ASM-LABEL: fp_frame:
; ASM:         ldy __rc0
; ASM-NEXT:    sty __rc30
; ASM-NEXT:    ldy __rc1
; ASM-NEXT:    sty __rc31
; ASM-NEXT:    .cfi_escape 0x16, 0x80, 0x80, 0x0c, 0x05, 0x92, 0x8f, 0x80, 0x0c, 0x00
; ASM:         jsr use
; ASM-NEXT:    ldx __rc30
; ASM-NEXT:    stx __rc0
; ASM-NEXT:    ldx __rc31
; ASM-NEXT:    stx __rc1
; ASM-NEXT:    .cfi_escape 0x16, 0x80, 0x80, 0x0c, 0x05, 0x92, 0x80, 0x80, 0x0c, 0x00
; ASM-NEXT:    pla
; CFI:       FDE
; CFI:         DW_CFA_val_expression: RS0 DW_OP_bregx RS15+0
; CFI:         DW_CFA_val_expression: RS0 DW_OP_bregx RS0+0
; CFI:         DW_CFA_restore_extended: RS0
  %vla = alloca i8, i16 %n
  call void @use(ptr %vla)
  ret void
}

; An interrupt handler that calls out saves registers beyond the four that go
; to the hardware stack on the soft stack, below a frame that also covers the
; extra page subtracted for atomicity. The save slot rules must name the slots
; the stores actually write.
define void @isr() "interrupt" {
; The interrupt-entry layout is described from the first instruction, before
; the CLD: CFA = S+4, PC from S+2/S+3 (no +1), P at S+1.
; ASM-LABEL: isr:
; ASM:         .cfi_startproc
; ASM-NEXT:  ; %bb.0:
; ASM-NEXT:    .cfi_escape 0x0f, 0x09, 0x74, 0x04,
; ASM-NEXT:    .cfi_escape 0x16, 0x05, 0x1a, 0x74, 0x02,
; ASM-NEXT:    .cfi_escape 0x16, 0x04, 0x09, 0x74, 0x03,
; ASM-NEXT:    .cfi_escape 0x10, 0x03, 0x09, 0x74, 0x01,
; ASM-NEXT:    cld
; The prologue subtracts 17 + 256 = 273 (adding 0xfeef).
; ASM:         adc #239
; ASM-NEXT:    sta __rc0
; ASM-NEXT:    lda __rc1
; ASM-NEXT:    adc #254
; ASM-NEXT:    sta __rc1
; ASM-NEXT:    .cfi_escape 0x16, 0x80, 0x80, 0x0c, 0x06, 0x92, 0x80, 0x80, 0x0c, 0x91, 0x02
; The first soft-stack save writes RC3 to (RS0),16 and the last writes RC19 to
; (RS0),0. The extra page is not part of these offsets.
; ASM:         lda __rc3
; ASM-NEXT:    ldy #16
; ASM-NEXT:    sta (__rc0),y
; ASM:         lda __rc19
; ASM-NEXT:    dey
; ASM-NEXT:    sta (__rc0),y
; ASM:         rti
; CFI:       FDE
; CFI:         DW_CFA_val_expression: RS0 DW_OP_bregx RS0+273
; CFI:         DW_CFA_expression: RC3 DW_OP_bregx RS0+16
; CFI:         DW_CFA_expression: RC19 DW_OP_bregx RS0+0
; CFI:         DW_CFA_restore_extended: RC3
; CFI:         DW_CFA_restore_extended: RC19
; CFI-NEXT:    DW_CFA_advance_loc:
; CFI-NEXT:    DW_CFA_restore_extended: RS0
  call void @ext()
  ret void
}

; With more than one epilogue, a block laid out after the first one still runs
; inside the frame; CFIFixup must restore the frame state for it. Tail merging
; would otherwise merge the two epilogues into one.
define i8 @two_epilogues(i8 %n) {
; ASM-LABEL: two_epilogues:
; ASM:         pha
; ASM-NEXT:    .cfi_escape
; ASM-NEXT:    .cfi_escape
; ASM-NEXT:    .cfi_escape
; ASM-NEXT:    .cfi_remember_state
; ASM:         .cfi_restore llvm_mos_rs0
; ASM:         rts
; ASM-NEXT:  .LBB3_2:
; ASM-NEXT:    .cfi_restore_state
; In the row table: after the entry row, the first row back in the entry state
; (CFA = S+3, RS0 = same) is the one after the first epilogue; the next block
; is back inside the frame (CFA = S+4, RS0 = RS0+4).
; CFI:       FDE
; CFI:         DW_CFA_remember_state:
; CFI:         DW_CFA_restore_state:
; CFI:         CFA=DW_OP_breg4 S+3,{{.*}} RS0=same
; CFI:         CFA=DW_OP_breg4 S+3,{{.*}} RS0=same
; CFI-NEXT:    CFA=DW_OP_breg4 S+4,{{.*}} RS0=DW_OP_bregx RS0+4
entry:
  %buf = alloca [4 x i8]
  call void @use(ptr %buf)
  %r = call i8 @g(i8 %n)
  %c = icmp eq i8 %r, 5
  br i1 %c, label %a, label %b, !prof !0
a:
  ret i8 %r
b:
  %t = call i8 @g(i8 %r)
  %u = add i8 %t, %n
  ret i8 %u
}

!0 = !{!"branch_weights", i32 1000, i32 1}
