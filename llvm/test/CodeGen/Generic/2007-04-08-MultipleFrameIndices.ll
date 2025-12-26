; RUN: llc -no-integrated-as < %s
; PR1308
; PR1557

; MOS uses GlobalISel which doesn't support inline asm with multi-register
; tied operands. The "0" constraint ties an input to the i32 output, but MOS
; needs 4 registers for i32 (8-bit registers). GlobalISel's InlineAsmLowering
; asserts that tied operands use exactly 1 register. See the FIXME comment in
; llvm/lib/CodeGen/GlobalISel/InlineAsmLowering.cpp.
; UNSUPPORTED: target=mos{{.*}}

; Bug: PR31336

define i32 @stuff(i32, ...) {
        %foo = alloca ptr
        %bar = alloca ptr
        %A = call i32 asm sideeffect "inline asm $0 $2 $3 $4", "=r,0,i,m,m"( i32 0, i32 1, ptr %foo, ptr %bar )
        ret i32 %A
}
