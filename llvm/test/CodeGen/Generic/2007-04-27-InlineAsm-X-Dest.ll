; RUN: llc -no-integrated-as < %s

; MOS uses GlobalISel which doesn't support indirect output constraints (=*).
; The "=*X" constraint writes through a pointer and returns void, but
; GlobalISel's InlineAsmLowering counts it as an output operand expecting a
; return register. See the FIXME comment in
; llvm/lib/CodeGen/GlobalISel/InlineAsmLowering.cpp near "ResRegs.size()".
; UNSUPPORTED: target=mos{{.*}}

; Test that we can have an "X" output constraint.

define void @test(ptr %t) {
        call void asm sideeffect "foo $0", "=*X,~{dirflag},~{fpsr},~{flags},~{memory}"( ptr elementtype( i16) %t )
        ret void
}
