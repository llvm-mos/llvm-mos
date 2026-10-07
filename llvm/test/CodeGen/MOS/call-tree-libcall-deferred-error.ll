; RUN: sed 's/@USE@/0/' %s | llc -O2 | FileCheck %s --check-prefix=UNUSED
; RUN: sed 's/@USE@/1/' %s | not llc -O2 2>&1 | FileCheck %s --check-prefix=USED

target datalayout = "e-m:e-p:16:8-p1:8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mos"

; __mulhi3 contains an indirect call, which can't be redirected to a clone. It
; is only speculatively cloned, so this is an error only if the NMI actually
; ends up calling it.
@g = global i16 0
@fp = global ptr null

define i16 @__mulhi3(i16 %a, i16 %b) noinline {
  %f = load ptr, ptr @fp
  %r = call i16 %f(i16 %a, i16 %b)
  ret i16 %r
}

define void @nmi() "interrupt-norecurse" "interrupt-rc-suffix"="__nmi" {
  %a = load volatile i16, ptr @g
  %use = icmp eq i8 @USE@, 1
  br i1 %use, label %mul, label %done
mul:
  %m = mul i16 %a, %a
  store volatile i16 %m, ptr @g
  br label %done
done:
  ret void
}

define void @main() {
  %a = load volatile i16, ptr @g
  %r = call i16 @__mulhi3(i16 %a, i16 %a)
  store volatile i16 %r, ptr @g
  ret void
}

; UNUSED-LABEL: nmi:
; UNUSED-NOT: __mulhi3__nmi

; USED: error:{{.*}}[MOS interrupt] indirect call (function pointer) in the call tree of interrupt handler "nmi" is not supported (function "__mulhi3__nmi")
