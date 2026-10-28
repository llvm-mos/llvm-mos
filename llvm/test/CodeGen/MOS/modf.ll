; RUN: llc -mtriple=mos -O2 -verify-machineinstrs < %s | FileCheck %s

; The IRTranslator turns llvm.modf into G_FMODF, which becomes a call to the C
; library's modff/modf. The integer part comes back through a stack temporary
; whose address is passed as the pointer argument and which is then copied to
; the caller's pointer.

define float @modf_f32(float %x, ptr %ip) {
; CHECK-LABEL: modf_f32:
; CHECK:         ldy #mos16lo(.Lmodf_f32_sstk)
; CHECK-NEXT:    sty __rc4
; CHECK-NEXT:    ldy #mos16hi(.Lmodf_f32_sstk)
; CHECK-NEXT:    sty __rc5
; CHECK-NEXT:    jsr modff
; CHECK:         lda .Lmodf_f32_sstk
; CHECK:         ldy .Lmodf_f32_sstk+3
; CHECK:         sta (__rc20),y
; CHECK:         rts
  %r = call { float, float } @llvm.modf.f32(float %x)
  %frac = extractvalue { float, float } %r, 0
  %int = extractvalue { float, float } %r, 1
  store float %int, ptr %ip
  ret float %frac
}

define double @modf_f64(double %x, ptr %ip) {
; CHECK-LABEL: modf_f64:
; CHECK:         ldy #mos16lo(.Lmodf_f64_sstk)
; CHECK-NEXT:    lda #mos16hi(.Lmodf_f64_sstk)
; CHECK-NEXT:    sty __rc8
; CHECK-NEXT:    sta __rc9
; CHECK:         jsr modf
; CHECK:         lda .Lmodf_f64_sstk
; CHECK:         ldy .Lmodf_f64_sstk+7
; CHECK:         sta (__rc20),y
; CHECK:         rts
  %r = call { double, double } @llvm.modf.f64(double %x)
  %frac = extractvalue { double, double } %r, 0
  %int = extractvalue { double, double } %r, 1
  store double %int, ptr %ip
  ret double %frac
}
