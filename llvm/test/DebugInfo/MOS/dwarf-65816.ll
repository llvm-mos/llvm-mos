; ROADMAP step 6 / #321: llvm-mos emits correct DWARF for the WDC 65816 (+mos-a16).
; Asserts the 65816-specific SHAPES (not exact addresses — RA may drift):
;   * 4-byte address size (CodePointerSize=4 conveys the 24-bit banking)
;   * the soft-stack frame base is imaginary register RS0 (DW_OP_regx RS0)
;   * 16-bit params/locals are located in imaginary-register pairs (DW_OP_regx RSn) —
;     the novel 65816 case: a 16-bit value lives in an Imag16 ZP pair, not on a stack
;   * the line table maps source lines with prologue_end + end_sequence
;   * llvm-dwarfdump --verify is clean
;
; IR captured from: mos-clang --target=mos -mcpu=mosw65816 -Xclang -target-feature
;   -Xclang +mos-a16 -g -Os -S -emit-llvm -fdebug-compilation-dir=. dwarf-65816.c
; where dwarf-65816.c is:
;   unsigned short add16(unsigned short a, unsigned short b) {
;     unsigned short t = a + b;
;     return t;
;   }
;
; NOTE: this lives in dev/lit/ (tracked) rather than vendor/llvm-mos/llvm/test/,
; because dev/regen-patch.sh mirrors only llvm/lib/Target/MOS into the tracked
; 0002 patch — a file under llvm/test/ would be lost on a clean vendor rebuild.
; It is staged here for the upstream llvm-mos PR (drop into test/DebugInfo/MOS/).
; The durable in-repo guard is dev/run.sh dwarf.

; RUN: llc --filetype=obj -mtriple=mos -mcpu=mosw65816 -o %t %s
; RUN: llvm-dwarfdump --debug-info %t | FileCheck %s
; RUN: llvm-dwarfdump --debug-line %t | FileCheck %s --check-prefix=LINE
; RUN: llvm-dwarfdump --verify %t

; CHECK: addr_size = 0x04
; CHECK: DW_TAG_subprogram
; CHECK: DW_AT_frame_base (DW_OP_regx RS0)
; CHECK: DW_AT_name ("add16")
; A 16-bit parameter lives in an imaginary-register pair (DW_OP_regx RSn).
; CHECK: DW_TAG_formal_parameter
; CHECK: DW_AT_location (DW_OP_regx RS{{[0-9]+}})
; CHECK: DW_AT_name ("a")
; The 16-bit local 't' has a location (register loclist under -Os).
; CHECK: DW_TAG_variable
; CHECK: DW_AT_location
; CHECK: DW_OP_regx RS{{[0-9]+}}
; CHECK: DW_AT_name ("t")

; LINE: prologue_end
; LINE: end_sequence

; ModuleID = 'dwarf-65816.c'
source_filename = "dwarf-65816.c"
target datalayout = "e-m:e-p:16:8-p1:8:8-p2:32:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mos"

; Function Attrs: mustprogress nofree norecurse nosync nounwind optsize willreturn memory(none)
define dso_local noundef zeroext i16 @add16(i16 noundef zeroext %0, i16 noundef zeroext %1) local_unnamed_addr #0 !dbg !10 {
    #dbg_value(i16 %0, !15, !DIExpression(), !18)
    #dbg_value(i16 %1, !16, !DIExpression(), !18)
  %3 = add i16 %1, %0, !dbg !19
    #dbg_value(i16 %3, !17, !DIExpression(), !18)
  ret i16 %3, !dbg !20
}

attributes #0 = { mustprogress nofree norecurse nosync nounwind optsize willreturn memory(none) "frame-pointer"="all" "no-trapping-math"="true" "stack-protector-buffer-size"="8" "target-cpu"="mosw65816" "target-features"="+mos-a16" }

!llvm.dbg.cu = !{!0}
!llvm.module.flags = !{!2, !3, !4}
!llvm.ident = !{!5}
!llvm.errno.tbaa = !{!6}

!0 = distinct !DICompileUnit(language: DW_LANG_C11, file: !1, producer: "llvm-mos", isOptimized: true, runtimeVersion: 0, emissionKind: FullDebug, splitDebugInlining: false, nameTableKind: None)
!1 = !DIFile(filename: "dwarf-65816.c", directory: ".")
!2 = !{i32 7, !"Dwarf Version", i32 5}
!3 = !{i32 2, !"Debug Info Version", i32 3}
!4 = !{i32 7, !"frame-pointer", i32 2}
!5 = !{!"llvm-mos"}
!6 = !{!7, !7, i64 0}
!7 = !{!"int", !8, i64 0}
!8 = !{!"omnipotent char", !9, i64 0}
!9 = !{!"Simple C/C++ TBAA"}
!10 = distinct !DISubprogram(name: "add16", scope: !1, file: !1, line: 1, type: !11, scopeLine: 1, flags: DIFlagPrototyped | DIFlagAllCallsDescribed, spFlags: DISPFlagDefinition | DISPFlagOptimized, unit: !0, retainedNodes: !14, keyInstructions: true)
!11 = !DISubroutineType(types: !12)
!12 = !{!13, !13, !13}
!13 = !DIBasicType(name: "unsigned short", size: 16, encoding: DW_ATE_unsigned)
!14 = !{!15, !16, !17}
!15 = !DILocalVariable(name: "a", arg: 1, scope: !10, file: !1, line: 1, type: !13)
!16 = !DILocalVariable(name: "b", arg: 2, scope: !10, file: !1, line: 1, type: !13)
!17 = !DILocalVariable(name: "t", scope: !10, file: !1, line: 2, type: !13)
!18 = !DILocation(line: 0, scope: !10)
!19 = !DILocation(line: 2, column: 24, scope: !10, atomGroup: 1, atomRank: 2)
!20 = !DILocation(line: 3, column: 3, scope: !10, atomGroup: 3, atomRank: 1)
