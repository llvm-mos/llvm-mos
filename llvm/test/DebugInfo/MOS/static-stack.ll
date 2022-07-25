; RUN: llc --filetype=obj -o %t < %s
; RUN: llc --filetype=asm -o - < %s | FileCheck %s --check-prefixes=ASM
; RUN: llvm-dwarfdump --debug-info %t | FileCheck %s
; RUN: llvm-dwarfdump --verify %t

; This file was based on output of
;
;   clang -target mos -S -emit-llvm -gdwarf-3 -O0 static-stack.c
;
; for the following static-stack.c
;
;   char f(char a, char b) {
;     return 42;
;   }
;

; ASM-LABEL: f

; ASM:  sta     .Lf_sstk
; ASM:  stx     .Lf_sstk+1

; ASM-LABEL: .Ldebug_info_start0

; ASM:  .byte   3                               ; Abbrev [3] 0x{{.*}}:0x{{.*}} DW_TAG_formal_parameter
; ASM:  .byte   3                               ; DW_AT_location
; (DW_OP_addr .Lf_sstk)
; ASM:  .byte   3
; ASM:  .short  .Lf_sstk

; ASM:  .byte   3                               ; Abbrev [3] 0x{{.*}}:0x{{.*}} DW_TAG_formal_parameter
; ASM:  .byte   5                               ; DW_AT_location
; (DW_OP_addr .Lf_sstk, DW_OP_plus_uconst 0x1)
; ASM:  .byte   3
; ASM:  .short  .Lf_sstk
; ASM:  .byte   35
; ASM:  .byte   1

; CHECK: file format elf32-mos

; CHECK: .debug_info contents:
; CHECK: Compile Unit: length = 0x{{.*}}, format = DWARF32, version = 0x0003, abbr_offset = 0x0000, addr_size = 0x02 (next unit at 0x{{.*}})

; CHECK: DW_TAG_compile_unit
; CHECK:   DW_AT_producer    ("clang version 14.0.0 (https://github.com/llvm/llvm-project ...)")
; CHECK:   DW_AT_language    (DW_LANG_C99)
; CHECK:   DW_AT_name        ("static-stack.c")
; CHECK:   DW_AT_stmt_list   (0x{{.*}})
; CHECK:   DW_AT_comp_dir    ("/tmp")
; CHECK:   DW_AT_low_pc      (0x{{.*}})
; CHECK:   DW_AT_high_pc     (0x{{.*}})

; CHECK:   DW_TAG_subprogram
; CHECK:     DW_AT_low_pc    (0x{{.*}})
; CHECK:     DW_AT_high_pc   (0x{{.*}})
; CHECK:     DW_AT_frame_base        (DW_OP_regx RS0)
; CHECK:     DW_AT_name      ("f")
; CHECK:     DW_AT_decl_file ("/tmp{{[/\\]}}static-stack.c")
; CHECK:     DW_AT_decl_line (1)
; CHECK:     DW_AT_prototyped        (0x01)
; CHECK:     DW_AT_type      (0x{{.*}} "char")
; CHECK:     DW_AT_external  (0x01)

; CHECK:       DW_TAG_formal_parameter
; CHECK:         DW_AT_location        (DW_OP_addr 0x0)
; CHECK:         DW_AT_name    ("a")
; CHECK:         DW_AT_decl_file       ("/tmp{{[/\\]}}static-stack.c")
; CHECK:         DW_AT_decl_line       (1)
; CHECK:         DW_AT_type    (0x{{.*}} "char")

; CHECK:       DW_TAG_formal_parameter
; CHECK:         DW_AT_location        (DW_OP_addr 0x0, DW_OP_plus_uconst 0x1)
; CHECK:         DW_AT_name    ("b")
; CHECK:         DW_AT_decl_file       ("/tmp{{[/\\]}}static-stack.c")
; CHECK:         DW_AT_decl_line       (1)
; CHECK:         DW_AT_type    (0x{{.*}} "char")

; CHECK:       NULL

; CHECK:     DW_TAG_base_type
; CHECK:       DW_AT_name      ("char")
; CHECK:       DW_AT_encoding  (DW_ATE_unsigned_char)
; CHECK:       DW_AT_byte_size (0x01)

; CHECK:     NULL

; ModuleID = 'static-stack.c'
source_filename = "static-stack.c"
target datalayout = "e-m:e-p:16:8-p1:8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mos"

; Function Attrs: noinline nounwind optnone
define dso_local zeroext i8 @f(i8 zeroext %a, i8 zeroext %b) #0 !dbg !7 {
entry:
  %a.addr = alloca i8, align 1
  %b.addr = alloca i8, align 1
  store i8 %a, i8* %a.addr, align 1
  call void @llvm.dbg.declare(metadata i8* %a.addr, metadata !12, metadata !DIExpression()), !dbg !13
  store i8 %b, i8* %b.addr, align 1
  call void @llvm.dbg.declare(metadata i8* %b.addr, metadata !14, metadata !DIExpression()), !dbg !15
  ret i8 42, !dbg !16
}

; Function Attrs: nofree nosync nounwind readnone speculatable willreturn
declare void @llvm.dbg.declare(metadata, metadata, metadata) #1

attributes #0 = { noinline nounwind optnone "frame-pointer"="all" "min-legal-vector-width"="0" "no-trapping-math"="true" "stack-protector-buffer-size"="8" }
attributes #1 = { nofree nosync nounwind readnone speculatable willreturn }

!llvm.dbg.cu = !{!0}
!llvm.module.flags = !{!2, !3, !4, !5}
!llvm.ident = !{!6}

!0 = distinct !DICompileUnit(language: DW_LANG_C99, file: !1, producer: "clang version 14.0.0 (https://github.com/llvm/llvm-project ...)", isOptimized: false, runtimeVersion: 0, emissionKind: FullDebug, splitDebugInlining: false, nameTableKind: None)
!1 = !DIFile(filename: "static-stack.c", directory: "/tmp")
!2 = !{i32 7, !"Dwarf Version", i32 3}
!3 = !{i32 2, !"Debug Info Version", i32 3}
!4 = !{i32 1, !"wchar_size", i32 4}
!5 = !{i32 7, !"frame-pointer", i32 2}
!6 = !{!"clang version 14.0.0 (https://github.com/llvm/llvm-project ...)"}
!7 = distinct !DISubprogram(name: "f", scope: !1, file: !1, line: 1, type: !8, scopeLine: 1, flags: DIFlagPrototyped, spFlags: DISPFlagDefinition, unit: !0, retainedNodes: !11)
!8 = !DISubroutineType(types: !9)
!9 = !{!10, !10, !10}
!10 = !DIBasicType(name: "char", size: 8, encoding: DW_ATE_unsigned_char)
!11 = !{}
!12 = !DILocalVariable(name: "a", arg: 1, scope: !7, file: !1, line: 1, type: !10)
!13 = !DILocation(line: 1, column: 13, scope: !7)
!14 = !DILocalVariable(name: "b", arg: 2, scope: !7, file: !1, line: 1, type: !10)
!15 = !DILocation(line: 1, column: 21, scope: !7)
!16 = !DILocation(line: 2, column: 3, scope: !7)
