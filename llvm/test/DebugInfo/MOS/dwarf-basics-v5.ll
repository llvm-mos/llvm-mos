; RUN: llc -generate-arange-section -minimize-addr-in-v5=Ranges --filetype=obj -o %t < %s
; RUN: llvm-dwarfdump --debug-info -debug-aranges -debug-addr %t | FileCheck %s
; RUN: llvm-dwarfdump --verify %t

; This file was based on output of
;
;   clang -target mos -S -emit-llvm -gdwarf-5 -Os dwarf-basics-v5.c
;
; for the following dwarf-basics-v5.c
;
;   struct X {
;     void *a;
;   };
;
;   char f(char y, struct X *p)
;   {
;     return 42;
;   }
;

; CHECK: file format elf32-mos

; CHECK: .debug_info contents:
; CHECK: Compile Unit: length = 0x{{.*}}, format = DWARF32, version = 0x0005, unit_type = DW_UT_compile, abbr_offset = 0x0000, addr_size = 0x04 (next unit at 0x{{.*}})

; CHECK: DW_TAG_compile_unit
; CHECK:   DW_AT_producer    ("clang version 14.0.0 (https://github.com/llvm/llvm-project ...)")
; CHECK:   DW_AT_language    (DW_LANG_C99)
; CHECK:   DW_AT_name        ("dwarf-basics-v5.c")
; CHECK:   DW_AT_str_offsets_base    (0x00000008)
; CHECK:   DW_AT_stmt_list   (0x{{.*}})
; CHECK:   DW_AT_comp_dir    ("/tmp")
; CHECK:   DW_AT_low_pc      (0x{{.*}})
; CHECK:   DW_AT_high_pc     (0x{{.*}})
; CHECK:   DW_AT_addr_base   (0x00000008)

; CHECK:   DW_TAG_subprogram
; CHECK:     DW_AT_low_pc    (0x{{.*}})
; CHECK:     DW_AT_high_pc   (0x{{.*}})
; CHECK:     DW_AT_frame_base        (DW_OP_regx RS0)
; CHECK:     DW_AT_call_all_calls    (true)
; CHECK:     DW_AT_name      ("f")
; CHECK:     DW_AT_decl_file ("/tmp{{[/\\]}}dwarf-basics-v5.c")
; CHECK:     DW_AT_decl_line (5)
; CHECK:     DW_AT_prototyped (true)
; CHECK:     DW_AT_type      (0x{{.*}} "char")
; CHECK:     DW_AT_external  (true)

; CHECK:       DW_TAG_formal_parameter
; CHECK:         DW_AT_name    ("y")
; CHECK:         DW_AT_decl_file       ("/tmp{{[/\\]}}dwarf-basics-v5.c")
; CHECK:         DW_AT_decl_line       (5)
; CHECK:         DW_AT_type    (0x{{.*}} "char")

; CHECK:       DW_TAG_formal_parameter
; CHECK:         DW_AT_name    ("p")
; CHECK:         DW_AT_decl_file       ("/tmp{{[/\\]}}dwarf-basics-v5.c")
; CHECK:         DW_AT_decl_line       (5)
; CHECK:         DW_AT_type    (0x{{.*}} "X *")

; CHECK:       NULL

; CHECK:     DW_TAG_base_type
; CHECK:       DW_AT_name      ("char")
; CHECK:       DW_AT_encoding  (DW_ATE_unsigned_char)
; CHECK:       DW_AT_byte_size (0x01)

; CHECK:     DW_TAG_pointer_type
; CHECK:       DW_AT_type      (0x{{.*}} "X")

; CHECK:     DW_TAG_structure_type
; CHECK:       DW_AT_name      ("X")
; CHECK:       DW_AT_byte_size (0x02)
; CHECK:       DW_AT_decl_file ("/tmp{{[/\\]}}dwarf-basics-v5.c")
; CHECK:       DW_AT_decl_line (1)

; CHECK:       DW_TAG_member
; CHECK:         DW_AT_name    ("a")
; CHECK:         DW_AT_type    (0x{{.*}} "void *")
; CHECK:         DW_AT_decl_file       ("/tmp{{[/\\]}}dwarf-basics-v5.c")
; CHECK:         DW_AT_decl_line       (2)
; CHECK:         DW_AT_data_member_location    (0x00)

; CHECK:       NULL

; CHECK:     DW_TAG_pointer_type

; CHECK:     NULL

; CHECK:      .debug_addr contents:
; CHECK-NEXT: Address table header: length = 0x{{.*}}, format = DWARF32, version = 0x0005, addr_size = 0x04, seg_size = 0x00
; CHECK-NEXT: Addrs: [
; CHECK-NEXT: 0x0000
; CHECK-NEXT: ]

; ModuleID = 'dwarf-basics-v5.c'
source_filename = "dwarf-basics-v5.c"
target datalayout = "e-m:e-p:16:8-p1:8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mos"

%struct.X = type { i8* }

; Function Attrs: mustprogress nofree norecurse nosync nounwind optsize readnone willreturn
define dso_local zeroext i8 @f(i8 zeroext %y, %struct.X* nocapture readnone %p) local_unnamed_addr #0 !dbg !7 {
entry:
  call void @llvm.dbg.value(metadata i8 %y, metadata !17, metadata !DIExpression()), !dbg !19
  call void @llvm.dbg.value(metadata %struct.X* %p, metadata !18, metadata !DIExpression()), !dbg !19
  ret i8 42, !dbg !20
}

; Function Attrs: nofree nosync nounwind readnone speculatable willreturn
declare void @llvm.dbg.value(metadata, metadata, metadata) #1

attributes #0 = { mustprogress nofree norecurse nosync nounwind optsize readnone willreturn "frame-pointer"="all" "min-legal-vector-width"="0" "no-trapping-math"="true" "stack-protector-buffer-size"="8" }
attributes #1 = { nofree nosync nounwind readnone speculatable willreturn }

!llvm.dbg.cu = !{!0}
!llvm.module.flags = !{!2, !3, !4, !5}
!llvm.ident = !{!6}

!0 = distinct !DICompileUnit(language: DW_LANG_C99, file: !1, producer: "clang version 14.0.0 (https://github.com/llvm/llvm-project ...)", isOptimized: true, runtimeVersion: 0, emissionKind: FullDebug, splitDebugInlining: false, nameTableKind: None)
!1 = !DIFile(filename: "dwarf-basics-v5.c", directory: "/tmp", checksumkind: CSK_MD5, checksum: "dbbaa7383ecf1705b3b4648f8af69a79")
!2 = !{i32 7, !"Dwarf Version", i32 5}
!3 = !{i32 2, !"Debug Info Version", i32 3}
!4 = !{i32 1, !"wchar_size", i32 4}
!5 = !{i32 7, !"frame-pointer", i32 2}
!6 = !{!"clang version 14.0.0 (https://github.com/llvm/llvm-project ...)"}
!7 = distinct !DISubprogram(name: "f", scope: !1, file: !1, line: 5, type: !8, scopeLine: 6, flags: DIFlagPrototyped | DIFlagAllCallsDescribed, spFlags: DISPFlagDefinition | DISPFlagOptimized, unit: !0, retainedNodes: !16)
!8 = !DISubroutineType(types: !9)
!9 = !{!10, !10, !11}
!10 = !DIBasicType(name: "char", size: 8, encoding: DW_ATE_unsigned_char)
!11 = !DIDerivedType(tag: DW_TAG_pointer_type, baseType: !12, size: 16)
!12 = distinct !DICompositeType(tag: DW_TAG_structure_type, name: "X", file: !1, line: 1, size: 16, elements: !13)
!13 = !{!14}
!14 = !DIDerivedType(tag: DW_TAG_member, name: "a", scope: !12, file: !1, line: 2, baseType: !15, size: 16)
!15 = !DIDerivedType(tag: DW_TAG_pointer_type, baseType: null, size: 16)
!16 = !{!17, !18}
!17 = !DILocalVariable(name: "y", arg: 1, scope: !7, file: !1, line: 5, type: !10)
!18 = !DILocalVariable(name: "p", arg: 2, scope: !7, file: !1, line: 5, type: !11)
!19 = !DILocation(line: 0, scope: !7)
!20 = !DILocation(line: 7, column: 3, scope: !7)
