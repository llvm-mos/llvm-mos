; RUN: llc -mtriple=mos -mcpu=mos6502 -O2 -filetype=obj %s -o %t.o
; RUN: llvm-dwarfdump --debug-info %t.o | FileCheck %s
; RUN: llc -mtriple=mos -mcpu=mos6502 -O2 -zp-avail=224 %s -o - \
; RUN:   | FileCheck %s --check-prefix=ZP

; Non-reentrant locals referenced by an indirect dbg.value (ptr-to-alloca with
; DW_OP_deref) get a memory-location DW_AT_location of the form
;   DW_OP_addr <symbol+offset>
; without a trailing DW_OP_stack_value.
;
; With zero page available, the same locals live on the zero-page stack. Their
; location is DW_OP_addr (3) of the zero-page stack symbol: 5 bytes in all,
; with no DW_OP_stack_value after it.

target triple = "mos"

declare void @sink8(ptr)
declare void @sink16(ptr)
declare void @sink32(ptr)
declare void @llvm.dbg.value(metadata, metadata, metadata)

; CHECK-LABEL: DW_TAG_subprogram
; CHECK:        DW_AT_name      ("test_i8")
; CHECK:        DW_TAG_variable
; CHECK:          DW_AT_location {{.*}}DW_OP_{{addr|addrx}}
; CHECK-NOT:                     DW_OP_stack_value
; CHECK:          DW_AT_name    ("x8")

; ZP:      Abbrev [{{[0-9]+}}] {{.*}} DW_TAG_variable
; ZP-NEXT: .byte 5 ; DW_AT_location
; ZP-NEXT: .byte 3
; ZP-NEXT: .long .Ltest_i8_zp_stk
; ZP-NEXT: ; DW_AT_name

define void @test_i8(i8 %v) #0 !dbg !10 {
entry:
  %x8 = alloca i8, align 1
  call void @llvm.dbg.value(metadata ptr %x8, metadata !14, metadata !DIExpression(DW_OP_deref)), !dbg !18
  store volatile i8 %v, ptr %x8, align 1, !dbg !18
  call void @sink8(ptr %x8), !dbg !18
  ret void, !dbg !18
}

; CHECK-LABEL: DW_TAG_subprogram
; CHECK:        DW_AT_name      ("test_i16")
; CHECK:        DW_TAG_variable
; CHECK:          DW_AT_location {{.*}}DW_OP_{{addr|addrx}}
; CHECK-NOT:                     DW_OP_stack_value
; CHECK:          DW_AT_name    ("x16")

; ZP:      Abbrev [{{[0-9]+}}] {{.*}} DW_TAG_variable
; ZP-NEXT: .byte 5 ; DW_AT_location
; ZP-NEXT: .byte 3
; ZP-NEXT: .long .Ltest_i16_zp_stk
; ZP-NEXT: ; DW_AT_name

define void @test_i16(i16 %v) #0 !dbg !20 {
entry:
  %x16 = alloca i16, align 1
  call void @llvm.dbg.value(metadata ptr %x16, metadata !24, metadata !DIExpression(DW_OP_deref)), !dbg !28
  store volatile i16 %v, ptr %x16, align 1, !dbg !28
  call void @sink16(ptr %x16), !dbg !28
  ret void, !dbg !28
}

; CHECK-LABEL: DW_TAG_subprogram
; CHECK:        DW_AT_name      ("test_i32")
; CHECK:        DW_TAG_variable
; CHECK:          DW_AT_location {{.*}}DW_OP_{{addr|addrx}}
; CHECK-NOT:                     DW_OP_stack_value
; CHECK:          DW_AT_name    ("x32")

; ZP:      Abbrev [{{[0-9]+}}] {{.*}} DW_TAG_variable
; ZP-NEXT: .byte 5 ; DW_AT_location
; ZP-NEXT: .byte 3
; ZP-NEXT: .long .Ltest_i32_zp_stk
; ZP-NEXT: ; DW_AT_name

define void @test_i32(i32 %v) #0 !dbg !30 {
entry:
  %x32 = alloca i32, align 1
  call void @llvm.dbg.value(metadata ptr %x32, metadata !34, metadata !DIExpression(DW_OP_deref)), !dbg !38
  store volatile i32 %v, ptr %x32, align 1, !dbg !38
  call void @sink32(ptr %x32), !dbg !38
  ret void, !dbg !38
}

attributes #0 = { noinline nounwind "nonreentrant" }

!llvm.dbg.cu = !{!0}
!llvm.module.flags = !{!2, !3}

!0 = distinct !DICompileUnit(language: DW_LANG_C99, file: !1, producer: "test", emissionKind: FullDebug)
!1 = !DIFile(filename: "test.c", directory: "/")
!2 = !{i32 2, !"Dwarf Version", i32 4}
!3 = !{i32 2, !"Debug Info Version", i32 3}

!10 = distinct !DISubprogram(name: "test_i8", scope: !1, file: !1, line: 1, type: !11, unit: !0, retainedNodes: !13)
!11 = !DISubroutineType(types: !12)
!12 = !{null, !15}
!13 = !{!14}
!14 = !DILocalVariable(name: "x8", scope: !10, file: !1, line: 2, type: !15)
!15 = !DIBasicType(name: "int8", size: 8, encoding: DW_ATE_signed)
!18 = !DILocation(line: 2, column: 1, scope: !10)

!20 = distinct !DISubprogram(name: "test_i16", scope: !1, file: !1, line: 10, type: !21, unit: !0, retainedNodes: !23)
!21 = !DISubroutineType(types: !22)
!22 = !{null, !25}
!23 = !{!24}
!24 = !DILocalVariable(name: "x16", scope: !20, file: !1, line: 11, type: !25)
!25 = !DIBasicType(name: "int16", size: 16, encoding: DW_ATE_signed)
!28 = !DILocation(line: 11, column: 1, scope: !20)

!30 = distinct !DISubprogram(name: "test_i32", scope: !1, file: !1, line: 20, type: !31, unit: !0, retainedNodes: !33)
!31 = !DISubroutineType(types: !32)
!32 = !{null, !35}
!33 = !{!34}
!34 = !DILocalVariable(name: "x32", scope: !30, file: !1, line: 21, type: !35)
!35 = !DIBasicType(name: "int32", size: 32, encoding: DW_ATE_signed)
!38 = !DILocation(line: 21, column: 1, scope: !30)
