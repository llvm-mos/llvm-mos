; RUN: llc --align-large-globals=0 -verify-machineinstrs < %s | FileCheck %s --match-full-lines

target datalayout = "e-m:e-p:16:8-p1:8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mos"

declare dso_local void @ext(ptr noundef) nocallback

; CHECK-LABEL: .Lstatic_stack:
; CHECK-NEXT:    .zero	29
; CHECK-NEXT:    .size	.Lstatic_stack, 29

; CHECK-LABEL: .set .Lleaf1_sstk, .Lstatic_stack
; CHECK-NEXT:    .size	.Lleaf1_sstk, 10
; CHECK-LABEL: .set .Lleaf2_sstk, .Lstatic_stack
; CHECK-NEXT:    .size	.Lleaf2_sstk, 1
; CHECK-LABEL: .set .Lpath_leaf_sstk, .Lstatic_stack+10
; CHECK-NEXT:    .size	.Lpath_leaf_sstk, 1
; CHECK-LABEL: .set .Lpath_root_sstk, .Lstatic_stack
; CHECK-NEXT:    .size	.Lpath_root_sstk, 10
; CHECK-LABEL: .set .Lrecur_leaf_sstk, .Lstatic_stack+1
; CHECK-NEXT:    .size	.Lrecur_leaf_sstk, 4
; CHECK-LABEL: .set .Lrecur_root_sstk, .Lstatic_stack
; CHECK-NEXT:    .size	.Lrecur_root_sstk, 1
; CHECK-LABEL: .set .Linr_sstk, .Lstatic_stack+11
; CHECK-NEXT:    .size	.Linr_sstk, 18

define void @leaf1() {
  %frame = alloca [10 x i8], align 1
  %arraydecay = getelementptr [10 x i8], ptr %frame, i16 0, i16 0
  call void @ext(ptr noundef %arraydecay) nocallback
  ret void
}

define void @leaf2() {
  %frame = alloca [1 x i8], align 1
  %arraydecay = getelementptr [1 x i8], ptr %frame, i16 0, i16 0
  call void @ext(ptr noundef %arraydecay) nocallback
  ret void
}

define void @path_root() {
  %frame = alloca [10 x i8], align 1
  %arraydecay = getelementptr [10 x i8], ptr %frame, i16 0, i16 0
  call void @path_leaf();
  call void @ext(ptr noundef %arraydecay) nocallback
  ret void
}

define void @path_leaf() {
  %frame = alloca [1 x i8], align 1
  %arraydecay = getelementptr [1 x i8], ptr %frame, i16 0, i16 0
  call void @ext(ptr noundef %arraydecay) nocallback
  ret void
}

define void @recur_root() norecurse {
  %frame = alloca [1 x i8], align 1
  %arraydecay = getelementptr [1 x i8], ptr %frame, i16 0, i16 0
  call void @recur_a()
  call void @ext(ptr noundef %arraydecay) nocallback
  ret void
}

define void @recur_a() {
  %frame = alloca [2 x i8], align 1
  %arraydecay = getelementptr [2 x i8], ptr %frame, i16 0, i16 0
  call void @recur_b()
  call void @ext(ptr noundef %arraydecay) nocallback
  ret void
}

define void @recur_b() {
  %frame = alloca [3 x i8], align 1
  %arraydecay = getelementptr [3 x i8], ptr %frame, i16 0, i16 0
  call void @recur_a()
  call void @recur_leaf()
  call void @ext(ptr noundef %arraydecay) nocallback
  ret void
}

define void @recur_leaf() norecurse {
  %frame = alloca [4 x i8], align 1
  %arraydecay = getelementptr [4 x i8], ptr %frame, i16 0, i16 0
  call void @ext(ptr noundef %arraydecay) nocallback
  ret void
}

define void @inr() "interrupt-norecurse" {
  %frame = alloca [1 x i8], align 1
  %arraydecay = getelementptr [1 x i8], ptr %frame, i16 0, i16 0
  call void @ext(ptr noundef %arraydecay) nocallback
  ret void
}
