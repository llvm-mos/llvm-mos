; RUN: opt -S -mtriple=amdgcn-unknown-amdhsa -mcpu=hawaii -passes=loop-unroll -unroll-threshold=57 -unroll-peel-count=0 -unroll-allow-partial=false -unroll-max-iteration-count-to-analyze=16 < %s | FileCheck %s

@indices = external global [16 x i32]

; CHECK-LABEL: @test_func_addrspacecast_cost_noop(
; CHECK-NOT: br i1
define amdgpu_kernel void @test_func_addrspacecast_cost_noop(ptr addrspace(1) noalias nocapture %out, ptr addrspace(1) noalias nocapture %in) #0 {
entry:
  br label %for.body

for.body:
  %indvars.iv = phi i32 [ %indvars.iv.next, %for.body ], [ 0, %entry ]
  %sum.02 = phi float [ %fmul, %for.body ], [ 0.0, %entry ]
  %idx.ptr = getelementptr inbounds [16 x i32], ptr @indices, i32 0, i32 %indvars.iv
  %index = load i32, ptr %idx.ptr
  %arrayidx.in = getelementptr inbounds float, ptr addrspace(1) %in, i32 %index
  %arrayidx.out = getelementptr inbounds float, ptr addrspace(1) %out, i32 %index
  %cast.in = addrspacecast ptr addrspace(1) %arrayidx.in to ptr
  %cast.out = addrspacecast ptr addrspace(1) %arrayidx.out to ptr
  %load = load float, ptr %cast.in
  %fmul = fmul float %load, %sum.02
  store float %fmul, ptr %cast.out
  %indvars.iv.next = add i32 %indvars.iv, 1
  %exitcond = icmp eq i32 %indvars.iv.next, 16
  br i1 %exitcond, label %for.end, label %for.body

for.end:
  ret void
}

; Free, but not a no-op
; CHECK-LABEL: @test_func_addrspacecast_cost_free(
; CHECK-NOT: br i1
define amdgpu_kernel void @test_func_addrspacecast_cost_free(ptr noalias nocapture %out, ptr noalias nocapture %in) #0 {
entry:
  br label %for.body

for.body:
  %indvars.iv = phi i32 [ %indvars.iv.next, %for.body ], [ 0, %entry ]
  %sum.02 = phi float [ %fmul, %for.body ], [ 0.0, %entry ]
  %idx.ptr = getelementptr inbounds [16 x i32], ptr @indices, i32 0, i32 %indvars.iv
  %index = load i32, ptr %idx.ptr
  %arrayidx.in = getelementptr inbounds float, ptr %in, i32 %index
  %arrayidx.out = getelementptr inbounds float, ptr %out, i32 %index
  %cast.in = addrspacecast ptr %arrayidx.in to ptr addrspace(3)
  %cast.out = addrspacecast ptr %arrayidx.out to ptr addrspace(3)
  %load = load float, ptr addrspace(3) %cast.in
  %fmul = fmul float %load, %sum.02
  store float %fmul, ptr addrspace(3) %cast.out
  %indvars.iv.next = add i32 %indvars.iv, 1
  %exitcond = icmp eq i32 %indvars.iv.next, 16
  br i1 %exitcond, label %for.end, label %for.body

for.end:
  ret void
}

; CHECK-LABEL: @test_func_addrspacecast_cost_nonfree(
; CHECK: br i1 %exitcond
define amdgpu_kernel void @test_func_addrspacecast_cost_nonfree(ptr addrspace(3) noalias nocapture %out, ptr addrspace(3) noalias nocapture %in) #0 {
entry:
  br label %for.body

for.body:
  %indvars.iv = phi i32 [ %indvars.iv.next, %for.body ], [ 0, %entry ]
  %sum.02 = phi float [ %fmul, %for.body ], [ 0.0, %entry ]
  %idx.ptr = getelementptr inbounds [16 x i32], ptr @indices, i32 0, i32 %indvars.iv
  %index = load i32, ptr %idx.ptr
  %arrayidx.in = getelementptr inbounds float, ptr addrspace(3) %in, i32 %index
  %arrayidx.out = getelementptr inbounds float, ptr addrspace(3) %out, i32 %index
  %cast.in = addrspacecast ptr addrspace(3) %arrayidx.in to ptr
  %cast.out = addrspacecast ptr addrspace(3) %arrayidx.out to ptr
  %load = load float, ptr %cast.in
  %fmul = fmul float %load, %sum.02
  store float %fmul, ptr %cast.out
  %indvars.iv.next = add i32 %indvars.iv, 1
  %exitcond = icmp eq i32 %indvars.iv.next, 16
  br i1 %exitcond, label %for.end, label %for.body

for.end:
  ret void
}

attributes #0 = { nounwind }
attributes #1 = { nounwind readnone speculatable }
