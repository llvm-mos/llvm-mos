//===-- MOSCallingConv.cpp - MOS Calling Convention ------------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MOS calling convention.
//
//===----------------------------------------------------------------------===//

#include "MOSCallingConv.h"

#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/IR/DataLayout.h"

#define GET_CALLING_CONV_IMPL
#include "MOSGenCallingConv.inc"

using namespace llvm;

CCAssignFn *llvm::CCAssignFnForCall(CallingConv::ID CC, bool IsVarArg) {
  switch (CC) {
  case CallingConv::PreserveMost:
    return CC_MOS_PreserveMost;
  default:
    return IsVarArg ? CC_MOS_VarArgs : CC_MOS;
  }
}

CCAssignFn *llvm::CCAssignFnForReturn(CallingConv::ID CC) {
  return CC == CallingConv::PreserveMost ? CC_MOS_PreserveMost : CC_MOS;
}
