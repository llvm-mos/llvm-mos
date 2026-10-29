//===-- MOSCallingConv.h - MOS Calling Convention-----------------*- C++
//-*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MOS calling convention.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOSCALLINGCONV_H
#define LLVM_LIB_TARGET_MOS_MOSCALLINGCONV_H

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGenTypes/MachineValueType.h"
#include "llvm/IR/CallingConv.h"

namespace llvm {

/// Regular calling convention.
bool CC_MOS(unsigned ValNo, MVT ValVT, MVT LocVT, CCValAssign::LocInfo LocInfo,
            ISD::ArgFlagsTy ArgFlags, Type *OrigTy, CCState &State);

/// Calling convention used for the dynamic portion of varargs calls. Just puts
/// everything on the stack.
bool CC_MOS_VarArgs(unsigned ValNo, MVT ValVT, MVT LocVT,
                    CCValAssign::LocInfo LocInfo, ISD::ArgFlagsTy ArgFlags,
                    Type *OrigTy, CCState &State);

/// Calling convention for preserve_most functions. Values are passed and
/// returned in the hard registers A, X, and Y only; all other values go on the
/// stack. See MOSCallingConv.td for the rationale.
bool CC_MOS_PreserveMost(unsigned ValNo, MVT ValVT, MVT LocVT,
                         CCValAssign::LocInfo LocInfo,
                         ISD::ArgFlagsTy ArgFlags, Type *OrigTy,
                         CCState &State);

/// Selects the CCAssignFn for passing arguments under the given calling
/// convention. If IsVarArg is true, the function for the variadic portion of
/// the call is returned instead. PreserveMost does not support variadic
/// functions, so the same function is returned for both cases.
CCAssignFn *CCAssignFnForCall(CallingConv::ID CC, bool IsVarArg);

/// Selects the CCAssignFn used to return values under the given calling
/// convention.
CCAssignFn *CCAssignFnForReturn(CallingConv::ID CC);

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSCALLINGCONV_H
