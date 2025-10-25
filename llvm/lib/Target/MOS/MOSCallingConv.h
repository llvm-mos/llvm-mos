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

namespace llvm {

/// Regular calling convention.
bool CC_MOS(unsigned ValNo, MVT ValVT, MVT LocVT, CCValAssign::LocInfo LocInfo,
            ISD::ArgFlagsTy ArgFlags, Type *OrigTy, CCState &State);

/// Calling convention used for the dynamic portion of varargs calls. Just puts
/// everything on the stack.
bool CC_MOS_VarArgs(unsigned ValNo, MVT ValVT, MVT LocVT,
                    CCValAssign::LocInfo LocInfo, ISD::ArgFlagsTy ArgFlags,
                    Type *OrigTy, CCState &State);

/// Calling convention intended for usecases like interrupts where the main 6502
/// registers A/X/Y can remain caller saved, but the ZP registers need to be
/// callee saved.
bool CC_MOS_PreserveMost(unsigned ValNo, MVT ValVT, MVT LocVT,
                        CCValAssign::LocInfo LocInfo, ISD::ArgFlagsTy ArgFlags,
                        CCState &State);

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSCALLINGCONV_H
