//===-- MOSInstructionSelector.h - MOS Instruction Selector -----*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MOS instruction selector.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOSINSTRUCTIONSELECTOR_H
#define LLVM_LIB_TARGET_MOS_MOSINSTRUCTIONSELECTOR_H

#include "MOSTargetMachine.h"
#include "MOSRegisterBankInfo.h"
#include "MOSSubtarget.h"
#include "llvm/CodeGen/GlobalISel/InstructionSelector.h"

namespace llvm {

InstructionSelector *
createMOSInstructionSelector(const MOSTargetMachine &TM,
                                 MOSSubtarget &STI,
                                 MOSRegisterBankInfo &RBI);

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSINSTRUCTIONSELECTOR_H
