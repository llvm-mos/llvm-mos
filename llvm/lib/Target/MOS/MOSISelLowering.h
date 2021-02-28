//===-- MOSISelLowering.h - MOS DAG Lowering Interface ----------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the interfaces that MOS uses to lower LLVM code into a
// selection DAG.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MOS_ISEL_LOWERING_H
#define LLVM_MOS_ISEL_LOWERING_H

#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGen/TargetLowering.h"

namespace llvm {

class MOSSubtarget;
class MOSTargetMachine;

/// Performs target lowering for the MOS.
class MOSTargetLowering : public TargetLowering {
public:
  explicit MOSTargetLowering(const MOSTargetMachine &TM,
                             const MOSSubtarget &STI);
protected:
  const MOSSubtarget &Subtarget;
};

} // end namespace llvm

#endif // LLVM_MOS_ISEL_LOWERING_H
