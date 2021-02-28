//===-- MOSFrameLowering.h - Define frame lowering for MOS ------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
// This file contains the MOS declaration of TargetFrameLowering class.
//===----------------------------------------------------------------------===//

#ifndef LLVM_MOS_FRAME_LOWERING_H
#define LLVM_MOS_FRAME_LOWERING_H

#include "llvm/CodeGen/TargetFrameLowering.h"

namespace llvm {

/// Utilities for creating function call frames.
class MOSFrameLowering : public TargetFrameLowering {
public:
  explicit MOSFrameLowering();

public:
  bool hasFP(const MachineFunction &MF) const override;
  void emitPrologue(MachineFunction &MF, MachineBasicBlock &MBB) const override;
  void emitEpilogue(MachineFunction &MF, MachineBasicBlock &MBB) const override;
};

} // end namespace llvm

#endif // LLVM_MOS_FRAME_LOWERING_H
