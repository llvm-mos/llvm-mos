//===-- MOSFrameLowering.h - Define frame lowering for MOS ------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
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
