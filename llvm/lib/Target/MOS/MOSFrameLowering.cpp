//===-- MOSFrameLowering.cpp - MOS Frame Information ----------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the MOS implementation of TargetFrameLowering class.
//
//===----------------------------------------------------------------------===//

#include "MOSFrameLowering.h"

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "MOSInstrInfo.h"
#include "MOSTargetMachine.h"

#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/IR/Function.h"
#include "llvm/Support/ErrorHandling.h"

#include <vector>

namespace llvm {

MOSFrameLowering::MOSFrameLowering()
    : TargetFrameLowering(TargetFrameLowering::StackGrowsDown, Align(1), 0) {}

void MOSFrameLowering::emitEpilogue(MachineFunction &MF,
                                    MachineBasicBlock &MBB) const {
  report_fatal_error("Not yet implemented");
}

void MOSFrameLowering::emitPrologue(MachineFunction &MF,
                                    MachineBasicBlock &MBB) const {

  report_fatal_error("Not yet implemented.");
}

// Return true if the specified function should have a dedicated frame
// pointer register. This is true if the function meets any of the following
// conditions:
//  - a register has been spilled
//  - has allocas
//  - input arguments are passed using the stack
//
// Notice that strictly this is not a frame pointer because it contains SP after
// frame allocation instead of having the original SP in function entry.
bool MOSFrameLowering::hasFP(const MachineFunction &MF) const {
  report_fatal_error("Not yet implemented.");
}

} // end of namespace llvm
