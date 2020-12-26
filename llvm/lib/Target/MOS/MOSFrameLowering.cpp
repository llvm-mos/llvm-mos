//===-- MOSFrameLowering.cpp - MOS Frame Information ----------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
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
#include "MOSMachineFunctionInfo.h"
#include "MOSTargetMachine.h"

#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/IR/Function.h"

#include <vector>

namespace llvm {

MOSFrameLowering::MOSFrameLowering()
    : TargetFrameLowering(TargetFrameLowering::StackGrowsDown, 1, -2) {}

bool MOSFrameLowering::canSimplifyCallFramePseudos(
    const MachineFunction &MF) const {
  // Always simplify call frame pseudo instructions, even when
  // hasReservedCallFrame is false.
  return true;
}

void MOSFrameLowering::determineCalleeSaves(MachineFunction &MF,
                                            BitVector &SavedRegs,
                                            RegScavenger *RS) const {
  TargetFrameLowering::determineCalleeSaves(MF, SavedRegs, RS);

  // If we have a frame pointer, the Y register needs to be saved as well.
  // We don't do that here however - the prologue and epilogue generation
  // code will handle it specially.
}

/// Replace pseudo store instructions that pass arguments through the stack with
/// real instructions. If insertPushes is true then all instructions are
/// replaced with push instructions, otherwise regular std instructions are
/// inserted.
/*
static void MOSFrameLowering::fixStackStores(MachineBasicBlock &MBB,
                           MachineBasicBlock::iterator MI,
                           const TargetInstrInfo &TII, bool insertPushes) {}
						   */

MachineBasicBlock::iterator MOSFrameLowering::eliminateCallFramePseudoInstr(
    MachineFunction &MF, MachineBasicBlock &MBB,
    MachineBasicBlock::iterator MI) const {
      return MI;
    }

void MOSFrameLowering::emitEpilogue(MachineFunction &MF,
                                    MachineBasicBlock &MBB) const {}

void MOSFrameLowering::emitPrologue(MachineFunction &MF,
                                    MachineBasicBlock &MBB) const {}

// Return true if the specified function should have a dedicated frame
// pointer register. This is true if the function meets any of the following
// conditions:
//  - a register has been spilled
//  - has allocas
//  - input arguments are passed using the stack
//
// Notice that strictly this is not a frame pointer because it contains SP after
// frame allocation instead of having the original SP in function entry.
bool MOSFrameLowering::hasFP(const MachineFunction &MF) const { return true; }

bool MOSFrameLowering::hasReservedCallFrame(const MachineFunction &MF) const {
  return false;
}

bool MOSFrameLowering::restoreCalleeSavedRegisters(
    MachineBasicBlock &MBB, MachineBasicBlock::iterator MI,
    std::vector<CalleeSavedInfo> &CSI, const TargetRegisterInfo *TRI) const {
  return true;
}

bool MOSFrameLowering::spillCalleeSavedRegisters(
    MachineBasicBlock &MBB, MachineBasicBlock::iterator MI,
    const std::vector<CalleeSavedInfo> &CSI,
    const TargetRegisterInfo *TRI) const {
  return true;
}

/// The frame analyzer pass.
///
/// Scans the function for allocas and used arguments
/// that are passed through the stack.
class MOSFrameAnalyzer : public MachineFunctionPass {
public:
  static char ID;
  MOSFrameAnalyzer() : MachineFunctionPass(ID) {}

  bool runOnMachineFunction(MachineFunction &MF) { return false; }

  StringRef getPassName() const { return "MOS Frame Analyzer"; }
};

char MOSFrameAnalyzer::ID = 0;

/// Creates instance of the frame analyzer pass.
FunctionPass *createMOSFrameAnalyzerPass() { return new MOSFrameAnalyzer(); }

/// Create the Dynalloca Stack Pointer Save/Restore pass.
/// Insert a copy of SP before allocating the dynamic stack memory and restore
/// it in function exit to restore the original SP state. This avoids the need
/// of reserving a register pair for a frame pointer.
class MOSDynAllocaSR : public MachineFunctionPass {
public:
  static char ID;
  MOSDynAllocaSR() : MachineFunctionPass(ID) {}

  bool runOnMachineFunction(MachineFunction &MF) { return true; }

  StringRef getPassName() const {
    return "MOS dynalloca stack pointer save/restore";
  }
};

char MOSDynAllocaSR::ID = 0;

/// createMOSDynAllocaSRPass - returns an instance of the dynalloca stack
/// pointer save/restore pass.
FunctionPass *createMOSDynAllocaSRPass() { return new MOSDynAllocaSR(); }

} // end of namespace llvm
