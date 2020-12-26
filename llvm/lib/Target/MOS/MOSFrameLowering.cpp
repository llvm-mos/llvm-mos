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

#include "MOS.h"
#include "MOSInstrInfo.h"
#include "MOSMachineFunctionInfo.h"
#include "MOSTargetMachine.h"
#include "MCTargetDesc/MOSMCTargetDesc.h"

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

bool MOSFrameLowering::hasReservedCallFrame(const MachineFunction &MF) const {
  // Reserve call frame memory in function prologue under the following
  // conditions:
  // - Y pointer is reserved to be the frame pointer.
  // - The function does not contain variable sized objects.

  const MachineFrameInfo &MFI = MF.getFrameInfo();
  return hasFP(MF) && !MFI.hasVarSizedObjects();
}

void MOSFrameLowering::emitPrologue(MachineFunction &MF,
                                    MachineBasicBlock &MBB) const {
//todo
}

void MOSFrameLowering::emitEpilogue(MachineFunction &MF,
                                    MachineBasicBlock &MBB) const {
//todo
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
  const MOSMachineFunctionInfo *FuncInfo = MF.getInfo<MOSMachineFunctionInfo>();

  return (FuncInfo->getHasSpills() || FuncInfo->getHasAllocas() ||
          FuncInfo->getHasStackArgs());
}

bool MOSFrameLowering::spillCalleeSavedRegisters(
    MachineBasicBlock &MBB, MachineBasicBlock::iterator MI,
    const std::vector<CalleeSavedInfo> &CSI,
    const TargetRegisterInfo *TRI) const {
  if (CSI.empty()) {
    return false;
  }

  unsigned CalleeFrameSize = 0;
  DebugLoc DL = MBB.findDebugLoc(MI);
  MachineFunction &MF = *MBB.getParent();
  const MOSSubtarget &STI = MF.getSubtarget<MOSSubtarget>();
  const TargetInstrInfo &TII = *STI.getInstrInfo();
  MOSMachineFunctionInfo *MOSFI = MF.getInfo<MOSMachineFunctionInfo>();

  for (unsigned i = CSI.size(); i != 0; --i) {
    unsigned Reg = CSI[i - 1].getReg();
    bool IsNotLiveIn = !MBB.isLiveIn(Reg);

    assert(TRI->getRegSizeInBits(*TRI->getMinimalPhysRegClass(Reg)) == 8 &&
           "Invalid register size");

    // Add the callee-saved register as live-in only if it is not already a
    // live-in register, this usually happens with arguments that are passed
    // through callee-saved registers.
    if (IsNotLiveIn) {
      MBB.addLiveIn(Reg);
    }

    // Do not kill the register when it is an input argument.
    BuildMI(MBB, MI, DL, TII.get(MOS::PUSHRr))
        .addReg(Reg, getKillRegState(IsNotLiveIn))
        .setMIFlag(MachineInstr::FrameSetup);
    ++CalleeFrameSize;
  }

  MOSFI->setCalleeSavedFrameSize(CalleeFrameSize);

  return true;
}

bool MOSFrameLowering::restoreCalleeSavedRegisters(
    MachineBasicBlock &MBB, MachineBasicBlock::iterator MI,
    std::vector<CalleeSavedInfo> &CSI,
    const TargetRegisterInfo *TRI) const {
  return true;
}

MachineBasicBlock::iterator MOSFrameLowering::eliminateCallFramePseudoInstr(
    MachineFunction &MF, MachineBasicBlock &MBB,
    MachineBasicBlock::iterator MI) const {
  return MBB.erase(MI);
}

void MOSFrameLowering::determineCalleeSaves(MachineFunction &MF,
                                            BitVector &SavedRegs,
                                            RegScavenger *RS) const {
  TargetFrameLowering::determineCalleeSaves(MF, SavedRegs, RS);

  // If we have a frame pointer, the Y register needs to be saved as well.
  // We don't do that here however - the prologue and epilogue generation
  // code will handle it specially.
}
/// The frame analyzer pass.
///
/// Scans the function for allocas and used arguments
/// that are passed through the stack.
struct MOSFrameAnalyzer : public MachineFunctionPass {
  static char ID;
  MOSFrameAnalyzer() : MachineFunctionPass(ID) {}

  bool runOnMachineFunction(MachineFunction &MF) {
    return false;
  }

  StringRef getPassName() const { return "MOS Frame Analyzer"; }
};

char MOSFrameAnalyzer::ID = 0;

/// Creates instance of the frame analyzer pass.
FunctionPass *createMOSFrameAnalyzerPass() { return new MOSFrameAnalyzer(); }

/// Create the Dynalloca Stack Pointer Save/Restore pass.
/// Insert a copy of SP before allocating the dynamic stack memory and restore
/// it in function exit to restore the original SP state. This avoids the need
/// of reserving a register pair for a frame pointer.
struct MOSDynAllocaSR : public MachineFunctionPass {
  static char ID;
  MOSDynAllocaSR() : MachineFunctionPass(ID) {}

  bool runOnMachineFunction(MachineFunction &MF) {
    // Early exit when there are no variable sized objects in the function.

    const MOSSubtarget &STI = MF.getSubtarget<MOSSubtarget>();
    const TargetInstrInfo &TII = *STI.getInstrInfo();
    MachineBasicBlock &EntryMBB = MF.front();
    MachineBasicBlock::iterator MBBI = EntryMBB.begin();
    DebugLoc DL = EntryMBB.findDebugLoc(MBBI);

    unsigned SPCopy =
        MF.getRegInfo().createVirtualRegister(&MOS::DREGSRegClass);

    // Create a copy of SP in function entry before any dynallocas are
    // inserted.
    BuildMI(EntryMBB, MBBI, DL, TII.get(MOS::COPY), SPCopy).addReg(MOS::SP);
    return true;
  }

  StringRef getPassName() const {
    return "MOS dynalloca stack pointer save/restore";
  }
};

char MOSDynAllocaSR::ID = 0;

/// createMOSDynAllocaSRPass - returns an instance of the dynalloca stack
/// pointer save/restore pass.
FunctionPass *createMOSDynAllocaSRPass() { return new MOSDynAllocaSR(); }

} // end of namespace llvm

