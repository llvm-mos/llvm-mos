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
  MachineBasicBlock::iterator MBBI = MBB.begin();
  CallingConv::ID CallConv = MF.getFunction().getCallingConv();
  DebugLoc DL = (MBBI != MBB.end()) ? MBBI->getDebugLoc() : DebugLoc();
  const MOSSubtarget &STI = MF.getSubtarget<MOSSubtarget>();
  const MOSInstrInfo &TII = *STI.getInstrInfo();
  bool HasFP = hasFP(MF);

  // Interrupt handlers re-enable interrupts in function entry.
  if (CallConv == CallingConv::MOS_INTR) {
    BuildMI(MBB, MBBI, DL, TII.get(MOS::BSETs))
        .addImm(0x07)
        .setMIFlag(MachineInstr::FrameSetup);
  }

  // Save the frame pointer if we have one.
  if (HasFP) {
    BuildMI(MBB, MBBI, DL, TII.get(MOS::PUSHWRr))
        .addReg(MOS::R29R28, RegState::Kill)
        .setMIFlag(MachineInstr::FrameSetup);
  }

  // Emit special prologue code to save R1, R0 and SREG in interrupt/signal
  // handlers before saving any other registers.
  if (CallConv == CallingConv::MOS_INTR ||
      CallConv == CallingConv::MOS_SIGNAL) {
    BuildMI(MBB, MBBI, DL, TII.get(MOS::PUSHWRr))
        .addReg(MOS::R1R0, RegState::Kill)
        .setMIFlag(MachineInstr::FrameSetup);

    BuildMI(MBB, MBBI, DL, TII.get(MOS::INRdA), MOS::R0)
        .addImm(0x3f)
        .setMIFlag(MachineInstr::FrameSetup);
    BuildMI(MBB, MBBI, DL, TII.get(MOS::PUSHRr))
        .addReg(MOS::R0, RegState::Kill)
        .setMIFlag(MachineInstr::FrameSetup);
    BuildMI(MBB, MBBI, DL, TII.get(MOS::EORRdRr))
        .addReg(MOS::R0, RegState::Define)
        .addReg(MOS::R0, RegState::Kill)
        .addReg(MOS::R0, RegState::Kill)
        .setMIFlag(MachineInstr::FrameSetup);
  }

  // Early exit if the frame pointer is not needed in this function.
  if (!HasFP) {
    return;
  }

  const MachineFrameInfo &MFI = MF.getFrameInfo();
  const MOSMachineFunctionInfo *AFI = MF.getInfo<MOSMachineFunctionInfo>();
  unsigned FrameSize = MFI.getStackSize() - AFI->getCalleeSavedFrameSize();

  // Skip the callee-saved push instructions.
  while (
      (MBBI != MBB.end()) && MBBI->getFlag(MachineInstr::FrameSetup) &&
      (MBBI->getOpcode() == MOS::PUSHRr || MBBI->getOpcode() == MOS::PUSHWRr)) {
    ++MBBI;
  }

  // Update Y with the new base value.
  BuildMI(MBB, MBBI, DL, TII.get(MOS::SPREAD), MOS::R29R28)
      .addReg(MOS::SP)
      .setMIFlag(MachineInstr::FrameSetup);

  // Mark the FramePtr as live-in in every block except the entry.
  for (MachineFunction::iterator I = std::next(MF.begin()), E = MF.end();
       I != E; ++I) {
    I->addLiveIn(MOS::R29R28);
  }

  if (!FrameSize) {
    return;
  }

  // Reserve the necessary frame memory by doing FP -= <size>.
  unsigned Opcode = (isUInt<6>(FrameSize)) ? MOS::SBIWRdK : MOS::SUBIWRdK;

  MachineInstr *MI = BuildMI(MBB, MBBI, DL, TII.get(Opcode), MOS::R29R28)
                         .addReg(MOS::R29R28, RegState::Kill)
                         .addImm(FrameSize)
                         .setMIFlag(MachineInstr::FrameSetup);
  // The SREG implicit def is dead.
  MI->getOperand(3).setIsDead();

  // Write back R29R28 to SP and temporarily disable interrupts.
  BuildMI(MBB, MBBI, DL, TII.get(MOS::SPWRITE), MOS::SP)
      .addReg(MOS::R29R28)
      .setMIFlag(MachineInstr::FrameSetup);
}

void MOSFrameLowering::emitEpilogue(MachineFunction &MF,
                                    MachineBasicBlock &MBB) const {
  CallingConv::ID CallConv = MF.getFunction().getCallingConv();
  bool isHandler = (CallConv == CallingConv::MOS_INTR ||
                    CallConv == CallingConv::MOS_SIGNAL);

  // Early exit if the frame pointer is not needed in this function except for
  // signal/interrupt handlers where special code generation is required.
  if (!hasFP(MF) && !isHandler) {
    return;
  }

  MachineBasicBlock::iterator MBBI = MBB.getLastNonDebugInstr();
  assert(MBBI->getDesc().isReturn() &&
         "Can only insert epilog into returning blocks");

  DebugLoc DL = MBBI->getDebugLoc();
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  const MOSMachineFunctionInfo *AFI = MF.getInfo<MOSMachineFunctionInfo>();
  unsigned FrameSize = MFI.getStackSize() - AFI->getCalleeSavedFrameSize();
  const MOSSubtarget &STI = MF.getSubtarget<MOSSubtarget>();
  const MOSInstrInfo &TII = *STI.getInstrInfo();

  // Emit special epilogue code to restore R1, R0 and SREG in interrupt/signal
  // handlers at the very end of the function, just before reti.
  if (isHandler) {
    BuildMI(MBB, MBBI, DL, TII.get(MOS::POPRd), MOS::R0);
    BuildMI(MBB, MBBI, DL, TII.get(MOS::OUTARr))
        .addImm(0x3f)
        .addReg(MOS::R0, RegState::Kill);
    BuildMI(MBB, MBBI, DL, TII.get(MOS::POPWRd), MOS::R1R0);
  }

  if (hasFP(MF))
    BuildMI(MBB, MBBI, DL, TII.get(MOS::POPWRd), MOS::R29R28);

  // Early exit if there is no need to restore the frame pointer.
  if (!FrameSize) {
    return;
  }

  // Skip the callee-saved pop instructions.
  while (MBBI != MBB.begin()) {
    MachineBasicBlock::iterator PI = std::prev(MBBI);
    int Opc = PI->getOpcode();

    if (Opc != MOS::POPRd && Opc != MOS::POPWRd && !PI->isTerminator()) {
      break;
    }

    --MBBI;
  }

  unsigned Opcode;

  // Select the optimal opcode depending on how big it is.
  if (isUInt<6>(FrameSize)) {
    Opcode = MOS::ADIWRdK;
  } else {
    Opcode = MOS::SUBIWRdK;
    FrameSize = -FrameSize;
  }

  // Restore the frame pointer by doing FP += <size>.
  MachineInstr *MI = BuildMI(MBB, MBBI, DL, TII.get(Opcode), MOS::R29R28)
                         .addReg(MOS::R29R28, RegState::Kill)
                         .addImm(FrameSize);
  // The SREG implicit def is dead.
  MI->getOperand(3).setIsDead();

  // Write back R29R28 to SP and temporarily disable interrupts.
  BuildMI(MBB, MBBI, DL, TII.get(MOS::SPWRITE), MOS::SP)
      .addReg(MOS::R29R28, RegState::Kill);
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
  if (CSI.empty()) {
    return false;
  }

  DebugLoc DL = MBB.findDebugLoc(MI);
  const MachineFunction &MF = *MBB.getParent();
  const MOSSubtarget &STI = MF.getSubtarget<MOSSubtarget>();
  const TargetInstrInfo &TII = *STI.getInstrInfo();

  for (const CalleeSavedInfo &CCSI : CSI) {
    unsigned Reg = CCSI.getReg();

    assert(TRI->getRegSizeInBits(*TRI->getMinimalPhysRegClass(Reg)) == 8 &&
           "Invalid register size");

    BuildMI(MBB, MI, DL, TII.get(MOS::POPRd), Reg);
  }

  return true;
}

/// Replace pseudo store instructions that pass arguments through the stack with
/// real instructions. If insertPushes is true then all instructions are
/// replaced with push instructions, otherwise regular std instructions are
/// inserted.
static void fixStackStores(MachineBasicBlock &MBB,
                           MachineBasicBlock::iterator MI,
                           const TargetInstrInfo &TII, bool insertPushes) {
  const MOSSubtarget &STI = MBB.getParent()->getSubtarget<MOSSubtarget>();
  const TargetRegisterInfo &TRI = *STI.getRegisterInfo();

  // Iterate through the BB until we hit a call instruction or we reach the end.
  for (auto I = MI, E = MBB.end(); I != E && !I->isCall();) {
    MachineBasicBlock::iterator NextMI = std::next(I);
    MachineInstr &MI = *I;
    unsigned Opcode = I->getOpcode();

    // Only care of pseudo store instructions where SP is the base pointer.
    if (Opcode != MOS::STDSPQRr && Opcode != MOS::STDWSPQRr) {
      I = NextMI;
      continue;
    }

    assert(MI.getOperand(0).getReg() == MOS::SP &&
           "Invalid register, should be SP!");
    if (insertPushes) {
      // Replace this instruction with a push.
      unsigned SrcReg = MI.getOperand(2).getReg();
      bool SrcIsKill = MI.getOperand(2).isKill();

      // We can't use PUSHWRr here because when expanded the order of the new
      // instructions are reversed from what we need. Perform the expansion now.
      if (Opcode == MOS::STDWSPQRr) {
        BuildMI(MBB, I, MI.getDebugLoc(), TII.get(MOS::PUSHRr))
            .addReg(TRI.getSubReg(SrcReg, MOS::sub_hi),
                    getKillRegState(SrcIsKill));
        BuildMI(MBB, I, MI.getDebugLoc(), TII.get(MOS::PUSHRr))
            .addReg(TRI.getSubReg(SrcReg, MOS::sub_lo),
                    getKillRegState(SrcIsKill));
      } else {
        BuildMI(MBB, I, MI.getDebugLoc(), TII.get(MOS::PUSHRr))
            .addReg(SrcReg, getKillRegState(SrcIsKill));
      }

      MI.eraseFromParent();
      I = NextMI;
      continue;
    }

    // Replace this instruction with a regular store. Use Y as the base
    // pointer since it is guaranteed to contain a copy of SP.
    unsigned STOpc =
        (Opcode == MOS::STDWSPQRr) ? MOS::STDWPtrQRr : MOS::STDPtrQRr;

    MI.setDesc(TII.get(STOpc));
    MI.getOperand(0).setReg(MOS::R29R28);

    I = NextMI;
  }
}

MachineBasicBlock::iterator MOSFrameLowering::eliminateCallFramePseudoInstr(
    MachineFunction &MF, MachineBasicBlock &MBB,
    MachineBasicBlock::iterator MI) const {
  const MOSSubtarget &STI = MF.getSubtarget<MOSSubtarget>();
  const TargetFrameLowering &TFI = *STI.getFrameLowering();
  const MOSInstrInfo &TII = *STI.getInstrInfo();

  // There is nothing to insert when the call frame memory is allocated during
  // function entry. Delete the call frame pseudo and replace all pseudo stores
  // with real store instructions.
  if (TFI.hasReservedCallFrame(MF)) {
    fixStackStores(MBB, MI, TII, false);
    return MBB.erase(MI);
  }

  DebugLoc DL = MI->getDebugLoc();
  unsigned int Opcode = MI->getOpcode();
  int Amount = TII.getFrameSize(*MI);

  // Adjcallstackup does not need to allocate stack space for the call, instead
  // we insert push instructions that will allocate the necessary stack.
  // For adjcallstackdown we convert it into an 'adiw reg, <amt>' handling
  // the read and write of SP in I/O space.
  if (Amount != 0) {
    assert(TFI.getStackAlignment() == 1 && "Unsupported stack alignment");

    if (Opcode == TII.getCallFrameSetupOpcode()) {
      fixStackStores(MBB, MI, TII, true);
    } else {
      assert(Opcode == TII.getCallFrameDestroyOpcode());

      // Select the best opcode to adjust SP based on the offset size.
      unsigned addOpcode;
      if (isUInt<6>(Amount)) {
        addOpcode = MOS::ADIWRdK;
      } else {
        addOpcode = MOS::SUBIWRdK;
        Amount = -Amount;
      }

      // Build the instruction sequence.
      BuildMI(MBB, MI, DL, TII.get(MOS::SPREAD), MOS::R31R30).addReg(MOS::SP);

      MachineInstr *New = BuildMI(MBB, MI, DL, TII.get(addOpcode), MOS::R31R30)
                              .addReg(MOS::R31R30, RegState::Kill)
                              .addImm(Amount);
      New->getOperand(3).setIsDead();

      BuildMI(MBB, MI, DL, TII.get(MOS::SPWRITE), MOS::SP)
          .addReg(MOS::R31R30, RegState::Kill);
    }
  }

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
    const MachineFrameInfo &MFI = MF.getFrameInfo();
    MOSMachineFunctionInfo *FuncInfo = MF.getInfo<MOSMachineFunctionInfo>();

    // If there are no fixed frame indexes during this stage it means there
    // are allocas present in the function.
    if (MFI.getNumObjects() != MFI.getNumFixedObjects()) {
      // Check for the type of allocas present in the function. We only care
      // about fixed size allocas so do not give false positives if only
      // variable sized allocas are present.
      for (unsigned i = 0, e = MFI.getObjectIndexEnd(); i != e; ++i) {
        // Variable sized objects have size 0.
        if (MFI.getObjectSize(i)) {
          FuncInfo->setHasAllocas(true);
          break;
        }
      }
    }

    // If there are fixed frame indexes present, scan the function to see if
    // they are really being used.
    if (MFI.getNumFixedObjects() == 0) {
      return false;
    }

    // Ok fixed frame indexes present, now scan the function to see if they
    // are really being used, otherwise we can ignore them.
    for (const MachineBasicBlock &BB : MF) {
      for (const MachineInstr &MI : BB) {
        int Opcode = MI.getOpcode();

        if ((Opcode != MOS::LDDRdPtrQ) && (Opcode != MOS::LDDWRdPtrQ) &&
            (Opcode != MOS::STDPtrQRr) && (Opcode != MOS::STDWPtrQRr)) {
          continue;
        }

        for (const MachineOperand &MO : MI.operands()) {
          if (!MO.isFI()) {
            continue;
          }

          if (MFI.isFixedObjectIndex(MO.getIndex())) {
            FuncInfo->setHasStackArgs(true);
            return false;
          }
        }
      }
    }

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
    if (!MF.getFrameInfo().hasVarSizedObjects()) {
      return false;
    }

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

    // Restore SP in all exit basic blocks.
    for (MachineBasicBlock &MBB : MF) {
      // If last instruction is a return instruction, add a restore copy.
      if (!MBB.empty() && MBB.back().isReturn()) {
        MBBI = MBB.getLastNonDebugInstr();
        DL = MBBI->getDebugLoc();
        BuildMI(MBB, MBBI, DL, TII.get(MOS::COPY), MOS::SP)
            .addReg(SPCopy, RegState::Kill);
      }
    }

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

