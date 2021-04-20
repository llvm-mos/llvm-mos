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
#include "MOSRegisterInfo.h"

#include "llvm/CodeGen/GlobalISel/CallLowering.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/PseudoSourceValue.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Support/ErrorHandling.h"

#define DEBUG_TYPE "mos-framelowering"

using namespace llvm;

MOSFrameLowering::MOSFrameLowering()
    : TargetFrameLowering(StackGrowsDown, /*StackAlignment=*/Align(1),
                          /*LocalAreaOffset=*/0) {}

bool MOSFrameLowering::assignCalleeSavedSpillSlots(
    MachineFunction &MF, const TargetRegisterInfo *TRI,
    std::vector<CalleeSavedInfo> &CSI) const {
  // The static stack is cheap, so just use that if possible. Note that it's
  // still sometimes worth using CSRs even if static stacks are available, since
  // it allows pointers to live across calls directly in ZP.
  if (MF.getFunction().doesNotRecurse())
    return false;

  // If static stack is unavailalbe, place the CSI on the hard stack, which we
  // don't explicitly model in PEI. Accordingly, this does nothing, but says
  // everything is fine. (spill/restore)CalleeSavedRegisters will emit the
  // spills and reloads sequentially to and from the hard stack.
  return true;
}

bool MOSFrameLowering::spillCalleeSavedRegisters(
    MachineBasicBlock &MBB, MachineBasicBlock::iterator MI,
    ArrayRef<CalleeSavedInfo> CSI, const TargetRegisterInfo *TRI) const {
  // The static stack is cheap, so just use that if possible.
  if (MBB.getParent()->getFunction().doesNotRecurse())
    return false;

  MachineIRBuilder Builder(MBB, MI);
  bool AMaybeLive = MBB.computeRegisterLiveness(TRI, MOS::A, MI) !=
                    MachineBasicBlock::LQR_Dead;

  // We cannot save/restore using PHA/PLA here: it would interfere with the PHA
  // of the CSRs.
  if (AMaybeLive)
    Builder.buildInstr(MOS::STAbs).addUse(MOS::A).addExternalSymbol("__save_a");
  // There are intentionally very few CSRs, few enough to place on the hard
  // stack without much risk of overflow. This is the only across-calls way the
  // compiler uses the hard stack, since the free CSRs can then be used with
  // impunity. This is slightly more expensive than saving/resting values
  // directly on the hard stack, but it's significantly simpler.
  for (const CalleeSavedInfo &CI : CSI) {
    Builder.buildCopy(MOS::A, CI.getReg());
    Builder.buildInstr(MOS::PH).addUse(MOS::A);
  }
  if (AMaybeLive)
    Builder.buildInstr(MOS::LDAbs).addDef(MOS::A).addExternalSymbol("__save_a");
  return true;
}

bool MOSFrameLowering::restoreCalleeSavedRegisters(
    MachineBasicBlock &MBB, MachineBasicBlock::iterator MI,
    MutableArrayRef<CalleeSavedInfo> CSI, const TargetRegisterInfo *TRI) const {
  // The static stack is cheap, so it was used if available.
  bool UsesHardStack = !MBB.getParent()->getFunction().doesNotRecurse();

  // Reverse the process of spillCalleeSavedRegisters.
  if (UsesHardStack) {
    bool AMaybeLive = MBB.computeRegisterLiveness(TRI, MOS::A, MI) !=
                      MachineBasicBlock::LQR_Dead;
    MachineIRBuilder Builder(MBB, MI);
    // We cannot save/restore using PHA/PLA here: it would interfere with the
    // PLA of the CSRs.
    if (AMaybeLive)
      Builder.buildInstr(MOS::STAbs)
          .addUse(MOS::A)
          .addExternalSymbol("__save_a");
    for (const CalleeSavedInfo &CI : reverse(CSI)) {
      Builder.buildInstr(MOS::PL).addDef(MOS::A);
      Builder.buildCopy(CI.getReg(), Register(MOS::A));
    }
    if (AMaybeLive)
      Builder.buildInstr(MOS::LDAbs)
          .addDef(MOS::A)
          .addExternalSymbol("__save_a");
  }

  // Mark the CSRs as used by the return to ensure Machine Copy Propagation
  // doesn't remove the copies that set them.
  if (MBB.succ_empty()) {
    assert(MBB.rbegin()->isReturn());
    for (const CalleeSavedInfo &CI : CSI) {
      MBB.rbegin()->addOperand(
          *MBB.getParent(), MachineOperand::CreateReg(
                                CI.getReg(), /*isDef=*/false, /*isImp=*/true));
    }
  }
  return UsesHardStack;
}

void MOSFrameLowering::processFunctionBeforeFrameFinalized(
    MachineFunction &MF, RegScavenger *RS) const {
  MachineFrameInfo &MFI = MF.getFrameInfo();

  // Assign all locals to static stack in non-recursive functions.
  if (MF.getFunction().doesNotRecurse()) {
    int64_t Offset = 0;
    for (int Idx = 0, End = MFI.getObjectIndexEnd(); Idx < End; ++Idx) {
      if (MFI.isDeadObjectIndex(Idx) || MFI.isVariableSizedObjectIndex(Idx))
        continue;

      MFI.setStackID(Idx, TargetStackID::NoAlloc);
      MFI.setObjectOffset(Idx, Offset);
      Offset += MFI.getObjectSize(Idx); // Static stack grows up.
    }
    return;
  }
}

MachineBasicBlock::iterator MOSFrameLowering::eliminateCallFramePseudoInstr(
    MachineFunction &MF, MachineBasicBlock &MBB,
    MachineBasicBlock::iterator MI) const {
  int64_t Offset = MI->getOperand(0).getImm();

  // If we've already reserved the outgoing call frame in the prolog/epilog, the
  // pseudo can be summarily removed.
  if (hasReservedCallFrame(MF) || !Offset)
    return MBB.erase(MI);

  // Increment/decrement the stack pointer to reserve space for the call frame.
  MachineIRBuilder Builder(MBB, MI);
  if (MI->getOpcode() ==
      MF.getSubtarget().getInstrInfo()->getCallFrameSetupOpcode())
    Offset = -Offset;
  emitIncSP(Builder, Offset);
  return MBB.erase(MI);
}

void MOSFrameLowering::emitPrologue(MachineFunction &MF,
                                    MachineBasicBlock &MBB) const {
  const MachineFrameInfo &MFI = MF.getFrameInfo();

  MachineIRBuilder Builder(MBB, MBB.begin());
  if (hasFP(MF))
    Builder.buildCopy(MOS::RS1, Register(MOS::RS0));

  // If soft stack is used, decrease the soft stack pointer SP.
  if (MFI.getStackSize())
    emitIncSP(Builder, -MFI.getStackSize());
}

void MOSFrameLowering::emitEpilogue(MachineFunction &MF,
                                    MachineBasicBlock &MBB) const {
  const MachineFrameInfo &MFI = MF.getFrameInfo();

  MachineIRBuilder Builder(MBB, MBB.getFirstTerminator());
  if (hasFP(MF))
    Builder.buildCopy(MOS::RS0, Register(MOS::RS1));

  // If soft stack is used, increase the soft stack pointer SP.
  if (MFI.getStackSize())
    emitIncSP(Builder, MFI.getStackSize());
}

bool MOSFrameLowering::hasFP(const MachineFunction &MF) const {
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  return MFI.isFrameAddressTaken() || MFI.hasVarSizedObjects();
}

uint64_t MOSFrameLowering::staticSize(const MachineFrameInfo &MFI) const {
  uint64_t Size = 0;
  for (int Idx = 0, End = MFI.getObjectIndexEnd(); Idx < End; ++Idx)
    if (MFI.getStackID(Idx) == TargetStackID::NoAlloc)
      Size += MFI.getObjectSize(Idx);
  return Size;
}

void MOSFrameLowering::emitIncSP(MachineIRBuilder &Builder,
                                 int64_t Offset) const {
  assert(Offset);
  assert(-32768 <= Offset && Offset < 32768);

  auto Bytes = static_cast<uint16_t>(Offset);
  int64_t LoBytes = Bytes & 0xFF;
  int64_t HiBytes = Bytes >> 8;
  assert(LoBytes || HiBytes);

  Register A = Builder.getMRI()->createVirtualRegister(&MOS::AcRegClass);

  Register C = Builder.buildInstr(MOS::LDCImm, {&MOS::CcRegClass}, {INT64_C(0)})
                   .getReg(0);
  if (LoBytes) {
    Builder.buildCopy(A, Register(MOS::RC0));
    Builder.buildInstr(MOS::ADCImm, {A, C, &MOS::VcRegClass}, {A, LoBytes, C});
    Builder.buildCopy(MOS::RC0, A);
  }

  auto LoCopy = Builder.buildCopy(A, Register(MOS::RC1));
  // The implicit use appeases the register scavenger, which wants to see one
  // clear sequence of definitions and redefinitions.
  if (LoBytes)
    LoCopy.addUse(A, RegState::Implicit);

  Builder.buildInstr(MOS::ADCImm, {A, C, &MOS::VcRegClass}, {A, HiBytes, C});
  Builder.buildCopy(MOS::RC1, A);
}