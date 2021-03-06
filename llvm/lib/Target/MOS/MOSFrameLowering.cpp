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
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
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

  // Targeted value of S at end of prologue, with S=0 on entry.
  int S = 0;

  // Extract out as many values as will fit onto the hard stack.
  for (int Idx = 0, End = MFI.getObjectIndexEnd(); Idx < End && S > -4; ++Idx) {
    int64_t Size = MFI.getObjectSize(Idx);
    if (MFI.isDeadObjectIndex(Idx) || MFI.isVariableSizedObjectIndex(Idx))
      continue;
    if (S - Size < -4)
      continue;
    MFI.setStackID(Idx, TargetStackID::Hard);
    S -= Size;
    // S points one byte below the real top of the stack.
    MFI.setObjectOffset(Idx, S + 1);
    MFI.setStackSize(MFI.getStackSize() - Size);
  }
}

MachineBasicBlock::iterator MOSFrameLowering::eliminateCallFramePseudoInstr(
    MachineFunction &MF, MachineBasicBlock &MBB,
    MachineBasicBlock::iterator MI) const {
  int64_t Offset = MI->getOperand(0).getImm();
  if (hasReservedCallFrame(MF) || !Offset)
    return MBB.erase(MI);

  const auto &TII = *MF.getSubtarget().getInstrInfo();
  if (MI->getOpcode() == TII.getCallFrameSetupOpcode())
    MI->getOperand(0).setImm(-Offset);
  MI->setDesc(TII.get(MOS::IncSP));
  MI->RemoveOperand(1);
  return MI;
}

void MOSFrameLowering::emitPrologue(MachineFunction &MF,
                                        MachineBasicBlock &MBB) const {
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();

  auto MI = MBB.begin();

  // If soft stack is used, decrease the soft stack pointer SP.
  if (MFI.getStackSize()) {
    MachineIRBuilder Builder(MBB, MI);
    Builder.buildInstr(MOS::IncSP).addImm(-MFI.getStackSize());
    if (hasFP(MF))
      Builder.buildCopy(MOS::RS1, Register(MOS::RS0));
  }

  uint64_t HsSize = hsSize(MFI);

  LLVM_DEBUG(dbgs() << "\nEmitting Prologue:\n");

  // Value of S at MI, relative to its value on entry.
  int S = 0;

  // Push undef to the stack until S has decreased to NewS.
  const auto PushUntil = [&](int NewS) {
    LLVM_DEBUG(dbgs() << "S:" << S << "\n");
    LLVM_DEBUG(dbgs() << "Pushing until S=" << NewS << "\n");

    MachineIRBuilder Builder(MBB, MI);
    // It doesn't matter what is pushed, just that the stack pointer is
    // decreased. This is still at least as efficient as increasing S using X
    // and/or A.
    for (; S > NewS; --S) {
      auto Push =
          Builder.buildInstr(MOS::Push).addUse(MOS::A, RegState::Undef);
      LLVM_DEBUG(dbgs() << *Push);
    }
  };

  // Defer pushing to the stack as long as possible. Hopefully, this will allow
  // folding STstk together with the push that decreases the stack pointer.
  for (; MI != MBB.end(); ++MI) {
    // For now, this is basic-block only.
    if (MI->isTerminator())
      break;
    if (MI->isCall())
      break;

    bool NeedsCorrectS = false;
    unsigned Idx;
    switch (MI->getOpcode()) {
    default:
      continue;
    case MOS::AddrLostk:
      Idx = MI->getOperand(2).getIndex();
      NeedsCorrectS = true;
      break;
    case MOS::LDstk:
    case MOS::AddrHistk:
    case MOS::Addrstk:
      Idx = MI->getOperand(1).getIndex();
      NeedsCorrectS = true;
      break;
    case MOS::STstk:
      Idx = MI->getOperand(1).getIndex();
      break;
    }

    // Soft stack operations don't interact with the hard stack.
    if (MFI.getStackID(Idx) != TargetStackID::Hard)
      continue;

    if (NeedsCorrectS)
      break;

    // Offset of the greatest (furthest from top) byte of the slot.
    int64_t Offset = MFI.getObjectOffset(Idx) + MFI.getObjectSize(Idx) - 1;
    LLVM_DEBUG(dbgs() << "S: " << S << "\n");
    LLVM_DEBUG(dbgs() << "Index Offset: " << Offset << "\n");

    // If the store is to a lower S, we can decrease S up to this point,
    // allowing us to fold the store.
    if (S > Offset)
      PushUntil(Offset);
    // If the store is to a higher S, it cannot be folded. It's relative to the
    // fully-offset S, so we're done.
    else if (S < Offset)
      break;

    // The store is to S, so it can folded together with a Push or two.
    assert(Offset == S);
    MachineIRBuilder Builder(MBB, MI);
    Register Src = MI->getOperand(0).getReg();
    if (MOS::ZP_PTRRegClass.contains(Src)) {
      // The high byte is pushed first, since it should have the higher memory
      // location (towards the bottom of the stack).
      auto Push = Builder.buildInstr(MOS::Push)
                      .addUse(TRI.getSubReg(Src, MOS::subhi));
      LLVM_DEBUG(dbgs() << *Push);
      Push = Builder.buildInstr(MOS::Push)
                 .addUse(TRI.getSubReg(Src, MOS::sublo));
      LLVM_DEBUG(dbgs() << *Push);
    } else {
      auto Push = Builder.buildInstr(MOS::Push).add(MI->getOperand(0));
      LLVM_DEBUG(dbgs() << *Push);
    }
    LLVM_DEBUG(dbgs() << "Erasing: " << *MI);
    MI = MBB.erase(MI);
    // MI will be incremented on the way out.
    --MI;
    S -= MFI.getObjectSize(Idx);
  }

  // Push any remaining bytes necessary to fully decrease S.
  PushUntil(-HsSize);
}

void MOSFrameLowering::emitEpilogue(MachineFunction &MF,
                                        MachineBasicBlock &MBB) const {
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();

  auto End = MBB.getFirstTerminator();

  // If soft stack is used, increase the soft stack pointer SP.
  if (MFI.getStackSize()) {
    MachineIRBuilder Builder(MBB, End);
    if (hasFP(MF))
      Builder.buildCopy(MOS::RS0, Register(MOS::RS1));
    Builder.buildInstr(MOS::IncSP).addImm(MFI.getStackSize());
    --End;
  }

  // Note: This causes MI to point to the instruction *before* End.
  auto MI = MachineBasicBlock::reverse_iterator(End);

  uint64_t HsSize = hsSize(MFI);

  LLVM_DEBUG(dbgs() << "\nEmitting Epilogue:\n");

  // Value of S at MI, relative to its value on exit.
  int S = 0;

  // Emit pull instructions before MI as necessary to decrease S to
  // NewS.
  const auto PullUntil = [&](int NewS) {
    MachineIRBuilder Builder(MBB, std::prev(MI).getReverse());

    LLVM_DEBUG(dbgs() << "S:" << S << "\n");
    LLVM_DEBUG(dbgs() << "Pulling until S=" << NewS << "\n");

    for (; S > NewS; --S) {
      auto Pull = Builder.buildInstr(MOS::PullPreserve);
      LLVM_DEBUG(dbgs() << *Pull);
    }
  };

  // Defer pulling from the stack to as early as possible. Hopefully, this will
  // allow folding loads together with the Pull that decreases the stack
  // pointer.
  for (; MI != MBB.rend(); ++MI) {
    LLVM_DEBUG(dbgs() << *MI);

    if (MI->isCall())
      break;

    bool IsPush = false;
    bool NeedsCorrectS = false;
    unsigned Idx;
    switch (MI->getOpcode()) {
    default:
      continue;
    case MOS::AddrLostk:
      Idx = MI->getOperand(2).getIndex();
      NeedsCorrectS = true;
      break;
    case MOS::STstk:
    case MOS::AddrHistk:
    case MOS::Addrstk:
      Idx = MI->getOperand(1).getIndex();
      NeedsCorrectS = true;
      break;
    case MOS::Push:
      IsPush = true;
      break;
    case MOS::LDstk:
      Idx = MI->getOperand(1).getIndex();
      break;
    }

    // Special case push, since it doesn't have an index operand.
    if (IsPush)
      break;

    // Soft stack operations don't interact with the hard stack.
    if (MFI.getStackID(Idx) != TargetStackID::Hard)
      continue;

    // Break if current instruction needs the fully offset S.
    if (NeedsCorrectS)
      break;

    // Offset of the greatest (furthest from top) byte of the slot.
    int64_t Offset = MFI.getObjectOffset(Idx) + MFI.getObjectSize(Idx) - 1;

    LLVM_DEBUG(dbgs() << "S: " << S << "\n");
    LLVM_DEBUG(dbgs() << "Index Offset: " << Offset << "\n");

    // If the load is to a lower S, we can decrease S by emitting pulls to reach
    // that S, allowing the load to be folded with a pull.
    if (S > Offset)
      PullUntil(Offset);
    // If the load is to a higher S, it cannot be folded and is relative to the
    // fully-offset S, so we're done.
    else if (S < Offset)
      break;

    // The load is to S, so it can folded together with a pull.
    assert(Offset == S);

    MachineIRBuilder Builder(MBB, std::prev(MI).getReverse());
    Register Dst = MI->getOperand(0).getReg();
    if (MOS::ZP_PTRRegClass.contains(Dst)) {
      // The low byte is pulled first, since it has the lower memory location
      // (towards the top of the stack).
      auto Pull = Builder.buildInstr(MOS::Pull)
                      .addDef(TRI.getSubReg(Dst, MOS::sublo));
      LLVM_DEBUG(dbgs() << *Pull);
      Pull = Builder.buildInstr(MOS::Pull)
                 .addDef(TRI.getSubReg(Dst, MOS::subhi));
      LLVM_DEBUG(dbgs() << *Pull);
    } else {
      auto Pull = Builder.buildInstr(MOS::Pull).add(MI->getOperand(0));
      LLVM_DEBUG(dbgs() << *Pull);
    }
    auto NextMI = std::next(MI);
    LLVM_DEBUG(dbgs() << "Erasing: " << *MI);
    MI->eraseFromParent();
    MI = NextMI;
    // MI will be incremented on the way out.
    --MI;
    S -= MFI.getObjectSize(Idx);
  }

  // Pull any remaining bytes necessary to fully decrease S.
  PullUntil(-HsSize);
}

bool MOSFrameLowering::hasFP(const MachineFunction &MF) const {
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  return MFI.isFrameAddressTaken() || MFI.hasVarSizedObjects();
}

bool MOSFrameLowering::isSupportedStackID(TargetStackID::Value ID) const {
  switch (ID) {
  default:
    return false;
  case TargetStackID::Default:
  case TargetStackID::NoAlloc:
  case TargetStackID::Hard:
    return true;
  }
}

uint64_t MOSFrameLowering::hsSize(const MachineFrameInfo &MFI) const {
  uint64_t Size = 0;
  for (int i = 0, e = MFI.getObjectIndexEnd(); i < e; ++i)
    if (MFI.getStackID(i) == TargetStackID::Hard)
      Size += MFI.getObjectSize(i);
  return Size;
}

uint64_t MOSFrameLowering::staticSize(const MachineFrameInfo &MFI) const {
  uint64_t Size = 0;
  for (int i = 0, e = MFI.getObjectIndexEnd(); i < e; ++i)
    if (MFI.getStackID(i) == TargetStackID::NoAlloc)
      Size += MFI.getObjectSize(i);
  return Size;
}
