//===-- MOSInsertCopies.cpp - MOS Copy Insertion --------------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MOS copy insertion pass.
//
// Register coalescing can tightly restrict the register classes of virtual
// registers in the name of avoiding copies. This is usually a good thing, but
// occasionally it's better (or at least not any worse) to copy, since it allows
// use of a faster addressing mode. This pass finds likely candidates for this
// and inserts copies to widen the register classes to include the fastest
// possible operands.
//
//===----------------------------------------------------------------------===//

#include "MOSInsertCopies.h"

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "MOSRegisterInfo.h"

#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"

#define DEBUG_TYPE "mos-insert-copies"

using namespace llvm;

namespace {

class MOSInsertCopies : public MachineFunctionPass {
public:
  static char ID;

  MOSInsertCopies() : MachineFunctionPass(ID) {
    llvm::initializeMOSInsertCopiesPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;
};

bool MOSInsertCopies::runOnMachineFunction(MachineFunction &MF) {
  if (skipFunction(MF.getFunction()))
    return false;

  MachineRegisterInfo &MRI = MF.getRegInfo();

  bool Changed = false;
  for (MachineBasicBlock &MBB : MF) {
    MachineBasicBlock::iterator Next;
    for (auto I = MBB.begin(), E = MBB.end(); I != E; I = Next) {
      Next = std::next(I);
      const TargetRegisterClass *WideRC;
      switch (I->getOpcode()) {
      default:
        continue;
      case MOS::ASL:
      case MOS::LSR:
      case MOS::ROL:
      case MOS::ROR:
        WideRC = &MOS::AImag8RegClass;
        break;
      case MOS::INC:
      case MOS::DEC:
      case MOS::IncNMOS:
      case MOS::DecNMOS:
      case MOS::IncMB:
      case MOS::DecMB:
      case MOS::DecDcpMB:
        WideRC = &MOS::Anyi8RegClass;
        break;
      }

      for (unsigned Idx = 0, EndIdx = I->getNumExplicitDefs(); Idx != EndIdx; ++Idx) {
        MachineOperand &DstOp = I->getOperand(Idx);
        if (!DstOp.isReg() || !DstOp.isTied() || !DstOp.getReg().isVirtual())
          continue;
        MachineOperand &SrcOp = I->getOperand(I->findTiedOperandIdx(Idx));

        const TargetRegisterClass *SrcRC = MRI.getRegClass(SrcOp.getReg());
        const TargetRegisterClass *DstRC = MRI.getRegClass(DstOp.getReg());

        // This may be an unrelated tied register; if so, ignore.
        if (!SrcRC->hasSuperClassEq(WideRC) || !DstRC->hasSuperClassEq(WideRC))
          continue;

        // Avoid copying to and from Imag8 just to make the regclass wider. This
        // could produce LDA ASL STA patterns, when it'd be better to just ASL.
        if (SrcRC == &MOS::Imag8RegClass && DstRC == &MOS::Imag8RegClass)
          continue;

        if (SrcRC != WideRC) {
          Changed = true;
          MachineIRBuilder Builder(MBB, I);
          SrcOp.setReg(Builder.buildCopy(WideRC, SrcOp).getReg(0));
        }
        if (DstRC != WideRC) {
          Changed = true;
          Register NewDst = MRI.createVirtualRegister(WideRC);
          MachineIRBuilder Builder(MBB, Next);
          Builder.buildCopy(DstOp, NewDst);
          DstOp.setReg(NewDst);
        }
      }
    }
  }
  return Changed;
}

} // namespace

char MOSInsertCopies::ID = 0;

INITIALIZE_PASS(MOSInsertCopies, DEBUG_TYPE, "MOS Copy Insertion", false, false)

MachineFunctionPass *llvm::createMOSInsertCopiesPass() {
  return new MOSInsertCopies;
}
