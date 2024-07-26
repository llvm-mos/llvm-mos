//===-- MOSLateOptimization.cpp - MOS Late Optimization -------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MOS late optimization pass.
//
// This pass performs simple optimizations once pseudo-instructions have been
// fully lowered. These optimizations might otherwise increase register pressure
// and cause spills, so they're done opportunistically at the very end.
//
//===----------------------------------------------------------------------===//

#include "MOSLateOptimization.h"

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "MOSInstrBuilder.h"
#include "MOSRegisterInfo.h"
#include "MOSSubtarget.h"

#include "llvm/ADT/SmallSet.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/LivePhysRegs.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"

#define DEBUG_TYPE "mos-late-opt"

using namespace llvm;

namespace {

class MOSLateOptimization : public MachineFunctionPass {
public:
  static char ID;

  MOSLateOptimization() : MachineFunctionPass(ID) {
    llvm::initializeMOSLateOptimizationPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;
  bool lowerCmpZeros(MachineBasicBlock &MBB) const;
  void lowerCmpZero(MachineInstr &MI) const;
  bool combineLdImm(MachineBasicBlock &MBB) const;
  bool tailJMP(MachineBasicBlock &MBB) const;
};

bool MOSLateOptimization::runOnMachineFunction(MachineFunction &MF) {
  bool Changed = false;
  for (MachineBasicBlock &MBB : MF) {
    Changed |= lowerCmpZeros(MBB);
    Changed |= combineLdImm(MBB);
    Changed |= tailJMP(MBB);
  }
  return Changed;
}

static bool definesNZ(const MachineInstr &MI, Register Val, const MOSSubtarget &STI) {
  if (MI.getOpcode() == MOS::CL)
    return false;
  if (STI.hasSPC700() && MI.getOpcode() == MOS::PL)
    return false;
  if (MI.getOpcode() == MOS::MOVImag8)
    return false;
  if (MI.getOpcode() == MOS::STImag8)
    return false;
  if (MI.definesRegister(Val, /*TRI=*/nullptr))
    return true;
  switch (MI.getOpcode()) {
  default:
    return false;
  case MOS::TA:
  case MOS::T_A:
  case MOS::TX:
  case MOS::LDImag8:
    return MI.getOperand(1).getReg() == Val;
  }
}

bool MOSLateOptimization::lowerCmpZeros(MachineBasicBlock &MBB) const {
  const auto &MRI = MBB.getParent()->getRegInfo();
  const auto &STI = MBB.getParent()->getSubtarget<MOSSubtarget>();
  const auto *TRI = MRI.getTargetRegisterInfo();
  bool Changed = false;
  for (MachineInstr &MI : make_early_inc_range(mbb_reverse(MBB))) {
    if (MI.getOpcode() != MOS::CmpZero)
      continue;

    if (MI.allDefsAreDead()) {
      MI.eraseFromParent();
      continue;
    }

    Register Val = MI.getOperand(0).getReg();

    for (auto &J : mbb_reverse(MBB.begin(), MI)) {
      if (J.isDebugInstr())
        continue;
      if (J.isCall() || J.isInlineAsm())
        break;
      if (definesNZ(J, Val, STI)) {
        Changed = true;
        J.addOperand(MachineOperand::CreateReg(MOS::NZ, /*isDef=*/true,
                                               /*isImp=*/true));
        MI.eraseFromParent();
        break;
      }
      if (J.modifiesRegister(MOS::NZ, TRI))
        break;
      bool ClobbersNZ = true;
      if (J.isBranch() || (J.mayStore() && !J.mayLoad()))
        ClobbersNZ = false;
      else
        switch (J.getOpcode()) {
        case MOS::CL:
        case MOS::CLV:
        case MOS::LDCImm:
        case MOS::MOVImag8:
        case MOS::STImag8:
        case MOS::PH:
        case MOS::SWAP:
          ClobbersNZ = false;
          break;
        case MOS::PL:
          if (STI.hasSPC700())
            ClobbersNZ = false;
          break;
        }
      if (ClobbersNZ)
        break;
    }
    if (Changed)
      continue;

    Changed = true;
    lowerCmpZero(MI);
  }
  return Changed;
}

void MOSLateOptimization::lowerCmpZero(MachineInstr &MI) const {
  auto &MBB = *MI.getParent();
  const auto &MRI = MBB.getParent()->getRegInfo();
  const auto *TRI = MRI.getTargetRegisterInfo();
  const MOSSubtarget &STI = MBB.getParent()->getSubtarget<MOSSubtarget>();
  Register Val = MI.getOperand(0).getReg();

  LivePhysRegs PhysRegs;
  PhysRegs.init(*TRI);
  PhysRegs.addLiveOuts(MBB);
  for (auto &J :
       make_range(MBB.rbegin(), MachineBasicBlock::reverse_iterator(MI)))
    PhysRegs.stepBackward(J);

  MachineIRBuilder Builder(MI);
  switch (Val) {
  default: {
    assert(MOS::Imag8RegClass.contains(Val));
    Register Tmp = 0;
    if (PhysRegs.available(MRI, MOS::A))
      Tmp = MOS::A;
    else if (PhysRegs.available(MRI, MOS::X))
      Tmp = MOS::X;
    else if (PhysRegs.available(MRI, MOS::Y))
      Tmp = MOS::Y;
    MachineInstrBuilder Access;
    if (Tmp) {
      Access = Builder.buildInstr(MOS::LDImag8, {Tmp}, {Val});
    } else {
      SmallSet<Register, 3> Defined;

      // At this point, unless we can find a GPR with the value, it'll take 4
      // bytes and 10 cycles for an INC ZP DEC ZP. So look really hard for the
      // value in a GPR.
      for (auto &J : mbb_reverse(MBB.begin(), MI)) {
        if (J.getOpcode() == MOS::LDImag8 && J.getOperand(1).getReg() == Val &&
            !Defined.contains(J.getOperand(0).getReg())) {
          Register GPR = J.getOperand(0).getReg();
          MI.getOperand(0).setReg(GPR);
          lowerCmpZero(MI);
          return;
        }

        if (J.getOpcode() == MOS::STImag8 && J.getOperand(0).getReg() == Val &&
            !Defined.contains(J.getOperand(1).getReg())) {
          Register GPR = J.getOperand(1).getReg();
          MI.getOperand(0).setReg(GPR);
          lowerCmpZero(MI);
          return;
        }

        // If Val was changed, then earlier GPRs couldn't have its new value.
        if (J.modifiesRegister(Val, TRI))
          break;

        if (J.modifiesRegister(MOS::A, TRI))
          Defined.insert(MOS::A);
        if (J.modifiesRegister(MOS::X, TRI))
          Defined.insert(MOS::X);
        if (J.modifiesRegister(MOS::Y, TRI))
          Defined.insert(MOS::Y);
        if (Defined.size() == 3)
          break;
      }

      Builder.buildInstr(MOS::INC, {Val}, {Val});
      Access = Builder.buildInstr(MOS::DEC, {Val}, {Val});
    }
    Access.addDef(MOS::NZ, RegState::Implicit);
    Access->getOperand(0).setIsDead();
    break;
  }
  case MOS::A: {
    MachineInstrBuilder Access;
    if (PhysRegs.available(MRI, MOS::X))
      Access = Builder.buildInstr(MOS::TA, {MOS::X}, {Register(MOS::A)});
    else if (PhysRegs.available(MRI, MOS::Y))
      Access = Builder.buildInstr(MOS::TA, {MOS::Y}, {Register(MOS::A)});
    else {
      Access = Builder.buildInstr(MOS::CMPImm, {MOS::C},
                                  {Register(MOS::A), INT64_C(0)});
    }
    Access.addDef(MOS::NZ, RegState::Implicit);
    Access->getOperand(0).setIsDead();
    break;
  }
  case MOS::X:
  case MOS::Y: {
    MachineInstrBuilder Access;
    auto OtherReg = Val == MOS::X ? MOS::Y : MOS::X;
    if (PhysRegs.available(MRI, MOS::A))
      Access = Builder.buildInstr(MOS::T_A, {MOS::A}, {Val});
    else if (PhysRegs.available(MRI, OtherReg) && STI.hasW65816Or65EL02())
      Access = Builder.buildInstr(MOS::TX, {OtherReg}, {Val});
    else
      Access = Builder.buildInstr(MOS::CMPImm, {MOS::C}, {Val, INT64_C(0)});
    Access.addDef(MOS::NZ, RegState::Implicit);
    Access->getOperand(0).setIsDead();
    break;
  }
  }
  MI.eraseFromParent();
}

bool MOSLateOptimization::combineLdImm(MachineBasicBlock &MBB) const {
  const auto &TII = *MBB.getParent()->getSubtarget().getInstrInfo();
  const auto *TRI = MBB.getParent()->getSubtarget().getRegisterInfo();
  const MOSSubtarget &STI = MBB.getParent()->getSubtarget<MOSSubtarget>();

  bool Changed = false;

  struct ImmLoad {
    MachineInstr *MI = nullptr;
    int64_t Val;
  } LoadA, LoadX, LoadY;

  for (MachineInstr &MI : MBB) {
    if (MI.getOpcode() != MOS::LDImm || !MI.getOperand(1).isImm()) {
      if (MI.modifiesRegister(MOS::A, TRI))
        LoadA.MI = nullptr;
      if (MI.modifiesRegister(MOS::X, TRI))
        LoadX.MI = nullptr;
      if (MI.modifiesRegister(MOS::Y, TRI))
        LoadY.MI = nullptr;
      continue;
    }

    Register Dst = MI.getOperand(0).getReg();
    int64_t Val = MI.getOperand(1).getImm();

    ImmLoad *Load = nullptr;
    // Try to replace with T__.
    switch (Dst) {
    case MOS::A: {
      Register Src;
      if (LoadX.MI && LoadX.Val == Val) {
        Src = MOS::X;
        Load = &LoadX;
      }
      if (LoadY.MI && LoadY.Val == Val) {
        Src = MOS::Y;
        Load = &LoadY;
      }
      if (Load) {
        MI.setDesc(TII.get(MOS::T_A));
        MI.getOperand(1).ChangeToRegister(Src, /*isDef=*/false);
      }
      break;
    }
    case MOS::X:
    case MOS::Y:
      if (LoadA.MI && LoadA.Val == Val) {
        Load = &LoadA;
        MI.setDesc(TII.get(MOS::TA));
        MI.getOperand(1).ChangeToRegister(MOS::A, /*isDef=*/false);
      } else if (STI.hasW65816Or65EL02()) {
        if (Dst == MOS::X && LoadY.MI && LoadY.Val == Val) {
          MI.setDesc(TII.get(MOS::TX));
          MI.getOperand(1).ChangeToRegister(MOS::Y, /*isDef=*/false);
        } else if (Dst == MOS::Y && LoadX.MI && LoadX.Val == Val) {
          MI.setDesc(TII.get(MOS::TX));
          MI.getOperand(1).ChangeToRegister(MOS::X, /*isDef=*/false);
        }
      }
      break;
    }

    // Try to replace with IN_ or DE_.
    if (!Load) {
      switch (Dst) {
      case MOS::A:
        if (STI.hasGPRIncDec())
          if (LoadA.MI && std::abs(LoadA.Val - Val) == 1)
            Load = &LoadA;
        break;
      case MOS::X:
        if (LoadX.MI && std::abs(LoadX.Val - Val) == 1)
          Load = &LoadX;
        break;
      case MOS::Y:
        if (LoadY.MI && std::abs(LoadY.Val - Val) == 1)
          Load = &LoadY;
        break;
      }

      if (Load) {
        MachineIRBuilder Builder(MI);
        MI.setDesc(TII.get(Val > Load->Val ? MOS::INC : MOS::DEC));
        MI.getOperand(1).ChangeToRegister(Dst, /*isDef=*/false, /*isImp=*/false,
                                          /*isKill=*/true);
        MI.tieOperands(0, 1);
      }
    }

    if (Load) {
      Changed = true;
      Load->MI->getOperand(0).setIsDead(false);
      for (MachineInstr &J : make_range(MachineBasicBlock::iterator(Load->MI),
                                        MachineBasicBlock::iterator(MI)))
        J.clearRegisterKills(Load->MI->getOperand(0).getReg(), TRI);
    }

    switch (Dst) {
    case MOS::A:
      Load = &LoadA;
      break;
    case MOS::X:
      Load = &LoadX;
      break;
    case MOS::Y:
      Load = &LoadY;
      break;
    }

    Load->MI = &MI;
    Load->Val = Val;
  }
  return Changed;
}

bool MOSLateOptimization::tailJMP(MachineBasicBlock &MBB) const {
  if (MBB.size() < 2)
    return false;
  auto It = std::prev(MBB.end());
  if (It->getOpcode() != MOS::RTS)
    return false;
  MachineInstr &RTS = *It;
  --It;
  if (It->getOpcode() != MOS::JSR)
    return false;
  MachineInstr &JSR = *It;
  RTS.eraseFromParent();
  JSR.setDesc(JSR.getMF()->getSubtarget().getInstrInfo()->get(MOS::TailJMP));
  return true;
}

} // namespace

char MOSLateOptimization::ID = 0;

INITIALIZE_PASS(MOSLateOptimization, DEBUG_TYPE, "MOS Late Optimizations",
                false, false)

MachineFunctionPass *llvm::createMOSLateOptimizationPass() {
  return new MOSLateOptimization;
}
