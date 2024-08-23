//===-- MOSISelLowering.cpp - MOS DAG Lowering Implementation -------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the interfaces that MOS uses to lower LLVM code into a
// selection DAG.
//
//===----------------------------------------------------------------------===//

#include "MOSISelLowering.h"

#include "llvm/ADT/StringSwitch.h"
#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/LivePhysRegs.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"
#include "llvm/IR/Function.h"
#include "llvm/Support/ErrorHandling.h"

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "MOSInstrBuilder.h"
#include "MOSInstrInfo.h"
#include "MOSRegisterInfo.h"
#include "MOSSubtarget.h"
#include "MOSTargetMachine.h"

using namespace llvm;

MOSTargetLowering::MOSTargetLowering(const MOSTargetMachine &TM,
                                     const MOSSubtarget &STI)
    : TargetLowering(TM) {
  addRegisterClass(MVT::i1, &MOS::Anyi1RegClass);
  addRegisterClass(MVT::i8, &MOS::Anyi8RegClass);
  addRegisterClass(MVT::i16, &MOS::Imag16RegClass);
  computeRegisterProperties(STI.getRegisterInfo());

  // Used in legalizer (etc.) to refer to the stack pointer.
  setStackPointerRegisterToSaveRestore(MOS::RS0);

  setMaximumJumpTableSize(std::min(256u, getMaximumJumpTableSize()));
}

MVT MOSTargetLowering::getRegisterType(MVT VT) const {
  // Even though a 16-bit register is available, it's not actually an integer
  // register, so split to 8 bits instead.
  if (VT.getSizeInBits() > 8)
    return MVT::i8;
  return TargetLowering::getRegisterType(VT);
}

unsigned
MOSTargetLowering::getNumRegisters(LLVMContext &Context, EVT VT,
                                   std::optional<MVT> RegisterVT) const {
  // Even though a 16-bit register is available, it's not actually an integer
  // register, so split to 8 bits instead.
  if (VT.getSizeInBits() > 8)
    return VT.getSizeInBits() / 8;
  return TargetLowering::getNumRegisters(Context, VT, RegisterVT);
}

MVT MOSTargetLowering::getRegisterTypeForCallingConv(
    LLVMContext &Context, CallingConv::ID CC, EVT VT,
    const ISD::ArgFlagsTy &Flags) const {
  if (Flags.isPointer())
    return Flags.getPointerAddrSpace() == MOS::AS_ZeroPage ? MVT::i8 : MVT::i16;
  return TargetLowering::getRegisterTypeForCallingConv(Context, CC, VT, Flags);
}

unsigned MOSTargetLowering::getNumRegistersForCallingConv(
    LLVMContext &Context, CallingConv::ID CC, EVT VT,
    const ISD::ArgFlagsTy &Flags) const {
  if (Flags.isPointer())
    return 1;
  return TargetLowering::getNumRegistersForCallingConv(Context, CC, VT, Flags);
}

unsigned MOSTargetLowering::getNumRegistersForInlineAsm(LLVMContext &Context,
                                                        EVT VT) const {
  // 16-bit inputs and outputs must be passed in Imag16 registers to allow using
  // pointer values in inline assembly.
  if (VT == MVT::i16)
    return 1;
  return TargetLowering::getNumRegistersForInlineAsm(Context, VT);
}

TargetLowering::ConstraintType
MOSTargetLowering::getConstraintType(StringRef Constraint) const {
  if (Constraint.size() == 1) {
    switch (Constraint[0]) {
    default:
      break;
    case 'a':
    case 'x':
    case 'y':
    case 'd':
    case 'c':
    case 'v':
      return C_Register;
    case 'R':
      return C_RegisterClass;
    }
  }
  return TargetLowering::getConstraintType(Constraint);
}

std::pair<unsigned, const TargetRegisterClass *>
MOSTargetLowering::getRegForInlineAsmConstraint(const TargetRegisterInfo *TRI,
                                                StringRef Constraint,
                                                MVT VT) const {
  if (Constraint.size() == 1) {
    switch (Constraint[0]) {
    default:
      break;
    case 'r':
      if (VT == MVT::i16)
        return std::make_pair(0U, &MOS::Imag16RegClass);
      return std::make_pair(0U, &MOS::Imag8RegClass);
    case 'R':
      return std::make_pair(0U, &MOS::GPRRegClass);
    case 'a':
      return std::make_pair(MOS::A, &MOS::GPRRegClass);
    case 'x':
      return std::make_pair(MOS::X, &MOS::GPRRegClass);
    case 'y':
      return std::make_pair(MOS::Y, &MOS::GPRRegClass);
    case 'd':
      return std::make_pair(0U, &MOS::XYRegClass);
    case 'c':
      return std::make_pair(MOS::C, &MOS::FlagRegClass);
    case 'v':
      return std::make_pair(MOS::V, &MOS::FlagRegClass);
    }
  }
  if (Constraint == "{cc}")
    return std::make_pair(MOS::P, &MOS::PcRegClass);

  return TargetLowering::getRegForInlineAsmConstraint(TRI, Constraint, VT);
}

bool is8BitIndex(Type *Ty) {
  if (!Ty)
    return false;
  return Ty == Type::getInt8Ty(Ty->getContext());
}

bool MOSTargetLowering::isLegalAddressingMode(const DataLayout &DL,
                                              const AddrMode &AM, Type *Ty,
                                              unsigned AddrSpace,
                                              Instruction *I) const {
  if (AM.Scale > 1 || AM.Scale < 0)
    return false;

  // Any Base + Index mode can be legally selected with zero page indexed
  // addressing.
  if (AddrSpace == MOS::AS_ZeroPage)
    return true;

  if (AM.Scale) {
    assert(AM.Scale == 1);
    if (!AM.HasBaseReg) {
      // Indexed addressing mode.
      if (is8BitIndex(AM.ScaleType))
        return true;

      // Consider a reg + 8-bit offset selectable via the indirect indexed
      // addressing mode.
      return !AM.BaseGV && 0 <= AM.BaseOffs && AM.BaseOffs < 256;
    }

    // Indirect indexed addressing mode: 16-bit register + 8-bit index register.
    // Doesn't matter which is 8-bit and which is 16-bit.
    return !AM.BaseGV && !AM.BaseOffs &&
           (is8BitIndex(AM.BaseType) || is8BitIndex(AM.ScaleType));
  }

  if (AM.HasBaseReg) {
    // Indexed addressing mode.
    if (is8BitIndex(AM.BaseType))
      return true;

    // Consider an reg + 8-bit offset selectable via the indirect indexed
    // addressing mode.
    return !AM.BaseGV && 0 <= AM.BaseOffs && AM.BaseOffs < 256;
  }

  // Any other combination of GV and BaseOffset are just global offsets.
  return true;
}

bool MOSTargetLowering::isTruncateFree(Type *SrcTy, Type *DstTy) const {
  if (!SrcTy->isIntegerTy() || !DstTy->isIntegerTy())
    return false;
  return SrcTy->getPrimitiveSizeInBits() > DstTy->getPrimitiveSizeInBits();
}

bool MOSTargetLowering::isZExtFree(Type *SrcTy, Type *DstTy) const {
  if (!SrcTy->isIntegerTy() || !DstTy->isIntegerTy())
    return false;
  return SrcTy->getPrimitiveSizeInBits() < DstTy->getPrimitiveSizeInBits();
}

static MachineBasicBlock *emitSelectImm(MachineInstr &MI,
                                        MachineBasicBlock *MBB);
static MachineBasicBlock *emitIncDecMB(MachineInstr &MI,
                                       MachineBasicBlock *MBB);
static MachineBasicBlock *emitCmpBrZeroMultiByte(MachineInstr &MI,
                                                 MachineBasicBlock *MBB);

MachineBasicBlock *
MOSTargetLowering::EmitInstrWithCustomInserter(MachineInstr &MI,
                                               MachineBasicBlock *MBB) const {
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Bad opcode.");
  case MOS::SelectImm:
    return emitSelectImm(MI, MBB);
  case MOS::IncMB:
  case MOS::DecMB:
  case MOS::DecDcpMB:
    return emitIncDecMB(MI, MBB);
  case MOS::CmpBrZeroMultiByte:
    return emitCmpBrZeroMultiByte(MI, MBB);
  }
}

static MachineBasicBlock *emitSelectImm(MachineInstr &MI,
                                        MachineBasicBlock *MBB) {
  // To "insert" Select* instructions, we actually have to insert the triangle
  // control-flow pattern.  The incoming instructions know the destination reg
  // to set, the flag to branch on, and the true/false values to select between.
  //
  // We produce the following control flow if the flag is neither N nor Z:
  //     HeadMBB
  //     |  \
  //     |  IfFalseMBB
  //     | /
  //    TailMBB
  //
  // If the flag is N or Z, then loading the true value in HeadMBB would clobber
  // the flag before the branch. We instead emit the following:
  //     HeadMBB
  //     |  \
  //     |  IfTrueMBB
  //     |      |
  //    IfFalse |
  //     |     /
  //     |    /
  //     TailMBB
  Register Dst = MI.getOperand(0).getReg();
  Register Flag = MI.getOperand(1).getReg();
  int64_t TrueValue = MI.getOperand(2).getImm();
  int64_t FalseValue = MI.getOperand(3).getImm();

  const BasicBlock *LLVM_BB = MBB->getBasicBlock();
  MachineFunction::iterator I = ++MBB->getIterator();
  MachineIRBuilder Builder(MI);

  MachineBasicBlock *HeadMBB = MBB;
  MachineFunction *F = MBB->getParent();

  const MOSSubtarget &STI = F->getSubtarget<MOSSubtarget>();

  // Split out all instructions after MI into a new basic block, updating
  // liveins.
  MachineBasicBlock *TailMBB = HeadMBB->splitAt(MI);

  // If MI is the last instruction, splitAt won't insert a new block. In that
  // case, the block must fall through, since there's no branch. Thus the tail
  // MBB is just the next MBB.
  if (TailMBB == HeadMBB)
    TailMBB = &*I;

  HeadMBB->removeSuccessor(TailMBB);

  // Add the false block between HeadMBB and TailMBB
  MachineBasicBlock *IfFalseMBB = F->CreateMachineBasicBlock(LLVM_BB);
  F->insert(TailMBB->getIterator(), IfFalseMBB);
  HeadMBB->addSuccessor(IfFalseMBB);
  for (const auto &LiveIn : TailMBB->liveins())
    if (LiveIn.PhysReg != Dst)
      IfFalseMBB->addLiveIn(LiveIn);
  IfFalseMBB->addSuccessor(TailMBB);

  // Add a true block if necessary to avoid clobbering NZ.
  MachineBasicBlock *IfTrueMBB = nullptr;
  if (Flag == MOS::N || Flag == MOS::Z) {
    IfTrueMBB = F->CreateMachineBasicBlock(LLVM_BB);
    F->insert(TailMBB->getIterator(), IfTrueMBB);
    IfTrueMBB->addSuccessor(TailMBB);

    // Add the unconditional branch from IfFalseMBB to TailMBB.
    Builder.setInsertPt(*IfFalseMBB, IfFalseMBB->begin());
    Builder.buildInstr(STI.hasBRA() ? MOS::BRA : MOS::JMP).addMBB(TailMBB);
    for (const auto &LiveIn : IfFalseMBB->liveins())
      IfTrueMBB->addLiveIn(LiveIn);

    Builder.setInsertPt(*HeadMBB, MI.getIterator());
  }

  const auto LDImm = [&Builder, &Dst](int64_t Val) {
    if (MOS::CV_GPR_LSBRegClass.contains(Dst)) {
      Builder.buildInstr(MOS::LDImm1, {Dst}, {Val});
      return;
    }

    assert(MOS::GPRRegClass.contains(Dst));
    Builder.buildInstr(MOS::LDImm, {Dst}, {Val});
  };

  if (IfTrueMBB) {
    // Insert branch.
    Builder.buildInstr(MOS::BR).addMBB(IfTrueMBB).addUse(Flag).addImm(1);
    HeadMBB->addSuccessor(IfTrueMBB);

    Builder.setInsertPt(*IfTrueMBB, IfTrueMBB->begin());
    // Load true value.
    LDImm(TrueValue);
  } else {
    // Load true value.
    LDImm(TrueValue);

    // Insert branch.
    Builder.buildInstr(MOS::BR).addMBB(TailMBB).addUse(Flag).addImm(1);
    HeadMBB->addSuccessor(TailMBB);
  }

  // Insert false load.
  Builder.setInsertPt(*IfFalseMBB, IfFalseMBB->begin());
  LDImm(FalseValue);

  MI.eraseFromParent();

  return TailMBB;
}

// Returns an IncMB that is safe to fold into the given CmpBrZeroMultiByte.
static MachineInstr *findCmpBrZeroMultiByteInc(MachineInstr &MI) {
  const TargetRegisterInfo *TRI = MI.getMF()->getSubtarget().getRegisterInfo();
  for (auto I = MachineBasicBlock::reverse_iterator(MI.getIterator()),
            E = MI.getParent()->rend();
       I != E; ++I) {
    if (I->hasUnmodeledSideEffects() || I->isCall())
      return nullptr;
    bool ReferencesCmpReg = false;
    for (const MachineOperand &MO : MI.explicit_uses()) {
      if (!MO.isReg())
        continue;
      if (I->readsRegister(MO.getReg(), TRI) ||
          I->definesRegister(MO.getReg(), TRI)) {
        ReferencesCmpReg = true;
        break;
      }
    }
    if (!ReferencesCmpReg)
      continue;
    if (I->getOpcode() != MOS::IncMB)
      return nullptr;
    for (unsigned MOI = MI.getNumExplicitDefs() + 2,
                  MOE = MI.getNumExplicitOperands();
         MOI != MOE; ++MOI) {
      if (I->getOperand(MOI - 2).getReg() != MI.getOperand(MOI).getReg())
        return nullptr;
    }
    return &*I;
  }
  return nullptr;
}

static MachineBasicBlock *emitIncDecMB(MachineInstr &MI,
                                       MachineBasicBlock *MBB) {
  if (!MBB->getParent()->getProperties().hasProperty(
          MachineFunctionProperties::Property::NoVRegs))
    return MBB;

  const MOSSubtarget &STI = MBB->getParent()->getSubtarget<MOSSubtarget>();

  // If this instruction will be folded into a later CmpBrZeroMultiByte, then
  // defer expanding it.
  if (MI.getOpcode() == MOS::IncMB && MI.getNumExplicitDefs() > 1) {
    auto Term = MBB->getFirstTerminator();
    if (Term != MBB->end() && Term->getOpcode() == MOS::CmpBrZeroMultiByte &&
        findCmpBrZeroMultiByteInc(*Term) == &MI) {
      if (std::prev(Term) != MI) {
        // The expansion of an intervening multi-byte instruction could separate
        // the IncMB from its CmpBrZeroMultiByte, so move it right before the
        // CmpBrZeroMultiByte. This is guaranteed safe by
        // findCmpBrZeroMultiByteInc.
        MBB->insert(Term, MI.removeFromParent());
      }
      return MBB;
    }
  }

  // Emitting INC/DEC sequences of N bytes is done in one of the following
  // three ways (? denotes a register):
  // 1. INC:            INC value / BNE increment_done
  // 2. DEC (register): DE? / CP? #$FF / BNE decrement_done
  // 3. DEC (memory):   LD? #$FF / DEC value / CP? value / BNE decrement_done
  // In addition:
  // - The comparison and branch are omitted for the final INC/DEC.
  // - For DEC value / CP? value, the unofficial 6502X opcode "DCP" is used
  //   instead, if enabled. This works with a scratch register A only, however.
  // - "3. DEC (memory)" is also used for imaginary registers, which are
  //   modeled as registers, but exist in memory.
  MachineIRBuilder Builder(MI);
  bool IsDec = MI.getOpcode() == MOS::DecMB || MI.getOpcode() == MOS::DecDcpMB;
  assert(IsDec || MI.getOpcode() == MOS::IncMB);
  unsigned FirstUseIdx = MI.getNumExplicitDefs();
  unsigned FirstDefIdx = IsDec ? 1 : 0;
  bool IsReg = MI.getOperand(FirstUseIdx).isReg();
  bool IsMemReg =
      IsReg && MOS::Imag8RegClass.contains(MI.getOperand(FirstUseIdx).getReg());
  bool IsLast = FirstUseIdx >= MI.getNumExplicitOperands() - 1;
  bool UseDcpOpcode = (!IsReg || IsMemReg) && !IsLast && STI.has6502X() &&
                      MI.getOpcode() == MOS::DecDcpMB;

  if (IsDec && !IsLast) {
    if (!IsReg || IsMemReg) {
      // 3. DEC (memory): LD? #$FF
      Builder.buildInstr(MOS::LDImm)
          .addDef(MI.getOperand(0).getReg())
          .addImm(INT64_C(0xFF));
    }
  }
  MachineInstrBuilder First;
  if (UseDcpOpcode) {
    // 3. DEC (memory): Emit DCP opcode, if requested.
    if (IsMemReg) {
      First = Builder.buildInstr(MOS::DCPImag8)
                  .addDef(MOS::C)
                  .addUse(MI.getOperand(0).getReg())
                  .addUse(MI.getOperand(FirstUseIdx).getReg());
      ++FirstDefIdx;
    } else {
      First = Builder.buildInstr(MOS::DCPAbs)
                  .addDef(MOS::C)
                  .addUse(MI.getOperand(0).getReg())
                  .add(MI.getOperand(FirstUseIdx));
    }
  } else {
    // 1/2/3. Emit INC/DEC.
    if (IsReg) {
      First = Builder.buildInstr(IsDec ? getDecPseudoOpcode(Builder)
                                       : getIncPseudoOpcode(Builder));
      if (IsDec && !IsLast) {
        // Avoid copying additional register flags here.
        // They will apply to the last opcode in the chain (CMP) instead.
        First.addDef(MI.getOperand(FirstDefIdx).getReg())
             .addUse(MI.getOperand(FirstUseIdx).getReg());
      } else {
        First.add(MI.getOperand(FirstDefIdx))
             .add(MI.getOperand(FirstUseIdx));
      }
      ++FirstDefIdx;
    } else {
      First = Builder.buildInstr(IsDec ? MOS::DECAbs : MOS::INCAbs)
                  .add(MI.getOperand(FirstUseIdx));
    }
  }
  if (IsLast) {
    MI.eraseFromParent();
    return MBB;
  }
  if (IsDec && !UseDcpOpcode) {
    // 2/3. DEC: Emit CMP.
    if (IsReg && !IsMemReg) {
      Builder.buildInstr(MOS::CMPImm)
          .addDef(MOS::C)
          .addUse(MI.getOperand(FirstUseIdx).getReg())
          .addImm(INT64_C(0xFF))
          .addDef(MOS::Z, RegState::Implicit);
    } else {
      Builder.buildInstr(IsMemReg ? MOS::CMPImag8 : MOS::CMPAbs)
          .addDef(MOS::C)
          .addUse(MI.getOperand(0).getReg())
          .add(MI.getOperand(FirstUseIdx))
          .addDef(MOS::Z, RegState::Implicit);
    }
  } else {
    // 1. INC: INC sets the Z flag; carry happens when value == 0.
    First.addDef(MOS::Z, RegState::Implicit);
  }

  MachineBasicBlock *TailMBB = MBB->splitAt(MI);
  // If MI is the last instruction, splitAt won't insert a new block. In that
  // case, the block must fall through, since there's no branch. Thus the tail
  // MBB is just the next MBB.
  if (TailMBB == MBB)
    TailMBB = &*std::next(MBB->getIterator());

  MachineFunction *F = MBB->getParent();
  MachineBasicBlock *RestMBB = F->CreateMachineBasicBlock(MBB->getBasicBlock());
  F->insert(TailMBB->getIterator(), RestMBB);
  for (const auto &LiveIn : TailMBB->liveins())
    RestMBB->addLiveIn(LiveIn);

  Builder.buildInstr(MOS::BR).addMBB(RestMBB).addUse(MOS::Z).addImm(
      INT64_C(-1));
  Builder.buildInstr(MOS::JMP).addMBB(TailMBB);
  MBB->addSuccessor(RestMBB);

  Builder.setInsertPt(*RestMBB, RestMBB->end());
  auto Rest = Builder.buildInstr(MI.getOpcode());
  if (IsDec)
    Rest.addDef(MI.getOperand(0).getReg());
  for (unsigned I = FirstDefIdx, E = MI.getNumExplicitOperands(); I != E; ++I) {
    if (I == FirstUseIdx)
      continue;
    Rest.add(MI.getOperand(I));
    if (MI.getOperand(I).isReg())
      RestMBB->addLiveIn(MI.getOperand(I).getReg());
  }
  Builder.buildInstr(MOS::JMP).addMBB(TailMBB);
  RestMBB->addSuccessor(TailMBB);
  RestMBB->sortUniqueLiveIns();

  MI.eraseFromParent();

  return RestMBB;
}

static MachineBasicBlock *emitCmpBrZeroMultiByte(MachineInstr &MI,
                                                 MachineBasicBlock *MBB) {
  if (!MBB->getParent()->getProperties().hasProperty(
          MachineFunctionProperties::Property::NoVRegs))
    return MBB;
  const MOSSubtarget &STI = MBB->getParent()->getSubtarget<MOSSubtarget>();
  const TargetInstrInfo &TII = *STI.getInstrInfo();

  MachineFunction &MF = *MBB->getParent();

  MachineBasicBlock *Target = MI.getOperand(0).getMBB();
  bool Val = MI.getOperand(1).getImm();

  MachineIRBuilder Builder(MI);

  if (MI.getNumExplicitOperands() == 3) {
    Builder.buildInstr(MOS::CmpBrZero)
        .addMBB(Target)
        .addUse(MOS::Z, RegState::Undef)
        .addImm(Val)
        .add(/*LowReg*/ MI.getOperand(2));
    MI.eraseFromParent();
    return MBB;
  }

  MachineBasicBlock *TBB;
  MachineBasicBlock *FBB;
  SmallVector<MachineOperand> Cond;
  bool CannotAnalyze = TII.analyzeBranch(*MBB, TBB, FBB, Cond);
  assert(!CannotAnalyze &&
         "all CmpBr branches structures should be analyzable");
  assert(!Cond.empty() && "expected conditional branch");
  assert(Cond.front().getImm() == MOS::CmpBrZeroMultiByte &&
         "expected CmpBrZeroMultiByte");

  // Normalize TBB and FBB to always be initialized.
  if (!TBB || !FBB) {
    auto FallthroughIter = std::next(MBB->getIterator());
    assert(
        FallthroughIter != MF.end() &&
        "unexpected fallthrough past CmpBrZeroMultiByte off end of function");
    MachineBasicBlock *Fallthrough = &*FallthroughIter;
    if (!TBB)
      TBB = Fallthrough;
    if (!FBB)
      FBB = Fallthrough;
  }

  MachineInstr *Inc = findCmpBrZeroMultiByteInc(MI);

  // Determine the byte to check for non-zero-ness first.
  unsigned CondIdx;
  if (Inc) {
    Builder.buildInstr(getIncPseudoOpcode(Builder))
        .add(Inc->getOperand(0))
        .add(Inc->getOperand(Inc->getNumExplicitDefs()));
    CondIdx = 2; // Opcode, Val, [LSB]
  } else {
    CondIdx = Cond.size() - 1; // MSB
  }
  MachineOperand CondMO = std::move(Cond[CondIdx]);
  Cond.erase(Cond.begin() + CondIdx);

  // If the byte comparison is zero, then maybe the whole comparison is. This
  // block makes this determination.
  MachineBasicBlock *MaybeZero =
      MF.CreateMachineBasicBlock(MBB->getBasicBlock());
  MF.insert(std::next(MBB->getIterator()), MaybeZero);

  TII.insertBranch(*MaybeZero, TBB, FBB, Cond, MBB->findDebugLoc(MBB->end()));
  for (auto I = MBB->succ_begin(), E = MBB->succ_end(); I != E; ++I)
    MaybeZero->copySuccessor(MBB, I);

  // Set up the byte non-zero comparison.
  SmallVector<MachineOperand> NonZeroCond;
  NonZeroCond.push_back(MachineOperand::CreateImm(MOS::CmpBrZero));
  NonZeroCond.push_back(MachineOperand::CreateReg(
      MOS::Z, /*isDef=*/false, /*isImp=*/false, /*isKill=*/false,
      /*isDead=*/false, /*isUndef=*/true));
  NonZeroCond.push_back(MachineOperand::CreateImm(0));
  NonZeroCond.push_back(std::move(CondMO));

  // Determine where to branch to if the byte is non-zero. At that point, we
  // know the whole value is non-zero, so we either use the original FBB or the
  // original target.
  MachineBasicBlock *NonZeroTBB = Val ? FBB : Target;

  // If the value is zero, fall through to MaybeZero.
  MachineBasicBlock *NonZeroFBB = MaybeZero;

  TII.removeBranch(*MBB);

  MachineBasicBlock *ZeroSucc = Val ? TBB : FBB;
  MachineBasicBlock *NonZeroSucc = Val ? FBB : TBB;
  MBB->replaceSuccessor(NonZeroSucc, NonZeroTBB);
  MBB->replaceSuccessor(ZeroSucc, NonZeroFBB);

  TII.insertBranch(*MBB, NonZeroTBB, NonZeroFBB, NonZeroCond,
                   MBB->findDebugLoc(MBB->end()));

  if (Inc) {
    Builder.setInsertPt(*MaybeZero, MaybeZero->begin());

    unsigned NumBytes = Inc->getNumExplicitDefs();

    // Illegal to remove tied operands, so recreate the increment one byte
    // smaller.
    auto NewInc = Builder.buildInstr(MOS::IncMB);
    for (unsigned I = 1, E = Inc->getNumOperands(); I != E; ++I)
      if (I != NumBytes)
        NewInc.add(Inc->getOperand(I));

    for (unsigned I = 0, E = NumBytes - 1; I != E; ++I)
      NewInc->tieOperands(I, I + NumBytes - 1);

    Inc->eraseFromParent();
  }

  // Update live regs.
  recomputeLiveIns(*MaybeZero);
  recomputeLiveIns(*MBB);

  return MaybeZero;
}
