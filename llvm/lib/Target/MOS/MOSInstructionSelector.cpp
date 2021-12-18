//===-- MOSInstructionSelector.cpp - MOS Instruction Selector -------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MOS instruction selector.
//
//===----------------------------------------------------------------------===//

#include "MOSInstructionSelector.h"

#include <set>

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOSRegisterInfo.h"
#include "MOSSubtarget.h"

#include "llvm/ADT/APFloat.h"
#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/CodeGen/GlobalISel/InstructionSelectorImpl.h"
#include "llvm/CodeGen/GlobalISel/MIPatternMatch.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/GlobalISel/RegisterBankInfo.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/InstrTypes.h"
#include "llvm/ObjectYAML/MachOYAML.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;
using namespace MIPatternMatch;

#define DEBUG_TYPE "mos-isel"

namespace llvm {
namespace MOS {
extern RegisterBank AnyRegBank;
} // namespace MOS
} // namespace llvm

namespace {

#define GET_GLOBALISEL_PREDICATE_BITSET
#include "MOSGenGlobalISel.inc"
#undef GET_GLOBALISEL_PREDICATE_BITSET

class MOSInstructionSelector : public InstructionSelector {
public:
  MOSInstructionSelector(const MOSTargetMachine &TM, MOSSubtarget &STI,
                         MOSRegisterBankInfo &RBI);

  bool select(MachineInstr &MI) override;
  static const char *getName() { return DEBUG_TYPE; }

private:
  const MOSSubtarget &STI;
  const MOSInstrInfo &TII;
  const MOSRegisterInfo &TRI;
  const MOSRegisterBankInfo &RBI;

  // Pre-tablegen selection functions. If these return false, fall through to
  // tablegen.
  bool selectAdd(MachineInstr &MI);

  // Post-tablegen selection functions. If these return false, it is an error.
  bool selectBrCondImm(MachineInstr &MI);
  bool selectSbc(MachineInstr &MI);
  bool selectFrameIndex(MachineInstr &MI);
  bool selectAddr(MachineInstr &MI);
  bool selectStore(MachineInstr &MI);
  bool selectLshrShlE(MachineInstr &MI);
  bool selectMergeValues(MachineInstr &MI);
  bool selectTrunc(MachineInstr &MI);
  bool selectAddE(MachineInstr &MI);
  bool selectUnMergeValues(MachineInstr &MI);

  // Select instructions that correspond 1:1 to a target instruction.
  bool selectGeneric(MachineInstr &MI);

  void composePtr(MachineIRBuilder &Builder, Register Dst, Register Lo,
                  Register Hi);

  void constrainGenericOp(MachineInstr &MI);

  void constrainOperandRegClass(MachineOperand &RegMO,
                                const TargetRegisterClass &RegClass);

  // Select all instructions in a given span, recursively. Allows selecting an
  // instruction sequence by reducing it to a more easily selectable sequence.
  bool selectAll(MachineInstrSpan MIS);

  /// tblgen-erated 'select' implementation, used as the initial selector for
  /// the patterns that don't require complex C++.
  bool selectImpl(MachineInstr &MI, CodeGenCoverage &CoverageInfo) const;

#define GET_GLOBALISEL_PREDICATES_DECL
#include "MOSGenGlobalISel.inc"
#undef GET_GLOBALISEL_PREDICATES_DECL

#define GET_GLOBALISEL_TEMPORARIES_DECL
#include "MOSGenGlobalISel.inc"
#undef GET_GLOBALISEL_TEMPORARIES_DECL
};

} // namespace

#define GET_GLOBALISEL_IMPL
#include "MOSGenGlobalISel.inc"
#undef GET_GLOBALISEL_IMPL

MOSInstructionSelector::MOSInstructionSelector(const MOSTargetMachine &TM,
                                               MOSSubtarget &STI,
                                               MOSRegisterBankInfo &RBI)
    : STI(STI), TII(*STI.getInstrInfo()), TRI(*STI.getRegisterInfo()), RBI(RBI),
#define GET_GLOBALISEL_PREDICATES_INIT
#include "MOSGenGlobalISel.inc"
#undef GET_GLOBALISEL_PREDICATES_INIT
#define GET_GLOBALISEL_TEMPORARIES_INIT
#include "MOSGenGlobalISel.inc"
#undef GET_GLOBALISEL_TEMPORARIES_INIT
{
}

// Returns the widest register class that can contain values of a given type.
// Used to ensure that every virtual register gets some register class by the
// time register allocation completes.
static const TargetRegisterClass &getRegClassForType(LLT Ty) {
  switch (Ty.getSizeInBits()) {
  default:
    llvm_unreachable("Invalid type size.");
  case 1:
    return MOS::Anyi1RegClass;
  case 8:
    return MOS::Anyi8RegClass;
  case 16:
    return MOS::Imag16RegClass;
  }
}

bool MOSInstructionSelector::select(MachineInstr &MI) {
  if (!MI.isPreISelOpcode()) {
    // Ensure that target-independent pseudos like COPY have register classes.
    constrainGenericOp(MI);
    return true;
  }

  switch (MI.getOpcode()) {
  case MOS::G_ADD:
    if (selectAdd(MI))
      return true;
  }

  if (selectImpl(MI, *CoverageInfo))
    return true;

  switch (MI.getOpcode()) {
  default:
    return false;
  case MOS::G_BRCOND_IMM:
    return selectBrCondImm(MI);
  case MOS::G_SBC:
    return selectSbc(MI);
  case MOS::G_FRAME_INDEX:
    return selectFrameIndex(MI);
  case MOS::G_BLOCK_ADDR:
  case MOS::G_GLOBAL_VALUE:
    return selectAddr(MI);
  case MOS::G_STORE_ABS:
  case MOS::G_STORE_ABS_IDX:
    return selectStore(MI);
  case MOS::G_LSHRE:
  case MOS::G_SHLE:
    return selectLshrShlE(MI);
  case MOS::G_MERGE_VALUES:
    return selectMergeValues(MI);
  case MOS::G_TRUNC:
    return selectTrunc(MI);
  case MOS::G_UADDE:
  case MOS::G_SADDE:
    return selectAddE(MI);
  case MOS::G_UNMERGE_VALUES:
    return selectUnMergeValues(MI);

  case MOS::G_BRINDIRECT:
  case MOS::G_IMPLICIT_DEF:
  case MOS::G_INTTOPTR:
  case MOS::G_LOAD_ABS:
  case MOS::G_LOAD_ABS_IDX:
  case MOS::G_LOAD_INDIR_IDX:
  case MOS::G_PHI:
  case MOS::G_PTRTOINT:
  case MOS::G_STORE_INDIR_IDX:
    return selectGeneric(MI);
  }
}

static bool shouldFoldMemAccess(const MachineInstr &Dst,
                                const MachineInstr &Src, AAResults *AA) {
  assert(Src.mayLoadOrStore());
  if (Dst.getParent() != Src.getParent())
    return false;
  for (MachineBasicBlock::const_iterator
           I = std::next(MachineBasicBlock::const_iterator(Src)),
           E = Dst;
       I != E; ++I) {
    if (I->isCall() || I->hasUnmodeledSideEffects())
      return false;
    if (I->mayLoadOrStore()) {
      if (Src.hasOrderedMemoryRef() || I->hasOrderedMemoryRef())
        return false;
      if (I->mayAlias(AA, Src, /*UseTBAA=*/true))
        return false;
    }
  }
  const auto &MRI = Dst.getMF()->getRegInfo();
  for (MachineInstr &UseMI :
       MRI.use_nodbg_instructions(Src.getOperand(0).getReg())) {
    switch (UseMI.getOpcode()) {
    default:
      return false;
    case MOS::G_ADD:
    case MOS::G_UADDE:
    case MOS::G_SADDE:
      break;
    }
  }
  return true;
}

struct FoldedLdAbs_match {
  const MachineInstr &Tgt;
  MachineOperand &Addr;
  AAResults *AA;

  FoldedLdAbs_match(const MachineInstr &Tgt, MachineOperand &Addr,
                    AAResults *AA)
      : Tgt(Tgt), Addr(Addr), AA(AA) {}

  bool match(const MachineRegisterInfo &MRI, Register Reg) {
    const MachineInstr *LdAbs = getOpcodeDef(MOS::G_LOAD_ABS, Reg, MRI);
    if (!LdAbs || !shouldFoldMemAccess(Tgt, *LdAbs, AA))
      return false;
    Addr = LdAbs->getOperand(1);
    return true;
  }
};

inline FoldedLdAbs_match m_FoldedLdAbs(const MachineInstr &Tgt,
                                       MachineOperand &Addr, AAResults *AA) {
  return {Tgt, Addr, AA};
}

bool MOSInstructionSelector::selectAdd(MachineInstr &MI) {
  assert(MI.getOpcode() == MOS::G_ADD);

  MachineIRBuilder Builder(MI);
  auto &MRI = *Builder.getMRI();

  LLT S1 = LLT::scalar(1);

  if (auto RHSConst =
          getIConstantVRegValWithLookThrough(MI.getOperand(2).getReg(), MRI)) {
    // Don't inhibit generation of INC/DEC.
    if (RHSConst->Value.abs().isOne())
      return false;
  }

  Register LHS;
  MachineOperand Addr = MachineOperand::CreateReg(0, false);
  if (!mi_match(MI.getOperand(0).getReg(), MRI,
                m_GAdd(m_Reg(LHS), m_FoldedLdAbs(MI, Addr, AA))))
    return false;
  Register CIn = Builder.buildInstr(MOS::LDCImm, {S1}, {INT64_C(0)}).getReg(0);
  auto Adc = Builder.buildInstr(MOS::ADCAbs)
                 .addDef(MI.getOperand(0).getReg())
                 .addDef(MRI.createGenericVirtualRegister(S1))
                 .addDef(MRI.createGenericVirtualRegister(S1))
                 .addUse(LHS)
                 .add(Addr)
                 .addUse(CIn);
  if (!constrainSelectedInstRegOperands(*Adc, TII, TRI, RBI))
    llvm_unreachable("Could not constrain ADCAbs");
  MI.eraseFromParent();
  return true;
}

// Given a G_SBC instruction Sbc and one of its flag output virtual registers,
// returns the flag that corresponds to the register.
static Register getSbcFlagForRegister(const MachineInstr &Sbc, Register Reg) {
  static Register Flags[] = {MOS::C, MOS::N, MOS::V, MOS::Z};
  for (int Idx = 0; Idx < 4; ++Idx)
    if (Sbc.getOperand(Idx + 1).getReg() == Reg)
      return Flags[Idx];
  llvm_unreachable("Could not find register in G_SBC outputs.");
}

struct CmpZTerm_match {
  Register &LHS;
  Register &Flag;

  CmpZTerm_match(Register &LHS, Register &Flag) : LHS(LHS), Flag(Flag) {}

  bool match(const MachineRegisterInfo &MRI, Register CondReg) {
    auto DefSrcReg = getDefSrcRegIgnoringCopies(CondReg, MRI);
    MachineInstr &CondMI = *DefSrcReg->MI;
    if (CondMI.getOpcode() != MOS::G_SBC)
      return false;

    auto RHSConst =
        getIConstantVRegValWithLookThrough(CondMI.getOperand(6).getReg(), MRI);
    if (!RHSConst)
      return false;
    if (!RHSConst->Value.isZero())
      return false;

    auto CInConst =
        getIConstantVRegValWithLookThrough(CondMI.getOperand(7).getReg(), MRI);
    if (!CInConst || CInConst->Value.isNullValue())
      return false;

    LHS = CondMI.getOperand(5).getReg();
    Flag = getSbcFlagForRegister(CondMI, DefSrcReg->Reg);

    return Flag == MOS::N || Flag == MOS::Z;
  }
};

inline CmpZTerm_match m_CmpZTerm(Register &LHS, Register &Flag) {
  return {LHS, Flag};
}

struct CmpImmTerm_match {
  Register &LHS;
  int64_t &RHS;
  Register &Flag;

  CmpImmTerm_match(Register &LHS, int64_t &RHS, Register &Flag)
      : LHS(LHS), RHS(RHS), Flag(Flag) {}

  bool match(const MachineRegisterInfo &MRI, Register CondReg) {
    auto DefSrcReg = getDefSrcRegIgnoringCopies(CondReg, MRI);
    MachineInstr &CondMI = *DefSrcReg->MI;
    if (CondMI.getOpcode() != MOS::G_SBC)
      return false;

    auto RHSConst =
        getIConstantVRegValWithLookThrough(CondMI.getOperand(6).getReg(), MRI);
    if (!RHSConst)
      return false;

    auto CInConst =
        getIConstantVRegValWithLookThrough(CondMI.getOperand(7).getReg(), MRI);
    if (!CInConst || CInConst->Value.isNullValue())
      return false;

    LHS = CondMI.getOperand(5).getReg();
    RHS = RHSConst->Value.getZExtValue();
    Flag = getSbcFlagForRegister(CondMI, DefSrcReg->Reg);

    return Flag == MOS::N || Flag == MOS::Z;
  }
};

// Match one of the outputs of a G_SBC to a CMPImmTerm operation. LHS and RHS
// are the left and right hand side of the comparison, while Flag is the
// physical (N or Z) register corresponding to the output by which the G_SBC was
// reached.
inline CmpImmTerm_match m_CmpImmTerm(Register &LHS, int64_t &RHS,
                                     Register &Flag) {
  return {LHS, RHS, Flag};
}

struct CmpImag8Term_match {
  Register &LHS;
  Register &RHS;
  Register &Flag;

  CmpImag8Term_match(Register &LHS, Register &RHS, Register &Flag)
      : LHS(LHS), RHS(RHS), Flag(Flag) {}

  bool match(const MachineRegisterInfo &MRI, Register CondReg) {
    auto DefSrcReg = getDefSrcRegIgnoringCopies(CondReg, MRI);
    MachineInstr &CondMI = *DefSrcReg->MI;
    if (CondMI.getOpcode() != MOS::G_SBC)
      return false;

    auto CInConst =
        getIConstantVRegValWithLookThrough(CondMI.getOperand(7).getReg(), MRI);
    if (!CInConst || CInConst->Value.isNullValue())
      return false;

    LHS = CondMI.getOperand(5).getReg();
    RHS = CondMI.getOperand(6).getReg();
    Flag = getSbcFlagForRegister(CondMI, DefSrcReg->Reg);

    return Flag == MOS::N || Flag == MOS::Z;
  }
};

// Match one of the outputs of a G_SBC to a CMPImag8Term operation. LHS and RHS
// are the left and right hand side of the comparison, while Flag is the
// physical (N or Z) register corresponding to the output by which the G_SBC was
// reached.
inline CmpImag8Term_match m_CmpImag8Term(Register &LHS, Register &RHS,
                                         Register &Flag) {
  return {LHS, RHS, Flag};
}

bool MOSInstructionSelector::selectBrCondImm(MachineInstr &MI) {
  MachineRegisterInfo &MRI = MI.getMF()->getRegInfo();

  Register CondReg = MI.getOperand(0).getReg();
  MachineBasicBlock *Tgt = MI.getOperand(1).getMBB();
  int64_t FlagVal = MI.getOperand(2).getImm();

  LLT S1 = LLT::scalar(1);

  MachineInstr *Compare = nullptr;
  Register Flag;

  MachineIRBuilder Builder(MI);

  Register LHS;
  if (!Compare && mi_match(CondReg, MRI, m_CmpZTerm(LHS, Flag)))
    Compare = Builder.buildInstr(MOS::CMPZTerm, {S1}, {LHS});
  int64_t RHSConst;
  if (!Compare && mi_match(CondReg, MRI, m_CmpImmTerm(LHS, RHSConst, Flag)))
    Compare = Builder.buildInstr(MOS::CMPImmTerm, {S1}, {LHS, RHSConst});
  Register RHS;
  if (!Compare && mi_match(CondReg, MRI, m_CmpImag8Term(LHS, RHS, Flag)))
    Compare = Builder.buildInstr(MOS::CMPImag8Term, {S1}, {LHS, RHS});

  if (Compare) {
    if (!constrainSelectedInstRegOperands(*Compare, TII, TRI, RBI))
      return false;
    assert(Flag != MOS::C);
    Builder.buildInstr(MOS::BR).addMBB(Tgt).addUse(Flag).addImm(FlagVal);
    MI.eraseFromParent();
    return true;
  }

  auto GBR = Builder.buildInstr(MOS::GBR)
                 .addMBB(MI.getOperand(1).getMBB())
                 .addUse(MI.getOperand(0).getReg())
                 .addImm(MI.getOperand(2).getImm());
  if (!constrainSelectedInstRegOperands(*GBR, TII, TRI, RBI))
    return false;
  MI.eraseFromParent();
  return true;
}

bool MOSInstructionSelector::selectSbc(MachineInstr &MI) {
  LLT S1 = LLT::scalar(1);
  LLT S8 = LLT::scalar(8);

  MachineIRBuilder Builder(MI);

  Register A = MI.getOperand(0).getReg();
  Register N = MI.getOperand(2).getReg();
  Register V = MI.getOperand(3).getReg();
  Register Z = MI.getOperand(4).getReg();
  Register R = MI.getOperand(6).getReg();

  if (Builder.getMRI()->use_nodbg_empty(A))
    A = MOS::NoRegister;
  if (Builder.getMRI()->use_nodbg_empty(N))
    N = MOS::NoRegister;
  if (Builder.getMRI()->use_nodbg_empty(V))
    V = MOS::NoRegister;
  if (Builder.getMRI()->use_nodbg_empty(Z))
    Z = MOS::NoRegister;

  auto CInConst = getIConstantVRegValWithLookThrough(MI.getOperand(7).getReg(),
                                                     *Builder.getMRI());
  bool CInSet = CInConst && !CInConst->Value.isNullValue();

  // We can only extract one of N or Z at a time, so if both are needed,
  // arbitrarily extract out the comparison that produces Z. This case
  // should very rarely be hit, if ever.
  if (N && Z) {
    MachineInstrSpan MIS(MI, MI.getParent());
    MI.getOperand(4).setReg(Builder.getMRI()->createGenericVirtualRegister(S1));
    Builder.setInsertPt(Builder.getMBB(), std::next(Builder.getInsertPt()));
    Builder.buildInstr(MOS::G_SBC, {S8, S1, S1, S1, Z},
                       {MI.getOperand(5), MI.getOperand(6), MI.getOperand(7)});
    return selectAll(MIS);
  }

  auto RConst = getIConstantVRegValWithLookThrough(R, *Builder.getMRI());
  MachineInstrBuilder Instr;
  if (!A && !V && CInSet) {
    if (RConst) {
      assert(RConst->Value.getBitWidth() == 8);
      if (N || Z) {
        Instr = Builder.buildInstr(
            MOS::CMPNZImm, {MI.getOperand(1), N, Z},
            {MI.getOperand(5), RConst->Value.getZExtValue()});
      } else {
        Instr = Builder.buildInstr(
            MOS::CMPImm, {MI.getOperand(1)},
            {MI.getOperand(5), RConst->Value.getZExtValue()});
      }
    } else {
      if (N || Z) {
        Instr = Builder.buildInstr(MOS::CMPNZImag8, {MI.getOperand(1), N, Z},
                                   {MI.getOperand(5), MI.getOperand(6)});
      } else {
        Instr = Builder.buildInstr(MOS::CMPImag8, {MI.getOperand(1)},
                                   {MI.getOperand(5), MI.getOperand(6)});
      }
    }
  } else {
    if (RConst) {
      assert(RConst->Value.getBitWidth() == 8);
      if (N || Z) {
        Instr = Builder.buildInstr(
            MOS::SBCNZImm,
            {MI.getOperand(0), MI.getOperand(1), N, MI.getOperand(3), Z},
            {MI.getOperand(5), RConst->Value.getZExtValue(), MI.getOperand(7)});
      } else {
        Instr = Builder.buildInstr(
            MOS::SBCImm, {MI.getOperand(0), MI.getOperand(1), MI.getOperand(3)},
            {MI.getOperand(5), RConst->Value.getZExtValue(), MI.getOperand(7)});
      }
    } else {
      if (N || Z) {
        Instr = Builder.buildInstr(
            MOS::SBCNZImag8,
            {MI.getOperand(0), MI.getOperand(1), N, MI.getOperand(3), Z},
            {MI.getOperand(5), MI.getOperand(6), MI.getOperand(7)});
      } else {
        Instr = Builder.buildInstr(
            MOS::SBCImag8,
            {MI.getOperand(0), MI.getOperand(1), MI.getOperand(3)},
            {MI.getOperand(5), MI.getOperand(6), MI.getOperand(7)});
      }
    }
  }
  if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
    return false;
  MI.eraseFromParent();
  return true;
}

bool MOSInstructionSelector::selectFrameIndex(MachineInstr &MI) {
  MachineIRBuilder Builder(MI);

  Register Dst = MI.getOperand(0).getReg();

  LLT S1 = LLT::scalar(1);
  LLT S8 = LLT::scalar(8);

  MachineInstrBuilder LoAddr;
  MachineInstrBuilder HiAddr;

  bool IsLocal = !MI.getMF()->getFrameInfo().isFixedObjectIndex(
      MI.getOperand(1).getIndex());
  if (MI.getMF()->getFunction().doesNotRecurse() && IsLocal)
    return selectAddr(MI);

  // Otherwise a soft stack needs to be used, so frame addresses are offsets
  // from the stack/frame pointer. Record this as a pseudo, since the best
  // code to emit depends heavily on the actual offset, which isn't known
  // until FEI.
  LoAddr = Builder.buildInstr(MOS::AddrLostk, {S8, S1, S1}, {})
               .add(MI.getOperand(1))
               .addImm(0);
  Register Carry = LoAddr.getReg(1);

  HiAddr = Builder.buildInstr(MOS::AddrHistk, {S8, S1, S1}, {})
               .add(MI.getOperand(1))
               .addImm(0)
               .addUse(Carry);

  if (!constrainSelectedInstRegOperands(*LoAddr, TII, TRI, RBI))
    return false;
  if (!constrainSelectedInstRegOperands(*HiAddr, TII, TRI, RBI))
    return false;
  composePtr(Builder, Dst, LoAddr.getReg(0), HiAddr.getReg(0));
  MI.eraseFromParent();
  return true;
}

bool MOSInstructionSelector::selectAddr(MachineInstr &MI) {
  MachineIRBuilder Builder(MI);
  LLT S8 = LLT::scalar(8);
  auto LoImm = Builder.buildInstr(MOS::LDImm, {S8}, {}).add(MI.getOperand(1));
  LoImm->getOperand(1).setTargetFlags(MOS::MO_LO);
  if (!constrainSelectedInstRegOperands(*LoImm, TII, TRI, RBI))
    return false;
  auto HiImm = Builder.buildInstr(MOS::LDImm, {S8}, {}).add(MI.getOperand(1));
  HiImm->getOperand(1).setTargetFlags(MOS::MO_HI);
  if (!constrainSelectedInstRegOperands(*HiImm, TII, TRI, RBI))
    return false;
  composePtr(Builder, MI.getOperand(0).getReg(), LoImm.getReg(0),
             HiImm.getReg(0));
  MI.eraseFromParent();
  return true;
}

bool MOSInstructionSelector::selectStore(MachineInstr &MI) {
  MachineIRBuilder Builder(MI);

  if (!STI.has65C02() ||
      !isOperandImmEqual(MI.getOperand(0), 0, *Builder.getMRI()))
    return selectGeneric(MI);

  unsigned Opcode;
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Unexpected opcode.");
  case MOS::G_STORE_ABS:
    Opcode = MOS::STZAbs;
    break;
  case MOS::G_STORE_ABS_IDX:
    Opcode = MOS::STZIdx;
    break;
  }

  MI.setDesc(TII.get(Opcode));
  MI.RemoveOperand(0);
  if (!constrainSelectedInstRegOperands(MI, TII, TRI, RBI))
    return false;
  return true;
}

bool MOSInstructionSelector::selectMergeValues(MachineInstr &MI) {
  Register Dst = MI.getOperand(0).getReg();
  Register Lo = MI.getOperand(1).getReg();
  Register Hi = MI.getOperand(2).getReg();

  MachineIRBuilder Builder(MI);
  composePtr(Builder, Dst, Lo, Hi);
  MI.eraseFromParent();
  return true;
}

bool MOSInstructionSelector::selectLshrShlE(MachineInstr &MI) {
  Register Dst = MI.getOperand(0).getReg();
  Register CarryOut = MI.getOperand(1).getReg();
  Register Src = MI.getOperand(2).getReg();
  Register CarryIn = MI.getOperand(3).getReg();

  unsigned ShiftOpcode, RotateOpcode;
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Unexpected opcode.");
  case MOS::G_SHLE:
    ShiftOpcode = MOS::ASL;
    RotateOpcode = MOS::ROL;
    break;
  case MOS::G_LSHRE:
    ShiftOpcode = MOS::LSR;
    RotateOpcode = MOS::ROR;
    break;
  }

  MachineIRBuilder Builder(MI);
  if (mi_match(CarryIn, *Builder.getMRI(), m_SpecificICst(0))) {
    auto Asl = Builder.buildInstr(ShiftOpcode, {Dst, CarryOut}, {Src});
    if (!constrainSelectedInstRegOperands(*Asl, TII, TRI, RBI))
      return false;
  } else {
    auto Rol =
        Builder.buildInstr(RotateOpcode, {Dst, CarryOut}, {Src, CarryIn});
    if (!constrainSelectedInstRegOperands(*Rol, TII, TRI, RBI))
      return false;
  }
  MI.eraseFromParent();
  return true;
}

bool MOSInstructionSelector::selectTrunc(MachineInstr &MI) {
  MachineIRBuilder Builder(MI);

  LLT S16 = LLT::scalar(16);
  LLT S8 = LLT::scalar(8);
  LLT S1 = LLT::scalar(1);

  Register From = MI.getOperand(1).getReg();
  Register To = MI.getOperand(0).getReg();

  LLT FromType = Builder.getMRI()->getType(From);
  LLT ToType = Builder.getMRI()->getType(To);
  assert(FromType == S16 && ToType == S1);

  MachineInstrSpan MIS(MI, MI.getParent());
  MI.getOperand(1).setReg(Builder.buildTrunc(S8, From).getReg(0));
  selectAll(MIS);
  return true;
}

bool MOSInstructionSelector::selectAddE(MachineInstr &MI) {
  Register Result = MI.getOperand(0).getReg();
  Register CarryOut = MI.getOperand(1).getReg();
  Register L = MI.getOperand(2).getReg();
  Register R = MI.getOperand(3).getReg();
  Register CarryIn = MI.getOperand(4).getReg();

  MachineIRBuilder Builder(MI);
  auto &MRI = *Builder.getMRI();

  LLT S1 = LLT::scalar(1);

  MachineInstrBuilder Instr = [&]() {
    if (auto RConst = getIConstantVRegValWithLookThrough(R, MRI)) {
      assert(RConst->Value.getBitWidth() == 8);
      return Builder.buildInstr(MOS::ADCImm, {Result, CarryOut, S1},
                                {L, RConst->Value.getZExtValue(), CarryIn});
    }
    MachineOperand Addr = MachineOperand::CreateReg(0, false);
    if (mi_match(L, MRI, m_FoldedLdAbs(MI, Addr, AA)))
      std::swap(L, R);
    if (mi_match(R, MRI, m_FoldedLdAbs(MI, Addr, AA))) {
      return Builder.buildInstr(MOS::ADCAbs)
          .addDef(Result)
          .addDef(CarryOut)
          .addDef(MRI.createGenericVirtualRegister(S1))
          .addUse(L)
          .add(Addr)
          .addUse(CarryIn);
    }
    return Builder.buildInstr(MOS::ADCImag8, {Result, CarryOut, S1},
                              {L, R, CarryIn});
  }();
  if (MI.getOpcode() == MOS::G_SADDE) {
    Register Tmp = Instr.getReg(1);
    Instr->getOperand(1).setReg(Instr.getReg(2));
    Instr->getOperand(2).setReg(Tmp);
  } else
    assert(MI.getOpcode() == MOS::G_UADDE);
  if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
    return false;

  MI.eraseFromParent();
  return true;
}

bool MOSInstructionSelector::selectUnMergeValues(MachineInstr &MI) {
  Register Lo = MI.getOperand(0).getReg();
  Register Hi = MI.getOperand(1).getReg();
  Register Src = MI.getOperand(2).getReg();

  MachineIRBuilder Builder(MI);

  auto LoCopy = Builder.buildCopy(Lo, Src);
  LoCopy->getOperand(1).setSubReg(MOS::sublo);
  constrainGenericOp(*LoCopy);
  auto HiCopy = Builder.buildCopy(Hi, Src);
  HiCopy->getOperand(1).setSubReg(MOS::subhi);
  constrainGenericOp(*HiCopy);
  MI.eraseFromParent();
  return true;
}

bool MOSInstructionSelector::selectGeneric(MachineInstr &MI) {
  unsigned Opcode;
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Unexpected opcode.");
  case MOS::G_BRINDIRECT:
    Opcode = MOS::JMPIndir;
    break;
  case MOS::G_INTTOPTR:
  case MOS::G_PTRTOINT:
    Opcode = MOS::COPY;
    break;
  case MOS::G_IMPLICIT_DEF:
    Opcode = MOS::IMPLICIT_DEF;
    break;
  case MOS::G_LOAD_ABS:
    Opcode = MOS::LDAbs;
    break;
  case MOS::G_LOAD_ABS_IDX:
    Opcode = MOS::LDIdx;
    break;
  case MOS::G_LOAD_INDIR_IDX:
    Opcode = MOS::LDYIndir;
    break;
  case MOS::G_PHI:
    Opcode = MOS::PHI;
    break;
  case MOS::G_STORE_ABS:
    Opcode = MOS::STAbs;
    break;
  case MOS::G_STORE_ABS_IDX:
    Opcode = MOS::STIdx;
    break;
  case MOS::G_STORE_INDIR_IDX:
    Opcode = MOS::STYIndir;
    break;
  }
  MI.setDesc(TII.get(Opcode));
  MI.addImplicitDefUseOperands(*MI.getMF());
  // Establish any tied operands and known register classes.
  if (!constrainSelectedInstRegOperands(MI, TII, TRI, RBI))
    return false;
  // Make sure that the outputs have register classes.
  constrainGenericOp(MI);
  return true;
}

// Produce a pointer vreg from a low and high vreg pair.
void MOSInstructionSelector::composePtr(MachineIRBuilder &Builder, Register Dst,
                                        Register Lo, Register Hi) {
  auto RegSeq = Builder.buildInstr(MOS::REG_SEQUENCE)
                    .addDef(Dst)
                    .addUse(Lo)
                    .addImm(MOS::sublo)
                    .addUse(Hi)
                    .addImm(MOS::subhi);
  constrainGenericOp(*RegSeq);

  // Rewrite uses of subregisters of the dst to Lo and Hi. Hopefully, this will
  // make the use of the REG_SEQUENCE dead. 16-bit pointer registers are hard to
  // come by in constrained zero pages. Unless their live ranges are limited,
  // register allocation may not be able to find a solution.
  std::vector<Register> Worklist = {Dst};
  std::vector<MachineOperand *> MOs;
  while (!Worklist.empty()) {
    Register Reg = Worklist.back();
    Worklist.pop_back();
    for (MachineInstr &MI : Builder.getMRI()->use_nodbg_instructions(Reg)) {
      if (MI.isCopy() && MI.getOperand(1).getReg().isVirtual() &&
          !MI.getOperand(1).getSubReg()) {
        Worklist.push_back(MI.getOperand(0).getReg());
        continue;
      }
      for (int Idx = 0, IdxEnd = MI.getNumOperands(); Idx != IdxEnd; Idx++) {
        MachineOperand &MO = MI.getOperand(Idx);
        if (MO.isReg() && MO.getReg() == Reg && MO.isUse() && MO.getSubReg())
          MOs.push_back(&MO);
      }
    }
  }

  // Machine operands cannot be directly modified in the above loop, since doing
  // so would upset use_nodbg_instructions. (The set of use instructions would
  // change.)
  for (MachineOperand *MO : MOs) {
    if (MO->getSubReg() == MOS::sublo) {
      MO->setReg(Lo);
      MO->setSubReg(0);
    } else {
      assert(MO->getSubReg() == MOS::subhi);

      MO->setReg(Hi);
      MO->setSubReg(0);
    }
  }
}

// Ensures that any virtual registers defined by this operation are given a
// register class. Otherwise, it's possible for chains of generic operations
// (PHI, COPY, etc.) to circularly define virtual registers in such a way that
// they never actually receive a register class. Since every virtual register is
// defined exactly once, making sure definitions are constrained suffices.
void MOSInstructionSelector::constrainGenericOp(MachineInstr &MI) {
  MachineRegisterInfo &MRI = MI.getMF()->getRegInfo();
  for (MachineOperand &Op : MI.operands()) {
    if (!Op.isReg() || !Op.isDef() || Op.getReg().isPhysical() ||
        MRI.getRegClassOrNull(Op.getReg()))
      continue;
    LLT Ty = MRI.getType(Op.getReg());
    constrainOperandRegClass(Op, getRegClassForType(Ty));
  }
}

void MOSInstructionSelector::constrainOperandRegClass(
    MachineOperand &RegMO, const TargetRegisterClass &RegClass) {
  MachineInstr &MI = *RegMO.getParent();
  MachineRegisterInfo &MRI = MI.getMF()->getRegInfo();
  RegMO.setReg(llvm::constrainOperandRegClass(*MF, TRI, MRI, TII, RBI, MI,
                                              RegClass, RegMO));
}

bool MOSInstructionSelector::selectAll(MachineInstrSpan MIS) {
  MachineRegisterInfo &MRI = MIS.begin()->getMF()->getRegInfo();

  // Ensure that all new generic virtual registers have a register bank.
  for (MachineInstr &MI : MIS)
    for (MachineOperand &MO : MI.operands()) {
      if (!MO.isReg())
        continue;
      Register Reg = MO.getReg();
      if (!MO.getReg().isVirtual())
        continue;
      if (MRI.getRegClassOrNull(MO.getReg()))
        continue;
      MRI.setRegBank(Reg, MOS::AnyRegBank);
    }

  // Select instructions in reverse block order. We permit erasing so have
  // to resort to manually iterating and recognizing the begin (rend) case.
  bool ReachedBegin = false;
  for (auto MII = std::prev(MIS.end()), Begin = MIS.begin(); !ReachedBegin;) {
    // Select this instruction.
    MachineInstr &MI = *MII;

    // And have our iterator point to the next instruction, if there is one.
    if (MII == Begin)
      ReachedBegin = true;
    else
      --MII;

    // We could have folded this instruction away already, making it dead.
    // If so, erase it.
    if (isTriviallyDead(MI, MRI)) {
      MI.eraseFromParent();
      continue;
    }

    if (!select(MI))
      return false;
  }
  return true;
}

InstructionSelector *llvm::createMOSInstructionSelector(
    const MOSTargetMachine &TM, MOSSubtarget &STI, MOSRegisterBankInfo &RBI) {
  return new MOSInstructionSelector(TM, STI, RBI);
}
