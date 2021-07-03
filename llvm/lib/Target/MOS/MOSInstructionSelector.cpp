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
#include "llvm/CodeGen/GlobalISel/InstructionSelectorImpl.h"
#include "llvm/CodeGen/GlobalISel/MIPatternMatch.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/GlobalISel/RegisterBankInfo.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstr.h"
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
  const MOSInstrInfo &TII;
  const MOSRegisterInfo &TRI;
  const MOSRegisterBankInfo &RBI;

  bool selectBrCondImm(MachineInstr &MI);
  bool selectCmp(MachineInstr &MI);
  bool selectConstant(MachineInstr &MI);
  bool selectIndex(MachineInstr &MI);
  bool selectFrameIndex(MachineInstr &MI);
  bool selectGlobalValue(MachineInstr &MI);
  bool selectLoadStore(MachineInstr &MI);
  bool selectLshrShlE(MachineInstr &MI);
  bool selectMergeValues(MachineInstr &MI);
  bool selectTrunc(MachineInstr &MI);
  bool selectAddSbcE(MachineInstr &MI);
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
    : TII(*STI.getInstrInfo()), TRI(*STI.getRegisterInfo()), RBI(RBI),
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
  if (selectImpl(MI, *CoverageInfo))
    return true;

  switch (MI.getOpcode()) {
  default:
    return false;
  case MOS::G_BRCOND_IMM:
    return selectBrCondImm(MI);
  case MOS::G_CONSTANT:
    return selectConstant(MI);
  case MOS::G_CMP:
    return selectCmp(MI);
  case MOS::G_FRAME_INDEX:
    return selectFrameIndex(MI);
  case MOS::G_GLOBAL_VALUE:
    return selectGlobalValue(MI);
  case MOS::G_INDEX:
    return selectIndex(MI);
  case MOS::G_LOAD:
  case MOS::G_STORE:
    return selectLoadStore(MI);
  case MOS::G_LSHRE:
  case MOS::G_SHLE:
    return selectLshrShlE(MI);
  case MOS::G_MERGE_VALUES:
    return selectMergeValues(MI);
  case MOS::G_TRUNC:
    return selectTrunc(MI);
  case MOS::G_UADDE:
  case MOS::G_SADDE:
  case MOS::G_USBCE:
  case MOS::G_SSBCE:
    return selectAddSbcE(MI);
  case MOS::G_UNMERGE_VALUES:
    return selectUnMergeValues(MI);

  case MOS::G_AND:
  case MOS::G_IMPLICIT_DEF:
  case MOS::G_INTTOPTR:
  case MOS::G_FREEZE:
  case MOS::G_OR:
  case MOS::G_PHI:
  case MOS::G_PTRTOINT:
  case MOS::G_XOR:
    return selectGeneric(MI);
  }
}

// Given a G_CMP instruction Cmp and one of its output virtual registers,
// returns the flag that corresponds to the register.
static Register getCmpFlagForRegister(const MachineInstr &Cmp, Register Reg) {
  static Register Flags[] = {MOS::C, MOS::N, MOS::V, MOS::Z};
  for (int Idx = 0; Idx < 4; ++Idx)
    if (Cmp.getOperand(Idx).getReg() == Reg)
      return Flags[Idx];
  llvm_unreachable("Could not find register in G_CMP outputs.");
}

struct CmpImm_match {
  Register &LHS;
  int64_t &RHS;
  Register &Flag;

  CmpImm_match(Register &LHS, int64_t &RHS, Register &Flag)
      : LHS(LHS), RHS(RHS), Flag(Flag) {}

  bool match(const MachineRegisterInfo &MRI, Register CondReg) {
    auto DefSrcReg = getDefSrcRegIgnoringCopies(CondReg, MRI);
    MachineInstr &CondMI = *DefSrcReg->MI;
    if (CondMI.getOpcode() != MOS::G_CMP)
      return false;
    auto RHSConst =
        getConstantVRegValWithLookThrough(CondMI.getOperand(5).getReg(), MRI);
    if (!RHSConst)
      return false;

    LHS = CondMI.getOperand(4).getReg();
    RHS = RHSConst->Value.getZExtValue();
    Flag = getCmpFlagForRegister(CondMI, DefSrcReg->Reg);

    // CMPImm cannot set the V register.
    return Flag != MOS::V;
  }
};

// Match one of the outputs of a G_CMP to a CMPImm operation. LHS and RHS are
// the left and right hand side of the comparison, while Flag is the virtual
// (for C) or physical (for N and Z) register corresponding to the output by
// which the G_CMP was reached.
inline CmpImm_match m_CmpImm(Register &LHS, int64_t &RHS, Register &Flag) {
  return {LHS, RHS, Flag};
}

bool MOSInstructionSelector::selectBrCondImm(MachineInstr &MI) {
  MachineRegisterInfo &MRI = MI.getMF()->getRegInfo();

  Register CondReg = MI.getOperand(0).getReg();
  MachineBasicBlock *Tgt = MI.getOperand(1).getMBB();
  int64_t FlagVal = MI.getOperand(2).getImm();

  LLT S1 = LLT::scalar(1);
  LLT S8 = LLT::scalar(8);

  MachineIRBuilder Builder(MI);

  Register LHS;
  int64_t RHS;
  Register Flag;
  if (!mi_match(CondReg, MRI, m_CmpImm(LHS, RHS, Flag))) {
    MachineInstrSpan MIS(MI, MI.getParent());
    auto CondExt = Builder.buildAnyExt(S8, CondReg);
    auto Zero = Builder.buildConstant(S8, 0);
    Register Z =
        Builder.buildInstr(MOS::G_CMP, {S1, S1, S1, S1 /*=Z*/}, {CondExt, Zero})
            .getReg(3);
    MI.getOperand(0).setReg(Z);
    MI.getOperand(2).setImm(!FlagVal);
    return selectAll(MIS);
  }

  // Use the terminator version of CMPImm to ensure that the live range of N and
  // Z is tightly curtailed.
  unsigned Opcode = Flag == MOS::C ? MOS::CMPImm : MOS::CMPImmTerm;
  auto Compare = Builder.buildInstr(Opcode, {S1}, {LHS, RHS});
  if (!constrainSelectedInstRegOperands(*Compare, TII, TRI, RBI))
    return false;

  if (Flag == MOS::C)
    Flag = Compare.getReg(0);

  Builder.buildInstr(MOS::BR).addMBB(Tgt).addUse(Flag).addImm(FlagVal);
  MI.eraseFromParent();
  return true;
}

bool MOSInstructionSelector::selectCmp(MachineInstr &MI) {
  MachineIRBuilder Builder(MI);

  Register CMPN = MI.getOperand(1).getReg();
  Register CMPZ = MI.getOperand(3).getReg();

  Register N = MOS::NoRegister;
  Register Z = MOS::NoRegister;
  if (!Builder.getMRI()->use_nodbg_empty(CMPN))
    N = CMPN;
  if (!Builder.getMRI()->use_nodbg_empty(CMPZ))
    Z = CMPZ;

  assert((!N || !Z) && "G_CMP can output at most one of N or Z.");

  auto C = Builder.buildInstr(MOS::LDCImm, {&MOS::CcRegClass}, {INT64_C(-1)});

  auto SBC = Builder.buildInstr(
      MOS::SBCNZImag8,
      {LLT::scalar(8), MI.getOperand(0), N, MI.getOperand(2), Z},
      {MI.getOperand(4), MI.getOperand(5), C});
  if (!constrainSelectedInstRegOperands(*SBC, TII, TRI, RBI))
    return false;
  MI.eraseFromParent();
  return true;
}

bool MOSInstructionSelector::selectConstant(MachineInstr &MI) {
  MachineIRBuilder Builder(MI);
  LLT S8 = LLT::scalar(8);

  Register Dst = MI.getOperand(0).getReg();
  uint64_t Imm = MI.getOperand(1).getCImm()->getZExtValue();

  LLT DstTy = Builder.getMRI()->getType(Dst);
  assert(DstTy.getSizeInBits() == 16);

  MachineInstrSpan MIS(MI, MI.getParent());
  auto Lo = Builder.buildConstant(S8, Imm & 0xFF);
  auto Hi = Builder.buildConstant(S8, Imm >> 8);
  Builder.buildMerge(MI.getOperand(0), {Lo, Hi});
  MI.eraseFromParent();
  selectAll(MIS);
  return true;
}

bool MOSInstructionSelector::selectIndex(MachineInstr &MI) {
  Register Dst = MI.getOperand(0).getReg();
  Register Base = MI.getOperand(1).getReg();
  Register Offset = MI.getOperand(2).getReg();

  MachineIRBuilder Builder(MI);

  LLT S1 = LLT::scalar(1);
  LLT S8 = LLT::scalar(8);

  MachineInstrSpan MIS(MI, MI.getParent());

  auto Unmerge = Builder.buildUnmerge(S8, Base);
  Register BaseLo = Unmerge.getReg(0), BaseHi = Unmerge.getReg(1);

  auto AddLo =
      Builder.buildUAdde(S8, S1, BaseLo, Offset, Builder.buildConstant(S1, 0));
  Register SumLo = AddLo.getReg(0);
  Register CarryLo = AddLo.getReg(1);
  auto AddHi =
      Builder.buildUAdde(S8, S1, BaseHi, Builder.buildConstant(S8, 0), CarryLo);
  Register SumHi = AddHi.getReg(0);
  composePtr(Builder, Dst, SumLo, SumHi);
  MI.eraseFromParent();
  return selectAll(MIS);
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
  if (MI.getMF()->getFunction().doesNotRecurse() && IsLocal) {
    // Non-recursive functions use static stack for their locals, so their frame
    // addresses are link-time constants that can be loaded as immediates.
    LoAddr = Builder.buildInstr(MOS::LDImm, {S8}, {}).add(MI.getOperand(1));
    LoAddr->getOperand(1).setTargetFlags(MOS::MO_LO);
    HiAddr = Builder.buildInstr(MOS::LDImm, {S8}, {}).add(MI.getOperand(1));
    HiAddr->getOperand(1).setTargetFlags(MOS::MO_HI);
  } else {
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
  }

  if (!constrainSelectedInstRegOperands(*LoAddr, TII, TRI, RBI))
    return false;
  if (!constrainSelectedInstRegOperands(*HiAddr, TII, TRI, RBI))
    return false;
  composePtr(Builder, Dst, LoAddr.getReg(0), HiAddr.getReg(0));
  MI.eraseFromParent();
  return true;
}

bool MOSInstructionSelector::selectGlobalValue(MachineInstr &MI) {
  Register Dst = MI.getOperand(0).getReg();

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
  composePtr(Builder, Dst, LoImm.getReg(0), HiImm.getReg(0));
  MI.eraseFromParent();
  return true;
}

// Determines if the memory address referenced by a load/store instruction
// is based on a constant value. Absolute or zero page addressing modes can
// be used under this condition.
static bool matchConstantAddr(Register Addr, MachineOperand &BaseOut,
                              const MachineRegisterInfo &MRI) {
  // Handle GlobalValues (including those with offsets for element access).
  if (MachineInstr *GV = getOpcodeDef(MOS::G_GLOBAL_VALUE, Addr, MRI)) {
    BaseOut = GV->getOperand(1);
    return true;
  }

  // Handle registers that can be resolved to constant values (e.g. IntToPtr).
  if (auto ConstAddr = getConstantVRegValWithLookThrough(Addr, MRI)) {
    BaseOut.ChangeToImmediate(ConstAddr->Value.getZExtValue());
    return true;
  }

  return false;
}

// Determines whether Addr can be referenced using the X/Y indexed addressing
// mode. If so, sets BaseOut to the base operand and OffsetOut to the value that
// should be in X/Y.
static bool matchIndexed(Register Addr, MachineOperand &BaseOut,
                         MachineOperand &OffsetOut,
                         const MachineRegisterInfo &MRI) {
  MachineInstr *SumAddr = getOpcodeDef(MOS::G_INDEX, Addr, MRI);
  if (!SumAddr)
    return false;

  Register Base = SumAddr->getOperand(1).getReg();
  Register Offset = SumAddr->getOperand(2).getReg();

  if (!matchConstantAddr(Base, BaseOut, MRI))
    return false;

  // Constant offsets should already have been folded into the base.
  OffsetOut.ChangeToRegister(Offset, /*isDef=*/false);
  return true;
}

// Determines whether Addr can be referenced using the indirect-indexed (addr),Y
// addressing mode. If so, sets BaseOut to the base operand and Offset to the
// value that should be in Y.
static void matchIndirectIndexed(Register Addr, MachineOperand &BaseOut,
                                 MachineOperand &OffsetOut,
                                 const MachineRegisterInfo &MRI) {
  MachineInstr *DefMI = getDefIgnoringCopies(Addr, MRI);
  if (DefMI->getOpcode() == MOS::G_INDEX) {
    Register Base = DefMI->getOperand(1).getReg();
    Register Offset = DefMI->getOperand(2).getReg();

    BaseOut.ChangeToRegister(Base, /*isDef=*/false);
    OffsetOut.ChangeToRegister(Offset, /*isDef=*/false);
    return;
  }

  // Any address can be accessed via (Addr),0.
  BaseOut.ChangeToRegister(Addr, /*isDef=*/false);
  OffsetOut.ChangeToImmediate(0);
}

bool MOSInstructionSelector::selectLoadStore(MachineInstr &MI) {
  MachineIRBuilder Builder(MI);
  MachineRegisterInfo &MRI = *Builder.getMRI();

  Register SrcDst = MI.getOperand(0).getReg();
  Register Addr = MI.getOperand(1).getReg();

  MachineOperand SrcDstOp = MachineOperand::CreateReg(SrcDst, /*isDef=*/false);

  unsigned AbsOpcode;
  unsigned IdxOpcode;
  unsigned YIndirOpcode;
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Unexpected opcode.");
  case MOS::G_LOAD:
    SrcDstOp.setIsDef();
    AbsOpcode = MOS::LDAbs;
    IdxOpcode = MOS::LDIdx;
    YIndirOpcode = MOS::LDYIndir;
    break;
  case MOS::G_STORE:
    AbsOpcode = MOS::STAbs;
    IdxOpcode = MOS::STIdx;
    YIndirOpcode = MOS::STYIndir;
    break;
  }

  MachineOperand Base = MachineOperand::CreateImm(0);
  MachineOperand Offset = MachineOperand::CreateImm(0);

  if (matchConstantAddr(Addr, Base, MRI)) {
    auto Instr =
        Builder.buildInstr(AbsOpcode).add(SrcDstOp).add(Base).cloneMemRefs(MI);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      return false;
    MI.eraseFromParent();
    return true;
  }

  if (matchIndexed(Addr, Base, Offset, MRI)) {
    auto Instr = Builder.buildInstr(IdxOpcode)
                     .add(SrcDstOp)
                     .add(Base)
                     .add(Offset)
                     .cloneMemRefs(MI);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      return false;
    MI.eraseFromParent();
    return true;
  }

  matchIndirectIndexed(Addr, Base, Offset, MRI);

  Register OffsetReg;
  if (Offset.isImm()) {
    OffsetReg =
        Builder.buildInstr(MOS::LDImm, {LLT::scalar(8)}, {Offset.getImm()})
            .getReg(0);
  } else
    OffsetReg = Offset.getReg();

  auto Instr = Builder.buildInstr(YIndirOpcode)
                   .add(SrcDstOp)
                   .add(Base)
                   .addUse(OffsetReg)
                   .cloneMemRefs(MI);
  if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
    return false;
  MI.eraseFromParent();
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

bool MOSInstructionSelector::selectAddSbcE(MachineInstr &MI) {
  unsigned ImmOpcode;
  unsigned Imag8Opcode;
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Unexpected opcode.");
  case MOS::G_UADDE:
  case MOS::G_SADDE:
    ImmOpcode = MOS::ADCImm;
    Imag8Opcode = MOS::ADCImag8;
    break;
  case MOS::G_USBCE:
  case MOS::G_SSBCE:
    ImmOpcode = MOS::SBCImm;
    Imag8Opcode = MOS::SBCImag8;
    break;
  }

  bool Signed;
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Unexpected opcode.");
  case MOS::G_UADDE:
  case MOS::G_USBCE:
    Signed = false;
    break;
  case MOS::G_SADDE:
  case MOS::G_SSBCE:
    Signed = true;
    break;
  }

  Register Result = MI.getOperand(0).getReg();
  Register CarryOut = MI.getOperand(1).getReg();
  Register L = MI.getOperand(2).getReg();
  Register R = MI.getOperand(3).getReg();
  Register CarryIn = MI.getOperand(4).getReg();

  MachineIRBuilder Builder(MI);

  LLT S1 = LLT::scalar(1);

  auto RConst = getConstantVRegValWithLookThrough(R, *Builder.getMRI());
  MachineInstrBuilder Instr;
  if (RConst) {
    assert(RConst->Value.getBitWidth() == 8);
    Instr = Builder.buildInstr(ImmOpcode, {Result, CarryOut, S1},
                               {L, RConst->Value.getZExtValue(), CarryIn});
  } else {
    Instr = Builder.buildInstr(Imag8Opcode, {Result, CarryOut, S1},
                               {L, R, CarryIn});
  }
  if (Signed) {
    Register Tmp = Instr.getReg(1);
    Instr->getOperand(1).setReg(Instr.getReg(2));
    Instr->getOperand(2).setReg(Tmp);
  }
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
  case MOS::G_AND:
    Opcode = MOS::ANDImag8;
    break;
  case MOS::G_FREEZE:
  case MOS::G_INTTOPTR:
  case MOS::G_PTRTOINT:
    Opcode = MOS::COPY;
    break;
  case MOS::G_IMPLICIT_DEF:
    Opcode = MOS::IMPLICIT_DEF;
    break;
  case MOS::G_OR:
    Opcode = MOS::ORAImag8;
    break;
  case MOS::G_PHI:
    Opcode = MOS::PHI;
    break;
  case MOS::G_XOR:
    Opcode = MOS::EORImag8;
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

  // Rewrite uses of subregisters of the dst to Lo and Hi. Otherwise, if the
  // uses of sublo and subhi are actually independent, dead lane elimination may
  // separate Dst into two separate virtual regisers, but without rewriting the
  // register class from imag16 to imag8. This wastes considerable space and
  // upsets the MachineVerifier.
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
  for (MachineInstr& MI : MIS)
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
      MI.eraseFromParentAndMarkDBGValuesForRemoval();
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
