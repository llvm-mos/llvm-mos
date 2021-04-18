//===-- MOSLegalizerInfo.cpp - MOS Legalizer-------------------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the interface that MOS uses to legalize generic MIR.
//
// Broadly only 8-bit integers and pointers are legal. It's legal to extract a
// 16-bit integer out of a pointer or to convert a 16-bit integer into one. The
// 16-bit integers must be lowered to a pair of 8-bit values for further
// manipulation, but they can be copied around and G_PHIed and so forth as-is.
//
//===----------------------------------------------------------------------===//

#include "MOSLegalizerInfo.h"

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOSMachineFunctionInfo.h"

#include "llvm/CodeGen/GlobalISel/LegalizerHelper.h"
#include "llvm/CodeGen/GlobalISel/LegalizerInfo.h"
#include "llvm/CodeGen/GlobalISel/MIPatternMatch.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/GlobalISel/RegisterBankInfo.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/PseudoSourceValue.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetOpcodes.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/IR/GlobalVariable.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;
using namespace TargetOpcode;
using namespace MIPatternMatch;

MOSLegalizerInfo::MOSLegalizerInfo() {
  using namespace LegalityPredicates;
  using namespace LegalizeMutations;

  LLT S1 = LLT::scalar(1);
  LLT S8 = LLT::scalar(8);
  LLT S16 = LLT::scalar(16);
  LLT S32 = LLT::scalar(32);
  LLT S64 = LLT::scalar(64);
  LLT P = LLT::pointer(0, 16);

  // Handle generation and copying of any type in the producer/consume type
  // sets.
  getActionDefinitionsBuilder({G_IMPLICIT_DEF, G_FREEZE, G_CONSTANT, G_PHI})
      .legalFor({S1, S8, S16, P})
      .clampScalar(0, S8, S8);

  // Constants

  getActionDefinitionsBuilder({G_FRAME_INDEX, G_GLOBAL_VALUE}).legalFor({P});

  // Integer Extension and Truncation

  getActionDefinitionsBuilder(G_ANYEXT).legalFor(
      {{S8, S1}, {S16, S8}, {S16, S1}});

  getActionDefinitionsBuilder(G_ZEXT)
      .legalFor({{S8, S1}})
      .clampScalar(0, S8, S8);

  getActionDefinitionsBuilder(G_TRUNC).legalFor(
      {{S1, S8}, {S1, S16}, {S8, S16}});

  // Type Conversions

  getActionDefinitionsBuilder(G_INTTOPTR).legalFor({{P, S16}});
  getActionDefinitionsBuilder(G_PTRTOINT).legalFor({{S16, P}});

  // Scalar Operations

  getActionDefinitionsBuilder(G_MERGE_VALUES)
      .legalForCartesianProduct({S16, P}, {S8});
  getActionDefinitionsBuilder(G_UNMERGE_VALUES)
      .legalForCartesianProduct({S8}, {S16, P});

  // Integer Operations

  getActionDefinitionsBuilder({G_ADD, G_SUB, G_AND, G_OR})
      .legalFor({S8})
      .clampScalar(0, S8, S8);

  getActionDefinitionsBuilder(G_XOR).legalFor({S8}).customFor({S1}).clampScalar(
      0, S8, S8);

  getActionDefinitionsBuilder(
      {G_MUL, G_SDIV, G_SREM, G_UDIV, G_UREM, G_CTLZ_ZERO_UNDEF})
      .libcall();

  // FIXME: Make this a libcall.
  getActionDefinitionsBuilder(G_UDIVREM).lower();

  getActionDefinitionsBuilder(G_ASHR)
      .legalFor({{S8, S8}})
      .clampScalar(0, S8, S8)
      // Truncate the shift amount to s8 once the resulting 8-bit shift
      // operations have been produced.
      .clampScalar(1, S8, S8);
  getActionDefinitionsBuilder(G_SHL).customFor({S8, S16, S32, S64});

  getActionDefinitionsBuilder(G_ROTL).legalFor({S8});

  // FIXME: The default narrowing of G_ICMP is terrible.
  getActionDefinitionsBuilder(G_ICMP)
      .customFor({{S1, P}, {S1, S8}})
      .minScalar(1, S8)
      .narrowScalarFor({{S1, S16}}, changeTo(1, S8))
      .narrowScalarFor({{S1, S32}}, changeTo(1, S16))
      .narrowScalarFor({{S1, S64}}, changeTo(1, S32));

  getActionDefinitionsBuilder(G_SELECT).legalFor({S1, S8}).clampScalar(0, S8,
                                                                       S8);

  // It's legal to G_PTR_ADD an 8-bit integer to a pointer, since there is at
  // least one addressing mode that performs this directly. The legalizer
  // endeavors to avoid producing G_PTR_ADDs where this addressing mode does not
  // apply, but it cannot always, so the instruction handler needs to handle
  // general 8-bit G_PTR_ADDs.
  getActionDefinitionsBuilder(G_PTR_ADD).legalFor({{P, S8}}).customFor(
      {{P, S16}});

  getActionDefinitionsBuilder({G_SMIN, G_SMAX, G_UMIN, G_UMAX}).lower();

  // FIXME: The default narrowing of G_ABS is terrible.
  getActionDefinitionsBuilder(G_ABS).lower();

  // Odd operations are handled via even ones: 6502 has only ADC/SBC.
  getActionDefinitionsBuilder({G_UADDO, G_USUBO}).customFor({S8});
  getActionDefinitionsBuilder({G_UADDE, G_USUBE}).legalFor({S8});

  // Floating Point Operations

  getActionDefinitionsBuilder({G_FADD,       G_FSUB,
                               G_FMUL,       G_FDIV,
                               G_FMA,        G_FPOW,
                               G_FREM,       G_FCOS,
                               G_FSIN,       G_FLOG10,
                               G_FLOG,       G_FLOG2,
                               G_FEXP,       G_FEXP2,
                               G_FCEIL,      G_FFLOOR,
                               G_FMINNUM,    G_FMAXNUM,
                               G_FSQRT,      G_FRINT,
                               G_FNEARBYINT, G_INTRINSIC_ROUNDEVEN,
                               G_FPEXT,      G_FPTRUNC,
                               G_FPTOSI,     G_FPTOUI,
                               G_SITOFP,     G_UITOFP})
      .libcall();

  // Memory Operations

  getActionDefinitionsBuilder({G_LOAD, G_STORE})
      .legalFor({{S8, P}})
      // Convert to int to load/store; that way the operation can be narrowed to
      // 8 bits.
      .customFor({{P, P}})
      .clampScalar(0, S8, S8);

  getActionDefinitionsBuilder({G_SEXTLOAD, G_ZEXTLOAD}).lower();

  getActionDefinitionsBuilder({G_MEMCPY, G_MEMMOVE, G_MEMSET}).libcall();

  // Control Flow

  getActionDefinitionsBuilder(G_BRCOND).customFor({S1});

  // Variadic Arguments

  getActionDefinitionsBuilder({G_VASTART, G_VAARG}).custom();

  // Other Operations

  getActionDefinitionsBuilder(G_DYN_STACKALLOC).lower();

  computeTables();
}

bool MOSLegalizerInfo::legalizeCustom(LegalizerHelper &Helper,
                                      MachineInstr &MI) const {
  MachineRegisterInfo &MRI = MI.getMF()->getRegInfo();

  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Invalid opcode for custom legalization.");
  case G_BRCOND:
    return legalizeBrCond(Helper, MRI, MI);
  case G_ICMP:
    return legalizeICmp(Helper, MRI, MI);
  case G_LOAD:
    return legalizeLoad(Helper, MRI, MI);
  case G_PTR_ADD:
    return legalizePtrAdd(Helper, MRI, MI);
  case G_SHL:
    return legalizeShl(Helper, MRI, MI);
  case G_STORE:
    return legalizeStore(Helper, MRI, MI);
  case G_UADDO:
  case G_USUBO:
    return legalizeUAddSubO(Helper, MRI, MI);
  case G_VAARG:
    return legalizeVAArg(Helper, MRI, MI);
  case G_VASTART:
    return legalizeVAStart(Helper, MRI, MI);
  case G_XOR:
    return legalizeXOR(Helper, MRI, MI);
  }
}

bool MOSLegalizerInfo::legalizeBrCond(LegalizerHelper &Helper,
                                      MachineRegisterInfo &MRI,
                                      MachineInstr &MI) const {
  assert(MI.getOpcode() == G_BRCOND);

  Register Tst = MI.getOperand(0).getReg();
  int64_t Val = 1;
  Register Not;
  if (mi_match(Tst, MRI, m_Not(m_Reg(Not)))) {
    Val = 0;
    Tst = Not;
  }

  MachineIRBuilder &Builder = Helper.MIRBuilder;
  Helper.Observer.changingInstr(MI);
  MI.setDesc(Builder.getTII().get(MOS::G_BRCOND_IMM));
  MI.getOperand(0).setReg(Tst);
  MI.addOperand(MachineOperand::CreateImm(Val));
  Helper.Observer.changedInstr(MI);
  return true;
}

// Lowers a comparison to the negation of the inverse comparison. For example,
// G_ICMP intpred(eq), A, B would become "not G_ICMP intpred(ne) A, B".
static void negateInverseComparison(LegalizerHelper &Helper, MachineInstr &MI) {
  Register Dst = MI.getOperand(0).getReg();
  auto Pred = static_cast<CmpInst::Predicate>(MI.getOperand(1).getPredicate());

  MachineIRBuilder &Builder = Helper.MIRBuilder;
  Register Not = Builder.getMRI()->createGenericVirtualRegister(LLT::scalar(1));
  Helper.Observer.changingInstr(MI);
  MI.getOperand(0).setReg(Not);
  MI.getOperand(1).setPredicate(CmpInst::getInversePredicate(Pred));
  Helper.Observer.changedInstr(MI);

  Builder.setInsertPt(Builder.getMBB(), std::next(Builder.getInsertPt()));
  Builder.buildNot(Dst, Not);
}

// Lowers a comparison to the swapped comparison on swapped operands. For
// example, G_ICMP intpred(ult), A, B would become "G_ICMP intpred(ugt) B, A".
static void swapComparison(LegalizerHelper &Helper, MachineInstr &MI) {
  Register LHS = MI.getOperand(2).getReg();
  Register RHS = MI.getOperand(3).getReg();
  auto Pred = static_cast<CmpInst::Predicate>(MI.getOperand(1).getPredicate());

  Helper.Observer.changingInstr(MI);
  MI.getOperand(1).setPredicate(CmpInst::getSwappedPredicate(Pred));
  MI.getOperand(2).setReg(RHS);
  MI.getOperand(3).setReg(LHS);
  Helper.Observer.changedInstr(MI);
}

bool MOSLegalizerInfo::legalizeICmp(LegalizerHelper &Helper,
                                    MachineRegisterInfo &MRI,
                                    MachineInstr &MI) const {
  assert(MI.getOpcode() == G_ICMP);
  MachineIRBuilder &Builder = Helper.MIRBuilder;

  Register Dst = MI.getOperand(0).getReg();
  Register LHS = MI.getOperand(2).getReg();
  Register RHS = MI.getOperand(3).getReg();

  LLT Type = MRI.getType(LHS);

  // Compare pointers by first converting to integer. This allows the comparison
  // to be reduced to 8-bit comparisons.
  if (Type.isPointer()) {
    LLT S16 = LLT::scalar(16);

    Helper.Observer.changingInstr(MI);
    MI.getOperand(2).setReg(Builder.buildPtrToInt(S16, LHS).getReg(0));
    MI.getOperand(3).setReg(Builder.buildPtrToInt(S16, RHS).getReg(0));
    Helper.Observer.changedInstr(MI);
    return true;
  }

  assert(Type == LLT::scalar(8));

  LLT S1 = LLT::scalar(1);

  // Lower 8-bit comparisons to a generic G_CMP instruction with similar
  // capabilities to the 6502's SBC and CMP instructions.
  // See www.6502.org/tutorials/compare_beyond.html.
  switch (MI.getOperand(1).getPredicate()) {
  case CmpInst::ICMP_EQ:
    Builder.buildInstr(MOS::G_CMP, {S1, S1, S1, Dst /*=Z*/}, {LHS, RHS});
    MI.eraseFromParent();
    break;
  case CmpInst::ICMP_UGE:
    Builder.buildInstr(MOS::G_CMP, {Dst /*=C*/, S1, S1, S1}, {LHS, RHS});
    MI.eraseFromParent();
    break;
  case CmpInst::ICMP_SLT: {
    auto Cmp = Builder.buildInstr(MOS::G_CMP, {S1, S1, S1, S1}, {LHS, RHS});
    Register N = Cmp.getReg(1);
    Register V = Cmp.getReg(2);
    Builder.buildXor(Dst, N, V);
    MI.eraseFromParent();
    break;
  }
  case CmpInst::ICMP_NE:
  case CmpInst::ICMP_ULT:
  case CmpInst::ICMP_SGE:
    negateInverseComparison(Helper, MI);
    break;
  case CmpInst::ICMP_ULE:
  case CmpInst::ICMP_UGT:
  case CmpInst::ICMP_SLE:
  case CmpInst::ICMP_SGT:
    swapComparison(Helper, MI);
    break;
  default:
    report_fatal_error("Not yet implemented.");
  }

  return true;
}

// Load pointers by loading a 16-bit integer, then converting to pointer. This
// allows the 16-bit loads to be reduced to a pair of 8-bit loads.
bool MOSLegalizerInfo::legalizeLoad(LegalizerHelper &Helper,
                                    MachineRegisterInfo &MRI,
                                    MachineInstr &MI) const {
  assert(MI.getOpcode() == G_LOAD);

  MachineIRBuilder &Builder = Helper.MIRBuilder;
  Register Tmp = MRI.createGenericVirtualRegister(LLT::scalar(16));
  Builder.setInsertPt(Builder.getMBB(), std::next(Builder.getInsertPt()));
  Builder.buildIntToPtr(MI.getOperand(0), Tmp);
  Helper.Observer.changingInstr(MI);
  MI.getOperand(0).setReg(Tmp);
  Helper.Observer.changedInstr(MI);
  return true;
}

bool MOSLegalizerInfo::legalizePtrAdd(LegalizerHelper &Helper,
                                      MachineRegisterInfo &MRI,
                                      MachineInstr &MI) const {
  assert(MI.getOpcode() == G_PTR_ADD);

  MachineIRBuilder &Builder = Helper.MIRBuilder;

  MachineOperand &Result = MI.getOperand(0);
  MachineOperand &Base = MI.getOperand(1);
  MachineOperand &Offset = MI.getOperand(2);

  MachineInstr *GlobalBase = getOpcodeDef(G_GLOBAL_VALUE, Base.getReg(), MRI);
  auto ConstOffset = getConstantVRegValWithLookThrough(Offset.getReg(), MRI);

  // Fold constant offsets into global value operand.
  if (GlobalBase && ConstOffset) {
    const MachineOperand &Op = GlobalBase->getOperand(1);
    Builder.buildInstr(G_GLOBAL_VALUE)
        .add(Result)
        .addGlobalAddress(Op.getGlobal(),
                          Op.getOffset() + ConstOffset->Value.getSExtValue());
    MI.eraseFromParent();
    return true;
  }

  // Adds of zero-extended offsets can instead use the legal 8-bit version of
  // G_PTR_ADD, with the goal of selecting indexed addressing modes.
  MachineInstr *ZExtOffset = getOpcodeDef(G_ZEXT, Offset.getReg(), MRI);
  if (ZExtOffset) {
    Helper.Observer.changingInstr(MI);
    Offset.setReg(ZExtOffset->getOperand(1).getReg());
    Helper.Observer.changedInstr(MI);
    return true;
  }

  // Similarly for offsets that fit in 8-bit unsigned constants.
  if (ConstOffset && ConstOffset->Value.isNonNegative() &&
      ConstOffset->Value.getActiveBits() <= 8) {
    auto Const =
        Builder.buildConstant(LLT::scalar(8), ConstOffset->Value.trunc(8));
    Helper.Observer.changingInstr(MI);
    Offset.setReg(Const.getReg(0));
    Helper.Observer.changedInstr(MI);
    return true;
  }

  // Generalized pointer additions must be lowered to 16-bit integer arithmetic.
  LLT S16 = LLT::scalar(16);
  Register PtrVal = Builder.buildPtrToInt(S16, MI.getOperand(1)).getReg(0);
  Register Sum = Builder.buildAdd(S16, PtrVal, MI.getOperand(2)).getReg(0);
  Builder.buildIntToPtr(MI.getOperand(0), Sum);
  MI.eraseFromParent();
  return true;
}

bool MOSLegalizerInfo::legalizeShl(LegalizerHelper &Helper,
                                   MachineRegisterInfo &MRI,
                                   MachineInstr &MI) const {
  assert(MI.getOpcode() == G_SHL);

  MachineIRBuilder &Builder = Helper.MIRBuilder;

  Register Dst = MI.getOperand(0).getReg();
  Register Src = MI.getOperand(1).getReg();
  Register Amt = MI.getOperand(2).getReg();

  // Presently, only left shifts by one bit are supported.
  auto ConstantAmt = getConstantVRegValWithLookThrough(Amt, MRI);
  if (!ConstantAmt || ConstantAmt->Value != 1)
    report_fatal_error("Only 1-bit left shifts are implemented.");

  LLT Ty = MRI.getType(Dst);
  assert(Ty == MRI.getType(Src));
  assert(Ty.isByteSized());

  LLT S1 = LLT::scalar(1);
  LLT S8 = LLT::scalar(8);

  auto Unmerge = Builder.buildUnmerge(S8, Src);
  SmallVector<Register> Parts;
  Register Carry = Builder.buildConstant(S1, 0).getReg(0);
  for (MachineOperand &SrcPart : Unmerge->defs()) {
    Parts.push_back(MRI.createGenericVirtualRegister(S8));
    Register NewCarry = MRI.createGenericVirtualRegister(S1);
    Builder.buildInstr(MOS::G_SHLE)
        .addDef(Parts.back())
        .addDef(NewCarry)
        .addUse(SrcPart.getReg())
        .addUse(Carry);
    Carry = NewCarry;
  }
  Builder.buildMerge(Dst, Parts);
  MI.eraseFromParent();

  return true;
}

// Converts pointer to integer before store, allowing the store to later be
// narrowed to 8 bits.
bool MOSLegalizerInfo::legalizeStore(LegalizerHelper &Helper,
                                     MachineRegisterInfo &MRI,
                                     MachineInstr &MI) const {
  assert(MI.getOpcode() == G_STORE);

  MachineIRBuilder &Builder = Helper.MIRBuilder;
  Register Tmp =
      Builder.buildPtrToInt(LLT::scalar(16), MI.getOperand(0)).getReg(0);
  Helper.Observer.changingInstr(MI);
  MI.getOperand(0).setReg(Tmp);
  Helper.Observer.changedInstr(MI);
  return true;
}

// Convert odd versions of generic add/sub to even versions, which can subsume
// the odd versions via a zero carry-in.
bool MOSLegalizerInfo::legalizeUAddSubO(LegalizerHelper &Helper,
                                        MachineRegisterInfo &MRI,
                                        MachineInstr &MI) const {
  unsigned Opcode;
  int64_t CarryInVal;
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Unexpected opcode.");
  case G_UADDO:
    Opcode = G_UADDE;
    CarryInVal = 0;
    break;
  case G_USUBO:
    Opcode = G_USUBE;
    CarryInVal = 1;
    break;
  }

  MachineIRBuilder &Builder = Helper.MIRBuilder;
  auto CarryIn = Builder.buildConstant(LLT::scalar(1), CarryInVal).getReg(0);
  Builder.buildInstr(Opcode, {MI.getOperand(0), MI.getOperand(1)},
                     {MI.getOperand(2), MI.getOperand(3), CarryIn});
  MI.eraseFromParent();
  return true;
}

// Lower variable argument access intrinsic.
bool MOSLegalizerInfo::legalizeVAArg(LegalizerHelper &Helper,
                                     MachineRegisterInfo &MRI,
                                     MachineInstr &MI) const {
  assert(MI.getOpcode() == G_VAARG);

  MachineIRBuilder &Builder = Helper.MIRBuilder;
  MachineFunction &MF = Builder.getMF();

  Register Dst = MI.getOperand(0).getReg();
  Register VaListPtr = MI.getOperand(1).getReg();

  LLT P = LLT::pointer(0, 16);

  // Load the current VAArg address out of the VAList.
  MachineMemOperand *AddrLoadMMO = MF.getMachineMemOperand(
      MachinePointerInfo::getUnknownStack(MF),
      MachineMemOperand::MOLoad | MachineMemOperand::MOInvariant, 2, Align());
  Register Addr = Builder.buildLoad(P, VaListPtr, *AddrLoadMMO).getReg(0);

  // Load the argument value out of the current VAArg address;
  unsigned Size = MRI.getType(Dst).getSizeInBytes();
  MachineMemOperand *ValueMMO = MF.getMachineMemOperand(
      MachinePointerInfo::getUnknownStack(MF),
      MachineMemOperand::MOLoad | MachineMemOperand::MOInvariant, Size,
      Align());
  Builder.buildLoad(Dst, Addr, *ValueMMO);

  // Increment the current VAArg address.
  Register SizeReg = Builder.buildConstant(LLT::scalar(16), Size).getReg(0);
  Register NextAddr = Builder.buildPtrAdd(P, Addr, SizeReg).getReg(0);
  MachineMemOperand *AddrStoreMMO =
      MF.getMachineMemOperand(MachinePointerInfo::getUnknownStack(MF),
                              MachineMemOperand::MOStore, 2, Align());
  Builder.buildStore(NextAddr, VaListPtr, *AddrStoreMMO);
  MI.eraseFromParent();
  return true;
}

// Lower variable argument pointer setup intrinsic.
bool MOSLegalizerInfo::legalizeVAStart(LegalizerHelper &Helper,
                                       MachineRegisterInfo &MRI,
                                       MachineInstr &MI) const {
  assert(MI.getOpcode() == G_VASTART);

  // Store the address of the fake varargs frame index into the valist.
  MachineIRBuilder &Builder = Helper.MIRBuilder;
  auto *FuncInfo = Builder.getMF().getInfo<MOSFunctionInfo>();
  Register Addr = Builder
                      .buildFrameIndex(LLT::pointer(0, 16),
                                       FuncInfo->getVarArgsStackIndex())
                      .getReg(0);
  Builder.buildStore(Addr, MI.getOperand(0), **MI.memoperands_begin());
  MI.eraseFromParent();
  return true;
}

bool MOSLegalizerInfo::legalizeXOR(LegalizerHelper &Helper,
                                   MachineRegisterInfo &MRI,
                                   MachineInstr &MI) const {
  assert(MI.getOpcode() == G_XOR);

  LLT S1 = LLT::scalar(1);

  Register Dst = MI.getOperand(0).getReg();
  assert(MRI.getType(Dst) == S1);

  Register Not;
  if (mi_match(Dst, MRI, m_Not(m_Reg(Not)))) {
    // The G_XOR may have been created by legalizing the definition of Dst.
    // If so, since uses are legalized before defs, the legalization of the use
    // of Dst has already occurred. Since the G_XOR didn't exist when the use
    // was being legalized, there hasn't yet been any opportunity to fold the
    // G_XOR in to the use. We do such folding here; hopefully that will make
    // the G_XOR dead.

    for (MachineInstr &UseMI : MRI.use_nodbg_instructions(Dst)) {
      if (UseMI.getOpcode() != MOS::G_BRCOND_IMM)
        continue;
      assert(UseMI.getOperand(0).getReg() == Dst);
      Helper.Observer.changingInstr(UseMI);
      UseMI.getOperand(0).setReg(Not);
      UseMI.getOperand(2).setImm(!UseMI.getOperand(2).getImm());
      Helper.Observer.changedInstr(UseMI);
    }

    if (!isTriviallyDead(MI, MRI)) {
      MachineIRBuilder &Builder = Helper.MIRBuilder;
      // If Not is true, select 0, otherwise select 1. This will eventually
      // lower to control flow.
      Helper.MIRBuilder.buildSelect(Dst, Not, Builder.buildConstant(S1, 0),
                                    Builder.buildConstant(S1, 1));
    }
    MI.eraseFromParent();
    return true;
  }

  if (isTriviallyDead(MI, MRI))
    MI.eraseFromParent();
  else
    Helper.widenScalar(MI, 0, LLT::scalar(8));

  return true;
}