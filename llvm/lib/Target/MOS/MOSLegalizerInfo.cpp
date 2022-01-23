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
#include "MOSInstrInfo.h"
#include "MOSMachineFunctionInfo.h"
#include "MOSRegisterInfo.h"
#include "MOSSubtarget.h"

#include "llvm/ADT/SmallVector.h"
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
#include "llvm/Demangle/Demangle.h"
#include "llvm/IR/GlobalVariable.h"
#include "llvm/IR/InstrTypes.h"
#include "llvm/IR/Instructions.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"

using namespace llvm;
using namespace TargetOpcode;
using namespace MIPatternMatch;

MOSLegalizerInfo::MOSLegalizerInfo(const MOSSubtarget &STI) {
  using namespace LegalityPredicates;
  using namespace LegalizeMutations;

  LLT S1 = LLT::scalar(1);
  LLT S8 = LLT::scalar(8);
  LLT S16 = LLT::scalar(16);
  LLT S32 = LLT::scalar(32);
  LLT S64 = LLT::scalar(64);
  LLT P = LLT::pointer(0, 16);

  // Constants

  getActionDefinitionsBuilder(G_CONSTANT)
      .legalFor({S1, S8})
      .customFor({P})
      .widenScalarToNextMultipleOf(0, 8)
      .maxScalar(0, S8)
      .unsupported();

  getActionDefinitionsBuilder(G_IMPLICIT_DEF)
      .legalFor({S1, S8, P})
      .widenScalarToNextMultipleOf(0, 8)
      .maxScalar(0, S8)
      .unsupported();

  getActionDefinitionsBuilder({G_FRAME_INDEX, G_GLOBAL_VALUE, G_BLOCK_ADDR})
      .legalFor({P})
      .unsupported();

  // Integer Extension and Truncation

  getActionDefinitionsBuilder(G_ANYEXT)
      .legalFor({{S8, S1}, {S16, S1}, {S16, S8}})
      .unsupported();
  getActionDefinitionsBuilder(G_TRUNC)
      .legalFor({{S1, S8}, {S1, S16}, {S8, S16}})
      .unsupported();

  getActionDefinitionsBuilder(G_SEXT).custom();

  getActionDefinitionsBuilder(G_SEXT_INREG).lower();

  getActionDefinitionsBuilder(G_ZEXT)
      .customIf(typeIs(1, S1))
      .maxScalar(0, S8)
      .unsupported();

  // Type Conversions

  getActionDefinitionsBuilder(G_INTTOPTR)
      .legalFor({{P, S16}})
      .clampScalar(1, S16, S16)
      .unsupported();
  getActionDefinitionsBuilder(G_PTRTOINT)
      .legalFor({{S16, P}})
      .clampScalar(0, S16, S16)
      .unsupported();

  // Scalar Operations

  getActionDefinitionsBuilder({G_EXTRACT, G_INSERT}).lower();

  getActionDefinitionsBuilder(G_MERGE_VALUES)
      .legalForCartesianProduct({S16, P}, {S8})
      .unsupported();
  getActionDefinitionsBuilder(G_UNMERGE_VALUES)
      .legalForCartesianProduct({S8}, {S16, P})
      .unsupported();

  getActionDefinitionsBuilder(G_BSWAP)
      .customFor({S8})
      .unsupportedIf(scalarNarrowerThan(0, 8))
      .widenScalarToNextMultipleOf(0, 8)
      .maxScalar(0, S8);

  getActionDefinitionsBuilder(G_BITREVERSE).lower();

  // Integer Operations

  getActionDefinitionsBuilder({G_ADD, G_SUB})
      .legalFor({S8})
      .widenScalarToNextMultipleOf(0, 8)
      .custom();

  getActionDefinitionsBuilder({G_AND, G_OR})
      .legalFor({S8})
      .widenScalarToNextMultipleOf(0, 8)
      .maxScalar(0, S8)
      .unsupported();

  getActionDefinitionsBuilder(G_XOR)
      .legalFor({S8})
      .customFor({S1})
      .widenScalarToNextMultipleOf(0, 8)
      .maxScalar(0, S8)
      .unsupported();

  getActionDefinitionsBuilder({G_MUL, G_SDIV, G_SREM, G_UDIV, G_UREM})
      .libcall();

  getActionDefinitionsBuilder({G_SDIVREM, G_UDIVREM})
      .customFor({S8, S16, S32, S64})
      .lower();

  getActionDefinitionsBuilder(
      {G_SADDSAT, G_UADDSAT, G_SSUBSAT, G_USUBSAT, G_SSHLSAT, G_USHLSAT})
      .lower();

  getActionDefinitionsBuilder({G_LSHR, G_SHL, G_ASHR})
      .widenScalarToNextMultipleOf(0, 8)
      .maxScalar(1, S8)
      .custom();

  getActionDefinitionsBuilder({G_ROTL, G_ROTR})
      .lowerIf([](const LegalityQuery &Query) {
        assert(Query.Types[0].isScalar());
        return !Query.Types[0].isByteSized();
      })
      .custom();

  getActionDefinitionsBuilder(G_ICMP)
      .customFor({{S1, P}, {S1, S8}})
      .widenScalarToNextMultipleOf(1, 8)
      .custom();

  getActionDefinitionsBuilder(G_SELECT)
      .customFor({P})
      .legalFor({S1, S8})
      .widenScalarToNextMultipleOf(0, 8)
      .maxScalar(0, S8)
      .unsupported();

  getActionDefinitionsBuilder(G_PTR_ADD).customFor({{P, S16}}).unsupported();

  getActionDefinitionsBuilder({G_SMIN, G_SMAX, G_UMIN, G_UMAX}).lower();

  getActionDefinitionsBuilder(G_ABS).custom();

  // Odd operations are handled via even ones: 6502 has only ADC/SBC.
  getActionDefinitionsBuilder({G_UADDO, G_SADDO, G_USUBO, G_SSUBO})
      .customFor({S8})
      .widenScalarToNextMultipleOf(0, 8)
      .maxScalar(0, S8)
      .unsupported();
  getActionDefinitionsBuilder({G_SMULO, G_UMULO}).lower();
  getActionDefinitionsBuilder({G_UADDE, G_SADDE})
      .legalFor({S8})
      .widenScalarToNextMultipleOf(0, 8)
      .maxScalar(0, S8)
      .unsupported();
  getActionDefinitionsBuilder({G_USUBE, G_SSUBE})
      .customFor({S8})
      .widenScalarToNextMultipleOf(0, 8)
      .maxScalar(0, S8)
      .unsupported();
  getActionDefinitionsBuilder({G_UMULH, G_SMULH}).lower();

  // WARNING: The default lowering of funnel shifts is terrible. Luckily, they
  // appear to mostly be rotations, which are combined away and handled
  // separately.
  getActionDefinitionsBuilder({G_FSHL, G_FSHR}).lower();

  getActionDefinitionsBuilder(
      {G_CTLZ, G_CTTZ, G_CTPOP, G_CTLZ_ZERO_UNDEF, G_CTTZ_ZERO_UNDEF})
      .lower();

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
      .unsupported();

  // Memory Operations

  getActionDefinitionsBuilder({G_LOAD, G_STORE})
      // Convert to int to load/store; that way the operation can be narrowed to
      // 8 bits. Once 8-bit, select an addressing mode to replace the generic
      // G_LOAD and G_STORE.
      .customFor({{S8, P}, {P, P}})
      .widenScalarToNextMultipleOf(0, 8)
      .maxScalar(0, S8)
      .unsupported();

  getActionDefinitionsBuilder({G_SEXTLOAD, G_ZEXTLOAD}).custom();

  getActionDefinitionsBuilder({G_MEMCPY, G_MEMMOVE, G_MEMSET}).libcall();

  // Control Flow

  getActionDefinitionsBuilder(G_PHI)
      .customFor({P})
      .legalFor({S1, S8})
      .widenScalarToNextMultipleOf(0, 8)
      .maxScalar(0, S8)
      .unsupported();

  getActionDefinitionsBuilder(G_BRCOND).customFor({S1}).unsupported();

  getActionDefinitionsBuilder(G_BRINDIRECT).legalFor({P});

  getActionDefinitionsBuilder(G_BRJT).customIf(
      all(typeIs(0, P), scalarWiderThan(1, 8)));

  getActionDefinitionsBuilder(G_JUMP_TABLE).unsupported();

  // Variadic Arguments

  getActionDefinitionsBuilder({G_VASTART, G_VAARG}).custom();

  // Other Operations

  getActionDefinitionsBuilder(G_DYN_STACKALLOC).custom();

  getActionDefinitionsBuilder(G_FREEZE)
      .customFor({S1, S8, P})
      .widenScalarToNextMultipleOf(0, 8)
      .maxScalar(0, S8)
      .unsupported();

  getLegacyLegalizerInfo().computeTables();
  verify(*STI.getInstrInfo());
}

bool MOSLegalizerInfo::legalizeIntrinsic(LegalizerHelper &Helper,
                                         MachineInstr &MI) const {
  LLT P = LLT::pointer(0, 16);
  MachineIRBuilder &Builder = Helper.MIRBuilder;
  switch (MI.getIntrinsicID()) {
  case Intrinsic::trap: {
    auto &Ctx = MI.getMF()->getFunction().getContext();
    auto *RetTy = Type::getVoidTy(Ctx);
    if (!createLibcall(Builder, "abort", {{}, RetTy, 0}, {}, CallingConv::C)) {
      return false;
    }
    MI.eraseFromParent();
    return true;
  }
  case Intrinsic::vacopy: {
    MachinePointerInfo MPO;
    auto Tmp =
        Builder.buildLoad(P, MI.getOperand(2),
                          *MI.getMF()->getMachineMemOperand(
                              MPO, MachineMemOperand::MOLoad, 2, Align()));
    Builder.buildStore(Tmp, MI.getOperand(1),
                       *MI.getMF()->getMachineMemOperand(
                           MPO, MachineMemOperand::MOStore, 2, Align()));
    MI.eraseFromParent();
    return true;
  }
  }
  return false;
}

bool MOSLegalizerInfo::legalizeCustom(LegalizerHelper &Helper,
                                      MachineInstr &MI) const {
  MachineRegisterInfo &MRI = MI.getMF()->getRegInfo();

  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Invalid opcode for custom legalization.");
  // Constants
  case G_CONSTANT:
    return legalizeConstant(Helper, MRI, MI);

  // Integer Extension and Truncation
  case G_SEXT:
    return legalizeSExt(Helper, MRI, MI);
  case G_ZEXT:
    return legalizeZExt(Helper, MRI, MI);

  // Scalar Operations
  case G_BSWAP:
    return legalizeBSwap(Helper, MRI, MI);

  // Integer Operations
  case G_ADD:
  case G_SUB:
    return legalizeAddSub(Helper, MRI, MI);
  case G_XOR:
    return legalizeXor(Helper, MRI, MI);
  case G_SDIVREM:
  case G_UDIVREM:
    return legalizeDivRem(Helper, MRI, MI);
  case G_LSHR:
  case G_SHL:
  case G_ASHR:
  case G_ROTL:
  case G_ROTR:
    return legalizeShiftRotate(Helper, MRI, MI);
  case G_ICMP:
    return legalizeICmp(Helper, MRI, MI);
  case G_SELECT:
    return legalizeSelect(Helper, MRI, MI);
  case G_ABS:
    return legalizeAbs(Helper, MRI, MI);
  case G_PTR_ADD:
    return legalizePtrAdd(Helper, MRI, MI);
  case G_UADDO:
  case G_SADDO:
  case G_USUBO:
  case G_SSUBO:
    return legalizeAddSubO(Helper, MRI, MI);
  case G_USUBE:
  case G_SSUBE:
    return legalizeSubE(Helper, MRI, MI);

  // Memory Operations
  case G_SEXTLOAD:
  case G_ZEXTLOAD:
  case G_LOAD:
    return legalizeLoad(Helper, MRI, MI);
  case G_STORE:
    return legalizeStore(Helper, MRI, MI);

  // Control Flow
  case G_PHI:
    return legalizePhi(Helper, MRI, MI);
  case G_BRCOND:
    return legalizeBrCond(Helper, MRI, MI);
  case G_BRJT:
    return legalizeBrJt(Helper, MRI, MI);

  // Variadic Arguments
  case G_VAARG:
    return legalizeVAArg(Helper, MRI, MI);
  case G_VASTART:
    return legalizeVAStart(Helper, MRI, MI);

  // Other Operations
  case G_DYN_STACKALLOC:
    return legalizeDynStackAlloc(Helper, MRI, MI);
  case G_FREEZE:
    return legalizeFreeze(Helper, MRI, MI);
  }
}

//===----------------------------------------------------------------------===//
// Constants
//===----------------------------------------------------------------------===//

bool MOSLegalizerInfo::legalizeConstant(LegalizerHelper &Helper,
                                        MachineRegisterInfo &MRI,
                                        MachineInstr &MI) const {
  MachineIRBuilder &Builder = Helper.MIRBuilder;

  Register Tmp = MRI.createGenericVirtualRegister(LLT::scalar(16));
  Register Dst = MI.getOperand(0).getReg();

  Helper.Observer.changingInstr(MI);
  MI.getOperand(0).setReg(Tmp);
  Helper.Observer.changedInstr(MI);
  Builder.setInsertPt(Builder.getMBB(), std::next(Builder.getInsertPt()));
  Builder.buildIntToPtr(Dst, Tmp);
  return true;
}

//===----------------------------------------------------------------------===//
// Integer Extension and Truncation
//===----------------------------------------------------------------------===//

static auto unmergeDefs(MachineInstr *MI) {
  assert(MI->getOpcode() == TargetOpcode::G_UNMERGE_VALUES);
  return make_range(MI->operands_begin(), MI->operands_end() - 1);
}

static auto unmergeHighDefs(MachineInstr *MI) {
  assert(MI->getOpcode() == TargetOpcode::G_UNMERGE_VALUES);
  return make_range(MI->operands_begin() + 1, MI->operands_end() - 1);
}

static auto unmergeDefsSplitHigh(MachineInstr *MI) {
  assert(MI->getOpcode() == TargetOpcode::G_UNMERGE_VALUES);
  struct LowsAndHigh {
    iterator_range<MachineInstr::mop_iterator> Lows;
    MachineOperand &High;
  };
  return LowsAndHigh{make_range(MI->operands_begin(), MI->operands_end() - 2),
                     MI->getOperand(MI->getNumOperands() - 2)};
}

bool MOSLegalizerInfo::legalizeSExt(LegalizerHelper &Helper,
                                    MachineRegisterInfo &MRI,
                                    MachineInstr &MI) const {
  LLT S1 = LLT::scalar(1);
  LLT S8 = LLT::scalar(8);
  MachineIRBuilder &Builder = Helper.MIRBuilder;

  Register Dst = MI.getOperand(0).getReg();
  Register Src = MI.getOperand(1).getReg();

  LLT DstTy = MRI.getType(Dst);
  LLT SrcTy = MRI.getType(Src);

  if (SrcTy == S1) {
    auto NegOne = Builder.buildConstant(DstTy, -1);
    auto Zero = Builder.buildConstant(DstTy, 0);
    Builder.buildSelect(Dst, Src, NegOne, Zero);
  } else {
    auto Neg = Builder.buildICmp(CmpInst::ICMP_SLT, S1, Src,
                                 Builder.buildConstant(SrcTy, 0));
    auto NegOne = Builder.buildConstant(S8, -1);
    auto Zero = Builder.buildConstant(S8, 0);

    Register Fill = Builder.buildSelect(S8, Neg, NegOne, Zero).getReg(0);

    SmallVector<Register> Parts;
    unsigned Bits;
    if (SrcTy == S8) {
      Parts.push_back(Src);
      Bits = 8;
    } else {
      auto Unmerge = Builder.buildUnmerge(S8, Src);
      Bits = 0;
      for (MachineOperand &Op : unmergeDefs(Unmerge)) {
        Parts.push_back(Op.getReg());
        Bits += 8;
      }
    }
    while (Bits < DstTy.getSizeInBits()) {
      Parts.push_back(Fill);
      Bits += 8;
    }
    Builder.buildMerge(Dst, Parts);
  }

  MI.eraseFromParent();
  return true;
}

bool MOSLegalizerInfo::legalizeZExt(LegalizerHelper &Helper,
                                    MachineRegisterInfo &MRI,
                                    MachineInstr &MI) const {
  MachineIRBuilder &Builder = Helper.MIRBuilder;
  Register Dst = MI.getOperand(0).getReg();
  Register Src = MI.getOperand(1).getReg();

  LLT DstTy = MRI.getType(Dst);
  LLT SrcTy = MRI.getType(Src);

  assert(SrcTy == LLT::scalar(1));
  auto One = Builder.buildConstant(DstTy, 1);
  auto Zero = Builder.buildConstant(DstTy, 0);
  Builder.buildSelect(Dst, Src, One, Zero);
  MI.eraseFromParent();
  return true;
}

//===----------------------------------------------------------------------===//
// Scalar Operations
//===----------------------------------------------------------------------===//

bool MOSLegalizerInfo::legalizeBSwap(LegalizerHelper &Helper,
                                     MachineRegisterInfo &MRI,
                                     MachineInstr &MI) const {
  LLT S8 = LLT::scalar(8);
  assert(MRI.getType(MI.getOperand(0).getReg()) == S8);
  assert(MRI.getType(MI.getOperand(1).getReg()) == S8);
  Helper.Observer.changingInstr(MI);
  MI.setDesc(Helper.MIRBuilder.getTII().get(COPY));
  Helper.Observer.changedInstr(MI);
  return true;
}

//===----------------------------------------------------------------------===//
// Integer Operations
//===----------------------------------------------------------------------===//

static std::pair<Register, Register> splitLowRest(Register Reg,
                                                  MachineIRBuilder &Builder) {
  LLT S8 = LLT::scalar(8);

  auto Unmerge = Builder.buildUnmerge(S8, Reg);
  Register Low = Unmerge.getReg(0);

  SmallVector<Register> RestParts;
  for (MachineOperand &Op : unmergeHighDefs(Unmerge))
    RestParts.push_back(Op.getReg());
  Register Rest =
      (RestParts.size() > 1)
          ? Builder.buildMerge(LLT::scalar(RestParts.size() * 8), RestParts)
                .getReg(0)
          : RestParts[0];

  return {Low, Rest};
}

static void mergeLowRest(Register Dst, Register Low, Register Rest,
                         MachineIRBuilder &Builder) {
  LLT S8 = LLT::scalar(8);
  const auto &MRI = *Builder.getMRI();
  LLT RestTy = MRI.getType(Rest);

  SmallVector<Register> DstParts = {Low};
  if (RestTy == S8)
    DstParts.push_back(Rest);
  else {
    auto Unmerge = Builder.buildUnmerge(S8, Rest);
    for (MachineOperand &Op : unmergeDefs(Unmerge))
      DstParts.push_back(Op.getReg());
  }
  Builder.buildMerge(Dst, DstParts);
}

static std::pair<Register, Register> splitHighRest(Register Reg,
                                                   MachineIRBuilder &Builder) {
  LLT S8 = LLT::scalar(8);

  auto Unmerge = Builder.buildUnmerge(S8, Reg);
  auto UnmergeDefs = unmergeDefsSplitHigh(Unmerge);

  SmallVector<Register> RestParts;
  for (MachineOperand &Op : UnmergeDefs.Lows)
    RestParts.push_back(Op.getReg());
  Register Rest =
      (RestParts.size() > 1)
          ? Builder.buildMerge(LLT::scalar(RestParts.size() * 8), RestParts)
                .getReg(0)
          : RestParts[0];

  return {UnmergeDefs.High.getReg(), Rest};
}

bool MOSLegalizerInfo::legalizeAddSub(LegalizerHelper &Helper,
                                      MachineRegisterInfo &MRI,
                                      MachineInstr &MI) const {
  auto &Builder = Helper.MIRBuilder;
  LLT S8 = LLT::scalar(8);

  Register Dst = MI.getOperand(0).getReg();
  LLT DstTy = MRI.getType(Dst);
  assert(DstTy.isByteSized());

  auto RHSConst =
      getIConstantVRegValWithLookThrough(MI.getOperand(2).getReg(), MRI);
  if (!RHSConst || std::abs(RHSConst->Value.getSExtValue()) != 1)
    return Helper.narrowScalarAddSub(MI, 0, S8) !=
           LegalizerHelper::UnableToLegalize;

  // Handle multi-byte increments and decrements.

  Register Low, Rest;
  std::tie(Low, Rest) = splitLowRest(MI.getOperand(1).getReg(), Builder);
  LLT RestTy = MRI.getType(Rest);

  SmallVector<Register> DstParts;

  assert(MI.getOpcode() == MOS::G_ADD || MI.getOpcode() == MOS::G_SUB);
  int64_t Amt = RHSConst->Value.getSExtValue();
  if (MI.getOpcode() == MOS::G_SUB)
    Amt = -Amt;

  auto AmtLow = Builder.buildConstant(S8, Amt);
  Register DstLow = Builder.buildAdd(S8, Low, AmtLow).getReg(0);
  auto CarryBorrow =
      Builder.buildICmp(CmpInst::ICMP_EQ, LLT::scalar(1), DstLow,
                        (Amt == 1) ? Builder.buildConstant(S8, 0)
                                   : Builder.buildConstant(S8, 255));
  auto AmtRest = Builder.buildConstant(RestTy, Amt);
  auto RestIncDec = Builder.buildAdd(RestTy, Rest, AmtRest);
  Register DstRest =
      Builder.buildSelect(RestTy, CarryBorrow, RestIncDec, Rest).getReg(0);

  mergeLowRest(Dst, DstLow, DstRest, Builder);
  MI.eraseFromParent();
  return true;
}

bool MOSLegalizerInfo::legalizeXor(LegalizerHelper &Helper,
                                   MachineRegisterInfo &MRI,
                                   MachineInstr &MI) const {
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
      if (UseMI.getOpcode() == MOS::G_BRCOND_IMM) {
        assert(UseMI.getOperand(0).getReg() == Dst);
        Helper.Observer.changingInstr(UseMI);
        UseMI.getOperand(0).setReg(Not);
        UseMI.getOperand(2).setImm(!UseMI.getOperand(2).getImm());
        Helper.Observer.changedInstr(UseMI);
      } else if (UseMI.getOpcode() == MOS::G_SELECT &&
                 mi_match(UseMI.getOperand(2).getReg(), MRI, m_ZeroInt()) &&
                 mi_match(UseMI.getOperand(3).getReg(), MRI, m_AllOnesInt())) {
        Helper.Observer.changingInstr(UseMI);
        UseMI.getOperand(1).setReg(Not);
        UseMI.RemoveOperand(3);
        UseMI.RemoveOperand(2);
        UseMI.setDesc(Helper.MIRBuilder.getTII().get(MOS::COPY));
        Helper.Observer.changedInstr(UseMI);
      }
    }

    if (!isTriviallyDead(MI, MRI)) {
      MachineIRBuilder &Builder = Helper.MIRBuilder;
      // If Not is true, select 0, otherwise select 1. This will eventually
      // lower to control flow.
      auto Zero = Builder.buildConstant(S1, 0);
      auto One = Builder.buildConstant(S1, 1);
      Helper.MIRBuilder.buildSelect(Dst, Not, Zero, One);
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

bool MOSLegalizerInfo::legalizeDivRem(LegalizerHelper &Helper,
                                      MachineRegisterInfo &MRI,
                                      MachineInstr &MI) const {
  LLT Ty = MRI.getType(MI.getOperand(0).getReg());
  auto &Ctx = MI.getMF()->getFunction().getContext();

  auto Libcall = getRTLibDesc(MI.getOpcode(), Ty.getSizeInBits());

  Type *HLTy = IntegerType::get(Ctx, Ty.getSizeInBits());

  SmallVector<CallLowering::ArgInfo, 3> Args;
  Args.push_back({MI.getOperand(2).getReg(), HLTy, 0});
  Args.push_back({MI.getOperand(3).getReg(), HLTy, 1});

  // Pass a pointer to receive the remainder.
  MachinePointerInfo PtrInfo;
  auto FI = Helper.createStackTemporary(TypeSize::Fixed(Ty.getSizeInBytes()),
                                        Align(), PtrInfo);

  Type *PtrTy = PointerType::get(HLTy, 0);
  Args.push_back({FI->getOperand(0).getReg(), PtrTy, 2});

  if (!createLibcall(Helper.MIRBuilder, Libcall,
                     {MI.getOperand(0).getReg(), HLTy, 0}, Args))
    return false;

  Helper.MIRBuilder.buildLoad(
      MI.getOperand(1), FI,
      *Helper.MIRBuilder.getMF().getMachineMemOperand(
          PtrInfo,
          MachineMemOperand::MOLoad | MachineMemOperand::MODereferenceable,
          Ty.getSizeInBytes(), Align()));

  MI.eraseFromParent();
  return true;
}

// Whether or not it's worth shifting/rotating by 8 (in a type one byte wider
// for shifts) and shifting/rotating in the opposite direction by 8-n.
static bool shouldOverCorrect(uint64_t Amt, LLT Ty, bool IsRotate) {
  assert(Amt < 8);

  if (IsRotate)
    return Amt > 4;

  // The choice is between emitting Amt operations at width Ty, or emitting 8 -
  // Amt operations (in the opposite direction) at width Ty + 8.
  return Amt * Ty.getSizeInBytes() > (8 - Amt) * (Ty.getSizeInBytes() + 1);
}

bool MOSLegalizerInfo::legalizeShiftRotate(LegalizerHelper &Helper,
                                           MachineRegisterInfo &MRI,
                                           MachineInstr &MI) const {
  MachineIRBuilder &Builder = Helper.MIRBuilder;

  Register Dst = MI.getOperand(0).getReg();
  Register Src = MI.getOperand(1).getReg();
  Register AmtReg = MI.getOperand(2).getReg();

  bool IsRotate = MI.getOpcode() == G_ROTL || MI.getOpcode() == G_ROTR;

  LLT Ty = MRI.getType(Dst);
  assert(Ty == MRI.getType(Src));
  assert(Ty.isByteSized());

  LLT S1 = LLT::scalar(1);
  LLT S8 = LLT::scalar(8);

  // Presently, only left shifts by one bit are supported.
  auto ConstantAmt = getIConstantVRegValWithLookThrough(AmtReg, MRI);
  if (!ConstantAmt) {
    if (IsRotate)
      return Helper.lowerRotate(MI);
    if (!isPowerOf2_32(Ty.getSizeInBits()))
      return Helper.widenScalar(MI, 0,
                                LLT::scalar(NextPowerOf2(Ty.getSizeInBits())));
    if (Ty.getSizeInBits() > 64)
      return Helper.narrowScalar(MI, 0, LLT::scalar(64));
    return shiftLibcall(Helper, MRI, MI);
  }

  uint64_t Amt = ConstantAmt->Value.getZExtValue();

  // This forms the base case for the problem decompositions below.
  if (Amt == 0) {
    Builder.buildCopy(Dst, Src);
    MI.eraseFromParent();
    return true;
  }

  Register Partial;
  Register NewAmt;
  // Shift by one multiples of one byte.
  if (Amt >= 8) {
    auto Unmerge = Builder.buildUnmerge(S8, Src);
    SmallVector<Register> DstBytes;
    for (MachineOperand &Op : unmergeDefs(Unmerge))
      DstBytes.push_back(Op.getReg());
    Register Fill;
    switch (MI.getOpcode()) {
    default:
      llvm_unreachable("Invalid opcode.");
    case MOS::G_ROTL:
    case MOS::G_ROTR:
      break;
    case MOS::G_ASHR: {
      Register Sign = Builder
                          .buildICmp(ICmpInst::ICMP_SLT, S1, Src,
                                     Builder.buildConstant(Ty, 0))
                          .getReg(0);
      Fill = Builder.buildSExt(S8, Sign).getReg(0);
      break;
    }
    case MOS::G_LSHR:
    case MOS::G_SHL:
      Fill = Builder.buildConstant(S8, 0).getReg(0);
      break;
    }
    // Instead of decomposing the problem recursively byte-by-byte, looping here
    // ensures that Fill is reused for G_ASHR.
    while (Amt >= 8) {
      switch (MI.getOpcode()) {
      default:
        llvm_unreachable("Invalid opcode.");
      case MOS::G_LSHR:
      case MOS::G_ASHR:
      case MOS::G_SHL:
        break;
      case MOS::G_ROTR:
        Fill = DstBytes.front();
        break;
      case MOS::G_ROTL:
        Fill = DstBytes.back();
        break;
      }
      switch (MI.getOpcode()) {
      default:
        llvm_unreachable("Invalid opcode.");
      case MOS::G_LSHR:
      case MOS::G_ASHR:
      case MOS::G_ROTR:
        DstBytes.erase(DstBytes.begin());
        DstBytes.push_back(Fill);
        break;
      case MOS::G_SHL:
      case MOS::G_ROTL:
        DstBytes.pop_back();
        DstBytes.insert(DstBytes.begin(), Fill);
        break;
      }
      Amt -= 8;
    }
    assert(Amt < 8);
    Partial = Builder.buildMerge(Ty, DstBytes).getReg(0);
    NewAmt = Builder.buildConstant(S8, Amt).getReg(0);
  } else if (shouldOverCorrect(Amt, Ty, IsRotate)) {
    Register LeftAmt, RightAmt;
    switch (MI.getOpcode()) {
    default:
      llvm_unreachable("Invalid opcode.");
    case G_SHL:
    case G_ROTL:
      if (Ty == S8 && MI.getOpcode() == G_ROTL)
        LeftAmt = Builder.buildConstant(S8, 0).getReg(0);
      else
        LeftAmt = Builder.buildConstant(S8, 8).getReg(0);
      RightAmt = Builder.buildConstant(S8, 8 - Amt).getReg(0);
      break;
    case G_LSHR:
    case G_ASHR:
    case G_ROTR:
      LeftAmt = Builder.buildConstant(S8, 8 - Amt).getReg(0);
      if (Ty == S8 && MI.getOpcode() == G_ROTR)
        RightAmt = Builder.buildConstant(S8, 0).getReg(0);
      else
        RightAmt = Builder.buildConstant(S8, 8).getReg(0);
    }
    if (IsRotate) {
      auto Left = Builder.buildRotateLeft(Ty, Src, LeftAmt);
      Builder.buildRotateRight(Dst, Left, RightAmt);
    } else {
      LLT WideTy = LLT::scalar(Ty.getSizeInBits() + 8);
      Register WideSrc;
      switch (MI.getOpcode()) {
      default:
        llvm_unreachable("Invalid opcode.");
      case G_SHL:
        WideSrc = Builder.buildAnyExt(WideTy, Src).getReg(0);
        break;
      case G_LSHR:
        WideSrc = Builder.buildZExt(WideTy, Src).getReg(0);
        break;
      case G_ASHR:
        WideSrc = Builder.buildSExt(WideTy, Src).getReg(0);
        break;
      }
      auto Left = Builder.buildShl(WideTy, WideSrc, LeftAmt);
      auto Right = Builder.buildLShr(WideTy, Left, RightAmt).getReg(0);
      Builder.buildTrunc(Dst, Right);
    }
    MI.eraseFromParent();
    return true;
  } else {
    // Shift by one, then shift the remainder.

    Register CarryIn;
    switch (MI.getOpcode()) {
    default:
      llvm_unreachable("Invalid opcode.");
    case G_SHL:
    case G_LSHR:
      CarryIn = Builder.buildConstant(S1, 0).getReg(0);
      break;
    case G_ROTR: {
      // Once selected, this places the low bit in the carry flag.
      Register LowByte =
          (Ty == S8) ? Src : Builder.buildUnmerge(S8, Src).getReg(0);
      CarryIn = Builder
                    .buildInstr(MOS::G_LSHRE, {S8, S1},
                                {LowByte, Builder.buildUndef(S1)})
                    .getReg(1);
      break;
    }
    case G_ASHR:
    case G_ROTL: {
      // Once selected, this places the high bit in the carry flag.
      Register HighByte =
          (Ty == S8)
              ? Src
              : Builder.buildUnmerge(S8, Src).getReg(Ty.getSizeInBytes() - 1);
      CarryIn = Builder
                    .buildICmp(ICmpInst::ICMP_UGE, S1, HighByte,
                               Builder.buildConstant(S8, 0x80))
                    .getReg(0);
      break;
    }
    }

    unsigned Opcode;
    switch (MI.getOpcode()) {
    default:
      llvm_unreachable("Invalid opcode.");
    case G_SHL:
    case G_ROTL:
      Opcode = MOS::G_SHLE;
      break;
    case G_LSHR:
    case G_ASHR:
    case G_ROTR:
      Opcode = MOS::G_LSHRE;
      break;
    }
    auto Even = Builder.buildInstr(Opcode, {Ty, S1}, {Src, CarryIn});
    Partial = Even.getReg(0);
    if (!legalizeLshrEShlE(Helper, MRI, *Even))
      return false;
    NewAmt = Builder.buildConstant(S8, Amt - 1).getReg(0);
  }

  Helper.Observer.changingInstr(MI);
  MI.getOperand(1).setReg(Partial);
  MI.getOperand(2).setReg(NewAmt);
  Helper.Observer.changedInstr(MI);
  return true;
}

bool MOSLegalizerInfo::legalizeLshrEShlE(LegalizerHelper &Helper,
                                         MachineRegisterInfo &MRI,
                                         MachineInstr &MI) const {
  MachineIRBuilder &Builder = Helper.MIRBuilder;
  LLT S1 = LLT::scalar(1);
  LLT S8 = LLT::scalar(8);

  Register Dst = MI.getOperand(0).getReg();
  Register CarryOut = MI.getOperand(1).getReg();
  Register Src = MI.getOperand(2).getReg();
  Register CarryIn = MI.getOperand(3).getReg();

  LLT Ty = MRI.getType(Dst);
  if (Ty == S8)
    return true;

  auto Unmerge = Builder.buildUnmerge(S8, Src);
  SmallVector<Register> Parts;

  SmallVector<Register> Defs;
  for (MachineOperand &SrcPart : unmergeDefs(Unmerge))
    Defs.push_back(SrcPart.getReg());

  if (MI.getOpcode() == MOS::G_LSHRE)
    std::reverse(Defs.begin(), Defs.end());

  Register Carry = CarryIn;
  for (const auto &I : enumerate(Defs)) {
    Parts.push_back(MRI.createGenericVirtualRegister(S8));
    Register NewCarry = I.index() == Defs.size() - 1
                            ? CarryOut
                            : MRI.createGenericVirtualRegister(S1);
    Builder.buildInstr(MI.getOpcode(), {Parts.back(), NewCarry},
                       {I.value(), Carry});
    Carry = NewCarry;
  }

  if (MI.getOpcode() == MOS::G_LSHRE)
    std::reverse(Parts.begin(), Parts.end());

  Builder.buildMerge(Dst, Parts).getReg(0);
  MI.eraseFromParent();
  return true;
}

bool MOSLegalizerInfo::shiftLibcall(LegalizerHelper &Helper,
                                    MachineRegisterInfo &MRI,
                                    MachineInstr &MI) const {
  unsigned Size = MRI.getType(MI.getOperand(0).getReg()).getSizeInBits();
  auto &Ctx = MI.getMF()->getFunction().getContext();

  auto Libcall = getRTLibDesc(MI.getOpcode(), Size);

  Type *HLTy = IntegerType::get(Ctx, Size);
  Type *HLAmtTy = IntegerType::get(Ctx, 8);

  SmallVector<CallLowering::ArgInfo, 3> Args;
  Args.push_back({MI.getOperand(1).getReg(), HLTy, 0});
  Args.push_back({MI.getOperand(2).getReg(), HLAmtTy, 1});
  if (!createLibcall(Helper.MIRBuilder, Libcall,
                     {MI.getOperand(0).getReg(), HLTy, 0}, Args))
    return false;

  MI.eraseFromParent();
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

// Adjust the constant RHS to swap strictness of the predicate. This keeps the
// RHS constant, as opposed to swapping the arguments, which forces a load of
// the RHS into a GPR.
static bool adjustConstRHS(LegalizerHelper &Helper, MachineInstr &MI) {
  auto &Builder = Helper.MIRBuilder;
  const auto &MRI = *Builder.getMRI();

  Register RHS = MI.getOperand(3).getReg();
  auto Pred = static_cast<CmpInst::Predicate>(MI.getOperand(1).getPredicate());

  auto ConstRHS = getIConstantVRegValWithLookThrough(RHS, MRI);
  if (!ConstRHS)
    return false;

  switch (Pred) {
  default:
    llvm_unreachable("Unexpected predicate.");
  case CmpInst::ICMP_ULE:
  case CmpInst::ICMP_UGT:
    if (ConstRHS->Value.isMaxValue())
      return false;
    break;
  case CmpInst::ICMP_SLE:
  case CmpInst::ICMP_SGT:
    if (ConstRHS->Value.isMaxSignedValue())
      return false;
    break;
  }

  Helper.Observer.changingInstr(MI);
  MI.getOperand(1).setPredicate(CmpInst::getFlippedStrictnessPredicate(Pred));
  MI.getOperand(3).setReg(
      Builder.buildConstant(MRI.getType(RHS), ConstRHS->Value + 1).getReg(0));
  Helper.Observer.changedInstr(MI);
  return true;
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
  MachineIRBuilder &Builder = Helper.MIRBuilder;

  Register Dst = MI.getOperand(0).getReg();
  CmpInst::Predicate Pred =
      static_cast<CmpInst::Predicate>(MI.getOperand(1).getPredicate());
  Register LHS = MI.getOperand(2).getReg();
  Register RHS = MI.getOperand(3).getReg();

  // Implement most comparisons in terms of EQ, UGE, and SLT, as these can be
  // implemented directly via 6502 flags.
  switch (Pred) {
  case CmpInst::ICMP_NE:
  case CmpInst::ICMP_ULT:
  case CmpInst::ICMP_SGE:
    negateInverseComparison(Helper, MI);
    return true;
  case CmpInst::ICMP_ULE:
  case CmpInst::ICMP_UGT:
  case CmpInst::ICMP_SLE:
  case CmpInst::ICMP_SGT:
    if (adjustConstRHS(Helper, MI))
      return true;
    swapComparison(Helper, MI);
    return true;
  default:
    break;
  }

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

  LLT S1 = LLT::scalar(1);
  LLT S8 = LLT::scalar(8);

  bool RHSIsZero = mi_match(RHS, MRI, m_SpecificICst(0));
  Register CIn;

  if (Type != S8) {
    if (Pred != CmpInst::ICMP_SLT) {
      Register LHSHigh, LHSRest;
      Register RHSHigh, RHSRest;
      std::tie(LHSHigh, LHSRest) = splitHighRest(LHS, Builder);
      std::tie(RHSHigh, RHSRest) = splitHighRest(RHS, Builder);

      auto EqHigh = Builder.buildICmp(CmpInst::ICMP_EQ, S1, LHSHigh, RHSHigh);
      // If EqHigh is false, we defer to CmpHigh, which is equal to EqHigh if
      // Pred==ICMP_EQ.
      auto CmpHigh = (Pred == CmpInst::ICMP_EQ)
                         ? Builder.buildConstant(S1, 0)
                         : Builder.buildICmp(Pred, S1, LHSHigh, RHSHigh);
      auto RestPred = Pred;
      if (CmpInst::isSigned(RestPred))
        RestPred = CmpInst::getUnsignedPredicate(Pred);
      auto CmpRest =
          Builder.buildICmp(RestPred, S1, LHSRest, RHSRest).getReg(0);

      // If the high byte is equal, defer to the unsigned comparison on the
      // rest. Otherwise, defer to the comparison on the high byte.
      Builder.buildSelect(Dst, EqHigh, CmpRest, CmpHigh);
      MI.eraseFromParent();
      return true;
    }

    auto LHSUnmerge = Builder.buildUnmerge(S8, LHS);
    auto LHSUnmergeDefs = unmergeDefsSplitHigh(LHSUnmerge);

    // Determining whether the LHS is negative only requires looking at the
    // highest byte (bit, really).
    if (RHSIsZero) {
      Helper.Observer.changingInstr(MI);
      MI.getOperand(2).setReg(LHSUnmergeDefs.High.getReg());
      MI.getOperand(3).setReg(Builder.buildConstant(S8, 0).getReg(0));
      Helper.Observer.changedInstr(MI);
      return true;
    }

    // Perform multibyte signed comparisons by a multibyte subtraction.
    auto RHSUnmerge = Builder.buildUnmerge(S8, RHS);
    auto RHSUnmergeDefs = unmergeDefsSplitHigh(RHSUnmerge);
    assert(LHSUnmerge->getNumOperands() == RHSUnmerge->getNumOperands());
    CIn = Builder.buildConstant(S1, 1).getReg(0);
    // TODO: C++17 structured bindings
    for (const auto &I : zip(LHSUnmergeDefs.Lows, RHSUnmergeDefs.Lows)) {
      auto Sbc = Builder.buildInstr(MOS::G_SBC, {S8, S1, S1, S1, S1},
                                    {std::get<0>(I), std::get<1>(I), CIn});
      CIn = Sbc.getReg(1);
    }
    Type = S8;
    LHS = LHSUnmergeDefs.High.getReg();
    RHS = RHSUnmergeDefs.High.getReg();
    // Fall through to produce the final SBC that determines the comparison
    // result.
  } else {
    CIn = Builder.buildConstant(S1, 1).getReg(0);
  }

  assert(Type == S8);

  // Lower 8-bit comparisons to a generic G_SBC instruction with similar
  // capabilities to the 6502's SBC and CMP instructions.  See
  // www.6502.org/tutorials/compare_beyond.html.
  switch (Pred) {
  case CmpInst::ICMP_EQ: {
    auto Sbc =
        Builder.buildInstr(MOS::G_SBC, {S8, S1, S1, S1, S1}, {LHS, RHS, CIn});
    Builder.buildCopy(Dst, Sbc.getReg(4) /*=Z*/);
    MI.eraseFromParent();
    break;
  }
  case CmpInst::ICMP_UGE: {
    auto Sbc =
        Builder.buildInstr(MOS::G_SBC, {S8, S1, S1, S1, S1}, {LHS, RHS, CIn});
    Builder.buildCopy(Dst, Sbc.getReg(1) /*=C*/);
    MI.eraseFromParent();
    break;
  }
  case CmpInst::ICMP_SLT: {
    // Subtractions of zero cannot overflow, so N is always correct.
    if (RHSIsZero) {
      auto Sbc =
          Builder.buildInstr(MOS::G_SBC, {S8, S1, S1, S1, S1}, {LHS, RHS, CIn});
      Builder.buildCopy(Dst, Sbc.getReg(2) /*=N*/);
    } else {
      // General subtractions can overflow; if so, N is flipped.
      auto Sbc =
          Builder.buildInstr(MOS::G_SBC, {S8, S1, S1, S1, S1}, {LHS, RHS, CIn});
      // The quickest way to XOR N with V is to XOR the accumulator with 0x80
      // iff V, then reexamine N of the accumulator.
      auto Eor = Builder.buildXor(S8, Sbc, Builder.buildConstant(S8, 0x80));
      auto Zero = Builder.buildConstant(S8, 0);
      auto One = Builder.buildConstant(S1, 1);
      Builder.buildInstr(
          MOS::G_SBC, {S8, S1, Dst /*=N*/, S1, S1},
          {Builder.buildSelect(S8, Sbc.getReg(3) /*=V*/, Eor, Sbc), Zero, One});
    }
    MI.eraseFromParent();
    break;
  }
  default:
    llvm_unreachable("Unexpected integer comparison type.");
  }

  return true;
}

bool MOSLegalizerInfo::legalizeSelect(LegalizerHelper &Helper,
                                      MachineRegisterInfo &MRI,
                                      MachineInstr &MI) const {
  MachineIRBuilder &Builder = Helper.MIRBuilder;

  LLT P = LLT::pointer(0, 16);
  LLT S16 = LLT::scalar(16);

  Register Dst = MI.getOperand(0).getReg();
  Register Test = MI.getOperand(1).getReg();
  Register LHS = MI.getOperand(2).getReg();
  Register RHS = MI.getOperand(3).getReg();

  assert(MRI.getType(Dst) == P);
  assert(MRI.getType(Test) == LLT::scalar(1));
  assert(MRI.getType(LHS) == P);
  assert(MRI.getType(RHS) == P);

  Helper.Observer.changingInstr(MI);
  MI.getOperand(2).setReg(Builder.buildPtrToInt(S16, LHS).getReg(0));
  MI.getOperand(3).setReg(Builder.buildPtrToInt(S16, RHS).getReg(0));
  Register Tmp = MRI.createGenericVirtualRegister(S16);
  MI.getOperand(0).setReg(Tmp);
  Helper.Observer.changedInstr(MI);

  Builder.setInsertPt(Builder.getMBB(), std::next(Builder.getInsertPt()));
  Builder.buildIntToPtr(Dst, Tmp);
  return true;
}

bool MOSLegalizerInfo::legalizeAbs(LegalizerHelper &Helper,
                                   MachineRegisterInfo &MRI,
                                   MachineInstr &MI) const {
  MachineIRBuilder &Builder = Helper.MIRBuilder;

  Register Arg = MI.getOperand(1).getReg();
  LLT Ty = MRI.getType(Arg);
  assert(MRI.getType(MI.getOperand(0).getReg()) == Ty);

  auto IsNeg = Builder.buildICmp(CmpInst::ICMP_SLT, LLT::scalar(1), Arg,
                                 Builder.buildConstant(Ty, 0));
  Builder.buildSelect(MI.getOperand(0), IsNeg, Builder.buildNeg(Ty, Arg), Arg);
  MI.eraseFromParent();
  return true;
}

bool MOSLegalizerInfo::legalizePtrAdd(LegalizerHelper &Helper,
                                      MachineRegisterInfo &MRI,
                                      MachineInstr &MI) const {
  MachineIRBuilder &Builder = Helper.MIRBuilder;

  Register Result = MI.getOperand(0).getReg();
  Register Base = MI.getOperand(1).getReg();
  Register Offset = MI.getOperand(2).getReg();

  MachineInstr *GlobalBase = getOpcodeDef(G_GLOBAL_VALUE, Base, MRI);
  auto ConstOffset = getIConstantVRegValWithLookThrough(Offset, MRI);

  // Fold constant offsets into global value operand.
  if (GlobalBase && ConstOffset) {
    const MachineOperand &Op = GlobalBase->getOperand(1);
    Builder.buildInstr(G_GLOBAL_VALUE)
        .addDef(Result)
        .addGlobalAddress(Op.getGlobal(),
                          Op.getOffset() + ConstOffset->Value.getSExtValue());
    MI.eraseFromParent();
    return true;
  }

  // Generalized pointer additions must be lowered to 16-bit integer
  // arithmetic.
  LLT S16 = LLT::scalar(16);
  auto PtrVal = Builder.buildPtrToInt(S16, Base);
  auto Sum = Builder.buildAdd(S16, PtrVal, Offset);
  Builder.buildIntToPtr(Result, Sum);
  MI.eraseFromParent();
  return true;
}

// Convert odd versions of generic add/sub to even versions, which can subsume
// the odd versions via a zero carry-in.
bool MOSLegalizerInfo::legalizeAddSubO(LegalizerHelper &Helper,
                                       MachineRegisterInfo &MRI,
                                       MachineInstr &MI) const {
  MachineIRBuilder &Builder = Helper.MIRBuilder;
  LLT S1 = LLT::scalar(1);

  unsigned Opcode;
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Unexpected opcode");
  case G_UADDO:
    Opcode = G_UADDE;
    break;
  case G_SADDO:
    Opcode = G_SADDE;
    break;
  case G_USUBO:
    Opcode = G_USUBE;
    break;
  case G_SSUBO:
    Opcode = G_SSUBE;
    break;
  }

  Builder.buildInstr(
      Opcode, {MI.getOperand(0), MI.getOperand(1)},
      {MI.getOperand(2), MI.getOperand(3), Builder.buildConstant(S1, 0)});
  MI.eraseFromParent();
  return true;
}

bool MOSLegalizerInfo::legalizeSubE(LegalizerHelper &Helper,
                                    MachineRegisterInfo &MRI,
                                    MachineInstr &MI) const {
  MachineIRBuilder &Builder = Helper.MIRBuilder;
  LLT S1 = LLT::scalar(1);
  LLT S8 = LLT::scalar(8);

  auto CarryIn = Builder.buildNot(S1, MI.getOperand(4));
  if (MI.getOpcode() == MOS::G_USUBE) {
    auto Sbc =
        Builder.buildInstr(MOS::G_SBC, {S8, S1, S1, S1, S1},
                           {MI.getOperand(2), MI.getOperand(3), CarryIn});
    Builder.setInsertPt(Builder.getMBB(), std::next(Builder.getInsertPt()));
    Builder.buildCopy(MI.getOperand(0), Sbc.getReg(0));
    Builder.buildNot(MI.getOperand(1), Sbc.getReg(1) /*=C*/);
  } else {
    assert(MI.getOpcode() == MOS::G_SSUBE);
    auto Sbc =
        Builder.buildInstr(MOS::G_SBC, {S8, S1, S1, S1, S1},
                           {MI.getOperand(2), MI.getOperand(3), CarryIn});
    Builder.setInsertPt(Builder.getMBB(), std::next(Builder.getInsertPt()));
    Builder.buildCopy(MI.getOperand(0), Sbc.getReg(0));
    Builder.buildCopy(MI.getOperand(1), Sbc.getReg(3) /*=V*/);
  }

  MI.eraseFromParent();
  return true;
}

//===----------------------------------------------------------------------===//
// Memory Operations
//===----------------------------------------------------------------------===//

// Load pointers by loading a 16-bit integer, then converting to pointer. This
// allows the 16-bit loads to be reduced to a pair of 8-bit loads.
bool MOSLegalizerInfo::legalizeLoad(LegalizerHelper &Helper,
                                    MachineRegisterInfo &MRI,
                                    MachineInstr &MI) const {
  if (MI.getOpcode() == G_LOAD &&
      MRI.getType(MI.getOperand(0).getReg()) == LLT::scalar(8))
    return selectAddressingMode(Helper, MRI, MI);

  MachineIRBuilder &Builder = Helper.MIRBuilder;
  Builder.setInsertPt(Builder.getMBB(), std::next(Builder.getInsertPt()));
  Register Tmp;
  const MachineMemOperand &MMO = **MI.memoperands_begin();
  switch (MI.getOpcode()) {
  case MOS::G_SEXTLOAD:
    Tmp = MRI.createGenericVirtualRegister(MMO.getType());
    Builder.buildSExt(MI.getOperand(0), Tmp);
    break;
  case MOS::G_ZEXTLOAD:
    Tmp = MRI.createGenericVirtualRegister(MMO.getType());
    Builder.buildZExt(MI.getOperand(0), Tmp);
    break;
  default:
    assert(MRI.getType(MI.getOperand(0).getReg()).isPointer());
    Tmp = MRI.createGenericVirtualRegister(LLT::scalar(16));
    Builder.buildIntToPtr(MI.getOperand(0), Tmp);
    break;
  }
  Helper.Observer.changingInstr(MI);
  MI.setDesc(Builder.getTII().get(MOS::G_LOAD));
  MI.getOperand(0).setReg(Tmp);
  Helper.Observer.changedInstr(MI);
  return true;
}

// Converts pointer to integer before store, allowing the store to later be
// narrowed to 8 bits.
bool MOSLegalizerInfo::legalizeStore(LegalizerHelper &Helper,
                                     MachineRegisterInfo &MRI,
                                     MachineInstr &MI) const {
  if (MRI.getType(MI.getOperand(0).getReg()) == LLT::scalar(8))
    return selectAddressingMode(Helper, MRI, MI);

  MachineIRBuilder &Builder = Helper.MIRBuilder;
  Register Tmp =
      Builder.buildPtrToInt(LLT::scalar(16), MI.getOperand(0)).getReg(0);
  Helper.Observer.changingInstr(MI);
  MI.getOperand(0).setReg(Tmp);
  Helper.Observer.changedInstr(MI);
  return true;
}

static bool willBeStaticallyAllocated(const MachineOperand &MO) {
  assert(MO.isFI());
  if (!MO.getParent()->getMF()->getFunction().doesNotRecurse())
    return false;
  return !MO.getParent()->getMF()->getFrameInfo().isFixedObjectIndex(
      MO.getIndex());
}

bool MOSLegalizerInfo::selectAddressingMode(LegalizerHelper &Helper,
                                            MachineRegisterInfo &MRI,
                                            MachineInstr &MI) const {
  if (tryAbsoluteAddressing(Helper, MRI, MI))
    return true;
  if (tryAbsoluteIndexedAddressing(Helper, MRI, MI))
    return true;
  return selectIndirectIndexedAddressing(Helper, MRI, MI);
}
bool MOSLegalizerInfo::tryAbsoluteAddressing(LegalizerHelper &Helper,
                                             MachineRegisterInfo &MRI,
                                             MachineInstr &MI) const {
  MachineIRBuilder &Builder = Helper.MIRBuilder;

  bool IsLoad = MI.getOpcode() == G_LOAD;
  assert(IsLoad || MI.getOpcode() == G_STORE);

  Register Addr = MI.getOperand(1).getReg();
  int64_t Offset = 0;

  unsigned Opcode = IsLoad ? MOS::G_LOAD_ABS : MOS::G_STORE_ABS;

  while (true) {
    if (auto ConstAddr = getIConstantVRegValWithLookThrough(Addr, MRI)) {
      Helper.Observer.changingInstr(MI);
      MI.setDesc(Builder.getTII().get(Opcode));
      MI.getOperand(1).ChangeToImmediate(Offset +
                                         ConstAddr->Value.getSExtValue());
      Helper.Observer.changedInstr(MI);
      return true;
    }
    if (const MachineInstr *GVAddr = getOpcodeDef(G_GLOBAL_VALUE, Addr, MRI)) {
      Helper.Observer.changingInstr(MI);
      MI.setDesc(Builder.getTII().get(Opcode));
      const MachineOperand &GV = GVAddr->getOperand(1);
      MI.getOperand(1).ChangeToGA(GV.getGlobal(), GV.getOffset() + Offset);
      Helper.Observer.changedInstr(MI);
      return true;
    }
    if (const MachineInstr *FIAddr = getOpcodeDef(G_FRAME_INDEX, Addr, MRI)) {
      const MachineOperand &FI = FIAddr->getOperand(1);
      if (willBeStaticallyAllocated(FI)) {
        Helper.Observer.changingInstr(MI);
        MI.setDesc(Builder.getTII().get(Opcode));
        MI.getOperand(1).ChangeToFrameIndex(FI.getIndex(),
                                            FI.getOffset() + Offset);
        Helper.Observer.changedInstr(MI);
        return true;
      }
    }
    if (const MachineInstr *PtrAddAddr = getOpcodeDef(G_PTR_ADD, Addr, MRI)) {
      Register Base = PtrAddAddr->getOperand(1).getReg();
      Register NewOffset = PtrAddAddr->getOperand(2).getReg();
      auto ConstOffset = getIConstantVRegValWithLookThrough(NewOffset, MRI);
      if (!ConstOffset)
        return false;
      Offset += ConstOffset->Value.getSExtValue();
      Addr = Base;
      continue;
    }
    return false;
  }
  return false;
}

bool MOSLegalizerInfo::tryAbsoluteIndexedAddressing(LegalizerHelper &Helper,
                                                    MachineRegisterInfo &MRI,
                                                    MachineInstr &MI) const {
  MachineIRBuilder &Builder = Helper.MIRBuilder;

  // Page crossing 6502 bug may generate spurious access to hardware registers.
  if ((*MI.memoperands_begin())->isVolatile())
    return false;

  bool IsLoad = MI.getOpcode() == G_LOAD;
  assert(IsLoad || MI.getOpcode() == G_STORE);

  Register Addr = MI.getOperand(1).getReg();
  int64_t Offset = 0;
  Register Index = 0;

  unsigned Opcode = IsLoad ? MOS::G_LOAD_ABS_IDX : MOS::G_STORE_ABS_IDX;

  while (true) {
    if (auto ConstAddr = getIConstantVRegValWithLookThrough(Addr, MRI)) {
      assert(Index); // Otherwise, Absolute addressing would have been selected.
      Builder.buildInstr(Opcode)
          .add(MI.getOperand(0))
          .addImm(ConstAddr->Value.getSExtValue() + Offset)
          .addUse(Index)
          .addMemOperand(*MI.memoperands_begin());
      MI.eraseFromParent();
      return true;
    }
    if (const MachineInstr *GVAddr = getOpcodeDef(G_GLOBAL_VALUE, Addr, MRI)) {
      assert(Index); // Otherwise, Absolute addressing would have been selected.
      const MachineOperand &GV = GVAddr->getOperand(1);
      Builder.buildInstr(Opcode)
          .add(MI.getOperand(0))
          .addGlobalAddress(GV.getGlobal(), GV.getOffset() + Offset)
          .addUse(Index)
          .addMemOperand(*MI.memoperands_begin());
      MI.eraseFromParent();
      return true;
    }
    if (const MachineInstr *FIAddr = getOpcodeDef(G_FRAME_INDEX, Addr, MRI)) {
      const MachineOperand &FI = FIAddr->getOperand(1);
      if (willBeStaticallyAllocated(FI)) {
        assert(
            Index); // Otherwise, Absolute addressing would have been selected.
        Builder.buildInstr(Opcode)
            .add(MI.getOperand(0))
            .addFrameIndex(FI.getIndex(), FI.getOffset() + Offset)
            .addUse(Index)
            .addMemOperand(*MI.memoperands_begin());
        MI.eraseFromParent();
        return true;
      }
    }
    if (const MachineInstr *PtrAddAddr = getOpcodeDef(G_PTR_ADD, Addr, MRI)) {
      Register Base = PtrAddAddr->getOperand(1).getReg();
      Register NewOffset = PtrAddAddr->getOperand(2).getReg();
      if (auto ConstOffset =
              getIConstantVRegValWithLookThrough(NewOffset, MRI)) {
        Offset += ConstOffset->Value.getSExtValue();
        Addr = Base;
        continue;
      }
      if (MachineInstr *ZExtOffset = getOpcodeDef(G_ZEXT, NewOffset, MRI)) {
        if (Index)
          return false;

        Register Src = ZExtOffset->getOperand(1).getReg();
        LLT SrcTy = MRI.getType(Src);
        if (SrcTy.getSizeInBits() > 8)
          return false;
        if (SrcTy.getSizeInBits() < 8)
          Src = Builder.buildZExt(LLT::scalar(8), Src).getReg(0);
        assert(MRI.getType(Src) == LLT::scalar(8));
        Index = Src;
        Addr = Base;
        continue;
      }
    }
    return false;
  }
  return false;
}

bool MOSLegalizerInfo::selectIndirectIndexedAddressing(LegalizerHelper &Helper,
                                                       MachineRegisterInfo &MRI,
                                                       MachineInstr &MI) const {
  MachineIRBuilder &Builder = Helper.MIRBuilder;

  bool IsLoad = MI.getOpcode() == G_LOAD;
  assert(IsLoad || MI.getOpcode() == G_STORE);

  Register Addr = MI.getOperand(1).getReg();
  Register Index = 0;

  unsigned Opcode = IsLoad ? MOS::G_LOAD_INDIR_IDX : MOS::G_STORE_INDIR_IDX;

  // Page crossing 6502 bug may generate spurious access to hardware registers.
  if (!(*MI.memoperands_begin())->isVolatile()) {
    if (const MachineInstr *PtrAddAddr = getOpcodeDef(G_PTR_ADD, Addr, MRI)) {
      Register Base = PtrAddAddr->getOperand(1).getReg();
      Register NewOffset = PtrAddAddr->getOperand(2).getReg();
      if (auto ConstOffset =
              getIConstantVRegValWithLookThrough(NewOffset, MRI)) {
        if (ConstOffset->Value.getActiveBits() <= 8) {
          Index = Builder
                      .buildConstant(LLT::scalar(8),
                                     ConstOffset->Value.getSExtValue())
                      .getReg(0);
          Addr = Base;
        }
      } else if (MachineInstr *ZExtOffset =
                     getOpcodeDef(G_ZEXT, NewOffset, MRI)) {
        Register Src = ZExtOffset->getOperand(1).getReg();
        LLT SrcTy = MRI.getType(Src);
        if (SrcTy.getSizeInBits() <= 8) {
          if (SrcTy.getSizeInBits() < 8)
            Src = Builder.buildZExt(LLT::scalar(8), Src).getReg(0);
          assert(MRI.getType(Src) == LLT::scalar(8));
          Index = Src;
          Addr = Base;
        }
      }
    }
  }

  if (!Index)
    Index = Builder.buildConstant(LLT::scalar(8), 0).getReg(0);
  Builder.buildInstr(Opcode)
      .add(MI.getOperand(0))
      .addUse(Addr)
      .addUse(Index)
      .addMemOperand(*MI.memoperands_begin());
  MI.eraseFromParent();
  return true;
}

//===----------------------------------------------------------------------===//
// Control Flow
//===----------------------------------------------------------------------===//

bool MOSLegalizerInfo::legalizePhi(LegalizerHelper &Helper,
                                   MachineRegisterInfo &MRI,
                                   MachineInstr &MI) const {
  LLT S16 = LLT::scalar(16);
  MachineIRBuilder &Builder = Helper.MIRBuilder;

  Helper.Observer.changingInstr(MI);
  for (unsigned I = 1, IE = MI.getNumOperands(); I < IE; I += 2) {
    Register Reg = MI.getOperand(I).getReg();
    MachineBasicBlock *Block = MI.getOperand(I + 1).getMBB();
    Builder.setInsertPt(*Block, Block->getFirstTerminator());
    MI.getOperand(I).setReg(Builder.buildPtrToInt(S16, Reg).getReg(0));
  }
  Register Tmp = MRI.createGenericVirtualRegister(S16);
  Builder.setInsertPt(*MI.getParent(), MI.getParent()->getFirstNonPHI());
  Builder.buildIntToPtr(MI.getOperand(0).getReg(), Tmp);
  MI.getOperand(0).setReg(Tmp);
  Helper.Observer.changedInstr(MI);
  return true;
}

bool MOSLegalizerInfo::legalizeBrCond(LegalizerHelper &Helper,
                                      MachineRegisterInfo &MRI,
                                      MachineInstr &MI) const {
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

bool MOSLegalizerInfo::legalizeBrJt(LegalizerHelper &Helper,
                                    MachineRegisterInfo &MRI,
                                    MachineInstr &MI) const {
  LLT S8 = LLT::scalar(8);
  MachineIRBuilder &Builder = Helper.MIRBuilder;

  const MachineInstr *Base =
      getOpcodeDef(G_JUMP_TABLE, MI.getOperand(0).getReg(), MRI);
  assert(Base && "Invalid first argument to G_BRJT; expected G_JUMP_TABLE.");

  assert(MI.getOperand(1).isJTI());
  assert(MI.getOperand(1).getIndex() == Base->getOperand(1).getIndex() &&
         "Expected G_JUMP_TABLE to have same index.");

  // Note: Jump table size is hard-limited to 256 entries.
  Register Offset = Builder.buildTrunc(S8, MI.getOperand(2)).getReg(0);

  Register LoAddr = MRI.createGenericVirtualRegister(S8);
  Builder.buildInstr(MOS::G_LOAD_ABS_IDX)
      .addDef(LoAddr)
      .add(MI.getOperand(1))
      .addUse(Offset);
  Register HiAddr = MRI.createGenericVirtualRegister(S8);
  Builder.buildInstr(MOS::G_LOAD_ABS_IDX)
      .addDef(HiAddr)
      .addJumpTableIndex(MI.getOperand(1).getIndex(), MOS::MO_HI_JT)
      .addUse(Offset);
  Builder.buildBrIndirect(
      Builder.buildMerge(LLT::pointer(0, 16), {LoAddr, HiAddr}).getReg(0));

  MI.eraseFromParent();
  return true;
}

//===----------------------------------------------------------------------===//
// Variadic Arguments
//===----------------------------------------------------------------------===//

// Lower variable argument access intrinsic.
bool MOSLegalizerInfo::legalizeVAArg(LegalizerHelper &Helper,
                                     MachineRegisterInfo &MRI,
                                     MachineInstr &MI) const {
  MachineIRBuilder &Builder = Helper.MIRBuilder;
  MachineFunction &MF = Builder.getMF();

  Register Dst = MI.getOperand(0).getReg();
  Register VaListPtr = MI.getOperand(1).getReg();

  LLT P = LLT::pointer(0, 16);
  LLT S16 = LLT::scalar(16);

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
  auto NextAddr =
      Builder.buildPtrAdd(P, Addr, Builder.buildConstant(S16, Size));
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
  LLT P = LLT::pointer(0, 16);

  // Store the address of the fake varargs frame index into the valist.
  MachineIRBuilder &Builder = Helper.MIRBuilder;
  auto *FuncInfo = Builder.getMF().getInfo<MOSFunctionInfo>();
  Builder.buildStore(
      Builder.buildFrameIndex(P, FuncInfo->getVarArgsStackIndex()),
      MI.getOperand(0), **MI.memoperands_begin());
  MI.eraseFromParent();
  return true;
}

bool MOSLegalizerInfo::legalizeDynStackAlloc(LegalizerHelper &Helper,
                                             MachineRegisterInfo &MRI,
                                             MachineInstr &MI) const {
  MachineIRBuilder &Builder = Helper.MIRBuilder;
  Register Dst = MI.getOperand(0).getReg();
  Register AllocSize = MI.getOperand(1).getReg();
  Align Alignment = assumeAligned(MI.getOperand(2).getImm());

  LLT PtrTy = MRI.getType(Dst);
  LLT IntPtrTy = LLT::scalar(PtrTy.getSizeInBits());

  auto SPTmp = Builder.buildCopy(PtrTy, Register(MOS::RS0));
  SPTmp = Builder.buildCast(IntPtrTy, SPTmp);

  // Subtract the final alloc from the SP. We use G_PTRTOINT here so we don't
  // have to generate an extra instruction to negate the alloc and then use
  // G_PTR_ADD to add the negative offset.
  auto Alloc = Builder.buildSub(IntPtrTy, SPTmp, AllocSize);
  if (Alignment > Align(1)) {
    APInt AlignMask(IntPtrTy.getSizeInBits(), Alignment.value(), true);
    AlignMask.negate();
    auto AlignCst = Builder.buildConstant(IntPtrTy, AlignMask);
    Alloc = Builder.buildAnd(IntPtrTy, Alloc, AlignCst);
  }

  SPTmp = Builder.buildCast(PtrTy, Alloc);
  Builder.buildCopy(MOS::RS0, SPTmp);
  Builder.buildCopy(Dst, SPTmp);

  MI.eraseFromParent();
  return true;
}

bool MOSLegalizerInfo::legalizeFreeze(LegalizerHelper &Helper,
                                      MachineRegisterInfo &MRI,
                                      MachineInstr &MI) const {
  // G_FREEZE is lowered to COPY here to ensure getDefSrcRegIgnoringCopies
  // will work across the entire function during instruction selection.
  Helper.Observer.changingInstr(MI);
  MI.setDesc(Helper.MIRBuilder.getTII().get(COPY));
  Helper.Observer.changedInstr(MI);
  return true;
}
