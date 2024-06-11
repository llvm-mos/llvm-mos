//===-- MOSCombiner.cpp - MOS GlobalIsel Combiner -------------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MOS global machine instruction combiner.
//
// This runs between phases of the GlobalISel process to optimize away
// inefficient patterns discovered in the global machine instructions.
//
//===----------------------------------------------------------------------===//

#include "MOSCombiner.h"

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "MOSLegalizerInfo.h"
#include "MOSSubtarget.h"

#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/CodeGen/GlobalISel/CSEInfo.h"
#include "llvm/CodeGen/GlobalISel/Combiner.h"
#include "llvm/CodeGen/GlobalISel/CombinerHelper.h"
#include "llvm/CodeGen/GlobalISel/CombinerInfo.h"
#include "llvm/CodeGen/GlobalISel/GIMatchTableExecutor.h"
#include "llvm/CodeGen/GlobalISel/GIMatchTableExecutorImpl.h"
#include "llvm/CodeGen/GlobalISel/GISelChangeObserver.h"
#include "llvm/CodeGen/GlobalISel/GISelKnownBits.h"
#include "llvm/CodeGen/GlobalISel/GenericMachineInstrs.h"
#include "llvm/CodeGen/GlobalISel/LegalizerHelper.h"
#include "llvm/CodeGen/GlobalISel/MIPatternMatch.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineDominators.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetOpcodes.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/PatternMatch.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Target/TargetMachine.h"

#define GET_GICOMBINER_DEPS
#include "MOSGenGICombiner.inc"
#undef GET_GICOMBINER_DEPS

#define DEBUG_TYPE "mos-combiner"

using namespace llvm;

namespace {

#define GET_GICOMBINER_TYPES
#include "MOSGenGICombiner.inc"
#undef GET_GICOMBINER_TYPES

class MOSCombinerImpl : public Combiner {
  // TODO: Make CombinerHelper methods const.
  mutable CombinerHelper Helper;
  const MOSCombinerImplRuleConfig &RuleConfig;
  AAResults *AA;

public:
  MOSCombinerImpl(MachineFunction &MF, CombinerInfo &CInfo,
                  const TargetPassConfig *TPC, bool IsPreLegalize,
                  GISelKnownBits &KB, GISelCSEInfo *CSEInfo,
                  const MOSCombinerImplRuleConfig &RuleConfig,
                  const MOSSubtarget &STI, MachineDominatorTree *MDT,
                  const LegalizerInfo *LI, AAResults *AA);

  static const char *getName() { return "MOSCombiner"; }

  bool tryCombineAll(MachineInstr &I) const override;

  // G_PTR_ADD (GLOBAL_VALUE @x + y_const), z_const =>
  // GLOBAL_VALUE @x + (y_const + z_const)
  bool matchFoldGlobalOffset(
      MachineInstr &MI,
      std::pair<const MachineOperand *, int64_t> &MatchInfo) const;
  // G_PTR_ADD (GLOBAL_VALUE @x + y_const), z_const =>
  // GLOBAL_VALUE @x + (y_const + z_const)
  void applyFoldGlobalOffset(
      MachineInstr &MI,
      std::pair<const MachineOperand *, int64_t> &MatchInfo) const;

  bool matchSBCEqual(MachineInstr &MI) const;
  void applySBCEqual(MachineInstr &MI) const;

  bool matchExtractLowBit(MachineInstr &MI, MachineInstr *&Shift) const;
  void applyExtractLowBit(MachineInstr &MI, MachineInstr *&Shift) const;

  bool matchUAddO1(MachineInstr &MI) const;
  void applyUAddO1(MachineInstr &MI) const;

  bool matchCMPZZero(MachineInstr &MI, MachineOperand *&Zero) const;
  void applyCMPZZero(MachineInstr &MI, MachineOperand *&Zero) const;

  // G_LOAD/G_STORE pair => G_MEMCPY_INLINE
  bool matchLoadStoreToMemcpy(MachineInstr &MI, GLoad *&Load) const;
  void applyLoadStoreToMemcpy(MachineInstr &MI, GLoad *&Load) const;

  // G_STORE => G_MEMSET
  bool matchStoreToMemset(MachineInstr &MI, uint8_t &Value) const;
  void applyStoreToMemset(MachineInstr &MI, uint8_t &Value) const;

  bool matchFoldAddE(MachineInstr &MI, BuildFnTy &MatchInfo) const;
  bool matchFoldSbc(MachineInstr &MI, BuildFnTy &MatchInfo) const;
  bool matchFoldShift(MachineInstr &MI, BuildFnTy &MatchInfo) const;

  bool matchShiftUnusedCarryIn(MachineInstr &MI, BuildFnTy &MatchInfo) const;

  bool matchMulToShiftAndAdd(MachineInstr &MI, BuildFnTy &MatchInfo) const;

  APInt getDemandedBits(Register R) const;
  APInt getDemandedBits(Register R, DenseMap<Register, APInt> &Cache) const;

private:
#define GET_GICOMBINER_CLASS_MEMBERS
#include "MOSGenGICombiner.inc"
#undef GET_GICOMBINER_CLASS_MEMBERS
};

#define GET_GICOMBINER_IMPL
#include "MOSGenGICombiner.inc"
#undef GET_GICOMBINER_IMPL

MOSCombinerImpl::MOSCombinerImpl(
    MachineFunction &MF, CombinerInfo &CInfo, const TargetPassConfig *TPC,
    bool IsPreLegalize, GISelKnownBits &KB, GISelCSEInfo *CSEInfo,
    const MOSCombinerImplRuleConfig &RuleConfig, const MOSSubtarget &STI,
    MachineDominatorTree *MDT, const LegalizerInfo *LI, AAResults *AA)
    : Combiner(MF, CInfo, TPC, &KB, CSEInfo),
      Helper(Observer, B, IsPreLegalize, &KB, MDT, LI), RuleConfig(RuleConfig),
      AA(AA),
#define GET_GICOMBINER_CONSTRUCTOR_INITS
#include "MOSGenGICombiner.inc"
#undef GET_GICOMBINER_CONSTRUCTOR_INITS
{
}

// G_PTR_ADD (GLOBAL_VALUE @x + y_const), z_const =>
// GLOBAL_VALUE @x + (y_const + z_const)
bool MOSCombinerImpl::matchFoldGlobalOffset(
    MachineInstr &MI,
    std::pair<const MachineOperand *, int64_t> &MatchInfo) const {
  const auto &Add = cast<GPtrAdd>(MI);
  using namespace TargetOpcode;

  MachineInstr *GlobalBase =
      getOpcodeDef(G_GLOBAL_VALUE, Add.getBaseReg(), MRI);
  auto ConstOffset =
      getIConstantVRegValWithLookThrough(Add.getOffsetReg(), MRI);

  if (!GlobalBase || !ConstOffset)
    return false;
  const MachineOperand *BaseGV = &GlobalBase->getOperand(1);
  int64_t NewOffset = BaseGV->getOffset() + ConstOffset->Value.getSExtValue();
  MatchInfo = {BaseGV, NewOffset};
  return true;
}

// G_PTR_ADD (GLOBAL_VALUE @x + y_const), z_const =>
// GLOBAL_VALUE @x + (y_const + z_const)
void MOSCombinerImpl::applyFoldGlobalOffset(
    MachineInstr &MI,
    std::pair<const MachineOperand *, int64_t> &MatchInfo) const {
  using namespace TargetOpcode;
  assert(MI.getOpcode() == G_PTR_ADD);
  const TargetInstrInfo &TII = B.getTII();
  Observer.changingInstr(MI);
  MI.setDesc(TII.get(TargetOpcode::G_GLOBAL_VALUE));
  MI.getOperand(1).ChangeToGA(MatchInfo.first->getGlobal(), MatchInfo.second,
                              MatchInfo.first->getTargetFlags());
  MI.removeOperand(2);
  Observer.changedInstr(MI);
}

bool MOSCombinerImpl::matchSBCEqual(MachineInstr &MI) const {
  assert(MI.getOpcode() == MOS::G_SBC);
  Register LHS = MI.getOperand(5).getReg();
  Register RHS = MI.getOperand(6).getReg();
  Register CarryIn = MI.getOperand(7).getReg();

  auto ConstCarryIn = getIConstantVRegValWithLookThrough(CarryIn, MRI);
  if (!ConstCarryIn)
    return false;
  if (!ConstCarryIn->Value.isAllOnes())
    return false;

  if (LHS == RHS)
    return true;

  auto ConstLHS = getIConstantVRegValWithLookThrough(LHS, MRI);
  auto ConstRHS = getIConstantVRegValWithLookThrough(RHS, MRI);
  if (!ConstLHS || !ConstRHS)
    return false;

  return ConstLHS->Value == ConstRHS->Value;
}

void MOSCombinerImpl::applySBCEqual(MachineInstr &MI) const {
  LLT S1 = LLT::scalar(1);

  B.setInstrAndDebugLoc(MI);
  B.buildCopy(MI.getOperand(0), B.buildConstant(LLT::scalar(8), 0));

  auto True = B.buildConstant(S1, 1);
  auto False = B.buildConstant(S1, 0);
  // C
  B.buildCopy(MI.getOperand(1), True);
  // N
  B.buildCopy(MI.getOperand(2), False);
  // V
  B.buildCopy(MI.getOperand(3), False);
  // Z
  B.buildCopy(MI.getOperand(4), True);
  MI.eraseFromParent();
}

bool MOSCombinerImpl::matchExtractLowBit(MachineInstr &MI,
                                         MachineInstr *&Shift) const {
  using namespace MIPatternMatch;
  Register Src;
  if (MI.getOpcode() == MOS::G_TRUNC) {
    if (MRI.getType(MI.getOperand(0).getReg()) != LLT::scalar(1))
      return false;
    Src = MI.getOperand(1).getReg();
    Register NewSrc;
    if (mi_match(Src, MRI,
                 m_GAnd(m_Reg(NewSrc), MIPatternMatch::m_SpecificICst(1))))
      Src = NewSrc;
  } else {
    assert(MI.getOpcode() == MOS::G_ICMP);
    ICmpInst::Predicate Pred;
    if (!mi_match(MI.getOperand(0).getReg(), MRI,
                  m_GICmp(m_Pred(Pred),
                          m_GAnd(m_Reg(Src), MIPatternMatch::m_SpecificICst(1)),
                          MIPatternMatch::m_SpecificICst(0))))
      return false;
    // The NE case handled automatically via an optimization that converts it to
    // a G_TRUNC.
    if (Pred != CmpInst::ICMP_EQ)
      return false;
  }

  for (MachineInstr &RefMI : MRI.reg_nodbg_instructions(Src)) {
    if (RefMI.getOpcode() != MOS::G_LSHR)
      continue;
    if (RefMI.getOperand(1).getReg() != Src)
      continue;
    auto ConstAmt =
        getIConstantVRegValWithLookThrough(RefMI.getOperand(2).getReg(), MRI);
    if (!ConstAmt || !ConstAmt->Value.isOne())
      continue;
    if (!Helper.dominates(RefMI, MI) && !Helper.dominates(MI, RefMI))
      continue;
    Shift = &RefMI;
    return true;
  }

  return false;
}

void MOSCombinerImpl::applyExtractLowBit(MachineInstr &MI,
                                         MachineInstr *&Shift) const {
  assert(Shift->getOpcode() == MOS::G_LSHR);
  LLT S1 = LLT::scalar(1);

  bool Negate = MI.getOpcode() == MOS::G_ICMP &&
                MI.getOperand(1).getPredicate() == CmpInst::ICMP_EQ;

  if (Helper.dominates(*Shift, MI)) {
    B.setInstrAndDebugLoc(*Shift);
  } else {
    assert(Helper.dominates(MI, *Shift));
    B.setInstrAndDebugLoc(MI);
  }

  auto EvenShift = B.buildInstr(MOS::G_LSHRE, {Shift->getOperand(0), S1},
                                {Shift->getOperand(1), B.buildConstant(S1, 0)});
  if (Negate)
    B.buildNot(MI.getOperand(0).getReg(), EvenShift.getReg(1));
  else
    B.buildCopy(MI.getOperand(0).getReg(), EvenShift.getReg(1));
  MOSLegalizerInfo Legalizer(B.getMF().getSubtarget<MOSSubtarget>());
  LegalizerHelper LegalizerHelper(B.getMF(), Legalizer, Observer, B);
  B.setInstrAndDebugLoc(*EvenShift);
  if (!Legalizer.legalizeLshrEShlE(LegalizerHelper, MRI, *EvenShift))
    llvm_unreachable("Failed to legalize shift.");
  Shift->eraseFromParent();
  MI.eraseFromParent();
}

// Use of the overflow flag from an increment is better done on the 6502 by
// comparing the result to zero, since this allows increment and decrement
// operators instead of just ADC.
bool MOSCombinerImpl::matchUAddO1(MachineInstr &MI) const {
  if (MI.getOpcode() != MOS::G_UADDO)
    return false;
  std::optional<ValueAndVReg> Val =
      getIConstantVRegValWithLookThrough(MI.getOperand(3).getReg(), MRI);
  if (!Val || !Val->Value.isOne())
    return false;
  return true;
}

void MOSCombinerImpl::applyUAddO1(MachineInstr &MI) const {
  LLT Ty = MRI.getType(MI.getOperand(0).getReg());

  Register Dst = MI.getOperand(0).getReg();
  Register Overflow = MI.getOperand(1).getReg();
  Register LHS = MI.getOperand(2).getReg();
  Register RHS = MI.getOperand(3).getReg();

  B.setInsertPt(*MI.getParent(), std::next(MachineBasicBlock::iterator(MI)));
  B.setDebugLoc(MI.getDebugLoc());
  MI.eraseFromParent();
  B.buildAdd(Dst, LHS, RHS);
  B.buildICmp(CmpInst::ICMP_EQ, Overflow, Dst, B.buildConstant(Ty, 0));
}

bool MOSCombinerImpl::matchCMPZZero(MachineInstr &MI,
                                    MachineOperand *&Zero) const {
  if (MI.getOpcode() != MOS::G_CMPZ)
    return false;
  for (unsigned I = 1, E = MI.getNumOperands(); I != E; ++I) {
    MachineOperand &MO = MI.getOperand(I);
    if (std::optional<ValueAndVReg> Val =
            getIConstantVRegValWithLookThrough(MO.getReg(), MRI);
        Val && Val->Value.isZero()) {
      Zero = &MO;
      return true;
    }
  }
  return false;
}

void MOSCombinerImpl::applyCMPZZero(MachineInstr &MI,
                                    MachineOperand *&Zero) const {
  B.setInstrAndDebugLoc(MI);
  auto New = B.buildInstr(MOS::G_CMPZ);
  for (unsigned I = 0, E = MI.getNumOperands(); I != E; ++I) {
    MachineOperand &MO = MI.getOperand(I);
    if (&MO != Zero)
      New.add(MO);
  }
  MI.eraseFromParent();
}

static bool isRepeatingBytePattern(uint64_t Value, uint32_t Bytes) {
  // Support variants like 0x01010101, 0x02020202...
  for (uint32_t I = 1; I < Bytes; I++) {
    if (((Value >> (I * 8)) & 0xFF) != (Value & 0xFF)) {
      return false;
    }
  }
  return true;
}

// G_LOAD/G_STORE pair => G_MEMCPY_INLINE
bool MOSCombinerImpl::matchLoadStoreToMemcpy(MachineInstr &MI,
                                             GLoad *&Load) const {
  const auto *Store = dyn_cast<GStore>(&MI);
  if (!Store)
    return false;

  Load = dyn_cast_or_null<GLoad>(MI.getPrevNode());
  if (!Load)
    return false;
  if (Load->getDstReg() != Store->getValueReg())
    return false;
  if (!Load->isUnordered() || !Store->isUnordered())
    return false;
  if (MI.mayAlias(AA, *Load, true))
    return false;

  return true;
}

void MOSCombinerImpl::applyLoadStoreToMemcpy(MachineInstr &MI,
                                             GLoad *&Load) const {
  const auto &Store = cast<GStore>(MI);
  B.setInstrAndDebugLoc(MI);

  B.buildInstr(
       MOS::G_MEMCPY_INLINE, {},
       {Store.getPointerReg(), Load->getPointerReg(),
        B.buildConstant(LLT::scalar(16),
                        MRI.getType(Store.getValueReg()).getSizeInBytes())})
      .addMemOperand(&Store.getMMO())
      .addMemOperand(&Load->getMMO());

  MI.eraseFromParent();
}

// G_STORE => G_MEMSET (large constant stores of repeating bytes)
bool MOSCombinerImpl::matchStoreToMemset(MachineInstr &MI,
                                         uint8_t &Value) const {
  const auto *Store = dyn_cast<GStore>(&MI);
  if (!Store)
    return false;

  LLT Ty = MRI.getType(Store->getValueReg());
  if (!Ty.isScalar())
    return false;

  uint32_t SrcBits = Ty.getSizeInBits();
  if ((SrcBits & 7) != 0 || SrcBits < 24)
    return false;

  uint32_t SrcBytes = SrcBits >> 3;
  auto SrcValue = getIConstantVRegValWithLookThrough(Store->getValueReg(), MRI);
  if (!SrcValue)
    return false;

  auto SrcUInt64 = SrcValue->Value.getZExtValue();
  if (!isRepeatingBytePattern(SrcUInt64, SrcBytes))
    return false;

  Value = SrcUInt64 & 0xFF;
  return true;
}

void MOSCombinerImpl::applyStoreToMemset(MachineInstr &MI,
                                         uint8_t &Value) const {
  const auto &Store = cast<GStore>(MI);
  B.setInstrAndDebugLoc(MI);

  auto Length = MRI.getType(Store.getValueReg()).getSizeInBytes();

  B.buildInstr(MOS::G_MEMSET, {},
               {Store.getPointerReg(), B.buildConstant(LLT::scalar(8), Value),
                B.buildConstant(LLT::scalar(16), Length), UINT64_C(0)})
      .addMemOperand(&Store.getMMO());

  MI.eraseFromParent();
}

bool MOSCombinerImpl::matchFoldAddE(MachineInstr &MI,
                                    BuildFnTy &MatchInfo) const {
  auto LHS = getIConstantVRegValWithLookThrough(MI.getOperand(2).getReg(), MRI);
  if (!LHS)
    return false;
  auto RHS = getIConstantVRegValWithLookThrough(MI.getOperand(3).getReg(), MRI);
  if (!RHS)
    return false;
  auto CIn = getIConstantVRegValWithLookThrough(MI.getOperand(4).getReg(), MRI);
  if (!CIn)
    return false;

  bool Overflow;
  APInt Result;
  if (MI.getOpcode() == MOS::G_UADDE) {
    Result = LHS->Value.uadd_ov(RHS->Value, Overflow);
    bool O;
    Result = Result.uadd_ov(CIn->Value.zext(Result.getBitWidth()), O);
    Overflow |= O;
  } else {
    Result = LHS->Value.sadd_ov(RHS->Value, Overflow);
    bool O;
    Result = Result.sadd_ov(CIn->Value.zext(Result.getBitWidth()), O);
    Overflow |= O;
  }

  Register Dst = MI.getOperand(0).getReg();
  Register COut = MI.getOperand(1).getReg();
  MatchInfo = [=](MachineIRBuilder &B) {
    B.buildConstant(Dst, Result);
    B.buildConstant(COut, Overflow);
  };
  return true;
}

bool MOSCombinerImpl::matchFoldSbc(MachineInstr &MI,
                                   BuildFnTy &MatchInfo) const {
  auto LHS = getIConstantVRegValWithLookThrough(MI.getOperand(5).getReg(), MRI);
  if (!LHS)
    return false;
  auto RHS = getIConstantVRegValWithLookThrough(MI.getOperand(6).getReg(), MRI);
  if (!RHS)
    return false;
  auto CarryIn =
      getIConstantVRegValWithLookThrough(MI.getOperand(7).getReg(), MRI);
  if (!CarryIn)
    return false;

  APInt NotRHS = ~RHS->Value;

  bool CarryOut;
  APInt Result;
  Result = LHS->Value.uadd_ov(~RHS->Value, CarryOut);
  bool O;
  Result = Result.uadd_ov(CarryIn->Value.zext(Result.getBitWidth()), O);
  CarryOut |= O;

  bool Overflow = LHS->Value.isNegative() == !RHS->Value.isNegative() &&
                  Result.isNegative() != LHS->Value.isNegative();

  bool Negative = Result.isNegative();
  bool Zero = Result.isZero();

  Register Dst = MI.getOperand(0).getReg();
  Register C = MI.getOperand(1).getReg();
  Register N = MI.getOperand(2).getReg();
  Register V = MI.getOperand(3).getReg();
  Register Z = MI.getOperand(4).getReg();
  MatchInfo = [=](MachineIRBuilder &B) {
    B.buildConstant(Dst, Result);
    B.buildConstant(C, CarryOut);
    B.buildConstant(N, Negative);
    B.buildConstant(V, Overflow);
    B.buildConstant(Z, Zero);
  };
  return true;
}

bool MOSCombinerImpl::matchFoldShift(MachineInstr &MI,
                                     BuildFnTy &MatchInfo) const {
  auto Val = getIConstantVRegValWithLookThrough(MI.getOperand(2).getReg(), MRI);
  if (!Val)
    return false;
  assert(Val->Value.getBitWidth() == 8);
  auto CarryIn =
      getIConstantVRegValWithLookThrough(MI.getOperand(3).getReg(), MRI);
  if (!CarryIn)
    return false;

  bool CarryOut;
  APInt Result;

  if (MI.getOpcode() == MOS::G_LSHRE) {
    CarryOut = (Val->Value & 1).getBoolValue();
    Result = Val->Value.lshr(1) | CarryIn->Value.zext(8).shl(7);
  } else {
    CarryOut = (Val->Value & 0x80).getBoolValue();
    Result = Val->Value.shl(1) | CarryIn->Value.zext(8);
  }

  Register Dst = MI.getOperand(0).getReg();
  Register C = MI.getOperand(1).getReg();
  MatchInfo = [=](MachineIRBuilder &B) {
    B.buildConstant(Dst, Result);
    B.buildConstant(C, CarryOut);
  };
  return true;
}

bool MOSCombinerImpl::matchShiftUnusedCarryIn(MachineInstr &MI,
                                              BuildFnTy &MatchInfo) const {
  const auto ConstCarryIn =
      getIConstantVRegValWithLookThrough(MI.getOperand(3).getReg(), MRI);
  if (ConstCarryIn && ConstCarryIn->Value.isZero())
    return false;

  APInt DemandedBits = getDemandedBits(MI.getOperand(0).getReg());
  assert(DemandedBits.getBitWidth() == 8);
  if (MI.getOpcode() == MOS::G_LSHRE) {
    if ((DemandedBits & 0x80).getBoolValue())
      return false;
  } else {
    if ((DemandedBits & 1).getBoolValue())
      return false;
  }

  MatchInfo = [=, &MI](MachineIRBuilder &B) {
    Observer.changingInstr(MI);
    MI.getOperand(3).setReg(B.buildConstant(LLT::scalar(1), 0).getReg(0));
    Observer.changedInstr(MI);
  };
  return true;
}

// Try to transform:
// (1) multiply-by-(power-of-2 +/- 1) into shift and add/sub.
// mul x, (2^N + 1) --> add (shl x, N), x
// mul x, (2^N - 1) --> sub (shl x, N), x
// Examples: x * 33 --> (x << 5) + x
//           x * 15 --> (x << 4) - x
//           x * -33 --> -((x << 5) + x)
//           x * -15 --> -((x << 4) - x) ; this reduces --> x - (x << 4)
// (2) multiply-by-(power-of-2 +/- power-of-2) into shifts and add/sub.
// mul x, (2^N + 2^M) --> (add (shl x, N), (shl x, M))
// mul x, (2^N - 2^M) --> (sub (shl x, N), (shl x, M))
// Examples: x * 0x8800 --> (x << 15) + (x << 11)
//           x * 0xf800 --> (x << 16) - (x << 11)
//           x * -0x8800 --> -((x << 15) + (x << 11))
//           x * -0xf800 --> -((x << 16) - (x << 11)) ; (x << 11) - (x << 16)
bool MOSCombinerImpl::matchMulToShiftAndAdd(MachineInstr &MI,
                                            BuildFnTy &MatchInfo) const {
  LLT Ty = MRI.getType(MI.getOperand(0).getReg());
  Register LHS = MI.getOperand(1).getReg();
  Register RHS = MI.getOperand(2).getReg();

  const auto RHSConstOr = getIConstantVRegValWithLookThrough(RHS, MRI);
  if (!RHSConstOr)
    return false;
  APInt RHSConst = RHSConstOr->Value;

  // This combine seems to overide the base mul to shl combine, so do that work
  // here too.
  unsigned ShiftVal = RHSConst.exactLogBase2();
  if (static_cast<int32_t>(ShiftVal) != -1) {
    MatchInfo = [=, &MI](MachineIRBuilder &B) {
      LLT ShiftTy = MRI.getType(MI.getOperand(0).getReg());
      auto ShiftCst = B.buildConstant(ShiftTy, ShiftVal);
      Observer.changingInstr(MI);
      MI.setDesc(B.getTII().get(MOS::G_SHL));
      MI.getOperand(2).setReg(ShiftCst.getReg(0));
      Observer.changedInstr(MI);
    };
    return true;
  }

  unsigned MathOp;
  APInt MulC = RHSConst.abs();
  // The constant `2` should be treated as (2^0 + 1).
  unsigned TZeros = MulC == 2 ? 0 : MulC.countr_zero();
  MulC.lshrInPlace(TZeros);
  if ((MulC - 1).isPowerOf2())
    MathOp = MOS::G_ADD;
  else if ((MulC + 1).isPowerOf2())
    MathOp = MOS::G_SUB;
  else
    return false;

  unsigned ShAmt =
      MathOp == MOS::G_ADD ? (MulC - 1).logBase2() : (MulC + 1).logBase2();
  ShAmt += TZeros;
  assert(ShAmt < Ty.getScalarSizeInBits() &&
         "multiply-by-constant generated out of bounds shift");
  MatchInfo = [=, &MI](MachineIRBuilder &B) {
    auto Shl = B.buildShl(Ty, LHS, B.buildConstant(Ty, ShAmt));
    auto R = TZeros
                 ? B.buildInstr(
                       MathOp, {Ty},
                       {Shl, B.buildShl(Ty, LHS, B.buildConstant(Ty, TZeros))})
                 : B.buildInstr(MathOp, {Ty}, {Shl, LHS});
    if (RHSConst.isNegative())
      R = B.buildNeg(Ty, R);
    B.buildCopy(MI.getOperand(0).getReg(), R);
    MI.eraseFromParent();
  };
  return true;
}

APInt MOSCombinerImpl::getDemandedBits(Register R) const {
  DenseMap<Register, APInt> Cache;
  return getDemandedBits(R, Cache);
}

APInt MOSCombinerImpl::getDemandedBits(Register R,
                                       DenseMap<Register, APInt> &Cache) const {
  auto It = Cache.find(R);
  if (It != Cache.end())
    return It->second;

  uint64_t Size = MRI.getType(R).getSizeInBits();

  APInt DemandedBits = APInt::getZero(Size);
  for (const MachineOperand &Use : MRI.use_nodbg_operands(R)) {
    const MachineInstr &MI = *Use.getParent();
    switch (MI.getOpcode()) {
    default:
      DemandedBits = APInt::getAllOnes(Size);
      break;
    case MOS::G_AND: {
      APInt Zeroes = KB->getKnownZeroes(
          MI.getOperand(Use.getOperandNo() == 1 ? 2 : 1).getReg());
      DemandedBits |= ~Zeroes;
      break;
    }
    case MOS::G_OR: {
      APInt Ones = KB->getKnownOnes(
          MI.getOperand(Use.getOperandNo() == 1 ? 2 : 1).getReg());
      DemandedBits |= ~Ones;
      break;
    }
    case MOS::G_LSHRE: {
      APInt DstDemandedBits = getDemandedBits(MI.getOperand(0).getReg(), Cache);
      if (Use.getOperandNo() == 2) {
        APInt CarryOutDemanded =
            getDemandedBits(MI.getOperand(1).getReg(), Cache);
        DemandedBits |= DstDemandedBits << 1 | CarryOutDemanded.zext(8);
      } else {
        assert(Use.getOperandNo() == 3);
        DemandedBits |= DstDemandedBits.lshr(7).trunc(1);
      }
      break;
    }
    case MOS::G_SHLE: {
      APInt DstDemandedBits = getDemandedBits(MI.getOperand(0).getReg(), Cache);
      if (Use.getOperandNo() == 2) {
        APInt CarryOutDemanded =
            getDemandedBits(MI.getOperand(1).getReg(), Cache);
        DemandedBits |=
            DstDemandedBits.lshr(1) | (CarryOutDemanded.zext(8) << 7);
      } else {
        assert(Use.getOperandNo() == 3);
        DemandedBits |= DstDemandedBits.trunc(1);
      }
      break;
    }
    }
    if (DemandedBits.isAllOnes())
      break;
  }
  Cache.try_emplace(R, DemandedBits);
  return DemandedBits;
}

// Pass boilerplate
// ================

class MOSCombiner : public MachineFunctionPass {
  MOSCombinerImplRuleConfig RuleConfig;

public:
  static char ID;

  MOSCombiner();

  StringRef getPassName() const override { return "MOSCombiner"; }

  bool runOnMachineFunction(MachineFunction &MF) override;

  void getAnalysisUsage(AnalysisUsage &AU) const override;
};

} // end anonymous namespace

void MOSCombiner::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<TargetPassConfig>();
  AU.setPreservesCFG();
  getSelectionDAGFallbackAnalysisUsage(AU);
  AU.addRequired<GISelKnownBitsAnalysis>();
  AU.addPreserved<GISelKnownBitsAnalysis>();
  AU.addRequired<MachineDominatorTreeWrapperPass>();
  AU.addPreserved<MachineDominatorTreeWrapperPass>();
  AU.addRequired<GISelCSEAnalysisWrapperPass>();
  AU.addPreserved<GISelCSEAnalysisWrapperPass>();
  AU.addRequired<AAResultsWrapperPass>();

  MachineFunctionPass::getAnalysisUsage(AU);
}

MOSCombiner::MOSCombiner() : MachineFunctionPass(ID) {
  initializeMOSCombinerPass(*PassRegistry::getPassRegistry());

  if (!RuleConfig.parseCommandLineOption())
    report_fatal_error("Invalid rule identifier");
}

bool MOSCombiner::runOnMachineFunction(MachineFunction &MF) {
  if (MF.getProperties().hasProperty(
          MachineFunctionProperties::Property::FailedISel))
    return false;

  auto &TPC = getAnalysis<TargetPassConfig>();

  // Enable CSE.
  GISelCSEAnalysisWrapper &Wrapper =
      getAnalysis<GISelCSEAnalysisWrapperPass>().getCSEWrapper();
  auto *CSEInfo = &Wrapper.get(TPC.getCSEConfig());

  const MOSSubtarget &ST = MF.getSubtarget<MOSSubtarget>();
  const auto *LI = ST.getLegalizerInfo();

  const Function &F = MF.getFunction();
  bool EnableOpt =
      MF.getTarget().getOptLevel() != CodeGenOptLevel::None && !skipFunction(F);
  bool IsPreLegalize = !MF.getProperties().hasProperty(
      MachineFunctionProperties::Property::Legalized);
  GISelKnownBits *KB = &getAnalysis<GISelKnownBitsAnalysis>().get(MF);
  MachineDominatorTree *MDT =
      &getAnalysis<MachineDominatorTreeWrapperPass>().getDomTree();
  AAResults *AA = &getAnalysis<AAResultsWrapperPass>().getAAResults();
  CombinerInfo CInfo(
      /*AllowIllegalOps*/ IsPreLegalize, /*ShouldLegalizeIllegal*/ false,
      /*LegalizerInfo*/ nullptr, EnableOpt, F.hasOptSize(), F.hasMinSize());
  MOSCombinerImpl Impl(MF, CInfo, &TPC, IsPreLegalize, *KB, CSEInfo, RuleConfig,
                       ST, MDT, LI, AA);
  return Impl.combineMachineInstrs();
}

char MOSCombiner::ID = 0;
INITIALIZE_PASS_BEGIN(MOSCombiner, DEBUG_TYPE, "Combine MOS machine instrs",
                      false, false)
INITIALIZE_PASS_DEPENDENCY(TargetPassConfig)
INITIALIZE_PASS_DEPENDENCY(GISelKnownBitsAnalysis)
INITIALIZE_PASS_END(MOSCombiner, DEBUG_TYPE, "Combine MOS machine instrs",
                    false, false)

namespace llvm {
FunctionPass *createMOSCombiner() { return new MOSCombiner; }
} // namespace llvm
