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
#include "MOS.h"
#include "MOSFrameLowering.h"
#include "MOSInstrInfo.h"
#include "MOSMachineFunctionInfo.h"
#include "MOSRegisterInfo.h"
#include "MOSSubtarget.h"

#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/GlobalISel/LegalizerHelper.h"
#include "llvm/CodeGen/GlobalISel/LegalizerInfo.h"
#include "llvm/CodeGen/GlobalISel/LostDebugLocObserver.h"
#include "llvm/CodeGen/GlobalISel/MIPatternMatch.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineJumpTableInfo.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/PseudoSourceValue.h"
#include "llvm/CodeGen/Register.h"
#include "llvm/CodeGen/RegisterBankInfo.h"
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
  LLT PZ = LLT::pointer(1, 8);

  // Constants

  getActionDefinitionsBuilder(G_CONSTANT)
      .legalFor({S1, S8, P, PZ})
      .widenScalarToNextMultipleOf(0, 8)
      .maxScalar(0, S8)
      .unsupported();

  getActionDefinitionsBuilder(G_IMPLICIT_DEF)
      .legalFor({S1, S8, P, PZ})
      .widenScalarToNextMultipleOf(0, 8)
      .maxScalar(0, S8)
      .unsupported();

  getActionDefinitionsBuilder({G_GLOBAL_VALUE, G_FRAME_INDEX, G_BLOCK_ADDR})
      .legalFor({P, PZ})
      .unsupported();

  // Integer Extension and Truncation

  getActionDefinitionsBuilder(G_ANYEXT)
      .legalFor({{S16, S8}})
      .customIf(typeIs(1, S1))
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
      .legalFor({{P, S16}, {PZ, S8}})
      .scalarSameSizeAs(1, 0)
      .unsupported();
  getActionDefinitionsBuilder(G_PTRTOINT)
      .legalFor({{S16, P}, {S8, PZ}})
      .scalarSameSizeAs(0, 1)
      .unsupported();
  getActionDefinitionsBuilder(G_ADDRSPACE_CAST)
      .customForCartesianProduct({P, PZ})
      .unsupported();

  // Scalar Operations

  getActionDefinitionsBuilder({G_EXTRACT, G_INSERT}).lower();

  getActionDefinitionsBuilder(G_MERGE_VALUES)
      .legalForCartesianProduct({S16, P}, {S8, PZ})
      .unsupported();
  getActionDefinitionsBuilder(G_UNMERGE_VALUES)
      .legalForCartesianProduct({S8, PZ}, {S16, P})
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

  getActionDefinitionsBuilder(G_MUL)
      .libcallFor({S8, S16, S32, S64})
      .widenScalarToNextPow2(0)
      // Multiplications can only be narrowed to sizes where a multiplication of
      // double that size is legal, since that's the lowered algorithm invokes
      // such multiplications. Lowering S128 to S64 would produce infinite
      // regress because of this, so instead it's lowered to S32.
      .clampScalar(0, S8, S32)
      .unsupported();

  getActionDefinitionsBuilder({G_SDIV, G_SREM, G_UDIV, G_UREM})
      .clampScalar(0, S8, S64)
      .widenScalarToNextPow2(0)
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
      .customFor({P, PZ})
      .legalFor({S1, S8})
      .widenScalarToNextMultipleOf(0, 8)
      .maxScalar(0, S8)
      .unsupported();

  getActionDefinitionsBuilder(G_PTR_ADD)
      .customFor({{P, S16}, {PZ, S8}})
      .scalarSameSizeAs(1, 0)
      .unsupported();
  getActionDefinitionsBuilder(G_PTRMASK)
      .customFor({{P, S16}, {PZ, S8}})
      .scalarSameSizeAs(1, 0)
      .unsupported();

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

  getActionDefinitionsBuilder({G_FADD,
                               G_FSUB,
                               G_FMUL,
                               G_FDIV,
                               G_FMA,
                               G_FREM,
                               G_FPOW,
                               G_FEXP,
                               G_FEXP2,
                               G_FLOG,
                               G_FLOG2,
                               G_FMINNUM,
                               G_FMINNUM,
                               G_FMAXNUM,
                               G_FCEIL,
                               G_FCOS,
                               G_FSIN,
                               G_FSQRT,
                               G_FFLOOR,
                               G_FRINT,
                               G_FNEARBYINT,
                               G_INTRINSIC_ROUND,
                               G_INTRINSIC_TRUNC,
                               G_FMINIMUM,
                               G_FMAXIMUM,
                               G_INTRINSIC_ROUNDEVEN})
      .libcallFor({S32, S64});

  getActionDefinitionsBuilder(G_FABS).custom();

  getActionDefinitionsBuilder({G_FCOPYSIGN, G_IS_FPCLASS}).lower();

  getActionDefinitionsBuilder(G_FCANONICALIZE).custom();

  getActionDefinitionsBuilder(G_FPEXT).libcallFor({{S64, S32}});
  getActionDefinitionsBuilder(G_FPTRUNC).libcallFor({{S32, S64}});

  getActionDefinitionsBuilder(G_FCONSTANT).customFor({S32, S64});

  setFCmpLibcallsGNU();

  getActionDefinitionsBuilder(G_FCMP).customForCartesianProduct({S1},
                                                                {S32, S64});

  getActionDefinitionsBuilder({G_FPTOSI, G_FPTOUI})
      .libcallForCartesianProduct({S32, S64}, {S32, S64})
      .minScalar(0, S32);

  getActionDefinitionsBuilder({G_SITOFP, G_UITOFP})
      .libcallForCartesianProduct({S32, S64}, {S32, S64})
      .minScalar(1, S32);

  // Memory Operations

  getActionDefinitionsBuilder({G_LOAD, G_STORE})
      // Convert to int to load/store; that way the operation can be narrowed to
      // 8 bits. Once 8-bit, select an addressing mode to replace the generic
      // G_LOAD and G_STORE.
      .customForCartesianProduct({S8, PZ, P}, {PZ, P})
      .widenScalarToNextMultipleOf(0, 8)
      .maxScalar(0, S8)
      .unsupported();

  getActionDefinitionsBuilder({G_SEXTLOAD, G_ZEXTLOAD}).custom();

  getActionDefinitionsBuilder({G_MEMCPY, G_MEMMOVE, G_MEMSET, G_MEMCPY_INLINE})
      .custom();

  // Control Flow

  getActionDefinitionsBuilder(G_PHI)
      .legalFor({S1, S8, P, PZ})
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

  getActionDefinitionsBuilder({G_STACKSAVE, G_STACKRESTORE}).lower();

  getActionDefinitionsBuilder(G_FREEZE)
      .customFor({S1, S8, P, PZ})
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
  switch (cast<GIntrinsic>(MI).getIntrinsicID()) {
  case Intrinsic::trap: {
    auto &Ctx = MI.getMF()->getFunction().getContext();
    auto *RetTy = Type::getVoidTy(Ctx);
    LostDebugLocObserver LocObserver("");
    if (!createLibcall(Builder, "abort", {{}, RetTy, 0}, {}, CallingConv::C,
                       LocObserver))
      return false;
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

bool MOSLegalizerInfo::legalizeCustom(LegalizerHelper &Helper, MachineInstr &MI,
                                      LostDebugLocObserver &LocObserver) const {
  MachineRegisterInfo &MRI = MI.getMF()->getRegInfo();

  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Invalid opcode for custom legalization.");
  // Integer Extension and Truncation
  case G_ANYEXT:
    return legalizeAnyExt(Helper, MRI, MI);
  case G_SEXT:
    return legalizeSExt(Helper, MRI, MI);
  case G_ZEXT:
    return legalizeZExt(Helper, MRI, MI);

  case G_BSWAP:
  case G_FCANONICALIZE:
  case G_FREEZE:
    return legalizeToCopy(Helper, MRI, MI);

  // Integer Operations
  case G_ADD:
  case G_SUB:
    return legalizeAddSub(Helper, MRI, MI);
  case G_XOR:
    return legalizeXor(Helper, MRI, MI);
  case G_SDIVREM:
  case G_UDIVREM:
    return legalizeDivRem(Helper, MRI, MI, LocObserver);
  case G_LSHR:
  case G_SHL:
  case G_ASHR:
  case G_ROTL:
  case G_ROTR:
    return legalizeShiftRotate(Helper, MRI, MI, LocObserver);
  case G_ICMP:
    return legalizeICmp(Helper, MRI, MI);
  case G_SELECT:
    return legalizeSelect(Helper, MRI, MI);
  case G_ABS:
    return legalizeAbs(Helper, MRI, MI);
  case G_PTR_ADD:
    return legalizePtrAdd(Helper, MRI, MI);
  case G_PTRMASK:
    return legalizePtrMask(Helper, MRI, MI);
  case G_ADDRSPACE_CAST:
    return legalizeAddrSpaceCast(Helper, MRI, MI);
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
    return legalizeLoad(Helper, MRI, cast<GAnyLoad>(MI));
  case G_STORE:
    return legalizeStore(Helper, MRI, cast<GStore>(MI));
  case G_MEMCPY:
  case G_MEMCPY_INLINE:
  case G_MEMMOVE:
  case G_MEMSET:
    return legalizeMemOp(Helper, MRI, MI, LocObserver);

  // Control Flow
  case G_BRCOND:
    return legalizeBrCond(Helper, MRI, MI);
  case G_BRJT:
    return legalizeBrJt(Helper, MRI, MI);

  // Variadic Arguments
  case G_VAARG:
    return legalizeVAArg(Helper, MRI, MI);
  case G_VASTART:
    return legalizeVAStart(Helper, MRI, MI);

  // Floating Point
  case G_FABS:
    return legalizeFAbs(Helper, MRI, MI);
  case G_FCMP:
    return legalizeFCmp(Helper, MRI, MI, LocObserver);
  case G_FCONSTANT:
    return legalizeFConst(Helper, MRI, MI);

  // Other Operations
  case G_DYN_STACKALLOC:
    return legalizeDynStackAlloc(Helper, MRI, MI);
  }
}

//===----------------------------------------------------------------------===//
// Integer Extension and Truncation
//===----------------------------------------------------------------------===//

static auto unmergeDefs(MachineInstr *MI) {
  assert(MI->getOpcode() == TargetOpcode::G_UNMERGE_VALUES);
  return make_range(MI->operands_begin(), MI->operands_end() - 1);
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

bool MOSLegalizerInfo::legalizeAnyExt(LegalizerHelper &Helper,
                                      MachineRegisterInfo &MRI,
                                      MachineInstr &MI) const {
  Helper.Observer.changingInstr(MI);
  MI.setDesc(Helper.MIRBuilder.getTII().get(G_ZEXT));
  Helper.Observer.changedInstr(MI);
  return true;
}

bool MOSLegalizerInfo::legalizeSExt(LegalizerHelper &Helper,
                                    MachineRegisterInfo &MRI,
                                    MachineInstr &MI) const {
  LLT S1 = LLT::scalar(1);
  LLT S8 = LLT::scalar(8);
  MachineIRBuilder &Builder = Helper.MIRBuilder;

  auto [Dst, DstTy, Src, SrcTy] = MI.getFirst2RegLLTs();

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
    Builder.buildMergeValues(Dst, Parts);
  }

  MI.eraseFromParent();
  return true;
}

bool MOSLegalizerInfo::legalizeZExt(LegalizerHelper &Helper,
                                    MachineRegisterInfo &MRI,
                                    MachineInstr &MI) const {
  MachineIRBuilder &Builder = Helper.MIRBuilder;
  auto [Dst, Src] = MI.getFirst2Regs();
  LLT DstTy = MRI.getType(Dst);

  assert(MRI.getType(Src) == LLT::scalar(1));
  auto One = Builder.buildConstant(DstTy, 1);
  auto Zero = Builder.buildConstant(DstTy, 0);
  Builder.buildSelect(Dst, Src, One, Zero);
  MI.eraseFromParent();
  return true;
}

//===----------------------------------------------------------------------===//
// Integer Operations
//===----------------------------------------------------------------------===//

bool MOSLegalizerInfo::legalizeAddSub(LegalizerHelper &Helper,
                                      MachineRegisterInfo &MRI,
                                      MachineInstr &MI) const {
  auto &Builder = Helper.MIRBuilder;
  LLT S8 = LLT::scalar(8);

  auto [Dst, Src] = MI.getFirst2Regs();
  assert(MRI.getType(Dst).isByteSized());

  auto RHSConst =
      getIConstantVRegValWithLookThrough(MI.getOperand(2).getReg(), MRI);
  if (!RHSConst || std::abs(RHSConst->Value.getSExtValue()) != 1)
    return Helper.narrowScalarAddSub(MI, 0, S8) !=
           LegalizerHelper::UnableToLegalize;

  // Handle multi-byte increments and decrements.

  assert(MI.getOpcode() == MOS::G_ADD || MI.getOpcode() == MOS::G_SUB);
  int64_t Amt = RHSConst->Value.getSExtValue();
  if (MI.getOpcode() == MOS::G_SUB)
    Amt = -Amt;

  auto Unmerge = Builder.buildUnmerge(S8, Src);
  size_t NumParts = llvm::size(unmergeDefs(Unmerge));
  auto IncDec = Builder.buildInstr(Amt == 1 ? MOS::G_INC : MOS::G_DEC);
  SmallVector<Register> DstParts;
  for (size_t Idx = 0; Idx < NumParts; ++Idx) {
    Register R = MRI.createGenericVirtualRegister(S8);
    IncDec.addDef(R);
    DstParts.push_back(R);
  }
  for (MachineOperand &MO : unmergeDefs(Unmerge))
    IncDec.addUse(MO.getReg());
  Builder.buildMergeValues(Dst, DstParts);
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
        UseMI.removeOperand(3);
        UseMI.removeOperand(2);
        UseMI.setDesc(Helper.MIRBuilder.getTII().get(MOS::COPY));
        Helper.Observer.changedInstr(UseMI);
      }
    }

    if (!isTriviallyDead(MI, MRI) &&
        llvm::all_of(MRI.use_nodbg_instructions(Dst),
                     [](const MachineInstr &UseMI) {
                       switch (UseMI.getOpcode()) {
                       case MOS::G_SBC:
                       case MOS::G_UADDE:
                       case MOS::G_BRCOND_IMM:
                       case MOS::G_SELECT:
                         return true;
                       default:
                         return false;
                       }
                     })) {
      MachineIRBuilder &Builder = Helper.MIRBuilder;
      // If Not is true, select 0, otherwise select 1. This will eventually
      // lower to control flow.
      auto Zero = Builder.buildConstant(S1, 0);
      auto One = Builder.buildConstant(S1, 1);
      Helper.MIRBuilder.buildSelect(Dst, Not, Zero, One);
      MI.eraseFromParent();
      return true;
    }
  }

  if (isTriviallyDead(MI, MRI))
    MI.eraseFromParent();
  else
    Helper.widenScalar(MI, 0, LLT::scalar(8));

  return true;
}

bool MOSLegalizerInfo::legalizeDivRem(LegalizerHelper &Helper,
                                      MachineRegisterInfo &MRI,
                                      MachineInstr &MI,
                                      LostDebugLocObserver &LocObserver) const {
  LLT Ty = MRI.getType(MI.getOperand(0).getReg());
  auto &Ctx = MI.getMF()->getFunction().getContext();

  auto Libcall = getRTLibDesc(MI.getOpcode(), Ty.getSizeInBits());

  Type *HLTy = IntegerType::get(Ctx, Ty.getSizeInBits());

  SmallVector<CallLowering::ArgInfo, 3> Args;
  Args.push_back({MI.getOperand(2).getReg(), HLTy, 0});
  Args.push_back({MI.getOperand(3).getReg(), HLTy, 1});

  // Pass a pointer to receive the remainder.
  MachinePointerInfo PtrInfo;
  auto FI = Helper.createStackTemporary(Ty.getSizeInBytes(), Align(), PtrInfo);

  Type *PtrTy = PointerType::get(HLTy, 0);
  Args.push_back({FI->getOperand(0).getReg(), PtrTy, 2});

  if (!createLibcall(Helper.MIRBuilder, Libcall,
                     {MI.getOperand(0).getReg(), HLTy, 0}, Args, LocObserver))
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

bool MOSLegalizerInfo::legalizeShiftRotate(
    LegalizerHelper &Helper, MachineRegisterInfo &MRI, MachineInstr &MI,
    LostDebugLocObserver &LocObserver) const {
  MachineIRBuilder &Builder = Helper.MIRBuilder;

  auto [Dst, Src, AmtReg] = MI.getFirst3Regs();

  bool IsRotate = MI.getOpcode() == G_ROTL || MI.getOpcode() == G_ROTR;

  LLT Ty = MRI.getType(Dst);
  assert(Ty == MRI.getType(Src));
  assert(Ty.isByteSized());

  LLT S1 = LLT::scalar(1);
  LLT S8 = LLT::scalar(8);

  // Presently, only left shifts by one bit are supported.
  auto ConstantAmt = getIConstantVRegValWithLookThrough(AmtReg, MRI);
  if (!ConstantAmt) {
    if (!isPowerOf2_32(Ty.getSizeInBits())) {
      return IsRotate
                 ? Helper.lowerRotate(MI)
                 : Helper.widenScalar(
                       MI, 0, LLT::scalar(NextPowerOf2(Ty.getSizeInBits())));
    }
    if (Ty.getSizeInBits() > 64) {
      return IsRotate ? Helper.lowerRotate(MI)
                      : Helper.narrowScalar(MI, 0, LLT::scalar(64));
    }
    LLT AmtTy = MRI.getType(AmtReg);
    if (AmtTy != S8)
      MI.getOperand(2).setReg(Builder.buildTrunc(S8, AmtReg).getReg(0));
    return shiftRotateLibcall(Helper, MRI, MI, LocObserver);
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
    Partial = Builder.buildMergeValues(Ty, DstBytes).getReg(0);
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

  auto [Dst, CarryOut, Src, CarryIn] = MI.getFirst4Regs();
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

  Builder.buildMergeValues(Dst, Parts).getReg(0);
  MI.eraseFromParent();
  return true;
}

bool MOSLegalizerInfo::shiftRotateLibcall(
    LegalizerHelper &Helper, MachineRegisterInfo &MRI, MachineInstr &MI,
    LostDebugLocObserver &LocObserver) const {
  unsigned Size = MRI.getType(MI.getOperand(0).getReg()).getSizeInBits();
  auto &Ctx = MI.getMF()->getFunction().getContext();

  auto Libcall = getRTLibDesc(MI.getOpcode(), Size);

  Type *HLTy = IntegerType::get(Ctx, Size);
  Type *HLAmtTy = IntegerType::get(Ctx, 8);

  SmallVector<CallLowering::ArgInfo, 3> Args;
  Args.push_back({MI.getOperand(1).getReg(), HLTy, 0});
  Args.push_back({MI.getOperand(2).getReg(), HLAmtTy, 1});
  if (!createLibcall(Helper.MIRBuilder, Libcall,
                     {MI.getOperand(0).getReg(), HLTy, 0}, Args, LocObserver))
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
          ? Builder
                .buildMergeValues(LLT::scalar(RestParts.size() * 8), RestParts)
                .getReg(0)
          : RestParts[0];

  return {UnmergeDefs.High.getReg(), Rest};
}

static bool isNZUseLegal(Register R, const MachineRegisterInfo &MRI) {
  for (MachineOperand &MO : MRI.use_nodbg_operands(R)) {
    MachineInstr &MI = *MO.getParent();
    switch (MO.getParent()->getOpcode()) {
    case MOS::COPY:
      if (isNZUseLegal(MI.getOperand(0).getReg(), MRI))
        continue;
      break;
    case MOS::G_BRCOND_IMM:
      continue;
    case MOS::G_SELECT:
      if (&MO == &MI.getOperand(1))
        continue;
      break;
    }
    return false;
  }

  return true;
}

static Register buildNZSelect(Register R, MachineIRBuilder &Builder) {
  LLT S1 = LLT::scalar(1);
  return Builder
      .buildSelect(S1, R, Builder.buildConstant(S1, -1),
                   Builder.buildConstant(S1, 0))
      .getReg(0);
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
    LLT S = LLT::scalar(Type.getScalarSizeInBits());

    Helper.Observer.changingInstr(MI);
    MI.getOperand(2).setReg(Builder.buildPtrToInt(S, LHS).getReg(0));
    MI.getOperand(3).setReg(Builder.buildPtrToInt(S, RHS).getReg(0));
    Helper.Observer.changedInstr(MI);
    return true;
  }

  LLT S1 = LLT::scalar(1);
  LLT S8 = LLT::scalar(8);

  bool RHSIsZero = mi_match(RHS, MRI, m_SpecificICst(0));
  Register CIn;

  if (Type != S8) {
    if (RHSIsZero && Pred == CmpInst::ICMP_EQ &&
        all_of(MRI.use_instructions(Dst), [](const MachineInstr &MI) {
          return MI.getOpcode() == MOS::G_BRCOND_IMM;
        })) {
      auto Unmerge = Builder.buildUnmerge(S8, LHS);
      auto Cmp = Builder.buildInstr(MOS::G_CMPZ, {Dst}, {});
      for (const MachineOperand &MO : unmergeDefs(Unmerge))
        Cmp.addUse(MO.getReg());
      MI.eraseFromParent();
      return true;
    }

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
    for (const auto &[LHS, RHS] :
         zip(LHSUnmergeDefs.Lows, RHSUnmergeDefs.Lows)) {
      auto Sbc =
          Builder.buildInstr(MOS::G_SBC, {S8, S1, S1, S1, S1}, {LHS, RHS, CIn});
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
    Register Z = Sbc.getReg(4);
    if (!isNZUseLegal(Dst, MRI))
      Z = buildNZSelect(Z, Builder);
    Builder.buildCopy(Dst, Z);
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
      Register N = Sbc.getReg(2);
      if (!isNZUseLegal(Dst, MRI))
        N = buildNZSelect(N, Builder);
      Builder.buildCopy(Dst, N);
    } else {
      // General subtractions can overflow; if so, N is flipped.
      auto Sbc =
          Builder.buildInstr(MOS::G_SBC, {S8, S1, S1, S1, S1}, {LHS, RHS, CIn});
      // The quickest way to XOR N with V is to XOR the accumulator with 0x80
      // iff V, then reexamine N of the accumulator.
      auto Eor = Builder.buildXor(S8, Sbc, Builder.buildConstant(S8, 0x80));
      auto Zero = Builder.buildConstant(S8, 0);
      auto One = Builder.buildConstant(S1, 1);
      Register N =
          Builder
              .buildInstr(
                  MOS::G_SBC, {S8, S1, S1, S1, S1},
                  {Builder.buildSelect(S8, Sbc.getReg(3) /*=V*/, Eor, Sbc),
                   Zero, One})
              .getReg(2);
      if (!isNZUseLegal(Dst, MRI))
        N = buildNZSelect(N, Builder);
      Builder.buildCopy(Dst, N);
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

  auto [Dst, Test, LHS, RHS] = MI.getFirst4Regs();

  LLT P = MRI.getType(Dst);
  assert(P.isPointer());

  LLT S = LLT::scalar(P.getScalarSizeInBits());

  assert(MRI.getType(Dst) == P);
  assert(MRI.getType(Test) == LLT::scalar(1));
  assert(MRI.getType(LHS) == P);
  assert(MRI.getType(RHS) == P);

  Helper.Observer.changingInstr(MI);
  MI.getOperand(2).setReg(Builder.buildPtrToInt(S, LHS).getReg(0));
  MI.getOperand(3).setReg(Builder.buildPtrToInt(S, RHS).getReg(0));
  Register Tmp = MRI.createGenericVirtualRegister(S);
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

  auto [Result, Base, Offset] = MI.getFirst3Regs();

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

  if (ConstOffset && ConstOffset->Value.abs().isOne()) {
    Builder.buildInstr(ConstOffset->Value.isOne() ? MOS::G_INC : MOS::G_DEC,
                       {Result}, {Base});
    MI.eraseFromParent();
    return true;
  }

  // Generalized pointer additions must be lowered to integer arithmetic.
  LLT S = LLT::scalar(MRI.getType(Base).getScalarSizeInBits());
  auto PtrVal = Builder.buildPtrToInt(S, Base);
  auto Sum = Builder.buildAdd(S, PtrVal, Offset);
  Builder.buildIntToPtr(Result, Sum);
  MI.eraseFromParent();
  return true;
}

bool MOSLegalizerInfo::legalizePtrMask(LegalizerHelper &Helper,
                                       MachineRegisterInfo &MRI,
                                       MachineInstr &MI) const {
  MachineIRBuilder &Builder = Helper.MIRBuilder;

  auto [Result, Base, Mask] = MI.getFirst3Regs();
  LLT S = LLT::scalar(MRI.getType(Base).getScalarSizeInBits());

  Builder.buildIntToPtr(
      Result, Builder.buildAnd(S, Builder.buildPtrToInt(S, Base), Mask));
  MI.eraseFromParent();
  return true;
}

bool MOSLegalizerInfo::legalizeAddrSpaceCast(LegalizerHelper &Helper,
                                             MachineRegisterInfo &MRI,
                                             MachineInstr &MI) const {
  MachineIRBuilder &Builder = Helper.MIRBuilder;
  const MOSSubtarget &STI = Builder.getMF().getSubtarget<MOSSubtarget>();

  auto [DestReg, DestTypeP, SrcReg, SrcTypeP] = MI.getFirst2RegLLTs();
  LLT DestTypeS = LLT::scalar(DestTypeP.getScalarSizeInBits());
  LLT SrcTypeS = LLT::scalar(SrcTypeP.getScalarSizeInBits());

  auto Tmp = Builder.buildPtrToInt(SrcTypeS, SrcReg);

  if (DestTypeS.getSizeInBits() < SrcTypeS.getSizeInBits()) {
    // larger -> smaller address space: truncate
    Tmp = Builder.buildTrunc(DestTypeS, Tmp);
  } else if (DestTypeS.getSizeInBits() > SrcTypeS.getSizeInBits()) {
    // smaller -> larger address space: extend
    assert(SrcTypeP.getAddressSpace() == MOS::AS_ZeroPage);
    assert(DestTypeP.getAddressSpace() == MOS::AS_Memory);
    Tmp = Builder.buildZExt(DestTypeS, Tmp);
    if (STI.getZeroPageOffset() != 0) {
      // Dest = (Src | ZeroPageOffset)
      Tmp = Builder.buildOr(
          DestTypeS, Tmp,
          Builder.buildConstant(DestTypeS, STI.getZeroPageOffset()));
    }
  }

  assert(MRI.getType(Tmp->getOperand(0).getReg()) == DestTypeS);

  Builder.buildIntToPtr(DestReg, Tmp);

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
                                    GAnyLoad &MI) const {
  if (auto *Load = dyn_cast<GLoad>(&MI))
    if (MRI.getType(Load->getDstReg()) == LLT::scalar(8))
      return selectAddressingMode(Helper, MRI, MI);

  MachineIRBuilder &Builder = Helper.MIRBuilder;
  Builder.setInsertPt(Builder.getMBB(), std::next(Builder.getInsertPt()));
  Register Tmp;
  const MachineMemOperand &MMO = **MI.memoperands_begin();
  switch (MI.getOpcode()) {
  case MOS::G_SEXTLOAD:
    Tmp = MRI.createGenericVirtualRegister(MMO.getType());
    Builder.buildSExt(MI.getDstReg(), Tmp);
    break;
  case MOS::G_ZEXTLOAD:
    Tmp = MRI.createGenericVirtualRegister(MMO.getType());
    Builder.buildZExt(MI.getDstReg(), Tmp);
    break;
  default:
    auto DstType = MRI.getType(MI.getDstReg());
    assert(DstType.isPointer());
    Tmp = MRI.createGenericVirtualRegister(
        LLT::scalar(DstType.getScalarSizeInBits()));
    Builder.buildIntToPtr(MI.getDstReg(), Tmp);
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
                                     GStore &MI) const {
  if (MRI.getType(MI.getValueReg()) == LLT::scalar(8))
    return selectAddressingMode(Helper, MRI, MI);

  MachineIRBuilder &Builder = Helper.MIRBuilder;
  auto ValueType = MRI.getType(MI.getValueReg());
  assert(ValueType.isPointer());

  Register Tmp =
      Builder
          .buildPtrToInt(LLT::scalar(ValueType.getScalarSizeInBits()),
                         MI.getValueReg())
          .getReg(0);
  Helper.Observer.changingInstr(MI);
  MI.getOperand(0).setReg(Tmp);
  Helper.Observer.changedInstr(MI);
  return true;
}

static bool willBeStaticallyAllocated(const MachineOperand &MO) {
  const MachineFunction &MF = *MO.getParent()->getMF();
  const MOSFrameLowering &TFL =
      *MF.getSubtarget<MOSSubtarget>().getFrameLowering();
  assert(MO.isFI());
  if (!TFL.usesStaticStack(MF))
    return false;
  return !MF.getFrameInfo().isFixedObjectIndex(MO.getIndex());
}

bool MOSLegalizerInfo::selectAddressingMode(LegalizerHelper &Helper,
                                            MachineRegisterInfo &MRI,
                                            GLoadStore &MI) const {
  switch (MRI.getType(MI.getPointerReg()).getScalarSizeInBits()) {
  case 8: {
    if (tryAbsoluteAddressing(Helper, MRI, MI, true))
      return true;
    if (tryAbsoluteIndexedAddressing(Helper, MRI, MI, true))
      return true;
    return selectZeroIndexedAddressing(Helper, MRI, MI);
  }
  case 16: {
    if (tryAbsoluteAddressing(Helper, MRI, MI, false))
      return true;
    if (tryAbsoluteIndexedAddressing(Helper, MRI, MI, false))
      return true;
    return selectIndirectAddressing(Helper, MRI, MI);
  }
  default:
    llvm_unreachable("unknown pointer size");
  }
}

std::optional<MachineOperand>
MOSLegalizerInfo::matchAbsoluteAddressing(MachineRegisterInfo &MRI,
                                          Register Addr) const {
  int64_t Offset = 0;

  while (true) {
    if (auto ConstAddr = getIConstantVRegValWithLookThrough(Addr, MRI)) {
      return MachineOperand::CreateImm(Offset +
                                       ConstAddr->Value.getSExtValue());
    }
    if (const MachineInstr *GVAddr = getOpcodeDef(G_GLOBAL_VALUE, Addr, MRI)) {
      const MachineOperand &GV = GVAddr->getOperand(1);
      return MachineOperand::CreateGA(GV.getGlobal(), GV.getOffset() + Offset);
    }
    if (const MachineInstr *FIAddr = getOpcodeDef(G_FRAME_INDEX, Addr, MRI)) {
      const MachineOperand &FI = FIAddr->getOperand(1);
      if (willBeStaticallyAllocated(FI)) {
        return MachineOperand::CreateFI(FI.getIndex(), FI.getOffset() + Offset);
      }
    }
    if (const auto *PtrAddAddr =
            cast_if_present<GPtrAdd>(getOpcodeDef(G_PTR_ADD, Addr, MRI))) {
      auto ConstOffset =
          getIConstantVRegValWithLookThrough(PtrAddAddr->getOffsetReg(), MRI);
      if (!ConstOffset)
        return std::nullopt;
      Offset += ConstOffset->Value.getSExtValue();
      Addr = PtrAddAddr->getBaseReg();
      continue;
    }
    return std::nullopt;
  }
  return std::nullopt;
}

bool MOSLegalizerInfo::tryAbsoluteAddressing(LegalizerHelper &Helper,
                                             MachineRegisterInfo &MRI,
                                             GLoadStore &MI, bool ZP) const {
  MachineIRBuilder &Builder = Helper.MIRBuilder;
  const MOSSubtarget &STI = Builder.getMF().getSubtarget<MOSSubtarget>();

  unsigned Opcode = isa<GLoad>(MI) ? MOS::G_LOAD_ABS : MOS::G_STORE_ABS;
  auto Operand = matchAbsoluteAddressing(MRI, MI.getPointerReg());

  if (Operand.has_value()) {
    Helper.Observer.changingInstr(MI);
    MI.setDesc(Builder.getTII().get(Opcode));
    MI.removeOperand(1);
    if (ZP && Operand->isImm())
      Operand->setImm(STI.getZeroPageOffset() + (Operand->getImm() & 0xFF));
    MI.addOperand(*Operand);
    if (ZP)
      MI.getOperand(1).setTargetFlags(MOS::MO_ZEROPAGE);
    Helper.Observer.changedInstr(MI);
    return true;
  }

  return false;
}

bool MOSLegalizerInfo::tryAbsoluteIndexedAddressing(LegalizerHelper &Helper,
                                                    MachineRegisterInfo &MRI,
                                                    GLoadStore &MI,
                                                    bool ZP) const {
  LLT S8 = LLT::scalar(8);
  MachineIRBuilder &Builder = Helper.MIRBuilder;
  const MOSSubtarget &STI = Builder.getMF().getSubtarget<MOSSubtarget>();

  Register Addr = MI.getPointerReg();
  int64_t Offset = 0;
  Register Index = 0;

  unsigned Opcode = isa<GLoad>(MI)
                        ? (ZP ? MOS::G_LOAD_ZP_IDX : MOS::G_LOAD_ABS_IDX)
                        : (ZP ? MOS::G_STORE_ZP_IDX : MOS::G_STORE_ABS_IDX);

  while (true) {
    if (auto ConstAddr = getIConstantVRegValWithLookThrough(Addr, MRI)) {
      assert(Index); // Otherwise, Absolute addressing would have been selected.

      Offset = ConstAddr->Value.getSExtValue() + Offset;
      if (ZP)
        Offset = STI.getZeroPageOffset() + (Offset & 0xFF);
      auto Inst = Builder.buildInstr(Opcode)
                      .add(MI.getOperand(0))
                      .addImm(Offset)
                      .addUse(Index)
                      .addMemOperand(*MI.memoperands_begin());
      if (ZP)
        Inst->getOperand(1).setTargetFlags(MOS::MO_ZEROPAGE);
      MI.eraseFromParent();
      return true;
    }
    if (const MachineInstr *GVAddr = getOpcodeDef(G_GLOBAL_VALUE, Addr, MRI)) {
      assert(Index); // Otherwise, Absolute addressing would have been selected.
      const MachineOperand &GV = GVAddr->getOperand(1);
      auto Inst = Builder.buildInstr(Opcode)
                      .add(MI.getOperand(0))
                      .addGlobalAddress(GV.getGlobal(), GV.getOffset() + Offset)
                      .addUse(Index)
                      .addMemOperand(*MI.memoperands_begin());
      if (ZP)
        Inst->getOperand(1).setTargetFlags(MOS::MO_ZEROPAGE);
      MI.eraseFromParent();
      return true;
    }
    if (const MachineInstr *FIAddr = getOpcodeDef(G_FRAME_INDEX, Addr, MRI)) {
      const MachineOperand &FI = FIAddr->getOperand(1);
      if (willBeStaticallyAllocated(FI)) {
        assert(
            Index); // Otherwise, Absolute addressing would have been selected.
        auto Inst = Builder.buildInstr(Opcode)
                        .add(MI.getOperand(0))
                        .addFrameIndex(FI.getIndex(), FI.getOffset() + Offset)
                        .addUse(Index)
                        .addMemOperand(*MI.memoperands_begin());
        if (ZP)
          Inst->getOperand(1).setTargetFlags(MOS::MO_ZEROPAGE);
        MI.eraseFromParent();
        return true;
      }
    }
    if (const auto *PtrAddAddr =
            cast_if_present<GPtrAdd>(getOpcodeDef(G_PTR_ADD, Addr, MRI))) {
      Addr = PtrAddAddr->getBaseReg();
      Register NewOffset = PtrAddAddr->getOffsetReg();
      if (auto ConstOffset =
              getIConstantVRegValWithLookThrough(NewOffset, MRI)) {
        Offset += ConstOffset->Value.getSExtValue();
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
          Src = Builder.buildZExt(S8, Src).getReg(0);
        assert(MRI.getType(Src) == S8);
        Index = Src;
        continue;
      }
      if (Helper.getKnownBits()->getKnownBits(NewOffset).countMaxActiveBits() <=
          8) {
        if (Index)
          return false;
        Index = Builder.buildZExtOrTrunc(S8, NewOffset).getReg(0);
        continue;
      }
    }
    return false;
  }
  return false;
}

bool MOSLegalizerInfo::selectZeroIndexedAddressing(LegalizerHelper &Helper,
                                                   MachineRegisterInfo &MRI,
                                                   GLoadStore &MI) const {
  // Selects absolute indexed, but with the pointer as the index.
  MachineIRBuilder &Builder = Helper.MIRBuilder;
  const MOSSubtarget &STI = Builder.getMF().getSubtarget<MOSSubtarget>();

  assert(MRI.getType(MI.getPointerReg()).getScalarSizeInBits() == 8);
  LLT S = LLT::scalar(8);
  auto Addr = MI.getOperand(1).getReg();
  int64_t Offset = 0;

  if (const auto *PtrAddAddr =
          cast_if_present<GPtrAdd>(getOpcodeDef(G_PTR_ADD, Addr, MRI))) {
    auto ConstOffset =
        getIConstantVRegValWithLookThrough(PtrAddAddr->getOffsetReg(), MRI);
    if (ConstOffset) {
      Offset += ConstOffset->Value.getSExtValue();
      Addr = PtrAddAddr->getBaseReg();
    }
  }

  Offset = STI.getZeroPageOffset() + (Offset & 0xFF);

  auto AddrP = Builder.buildPtrToInt(S, Addr).getReg(0);
  unsigned Opcode = isa<GLoad>(MI) ? MOS::G_LOAD_ZP_IDX : MOS::G_STORE_ZP_IDX;
  auto Inst = Builder.buildInstr(Opcode)
                  .add(MI.getOperand(0))
                  .addImm(Offset)
                  .addUse(AddrP)
                  .addMemOperand(*MI.memoperands_begin());
  Inst->getOperand(1).setTargetFlags(MOS::MO_ZEROPAGE);
  MI.eraseFromParent();
  return true;
}

bool MOSLegalizerInfo::selectIndirectAddressing(LegalizerHelper &Helper,
                                                MachineRegisterInfo &MRI,
                                                GLoadStore &MI) const {
  LLT S8 = LLT::scalar(8);
  MachineIRBuilder &Builder = Helper.MIRBuilder;
  const MOSSubtarget &STI = Builder.getMF().getSubtarget<MOSSubtarget>();

  Register Addr = MI.getPointerReg();
  Register Index = 0;

  unsigned Opcode =
      isa<GLoad>(MI) ? MOS::G_LOAD_INDIR_IDX : MOS::G_STORE_INDIR_IDX;

  if (const auto *PtrAddAddr =
          cast_if_present<GPtrAdd>(getOpcodeDef(G_PTR_ADD, Addr, MRI))) {
    Register Base = PtrAddAddr->getBaseReg();
    Register Offset = PtrAddAddr->getOffsetReg();
    if (auto ConstOffset = getIConstantVRegValWithLookThrough(Offset, MRI)) {
      if (ConstOffset->Value.getActiveBits() <= 8) {
        Index = Builder.buildConstant(S8, ConstOffset->Value.getSExtValue())
                    .getReg(0);
        Addr = Base;
      }
    } else if (MachineInstr *ZExtOffset = getOpcodeDef(G_ZEXT, Offset, MRI)) {
      Register Src = ZExtOffset->getOperand(1).getReg();
      LLT SrcTy = MRI.getType(Src);
      if (SrcTy.getSizeInBits() <= 8) {
        if (SrcTy.getSizeInBits() < 8)
          Src = Builder.buildZExt(S8, Src).getReg(0);
        assert(MRI.getType(Src) == S8);
        Index = Src;
        Addr = Base;
      }
    } else if (Helper.getKnownBits()
                   ->getKnownBits(Offset)
                   .countMaxActiveBits() <= 8) {
      Index = Builder.buildZExtOrTrunc(S8, Offset).getReg(0);
      Addr = Base;
    }
  }

  if (!Index) {
    if (STI.has65C02()) {
      Opcode = isa<GLoad>(MI) ? MOS::G_LOAD_INDIR : MOS::G_STORE_INDIR;
      Builder.buildInstr(Opcode)
          .add(MI.getOperand(0))
          .addUse(Addr)
          .addMemOperand(*MI.memoperands_begin());
      MI.eraseFromParent();
      return true;
    }
    Index = Builder.buildConstant(S8, 0).getReg(0);
  }
  Builder.buildInstr(Opcode)
      .add(MI.getOperand(0))
      .addUse(Addr)
      .addUse(Index)
      .addMemOperand(*MI.memoperands_begin());
  MI.eraseFromParent();
  return true;
}

bool MOSLegalizerInfo::legalizeMemOp(LegalizerHelper &Helper,
                                     MachineRegisterInfo &MRI, MachineInstr &MI,
                                     LostDebugLocObserver &LocObserver) const {
  MachineIRBuilder &Builder = Helper.MIRBuilder;
  const MOSSubtarget &STI = Builder.getMF().getSubtarget<MOSSubtarget>();

  // Special handling for HuC6280 block copy extensions.
  if (STI.hasHUC6280())
    if (tryHuCBlockCopy(Helper, MRI, MI))
      return true;

  bool IsSet = MI.getOpcode() == MOS::G_MEMSET;
  bool IsInline = MI.getOpcode() == MOS::G_MEMCPY_INLINE;
  uint32_t SizeLimit;
  if (IsInline) {
    SizeLimit = UINT16_MAX;
  } else {
    MachineFunction *MF = MI.getParent()->getParent();
    if (MF && MF->getFunction().hasMinSize()) {
      // Copies:
      // => inline LDA/STA: 6n bytes
      // => memcpy(): ~23 bytes
      // Sets:
      // => inline LDA/STA: 2 + 3n bytes
      // => __memset(): ~21? bytes
      SizeLimit = IsSet ? 6 : 3;
    } else if (!MF || MF->getFunction().hasOptSize()) {
      SizeLimit = IsSet ? 8 : 4;
    } else {
      SizeLimit = IsSet ? 16 : 8;
    }
  }

  LegalizerHelper::LegalizeResult Result;

  // Try lowering, keeping in mind the size limit.
  if (IsInline) {
    Result = Helper.lowerMemcpyInline(MI);
  } else {
    Result = Helper.lowerMemCpyFamily(MI, SizeLimit);
  }
  if (Result == LegalizerHelper::Legalized) {
    return true;
  }

  // Try emitting a libcall.
  Result = createMemLibcall(Builder, MRI, MI, LocObserver);
  if (Result == LegalizerHelper::Legalized) {
    MI.eraseFromParent();
    return true;
  }

  return false;
}

static std::optional<uint64_t>
getUInt64FromConstantOper(const MachineOperand &Operand) {
  if (Operand.isImm())
    return Operand.getImm();
  if (Operand.isCImm())
    return Operand.getCImm()->getZExtValue();
  return std::nullopt;
}

static MachineOperand offsetMachineOperand(MachineOperand &Operand,
                                           int64_t Offset) {
  if (Offset == 0)
    return Operand;
  if (Operand.isImm())
    return MachineOperand::CreateImm(Operand.getImm() + Offset);
  if (Operand.isCImm())
    return MachineOperand::CreateCImm(Operand.getCImm() + Offset);
  if (Operand.isGlobal())
    return MachineOperand::CreateGA(Operand.getGlobal(),
                                    Operand.getOffset() + Offset);
  if (Operand.isFI())
    return MachineOperand::CreateFI(Operand.getIndex(),
                                    Operand.getOffset() + Offset);
  llvm_unreachable("Unsupported machine operand type!");
}

template <typename T> static inline int compareNumbers(T A, T B) {
  return A < B ? -1 : (A > B ? 1 : 0);
}

static std::optional<int> compareOperandLocations(const MachineOperand &A,
                                                  const MachineOperand &B) {
  if (A.isImm() && B.isImm())
    return compareNumbers(A.getImm(), B.getImm());
  if (A.isGlobal() && B.isGlobal())
    if (A.getGlobal()->getGlobalIdentifier() ==
        B.getGlobal()->getGlobalIdentifier())
      return compareNumbers(A.getOffset(), B.getOffset());
  return std::nullopt;
}

bool MOSLegalizerInfo::tryHuCBlockCopy(LegalizerHelper &Helper,
                                       MachineRegisterInfo &MRI,
                                       MachineInstr &MI) const {
  MachineIRBuilder &Builder = Helper.MIRBuilder;
  MachineFunction &MF = Builder.getMF();

  bool IsSet = MI.getOpcode() == MOS::G_MEMSET;
  bool IsInline = MI.getOpcode() == MOS::G_MEMCPY_INLINE;

  // Match supported combinations.
  if (MI.getOpcode() != MOS::G_MEMCPY &&
      MI.getOpcode() != MOS::G_MEMCPY_INLINE &&
      MI.getOpcode() != MOS::G_MEMMOVE && MI.getOpcode() != MOS::G_MEMSET) {
    return false;
  }

  Register DstReg = MI.getOperand(0).getReg();
  auto Dst = matchAbsoluteAddressing(MRI, DstReg);
  Register SrcReg = MI.getOperand(1).getReg();
  auto Src = matchAbsoluteAddressing(MRI, SrcReg);
  auto Len = matchAbsoluteAddressing(MRI, MI.getOperand(2).getReg());
  bool Descending = false;
  if (!Src.has_value() || !Dst.has_value() || !Len.has_value())
    return false;

  if (IsSet) {
    // A TII-based memory set is always slower than the alternative.
    // Skip using it unless -Os, -Oz is set.
    if (!MF.getFunction().hasOptSize())
      return false;

    auto SrcValue = getUInt64FromConstantOper(Src.value());
    if (!SrcValue.has_value())
      return false;
    if (MRI.getType(SrcReg).getSizeInBytes() != 1)
      return false;
  }
  if (MI.getOpcode() == MOS::G_MEMMOVE) {
    auto OperandOrder = compareOperandLocations(Src.value(), Dst.value());
    // TODO: Handle case when two G_MEMMOVE destinations cannot alias.
    if (!OperandOrder.has_value())
      return false;
    if (OperandOrder.value() == -1)
      Descending = true;
  }

  // On HuC platforms, block copies can be emitted, and sets can be done
  // with them too. However, some requirements have to be considered:
  // 1) The source, destination, and length have to be constant; however,
  //    they can be opaque constants (such as symbols).
  // 2) A block copy instruction stalls all interrupts until it completes.
  //    As such, one instruction should only do some amount of transfers,
  //    to prevent stalling video interrupts mid-execution.
  // Each transfer is 7 bytes and (17 + 6n) cycles, where n is the length
  // of the transfer in bytes.
  bool HuCIrqSafeBlockCopies = true; // TODO
  uint64_t BytesPerTransfer = HuCIrqSafeBlockCopies ? 16 : UINT16_MAX;
  uint64_t SizeMin, SizeMax;
  // Note that non-indexed LDA/STA memory calls are 1 cycle slower on
  // HuC6280 compared to other 6502 derivatives.
  if (MF.getFunction().hasMinSize()) {
    // Copies:
    // => inline LDA/STA: 6n bytes
    // => TII: 7 bytes
    // => memcpy(): ~23 bytes
    // Sets:
    // => inline LDA/STA: 2 + 3n bytes
    // => (LDA/STA|STZ)/TII: 3-5 + 7 bytes
    // => __memset(): ~21? bytes
    SizeMin = IsSet ? 5 : 2;
    SizeMax = IsSet ? (BytesPerTransfer * 2 + 1) : (BytesPerTransfer * 3);
  } else if (MF.getFunction().hasOptSize()) {
    // Try to strike a balance.
    SizeMin = IsSet ? 5 : 4;
    SizeMax = IsSet ? (BytesPerTransfer * 3 + 1) : (BytesPerTransfer * 4);
  } else {
    // Copies:
    // => inline LDA/STA: 10n cycles
    // => TII: 17 + 6n cycles
    // Sets:
    // => inline LDA/STA: 2 + 5n cycles
    // => (LDA/STA|STZ)/TII: 22-24 + 6n cycles
    SizeMin = 5;
    SizeMax = BytesPerTransfer * 5;
  }
  if (IsInline)
    SizeMax = UINT16_MAX;
  uint64_t KnownLen = UINT16_MAX;

  // If we require IRQ-safe chunks, the length has to be known.
  auto LenValue = getUInt64FromConstantOper(Len.value());
  if (LenValue.has_value()) {
    KnownLen = LenValue.value();
    if (KnownLen < SizeMin || KnownLen > SizeMax) {
      return false;
    }
  } else if (BytesPerTransfer < UINT16_MAX || Descending) {
    return false;
  }

  auto DstPointerInfo = MI.memoperands()[0]->getPointerInfo();
  auto SrcPointerInfo = MI.memoperands()[IsSet ? 0 : 1]->getPointerInfo();

  // Proceed with the custom lowering.
  if (IsSet) {
    // Emit a G_STORE, then set Src = Dst, Dst = Dst + 1, Len = Len - 1.
    auto StoreReg = MRI.createGenericVirtualRegister(LLT::scalar(8));
    Builder.buildConstant(
        StoreReg, getUInt64FromConstantOper(Src.value()).value() & 0xFF);
    Builder.buildStore(StoreReg, DstReg,
                       *MF.getMachineMemOperand(SrcPointerInfo,
                                                MachineMemOperand::MOStore, 1,
                                                Align(1)));

    Src = Dst;
    Dst = offsetMachineOperand(Dst.value(), 1);
    DstPointerInfo = DstPointerInfo.getWithOffset(1);
    Len = offsetMachineOperand(Len.value(), -1);
    KnownLen -= 1;
  }

  // Note that Descending transfers must be done in backwards order.
  if (KnownLen <= BytesPerTransfer) {
    uint64_t AdjOfs = Descending ? (KnownLen - 1) : 0;
    Builder.buildInstr(MOS::HuCMemcpy)
        .add(offsetMachineOperand(Src.value(), AdjOfs))
        .add(offsetMachineOperand(Dst.value(), AdjOfs))
        .add(Len.value())
        .addImm(Descending)
        .addMemOperand(MF.getMachineMemOperand(
            SrcPointerInfo, MachineMemOperand::MOLoad, 1, Align(1)))
        .addMemOperand(MF.getMachineMemOperand(
            DstPointerInfo, MachineMemOperand::MOStore, 1, Align(1)));
  } else {
    // Transfer Offset
    for (uint64_t TOfs = 0; TOfs < KnownLen; TOfs += BytesPerTransfer) {
      // Transfer Length
      uint64_t TLen = std::min(KnownLen - TOfs, BytesPerTransfer);
      // Adjusted Transfer Offset (opcode)
      uint64_t AdjTOfs = Descending ? (KnownLen - TOfs - 1) : TOfs;
      // Adjusted Transfer Offset (memory)
      uint64_t AdjTOfsMO = Descending ? (KnownLen - TOfs - TLen) : TOfs;
      Builder.buildInstr(MOS::HuCMemcpy)
          .add(offsetMachineOperand(Src.value(), AdjTOfs))
          .add(offsetMachineOperand(Dst.value(), AdjTOfs))
          .add(MachineOperand::CreateImm(TLen))
          .addImm(Descending)
          .addMemOperand(
              MF.getMachineMemOperand(SrcPointerInfo.getWithOffset(AdjTOfsMO),
                                      MachineMemOperand::MOLoad, 1, Align(1)))
          .addMemOperand(
              MF.getMachineMemOperand(DstPointerInfo.getWithOffset(AdjTOfsMO),
                                      MachineMemOperand::MOStore, 1, Align(1)));
    }
  }

  MI.eraseFromParent();
  return true;
}

//===----------------------------------------------------------------------===//
// Control Flow
//===----------------------------------------------------------------------===//

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
  const MOSSubtarget &STI = Builder.getMF().getSubtarget<MOSSubtarget>();

#ifndef NDEBUG
  const MachineInstr *Base =
      getOpcodeDef(G_JUMP_TABLE, MI.getOperand(0).getReg(), MRI);
#endif
  assert(Base && "Invalid first argument to G_BRJT; expected G_JUMP_TABLE.");

  assert(MI.getOperand(1).isJTI());
  assert(MI.getOperand(1).getIndex() == Base->getOperand(1).getIndex() &&
         "Expected G_JUMP_TABLE to have same index.");

  const MachineJumpTableInfo *JTI = MI.getMF()->getJumpTableInfo();
  const auto &Table = JTI->getJumpTables()[MI.getOperand(1).getIndex()];

  // Note: Jump table size is hard-limited to 256 entries.
  Register Offset = Builder.buildTrunc(S8, MI.getOperand(2)).getReg(0);

  if (STI.hasJMPIdxIndir() && Table.MBBs.size() <= 128) {
    Offset =
        Builder.buildShl(S8, Offset, Builder.buildConstant(S8, 1)).getReg(0);
    Builder.buildInstr(MOS::G_BRINDIRECT_IDX)
        .add(MI.getOperand(1))
        .addUse(Offset);
  } else {
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
        Builder.buildMergeValues(LLT::pointer(0, 16), {LoAddr, HiAddr})
            .getReg(0));
  }

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
  Builder.buildStore(Builder.buildFrameIndex(P, FuncInfo->VarArgsStackIndex),
                     MI.getOperand(0), **MI.memoperands_begin());
  MI.eraseFromParent();
  return true;
}

bool MOSLegalizerInfo::legalizeFAbs(LegalizerHelper &Helper,
                                    MachineRegisterInfo &MRI,
                                    MachineInstr &MI) const {
  MachineIRBuilder &Builder = Helper.MIRBuilder;
  Register Dst = MI.getOperand(0).getReg();
  LLT Ty = MRI.getType(Dst);
  Register Src = MI.getOperand(1).getReg();

  Builder.buildAnd(
      Dst, Src,
      Builder.buildConstant(Ty, ~APInt::getSignMask(Ty.getScalarSizeInBits())));
  MI.eraseFromParent();
  return true;
}

// Legalize floating-point comparisons to libcalls.
bool MOSLegalizerInfo::legalizeFCmp(LegalizerHelper &Helper,
                                    MachineRegisterInfo &MRI, MachineInstr &MI,
                                    LostDebugLocObserver &LocObserver) const {
  assert(MRI.getType(MI.getOperand(2).getReg()) ==
             MRI.getType(MI.getOperand(3).getReg()) &&
         "Mismatched operands for G_FCMP");
  auto OpSize = MRI.getType(MI.getOperand(2).getReg()).getSizeInBits();

  auto OriginalResult = MI.getOperand(0).getReg();
  auto Predicate =
      static_cast<CmpInst::Predicate>(MI.getOperand(1).getPredicate());
  auto Libcalls = getFCmpLibcalls(Predicate, OpSize);

  MachineIRBuilder &Builder = Helper.MIRBuilder;
  LLVMContext &Ctx = Builder.getMF().getFunction().getContext();

  if (Libcalls.empty()) {
    assert(
        (Predicate == CmpInst::FCMP_TRUE || Predicate == CmpInst::FCMP_FALSE) &&
        "Predicate needs libcalls, but none specified");
    Builder.buildConstant(OriginalResult,
                          Predicate == CmpInst::FCMP_TRUE ? 1 : 0);
    MI.eraseFromParent();
    return true;
  }

  assert((OpSize == 32 || OpSize == 64) && "Unsupported operand size");
  auto *ArgTy = OpSize == 32 ? Type::getFloatTy(Ctx) : Type::getDoubleTy(Ctx);
  auto *RetTy = Type::getInt32Ty(Ctx);

  SmallVector<Register, 2> Results;
  for (auto Libcall : Libcalls) {
    auto LibcallResult = MRI.createGenericVirtualRegister(LLT::scalar(32));
    auto Status =
        createLibcall(Builder, Libcall.LibcallID, {LibcallResult, RetTy, 0},
                      {{MI.getOperand(2).getReg(), ArgTy, 0},
                       {MI.getOperand(3).getReg(), ArgTy, 0}},
                      LocObserver);

    if (Status != LegalizerHelper::Legalized)
      return false;

    auto ProcessedResult =
        Libcalls.size() == 1
            ? OriginalResult
            : MRI.createGenericVirtualRegister(MRI.getType(OriginalResult));

    // We have a result, but we need to transform it into a proper 1-bit 0 or
    // 1, taking into account the different peculiarities of the values
    // returned by the comparison functions.
    CmpInst::Predicate ResultPred = Libcall.Predicate;
    if (ResultPred == CmpInst::BAD_ICMP_PREDICATE) {
      // We have a nice 0 or 1, and we just need to truncate it back to 1 bit
      // to keep the types consistent.
      Builder.buildTrunc(ProcessedResult, LibcallResult);
    } else {
      // We need to compare against 0.
      assert(CmpInst::isIntPredicate(ResultPred) && "Unsupported predicate");
      auto Zero = Builder.buildConstant(LLT::scalar(32), 0);
      Builder.buildICmp(ResultPred, ProcessedResult, LibcallResult, Zero);
    }
    Results.push_back(ProcessedResult);
  }

  if (Results.size() != 1) {
    assert(Results.size() == 2 && "Unexpected number of results");
    Builder.buildOr(OriginalResult, Results[0], Results[1]);
  }

  MI.eraseFromParent();
  return true;
}

// Convert floating-point constants into their binary integer equivalents.
bool MOSLegalizerInfo::legalizeFConst(LegalizerHelper &Helper,
                                      MachineRegisterInfo &MRI,
                                      MachineInstr &MI) const {
  MachineIRBuilder &Builder = Helper.MIRBuilder;
  LLVMContext &Ctx = Builder.getMF().getFunction().getContext();

  // Convert to integer constants, while preserving the binary representation.
  auto AsInteger = MI.getOperand(1).getFPImm()->getValueAPF().bitcastToAPInt();
  Builder.buildConstant(MI.getOperand(0), *ConstantInt::get(Ctx, AsInteger));
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

bool MOSLegalizerInfo::legalizeToCopy(LegalizerHelper &Helper,
                                      MachineRegisterInfo &MRI,
                                      MachineInstr &MI) const {
  Helper.Observer.changingInstr(MI);
  MI.setDesc(Helper.MIRBuilder.getTII().get(COPY));
  Helper.Observer.changedInstr(MI);
  return true;
}

void MOSLegalizerInfo::setFCmpLibcallsGNU() {
  // FCMP_TRUE and FCMP_FALSE don't need libcalls, they should be
  // default-initialized.
  FCmp32Libcalls.resize(CmpInst::LAST_FCMP_PREDICATE + 1);
  FCmp32Libcalls[CmpInst::FCMP_OEQ] = {{RTLIB::OEQ_F32, CmpInst::ICMP_EQ}};
  FCmp32Libcalls[CmpInst::FCMP_OGE] = {{RTLIB::OGE_F32, CmpInst::ICMP_SGE}};
  FCmp32Libcalls[CmpInst::FCMP_OGT] = {{RTLIB::OGT_F32, CmpInst::ICMP_SGT}};
  FCmp32Libcalls[CmpInst::FCMP_OLE] = {{RTLIB::OLE_F32, CmpInst::ICMP_SLE}};
  FCmp32Libcalls[CmpInst::FCMP_OLT] = {{RTLIB::OLT_F32, CmpInst::ICMP_SLT}};
  FCmp32Libcalls[CmpInst::FCMP_ORD] = {{RTLIB::UO_F32, CmpInst::ICMP_EQ}};
  FCmp32Libcalls[CmpInst::FCMP_UGE] = {{RTLIB::OLT_F32, CmpInst::ICMP_SGE}};
  FCmp32Libcalls[CmpInst::FCMP_UGT] = {{RTLIB::OLE_F32, CmpInst::ICMP_SGT}};
  FCmp32Libcalls[CmpInst::FCMP_ULE] = {{RTLIB::OGT_F32, CmpInst::ICMP_SLE}};
  FCmp32Libcalls[CmpInst::FCMP_ULT] = {{RTLIB::OGE_F32, CmpInst::ICMP_SLT}};
  FCmp32Libcalls[CmpInst::FCMP_UNE] = {{RTLIB::UNE_F32, CmpInst::ICMP_NE}};
  FCmp32Libcalls[CmpInst::FCMP_UNO] = {{RTLIB::UO_F32, CmpInst::ICMP_NE}};
  FCmp32Libcalls[CmpInst::FCMP_ONE] = {{RTLIB::OGT_F32, CmpInst::ICMP_SGT},
                                       {RTLIB::OLT_F32, CmpInst::ICMP_SLT}};
  FCmp32Libcalls[CmpInst::FCMP_UEQ] = {{RTLIB::OEQ_F32, CmpInst::ICMP_EQ},
                                       {RTLIB::UO_F32, CmpInst::ICMP_NE}};

  FCmp64Libcalls.resize(CmpInst::LAST_FCMP_PREDICATE + 1);
  FCmp64Libcalls[CmpInst::FCMP_OEQ] = {{RTLIB::OEQ_F64, CmpInst::ICMP_EQ}};
  FCmp64Libcalls[CmpInst::FCMP_OGE] = {{RTLIB::OGE_F64, CmpInst::ICMP_SGE}};
  FCmp64Libcalls[CmpInst::FCMP_OGT] = {{RTLIB::OGT_F64, CmpInst::ICMP_SGT}};
  FCmp64Libcalls[CmpInst::FCMP_OLE] = {{RTLIB::OLE_F64, CmpInst::ICMP_SLE}};
  FCmp64Libcalls[CmpInst::FCMP_OLT] = {{RTLIB::OLT_F64, CmpInst::ICMP_SLT}};
  FCmp64Libcalls[CmpInst::FCMP_ORD] = {{RTLIB::UO_F64, CmpInst::ICMP_EQ}};
  FCmp64Libcalls[CmpInst::FCMP_UGE] = {{RTLIB::OLT_F64, CmpInst::ICMP_SGE}};
  FCmp64Libcalls[CmpInst::FCMP_UGT] = {{RTLIB::OLE_F64, CmpInst::ICMP_SGT}};
  FCmp64Libcalls[CmpInst::FCMP_ULE] = {{RTLIB::OGT_F64, CmpInst::ICMP_SLE}};
  FCmp64Libcalls[CmpInst::FCMP_ULT] = {{RTLIB::OGE_F64, CmpInst::ICMP_SLT}};
  FCmp64Libcalls[CmpInst::FCMP_UNE] = {{RTLIB::UNE_F64, CmpInst::ICMP_NE}};
  FCmp64Libcalls[CmpInst::FCMP_UNO] = {{RTLIB::UO_F64, CmpInst::ICMP_NE}};
  FCmp64Libcalls[CmpInst::FCMP_ONE] = {{RTLIB::OGT_F64, CmpInst::ICMP_SGT},
                                       {RTLIB::OLT_F64, CmpInst::ICMP_SLT}};
  FCmp64Libcalls[CmpInst::FCMP_UEQ] = {{RTLIB::OEQ_F64, CmpInst::ICMP_EQ},
                                       {RTLIB::UO_F64, CmpInst::ICMP_NE}};
}

MOSLegalizerInfo::FCmpLibcallsList
MOSLegalizerInfo::getFCmpLibcalls(CmpInst::Predicate Predicate,
                                  unsigned Size) const {
  assert(CmpInst::isFPPredicate(Predicate) && "Unsupported FCmp predicate");
  if (Size == 32)
    return FCmp32Libcalls[Predicate];
  if (Size == 64)
    return FCmp64Libcalls[Predicate];
  llvm_unreachable("Unsupported size for FCmp predicate");
}
