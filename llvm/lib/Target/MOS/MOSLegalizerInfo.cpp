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
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/PseudoSourceValue.h"
#include "llvm/CodeGen/TargetOpcodes.h"
#include "llvm/IR/GlobalVariable.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

MOSLegalizerInfo::MOSLegalizerInfo() {
  using namespace TargetOpcode;
  using namespace LegalityPredicates;
  using namespace LegalizeMutations;

  LLT S1 = LLT::scalar(1);
  LLT S8 = LLT::scalar(8);
  LLT S16 = LLT::scalar(16);
  LLT S32 = LLT::scalar(32);
  LLT S64 = LLT::scalar(64);
  LLT P = LLT::pointer(0, 16);

  // Constants

  // Any type that can be operated on directly can be undef.
  getActionDefinitionsBuilder(G_IMPLICIT_DEF)
      .legalFor({S1, S8, P})
      .clampScalar(0, S8, S8);

  // S16 is legal because of the absolute addressing mode.
  getActionDefinitionsBuilder(G_CONSTANT)
      .legalFor({S1, S8, S16})
      .clampScalar(0, S8, S8);

  getActionDefinitionsBuilder({G_FRAME_INDEX, G_GLOBAL_VALUE}).legalFor({P});

  // Integer Extension and Truncation

  // Narrowing ZEXT to 8 bits should remove it entirely.
  getActionDefinitionsBuilder(G_ZEXT).clampScalar(0, S8, S8).unsupported();

  // Type Conversions

  getActionDefinitionsBuilder(G_INTTOPTR).legalFor({{P, S16}});
  getActionDefinitionsBuilder(G_PTRTOINT).legalFor({{S16, P}});

  // Scalar Operations

  getActionDefinitionsBuilder(G_MERGE_VALUES).legalFor({{S16, S8}});
  getActionDefinitionsBuilder(G_UNMERGE_VALUES).legalFor({{S8, S16}});

  // Integer Operations

  getActionDefinitionsBuilder({G_ADD, G_SUB, G_AND, G_OR, G_XOR})
      .legalFor({S8})
      .clampScalar(0, S8, S8);

  getActionDefinitionsBuilder(
      {G_MUL, G_SDIV, G_SREM, G_UDIV, G_UREM, G_CTLZ_ZERO_UNDEF})
      .libcall();

  getActionDefinitionsBuilder(G_ASHR).legalFor({S8}).clampScalar(0, S8, S8);
  getActionDefinitionsBuilder(G_SHL).customFor({S8, S16, S32, S64});

  // FIXME: The default narrowing of G_ICMP is terrible.
  getActionDefinitionsBuilder(G_ICMP)
      .legalFor({{S1, S8}})
      .minScalar(1, S8)
      .narrowScalarFor({{S1, S16}}, changeTo(1, S8))
      .narrowScalarFor({{S1, S32}}, changeTo(1, S16))
      .narrowScalarFor({{S1, S64}}, changeTo(1, S32));

  getActionDefinitionsBuilder(G_SELECT)
      .legalFor({{S8, S1}})
      .clampScalar(0, S8, S8);

  // It's legal to G_PTR_ADD an 8-bit integer to a pointer, since there is at
  // least one addressing mode that performs this directly. The legalizer
  // endeavors to avoid producing G_PTR_ADDs where this addressing mode does not
  // apply, but it cannot always, so the instruction handler needs to handle
  // general 8-bit G_PTR_ADDs.
  getActionDefinitionsBuilder(G_PTR_ADD).legalFor({{P, S8}}).customFor(
      {{P, S16}});

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

  getActionDefinitionsBuilder({G_MEMCPY, G_MEMMOVE, G_MEMSET}).libcall();

  // Control Flow

  getActionDefinitionsBuilder(G_PHI).legalFor({P, S8}).clampScalar(0, S8, S8);

  getActionDefinitionsBuilder(G_BRCOND).legalFor({S1});

  // Variadic Arguments

  getActionDefinitionsBuilder({G_VASTART, G_VAARG}).custom();

  // Other Operations

  getActionDefinitionsBuilder(G_DYN_STACKALLOC).lower();

  computeTables();
}

bool MOSLegalizerInfo::legalizeCustom(LegalizerHelper &Helper,
                                      MachineInstr &MI) const {
  using namespace TargetOpcode;
  MachineRegisterInfo &MRI = MI.getMF()->getRegInfo();

  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Invalid opcode for custom legalization.");
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
  }
}

// Load pointers by loading a 16-bit integer, then converting to pointer. This
// allows the 16-bit loads to be reduced to a pair of 8-bit loads.
bool MOSLegalizerInfo::legalizeLoad(LegalizerHelper &Helper,
                                    MachineRegisterInfo &MRI,
                                    MachineInstr &MI) const {
  using namespace TargetOpcode;

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
  using namespace TargetOpcode;

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
  using namespace TargetOpcode;

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
  using namespace TargetOpcode;

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
  using namespace TargetOpcode;

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
  using namespace TargetOpcode;

  assert(MI.getOpcode() == G_VAARG);

  MachineIRBuilder &Builder = Helper.MIRBuilder;
  MachineFunction &MF = Builder.getMF();

  Register Dst = MI.getOperand(0).getReg();
  Register VaListPtr = MI.getOperand(1).getReg();

  LLT p = LLT::pointer(0, 16);

  // Load the current VAArg address out of the VAList.
  MachineMemOperand *AddrLoadMMO = MF.getMachineMemOperand(
      MachinePointerInfo::getUnknownStack(MF),
      MachineMemOperand::MOLoad | MachineMemOperand::MOInvariant, 2, Align());
  Register Addr = Builder.buildLoad(p, VaListPtr, *AddrLoadMMO).getReg(0);

  // Load the argument value out of the current VAArg address;
  unsigned Size = MRI.getType(Dst).getSizeInBytes();
  MachineMemOperand *ValueMMO = MF.getMachineMemOperand(
      MachinePointerInfo::getUnknownStack(MF),
      MachineMemOperand::MOLoad | MachineMemOperand::MOInvariant, Size,
      Align());
  Builder.buildLoad(Dst, Addr, *ValueMMO);

  // Increment the current VAArg address.
  Register SizeReg = Builder.buildConstant(LLT::scalar(16), Size).getReg(0);
  Register NextAddr = Builder.buildPtrAdd(p, Addr, SizeReg).getReg(0);
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
  using namespace TargetOpcode;

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
