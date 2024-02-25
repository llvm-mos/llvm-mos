//===-- MOSInstructionSelector.cpp - MOS Instruction Selector -------------===
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
#include "MOS.h"
#include "MOSFrameLowering.h"
#include "MOSInstrBuilder.h"
#include "MOSRegisterInfo.h"
#include "MOSSubtarget.h"

#include "llvm/ADT/APFloat.h"
#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/CodeGen/GlobalISel/GIMatchTableExecutorImpl.h"
#include "llvm/CodeGen/GlobalISel/GenericMachineInstrs.h"
#include "llvm/CodeGen/GlobalISel/InstructionSelector.h"
#include "llvm/CodeGen/GlobalISel/MIPatternMatch.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/Register.h"
#include "llvm/CodeGen/RegisterBankInfo.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/InstrTypes.h"
#include "llvm/IR/Instruction.h"
#include "llvm/ObjectYAML/MachOYAML.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;
using namespace MIPatternMatch;

#define DEBUG_TYPE "mos-isel"

namespace {

#define GET_GLOBALISEL_PREDICATE_BITSET
#include "MOSGenGlobalISel.inc"
#undef GET_GLOBALISEL_PREDICATE_BITSET

class MOSInstructionSelector : public InstructionSelector {
public:
  MOSInstructionSelector(const MOSTargetMachine &TM, MOSSubtarget &STI,
                         MOSRegisterBankInfo &RBI);

  void setupMF(MachineFunction &MF, GISelKnownBits *KB,
               CodeGenCoverage *CovInfo, ProfileSummaryInfo *PSI,
               BlockFrequencyInfo *BFI, AAResults *AA) override;

  bool select(MachineInstr &MI) override;
  static const char *getName() { return DEBUG_TYPE; }

private:
  const MOSSubtarget &STI;
  const MOSInstrInfo &TII;
  const MOSRegisterInfo &TRI;
  const MOSRegisterBankInfo &RBI;

  // Pre-tablegen selection functions. If these return false, fall through to
  // tablegen.
  bool selectAddSub(MachineInstr &MI);
  bool selectLogical(MachineInstr &MI);

  // Post-tablegen selection functions. If these return false, it is an error.
  bool selectBrCondImm(MachineInstr &MI);
  bool selectSbc(MachineInstr &MI);
  bool selectFrameIndex(MachineInstr &MI);
  std::pair<Register, Register> selectFrameIndexLoHi(MachineInstr &MI);
  bool selectAddr(MachineInstr &MI);
  std::pair<Register, Register> selectAddrLoHi(MachineInstr &MI);
  bool selectStore(MachineInstr &MI);
  bool selectRMW(MachineInstr &MI);
  bool selectLshrShlE(MachineInstr &MI);
  bool selectMergeValues(MachineInstr &MI);
  bool selectTrunc(MachineInstr &MI);
  bool selectAddE(MachineInstr &MI);
  bool selectIncDecMB(MachineInstr &MI);
  bool selectUnMergeValues(MachineInstr &MI);
  bool selectBrIndirect(MachineInstr &MI);

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

void MOSInstructionSelector::setupMF(MachineFunction &MF, GISelKnownBits *KB,
                                     CodeGenCoverage *CovInfo,
                                     ProfileSummaryInfo *PSI,
                                     BlockFrequencyInfo *BFI, AAResults *AA) {
  InstructionSelector::setupMF(MF, KB, CovInfo, PSI, BFI, AA);

  // The machine verifier doesn't allow COPY instructions to have differing
  // types, but the various GlobalISel utilities used in the instruction
  // selector really need to be able to look through G_PTRTOINT and G_INTTOPTR
  // as if they were copies. To avoid maintaining separate versions of these, we
  // temporarily lower these to technically-illegal COPY instructions, but only
  // for the duration of this one pass.
  for (MachineBasicBlock &MBB : MF) {
    for (MachineInstr &MI : MBB) {
      switch (MI.getOpcode()) {
      case MOS::G_PTRTOINT:
      case MOS::G_INTTOPTR:
        MI.setDesc(TII.get(MOS::COPY));
        break;
      }
    }
  }
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
  case MOS::G_SUB:
    if (selectAddSub(MI))
      return true;
    break;
  case MOS::G_AND:
  case MOS::G_OR:
  case MOS::G_XOR:
    if (selectLogical(MI))
      return true;
    break;
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
  case MOS::G_CONSTANT:
  case MOS::G_GLOBAL_VALUE:
    return selectAddr(MI);
  case MOS::G_STORE_ZP_IDX:
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
  case MOS::G_INC:
  case MOS::G_DEC:
  case MOS::G_INC_TMP:
  case MOS::G_DEC_TMP:
    return selectIncDecMB(MI);
  case MOS::G_UNMERGE_VALUES:
    return selectUnMergeValues(MI);

  case MOS::G_BRINDIRECT:
    return selectBrIndirect(MI);

  case MOS::G_IMPLICIT_DEF:
  case MOS::G_LOAD_ZP_IDX:
  case MOS::G_LOAD_ABS:
  case MOS::G_LOAD_ABS_IDX:
  case MOS::G_LOAD_INDIR:
  case MOS::G_LOAD_INDIR_IDX:
  case MOS::G_PHI:
  case MOS::G_STORE_INDIR:
  case MOS::G_STORE_INDIR_IDX:
  case MOS::G_BRINDIRECT_IDX:
    return selectGeneric(MI);
  }
}

static bool shouldFoldMemAccess(const MachineInstr &Dst,
                                const MachineInstr &Src, AAResults *AA) {
  assert(Src.mayLoadOrStore());

  // For now, don't attempt to fold across basic block boundaries.
  if (Dst.getParent() != Src.getParent())
    return false;

  if ((*Src.memoperands_begin())->isVolatile())
    return false;

  // Does it pay off to fold the access? Depends on the number of users.
  const auto &STI = Dst.getMF()->getSubtarget<MOSSubtarget>();
  const auto &MRI = Dst.getMF()->getRegInfo();
  const auto Users = MRI.use_nodbg_instructions(Src.getOperand(0).getReg());
  const auto NumUsers = std::distance(Users.begin(), Users.end());

  // Looking at this pessimistically, if we don't fold the access, all
  // references may refer to an Imag8 reg that needs to be copied to/from a GPR.
  // This costs 2 bytes and 3 cycles. We also need to do the actual load/store.
  // If we do fold the access, then we get rid of both that and the load/store.
  // This makes the first reference free; as it's not any more expensive than
  // the load/store. However, for each reference past the first, we pay an
  // overhead for using the addressing over the imaginary addressing mode. This
  // cost is: Absolute: 1 byte, 1 cycle Absolute Indexed: 1 byte, 1.5 cycles
  // Indirect: 2 cycles Indirect Indexed: 2.5 cycles
  // So, it pays off to fold k references of each addressing mode if:
  // Absolute: k*(1+1) < (2+3) = 5; 2k < 5; k < 2.5; k <= 2
  // Absolute Indexed: k*(1+1.5) < 5; 2.5k < 5; k <= 1
  // Indirect: k*(0+2) < 5; 2k < 5; k <= 2
  // Indirect Indexed: k*(0+2.5) < 5; 2.5k < 5; k <= 1
  //
  // For HuC6280, we have different timings, so:
  // Absolute: k*(1+2) < (2+3) = 5; 3k < 5; k <= 1
  // Absolute Indexed: k*(1+2.5) < 5; 3.5k < 5; k <= 1
  // Indirect: k*(0+4) < 5; 4k < 5; k <= 1
  // Indirect Indexed: k*(0+4.5) < 5; 4.5k < 5; k <= 1
  //
  // For SPC700:
  // Absolute: k*(1+1) < (2+3) = 5; 2k < 5; k < 2.5; k <= 2
  // Absolute Indexed: k*(1+2) < 5; 3k < 5; k <= 1
  // Indirect: k*(0+3) < 5; 3k < 5; k <= 1
  // Indirect Indexed: k*(0+3) < 5; 3k < 5; k <= 1
  int MaxNumUsers;
  switch (Src.getOpcode()) {
  default:
    MaxNumUsers = 1;
    break;
  case MOS::G_LOAD_ABS:
    MaxNumUsers = STI.hasHUC6280() ? 1 : 2;
    break;
  case MOS::G_LOAD_INDIR:
    MaxNumUsers = (STI.hasHUC6280() || STI.hasSPC700()) ? 1 : 2;
    break;
  }
  if (NumUsers > MaxNumUsers)
    return false;

  // Look for intervening instructions that cannot be folded across.
  for (const MachineInstr &I :
       make_range(std::next(MachineBasicBlock::const_iterator(Src)),
                  MachineBasicBlock::const_iterator(Dst))) {
    if (I.isCall() || I.hasUnmodeledSideEffects())
      return false;
    if (I.mayLoadOrStore()) {
      if (Src.hasOrderedMemoryRef() || I.hasOrderedMemoryRef())
        return false;
      if (I.mayAlias(AA, Src, /*UseTBAA=*/true))
        return false;
      // Note: Dst may be a store, indicating that the whole sequence is a RMW
      // operation.
      if (I.mayAlias(AA, Dst, /*UseTBAA=*/true))
        return false;
    }
  }

  return true;
}

struct FoldedLdAbs_match {
  const MachineInstr &Tgt;
  MachineOperand &Addr;
  AAResults *AA;

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

struct FoldedLdIdx_match {
  const MachineInstr &Tgt;
  MachineOperand &Addr;
  Register &Idx;
  bool &ZP;
  AAResults *AA;

  bool match(const MachineRegisterInfo &MRI, Register Reg) {
    const MachineInstr *LDZpIdx = getOpcodeDef(MOS::G_LOAD_ZP_IDX, Reg, MRI);
    if (LDZpIdx) {
      if (!shouldFoldMemAccess(Tgt, *LDZpIdx, AA))
        return false;
      ZP = true;
      Addr = LDZpIdx->getOperand(1);
      Idx = LDZpIdx->getOperand(2).getReg();
      return true;
    }

    const MachineInstr *LDAbsIdx = getOpcodeDef(MOS::G_LOAD_ABS_IDX, Reg, MRI);
    if (LDAbsIdx) {
      if (!shouldFoldMemAccess(Tgt, *LDAbsIdx, AA))
        return false;
      ZP = false;
      Addr = LDAbsIdx->getOperand(1);
      Idx = LDAbsIdx->getOperand(2).getReg();
      return true;
    }

    return false;
  }
};
inline FoldedLdIdx_match m_FoldedLdIdx(const MachineInstr &Tgt,
                                       MachineOperand &Addr, Register &Idx,
                                       bool &ZP, AAResults *AA) {
  return {Tgt, Addr, Idx, ZP, AA};
}

struct FoldedLdIndir_match {
  const MachineInstr &Tgt;
  Register &Addr;
  AAResults *AA;

  bool match(const MachineRegisterInfo &MRI, Register Reg) {
    const MachineInstr *LdIndir = getOpcodeDef(MOS::G_LOAD_INDIR, Reg, MRI);
    if (!LdIndir || !shouldFoldMemAccess(Tgt, *LdIndir, AA))
      return false;
    Addr = LdIndir->getOperand(1).getReg();
    return true;
  }
};
inline FoldedLdIndir_match m_FoldedLdIndir(const MachineInstr &Tgt,
                                           Register &Addr, AAResults *AA) {
  return {Tgt, Addr, AA};
}

struct FoldedLdIndirIdx_match {
  const MachineInstr &Tgt;
  Register &Addr;
  Register &Idx;
  AAResults *AA;

  bool match(const MachineRegisterInfo &MRI, Register Reg) {
    const MachineInstr *LdIndirIdx =
        getOpcodeDef(MOS::G_LOAD_INDIR_IDX, Reg, MRI);
    if (!LdIndirIdx || !shouldFoldMemAccess(Tgt, *LdIndirIdx, AA))
      return false;
    Addr = LdIndirIdx->getOperand(1).getReg();
    Idx = LdIndirIdx->getOperand(2).getReg();
    return true;
  }
};
inline FoldedLdIndirIdx_match m_FoldedLdIndirIdx(const MachineInstr &Tgt,
                                                 Register &Addr, Register &Idx,
                                                 AAResults *AA) {
  return {Tgt, Addr, Idx, AA};
}

bool MOSInstructionSelector::selectAddSub(MachineInstr &MI) {
  assert(MI.getOpcode() == MOS::G_ADD || MI.getOpcode() == MOS::G_SUB);

  MachineIRBuilder Builder(MI);
  auto &MRI = *Builder.getMRI();

  Register Dst = MI.getOperand(0).getReg();

  LLT S1 = LLT::scalar(1);

  if (auto RHSConst =
          getIConstantVRegValWithLookThrough(MI.getOperand(2).getReg(), MRI)) {
    // Don't inhibit generation of INC/DEC.
    if (RHSConst->Value.abs().isOne())
      return false;
  }

  int64_t CarryInVal = MI.getOpcode() == MOS::G_ADD ? 0 : -1;

  bool Success;

  MachineInstr *Load;

  Register LHS;
  MachineOperand Addr = MachineOperand::CreateReg(0, false);
  unsigned Opcode;
  if (MI.getOpcode() == MOS::G_ADD) {
    Success =
        mi_match(Dst, MRI,
                 m_GAdd(m_Reg(LHS),
                        m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA))));
    Opcode = MOS::ADCAbs;
  } else {
    Success =
        mi_match(Dst, MRI,
                 m_GSub(m_Reg(LHS),
                        m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA))));
    Opcode = MOS::SBCAbs;
  }

  if (Success) {
    Register CIn =
        Builder.buildInstr(MOS::LDCImm, {S1}, {CarryInVal}).getReg(0);
    auto Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addDef(MRI.createGenericVirtualRegister(S1))
                     .addDef(MRI.createGenericVirtualRegister(S1))
                     .addUse(LHS)
                     .add(Addr)
                     .addUse(CIn)
                     .cloneMemRefs(*Load);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain absolute instruction.");
    MI.eraseFromParent();
    return true;
  }

  Register Idx;
  bool ZP;
  if (MI.getOpcode() == MOS::G_ADD) {
    Success = mi_match(
        Dst, MRI,
        m_GAdd(m_Reg(LHS),
               m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Addr, Idx, ZP, AA))));
    Opcode = ZP ? MOS::ADCZpIdx : MOS::ADCAbsIdx;
  } else {
    Success = mi_match(
        Dst, MRI,
        m_GSub(m_Reg(LHS),
               m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Addr, Idx, ZP, AA))));
    Opcode = ZP ? MOS::SBCZpIdx : MOS::SBCAbsIdx;
  }
  if (Success) {
    Register CIn =
        Builder.buildInstr(MOS::LDCImm, {S1}, {CarryInVal}).getReg(0);
    auto Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addDef(MRI.createGenericVirtualRegister(S1))
                     .addDef(MRI.createGenericVirtualRegister(S1))
                     .addUse(LHS)
                     .add(Addr)
                     .addUse(Idx)
                     .addUse(CIn)
                     .cloneMemRefs(*Load);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain absolute indexed instruction.");
    MI.eraseFromParent();
    return true;
  }

  Register IndirAddr;
  if (MI.getOpcode() == MOS::G_ADD) {
    Success = mi_match(
        Dst, MRI,
        m_GAdd(m_Reg(LHS),
               m_all_of(m_MInstr(Load), m_FoldedLdIndir(MI, IndirAddr, AA))));
    Opcode = MOS::ADCIndir;
  } else {
    Success = mi_match(
        Dst, MRI,
        m_GSub(m_Reg(LHS),
               m_all_of(m_MInstr(Load), m_FoldedLdIndir(MI, IndirAddr, AA))));
    Opcode = MOS::SBCIndir;
  }
  if (Success) {
    Register CIn =
        Builder.buildInstr(MOS::LDCImm, {S1}, {CarryInVal}).getReg(0);
    auto Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addDef(MRI.createGenericVirtualRegister(S1))
                     .addDef(MRI.createGenericVirtualRegister(S1))
                     .addUse(LHS)
                     .addUse(IndirAddr)
                     .addUse(CIn)
                     .cloneMemRefs(*Load);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain indirect instruction.");
    MI.eraseFromParent();
    return true;
  }

  if (MI.getOpcode() == MOS::G_ADD) {
    Success =
        mi_match(Dst, MRI,
                 m_GAdd(m_Reg(LHS),
                        m_all_of(m_MInstr(Load),
                                 m_FoldedLdIndirIdx(MI, IndirAddr, Idx, AA))));
    Opcode = MOS::ADCIndirIdx;
  } else {
    Success =
        mi_match(Dst, MRI,
                 m_GSub(m_Reg(LHS),
                        m_all_of(m_MInstr(Load),
                                 m_FoldedLdIndirIdx(MI, IndirAddr, Idx, AA))));
    Opcode = MOS::SBCIndirIdx;
  }
  if (Success) {
    Register CIn =
        Builder.buildInstr(MOS::LDCImm, {S1}, {CarryInVal}).getReg(0);
    auto Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addDef(MRI.createGenericVirtualRegister(S1))
                     .addDef(MRI.createGenericVirtualRegister(S1))
                     .addUse(LHS)
                     .addUse(IndirAddr)
                     .addUse(Idx)
                     .addUse(CIn)
                     .cloneMemRefs(*Load);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain indirect indexed instruction.");
    MI.eraseFromParent();
    return true;
  }

  return false;
}

bool MOSInstructionSelector::selectLogical(MachineInstr &MI) {
  MachineIRBuilder Builder(MI);
  auto &MRI = *Builder.getMRI();

  Register Dst = MI.getOperand(0).getReg();
  Register LHS;
  MachineOperand Addr = MachineOperand::CreateReg(0, false);

  MachineInstr *Load;

  bool Success;
  Register Opcode;
  switch (MI.getOpcode()) {
  case MOS::G_AND:
    Success =
        mi_match(Dst, MRI,
                 m_GAnd(m_Reg(LHS),
                        m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA))));
    Opcode = MOS::ANDAbs;
    break;
  case MOS::G_XOR:
    Success =
        mi_match(Dst, MRI,
                 m_GXor(m_Reg(LHS),
                        m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA))));
    Opcode = MOS::EORAbs;
    break;
  case MOS::G_OR:
    Success =
        mi_match(Dst, MRI,
                 m_GOr(m_Reg(LHS),
                       m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA))));
    Opcode = MOS::ORAAbs;
    break;
  }
  if (Success) {
    auto Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addUse(LHS)
                     .add(Addr)
                     .cloneMemRefs(*Load);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain absolute logical instruction.");
    MI.eraseFromParent();
    return true;
  }

  Register Idx;
  bool ZP;
  switch (MI.getOpcode()) {
  case MOS::G_AND:
    Success = mi_match(
        Dst, MRI,
        m_GAnd(m_Reg(LHS),
               m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Addr, Idx, ZP, AA))));
    Opcode = ZP ? MOS::ANDZpIdx : MOS::ANDAbsIdx;
    break;
  case MOS::G_XOR:
    Success = mi_match(
        Dst, MRI,
        m_GXor(m_Reg(LHS),
               m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Addr, Idx, ZP, AA))));
    Opcode = ZP ? MOS::EORZpIdx : MOS::EORAbsIdx;
    break;
  case MOS::G_OR:
    Success = mi_match(
        Dst, MRI,
        m_GOr(m_Reg(LHS),
              m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Addr, Idx, ZP, AA))));
    Opcode = ZP ? MOS::ORAZpIdx : MOS::ORAAbsIdx;
    break;
  }
  if (Success) {
    auto Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addUse(LHS)
                     .add(Addr)
                     .addUse(Idx)
                     .cloneMemRefs(*Load);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable(
          "Could not constrain absolute indexed logical instruction.");
    MI.eraseFromParent();
    return true;
  }

  Register IndirAddr;
  switch (MI.getOpcode()) {
  case MOS::G_AND:
    Success = mi_match(
        Dst, MRI,
        m_GAnd(m_Reg(LHS),
               m_all_of(m_MInstr(Load), m_FoldedLdIndir(MI, IndirAddr, AA))));
    Opcode = MOS::ANDIndir;
    break;
  case MOS::G_XOR:
    Success = mi_match(
        Dst, MRI,
        m_GXor(m_Reg(LHS),
               m_all_of(m_MInstr(Load), m_FoldedLdIndir(MI, IndirAddr, AA))));
    Opcode = MOS::EORIndir;
    break;
  case MOS::G_OR:
    Success = mi_match(
        Dst, MRI,
        m_GOr(m_Reg(LHS),
              m_all_of(m_MInstr(Load), m_FoldedLdIndir(MI, IndirAddr, AA))));
    Opcode = MOS::ORAIndir;
    break;
  }
  if (Success) {
    auto Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addUse(LHS)
                     .addUse(IndirAddr)
                     .cloneMemRefs(*Load);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain indirect logical instruction.");
    MI.eraseFromParent();
    return true;
  }

  switch (MI.getOpcode()) {
  case MOS::G_AND:
    Success =
        mi_match(Dst, MRI,
                 m_GAnd(m_Reg(LHS),
                        m_all_of(m_MInstr(Load),
                                 m_FoldedLdIndirIdx(MI, IndirAddr, Idx, AA))));
    Opcode = MOS::ANDIndirIdx;
    break;
  case MOS::G_XOR:
    Success =
        mi_match(Dst, MRI,
                 m_GXor(m_Reg(LHS),
                        m_all_of(m_MInstr(Load),
                                 m_FoldedLdIndirIdx(MI, IndirAddr, Idx, AA))));
    Opcode = MOS::EORIndirIdx;
    break;
  case MOS::G_OR:
    Success =
        mi_match(Dst, MRI,
                 m_GOr(m_Reg(LHS),
                       m_all_of(m_MInstr(Load),
                                m_FoldedLdIndirIdx(MI, IndirAddr, Idx, AA))));
    Opcode = MOS::ORAIndirIdx;
    break;
  }
  if (Success) {
    auto Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addUse(LHS)
                     .addUse(IndirAddr)
                     .addUse(Idx)
                     .cloneMemRefs(*Load);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable(
          "Could not constrain indirect indexed logical instruction.");
    MI.eraseFromParent();
    return true;
  }

  return false;
}

// Given a G_SBC instruction Sbc and one of its flag output virtual registers,
// returns the flag that corresponds to the register.
static Register getSbcFlagForRegister(const MachineInstr &Sbc, Register Reg) {
  static const Register Flags[] = {MOS::C, MOS::N, MOS::V, MOS::Z};
  // TODO: C++17 structured bindings
  for (const auto &I : zip(Flags, seq(1, 5)))
    if (Sbc.getOperand(std::get<1>(I)).getReg() == Reg)
      return std::get<0>(I);
  llvm_unreachable("Could not find register in G_SBC outputs.");
}

// Match criteria common to all Cmp where N or Z are used.
struct CmpNZ_match {
  Register &LHS;
  Register &Flag;

  // The matched G_SBC representing a CMP.
  MachineInstr *CondMI;

  CmpNZ_match(Register &LHS, Register &Flag) : LHS(LHS), Flag(Flag) {}

  bool match(const MachineRegisterInfo &MRI, Register CondReg) {
    auto DefSrcReg = getDefSrcRegIgnoringCopies(CondReg, MRI);
    CondMI = DefSrcReg->MI;
    if (CondMI->getOpcode() != MOS::G_SBC)
      return false;

    auto CInConst =
        getIConstantVRegValWithLookThrough(CondMI->getOperand(7).getReg(), MRI);
    if (!CInConst || CInConst->Value.isZero())
      return false;

    LHS = CondMI->getOperand(5).getReg();
    Flag = getSbcFlagForRegister(*CondMI, DefSrcReg->Reg);
    return Flag == MOS::N || Flag == MOS::Z;
  }
};

struct CmpNZZero_match : public CmpNZ_match {
  CmpNZZero_match(Register &LHS, Register &Flag) : CmpNZ_match(LHS, Flag) {}

  bool match(const MachineRegisterInfo &MRI, Register CondReg) {
    if (!CmpNZ_match::match(MRI, CondReg))
      return false;

    auto RHSConst =
        getIConstantVRegValWithLookThrough(CondMI->getOperand(6).getReg(), MRI);
    return RHSConst && RHSConst->Value.isZero();
  }
};

inline CmpNZZero_match m_CmpNZZero(Register &LHS, Register &Flag) {
  return {LHS, Flag};
}

struct CmpNZImm_match : public CmpNZ_match {
  int64_t &RHS;

  CmpNZImm_match(Register &LHS, int64_t &RHS, Register &Flag)
      : CmpNZ_match(LHS, Flag), RHS(RHS) {}

  bool match(const MachineRegisterInfo &MRI, Register CondReg) {
    if (!CmpNZ_match::match(MRI, CondReg))
      return false;

    auto RHSConst =
        getIConstantVRegValWithLookThrough(CondMI->getOperand(6).getReg(), MRI);
    if (!RHSConst)
      return false;

    RHS = RHSConst->Value.getZExtValue();
    return true;
  }
};

// Match one of the outputs of a G_SBC to a CmpNZImm operation. LHS and RHS
// are the left and right hand side of the comparison, while Flag is the
// physical (N or Z) register corresponding to the output by which the G_SBC
// was reached.
inline CmpNZImm_match m_CmpNZImm(Register &LHS, int64_t &RHS, Register &Flag) {
  return {LHS, RHS, Flag};
}

struct CmpNZImag8_match : public CmpNZ_match {
  Register &RHS;

  CmpNZImag8_match(Register &LHS, Register &RHS, Register &Flag)
      : CmpNZ_match(LHS, Flag), RHS(RHS) {}

  bool match(const MachineRegisterInfo &MRI, Register CondReg) {
    if (!CmpNZ_match::match(MRI, CondReg))
      return false;
    RHS = CondMI->getOperand(6).getReg();
    return true;
  }
};

// Match one of the outputs of a G_SBC to a CmpNZImag8 operation. LHS and
// RHS are the left and right hand side of the comparison, while Flag is the
// physical (N or Z) register corresponding to the output by which the G_SBC
// was reached.
inline CmpNZImag8_match m_CmpNZImag8(Register &LHS, Register &RHS,
                                     Register &Flag) {
  return {LHS, RHS, Flag};
}

struct CmpNZAbs_match : public CmpNZ_match {
  MachineOperand &Addr;
  MachineInstr *&Load;
  AAResults *AA;

  CmpNZAbs_match(Register &LHS, MachineOperand &Addr, Register &Flag,
                 MachineInstr *&Load, AAResults *AA)
      : CmpNZ_match(LHS, Flag), Addr(Addr), Load(Load), AA(AA) {}

  bool match(const MachineRegisterInfo &MRI, Register CondReg) {
    if (!CmpNZ_match::match(MRI, CondReg))
      return false;
    return mi_match(CondMI->getOperand(6).getReg(), MRI,
                    m_all_of(m_MInstr(Load), m_FoldedLdAbs(*CondMI, Addr, AA)));
  }
};

// Match one of the outputs of a G_SBC to a CmpNZAbs operation. Flag is the
// physical (N or Z) register corresponding to the output by which the G_SBC
// was reached.
inline CmpNZAbs_match m_CmpNZAbs(Register &LHS, MachineOperand &Addr,
                                 Register &Flag, MachineInstr *&Load,
                                 AAResults *AA) {
  return {LHS, Addr, Flag, Load, AA};
}

struct CmpNZIdx_match : public CmpNZ_match {
  MachineOperand &Addr;
  Register &Idx;
  MachineInstr *&Load;
  bool &ZP;
  AAResults *AA;

  CmpNZIdx_match(Register &LHS, MachineOperand &Addr, Register &Idx,
                 Register &Flag, MachineInstr *&Load, bool &ZP, AAResults *AA)
      : CmpNZ_match(LHS, Flag), Addr(Addr), Idx(Idx), Load(Load), ZP(ZP),
        AA(AA) {}

  bool match(const MachineRegisterInfo &MRI, Register CondReg) {
    if (!CmpNZ_match::match(MRI, CondReg))
      return false;
    return mi_match(
        CondMI->getOperand(6).getReg(), MRI,
        m_all_of(m_MInstr(Load), m_FoldedLdIdx(*CondMI, Addr, Idx, ZP, AA)));
  }
};

// Match one of the outputs of a G_SBC to a CmpNZIdx operation. Flag is the
// physical (N or Z) register corresponding to the output by which the G_SBC
// was reached.
inline CmpNZIdx_match m_CmpNZIdx(Register &LHS, MachineOperand &Addr,
                                 Register &Idx, Register &Flag,
                                 MachineInstr *&Load, bool &ZP, AAResults *AA) {
  return {LHS, Addr, Idx, Flag, Load, ZP, AA};
}

struct CmpNZIndir_match : public CmpNZ_match {
  Register &Addr;
  MachineInstr *&Load;
  AAResults *AA;

  CmpNZIndir_match(Register &LHS, Register &Addr, Register &Flag,
                   MachineInstr *&Load, AAResults *AA)
      : CmpNZ_match(LHS, Flag), Addr(Addr), Load(Load), AA(AA) {}

  bool match(const MachineRegisterInfo &MRI, Register CondReg) {
    if (!CmpNZ_match::match(MRI, CondReg))
      return false;
    return mi_match(
        CondMI->getOperand(6).getReg(), MRI,
        m_all_of(m_MInstr(Load), m_FoldedLdIndir(*CondMI, Addr, AA)));
  }
};

// Match one of the outputs of a G_SBC to a CmpNZIndir operation. Flag is the
// physical (N or Z) register corresponding to the output by which the G_SBC
// was reached.
inline CmpNZIndir_match m_CmpNZIndir(Register &LHS, Register &Addr,
                                     Register &Flag, MachineInstr *&Load,
                                     AAResults *AA) {
  return {LHS, Addr, Flag, Load, AA};
}

struct CmpNZIndirIdx_match : public CmpNZ_match {
  Register &Addr;
  Register &Idx;
  MachineInstr *&Load;
  AAResults *AA;

  CmpNZIndirIdx_match(Register &LHS, Register &Addr, Register &Idx,
                      Register &Flag, MachineInstr *&Load, AAResults *AA)
      : CmpNZ_match(LHS, Flag), Addr(Addr), Idx(Idx), Load(Load), AA(AA) {}

  bool match(const MachineRegisterInfo &MRI, Register CondReg) {
    if (!CmpNZ_match::match(MRI, CondReg))
      return false;
    return mi_match(
        CondMI->getOperand(6).getReg(), MRI,
        m_all_of(m_MInstr(Load), m_FoldedLdIndirIdx(*CondMI, Addr, Idx, AA)));
  }
};

// Match one of the outputs of a G_SBC to a CmpNZIndirIdx operation. Flag is
// the physical (N or Z) register corresponding to the output by which the G_SBC
// was reached.
inline CmpNZIndirIdx_match m_CmpNZIndirIdx(Register &LHS, Register &Addr,
                                           Register &Idx, Register &Flag,
                                           MachineInstr *&Load, AAResults *AA) {
  return {LHS, Addr, Idx, Flag, Load, AA};
}

bool MOSInstructionSelector::selectBrCondImm(MachineInstr &MI) {
  MachineRegisterInfo &MRI = MI.getMF()->getRegInfo();

  Register CondReg = MI.getOperand(0).getReg();
  MachineBasicBlock *Tgt = MI.getOperand(1).getMBB();
  int64_t FlagVal = MI.getOperand(2).getImm();

  Register Flag;

  MachineIRBuilder Builder(MI);

  MachineInstr *Load;

  Register LHS;
  if (MachineInstr *CMPZ = getOpcodeDef(MOS::G_CMPZ, CondReg, MRI)) {
    auto Branch =
        Builder.buildInstr(MOS::CmpBrZeroMultiByte).addMBB(Tgt).addImm(FlagVal);
    for (const MachineOperand &MO : CMPZ->uses())
      Branch.addUse(MO.getReg());
    if (!constrainSelectedInstRegOperands(*Branch, TII, TRI, RBI))
      return false;
    MI.eraseFromParent();
    return true;
  }
  if (mi_match(CondReg, MRI, m_CmpNZZero(LHS, Flag))) {
    auto Branch = Builder.buildInstr(MOS::CmpBrZero)
                      .addMBB(Tgt)
                      .addUse(Flag, RegState::Undef)
                      .addImm(FlagVal)
                      .addUse(LHS);
    if (!constrainSelectedInstRegOperands(*Branch, TII, TRI, RBI))
      return false;
    MI.eraseFromParent();
    return true;
  }
  int64_t RHSConst;
  if (mi_match(CondReg, MRI, m_CmpNZImm(LHS, RHSConst, Flag))) {
    auto Branch = Builder.buildInstr(MOS::CmpBrImm)
                      .addMBB(Tgt)
                      .addUse(Flag, RegState::Undef)
                      .addImm(FlagVal)
                      .addUse(LHS)
                      .addImm(RHSConst);
    if (!constrainSelectedInstRegOperands(*Branch, TII, TRI, RBI))
      return false;
    MI.eraseFromParent();
    return true;
  }
  MachineOperand Addr =
      MachineOperand::CreateReg(MOS::NoRegister, /*isDef=*/false);
  if (mi_match(CondReg, MRI, m_CmpNZAbs(LHS, Addr, Flag, Load, AA))) {
    auto Branch = Builder.buildInstr(MOS::CmpBrAbs)
                      .addMBB(Tgt)
                      .addUse(Flag, RegState::Undef)
                      .addImm(FlagVal)
                      .addUse(LHS)
                      .add(Addr)
                      .cloneMemRefs(*Load);
    if (!constrainSelectedInstRegOperands(*Branch, TII, TRI, RBI))
      return false;
    MI.eraseFromParent();
    return true;
  }
  Register Idx;
  bool ZP;
  if (mi_match(CondReg, MRI, m_CmpNZIdx(LHS, Addr, Idx, Flag, Load, ZP, AA))) {
    auto Branch = Builder.buildInstr(ZP ? MOS::CmpBrZpIdx : MOS::CmpBrAbsIdx)
                      .addMBB(Tgt)
                      .addUse(Flag, RegState::Undef)
                      .addImm(FlagVal)
                      .addUse(LHS)
                      .add(Addr)
                      .addUse(Idx)
                      .cloneMemRefs(*Load);
    if (!constrainSelectedInstRegOperands(*Branch, TII, TRI, RBI))
      return false;
    MI.eraseFromParent();
    return true;
  }
  Register RegAddr;
  if (mi_match(CondReg, MRI, m_CmpNZIndir(LHS, RegAddr, Flag, Load, AA))) {
    auto Branch = Builder.buildInstr(MOS::CmpBrIndir)
                      .addMBB(Tgt)
                      .addUse(Flag, RegState::Undef)
                      .addImm(FlagVal)
                      .addUse(LHS)
                      .addUse(RegAddr)
                      .cloneMemRefs(*Load);
    if (!constrainSelectedInstRegOperands(*Branch, TII, TRI, RBI))
      return false;
    MI.eraseFromParent();
    return true;
  }
  if (mi_match(CondReg, MRI,
               m_CmpNZIndirIdx(LHS, RegAddr, Idx, Flag, Load, AA))) {
    auto Branch = Builder.buildInstr(MOS::CmpBrIndirIdx)
                      .addMBB(Tgt)
                      .addUse(Flag, RegState::Undef)
                      .addImm(FlagVal)
                      .addUse(LHS)
                      .addUse(RegAddr)
                      .addUse(Idx)
                      .cloneMemRefs(*Load);
    if (!constrainSelectedInstRegOperands(*Branch, TII, TRI, RBI))
      return false;
    MI.eraseFromParent();
    return true;
  }
  Register RHS;
  if (mi_match(CondReg, MRI, m_CmpNZImag8(LHS, RHS, Flag))) {
    auto Branch = Builder.buildInstr(MOS::CmpBrImag8)
                      .addMBB(Tgt)
                      .addUse(Flag, RegState::Undef)
                      .addImm(FlagVal)
                      .addUse(LHS)
                      .addUse(RHS);
    if (!constrainSelectedInstRegOperands(*Branch, TII, TRI, RBI))
      return false;
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

// Although some G_SBC instructions can be folded in to their (branch) uses,
// others need to be selected directly.
bool MOSInstructionSelector::selectSbc(MachineInstr &MI) {
  MachineIRBuilder Builder(MI);
  const auto &MRI = *Builder.getMRI();

  Register A = MI.getOperand(0).getReg();
  Register N = MI.getOperand(2).getReg();
  Register V = MI.getOperand(3).getReg();
  Register Z = MI.getOperand(4).getReg();
  Register R = MI.getOperand(6).getReg();

  // Outputs that are unused may not need to be generated.
  if (Builder.getMRI()->use_nodbg_empty(A))
    A = MOS::NoRegister;
  if (Builder.getMRI()->use_nodbg_empty(N))
    N = MOS::NoRegister;
  if (Builder.getMRI()->use_nodbg_empty(V))
    V = MOS::NoRegister;
  if (Builder.getMRI()->use_nodbg_empty(Z))
    Z = MOS::NoRegister;

  assert(!N && !Z &&
         "All N and Z uses must be selected to terminator instructions.");

  auto CInConst =
      getIConstantVRegValWithLookThrough(MI.getOperand(7).getReg(), MRI);
  bool CInSet = CInConst && !CInConst->Value.isZero();

  auto RConst = getIConstantVRegValWithLookThrough(R, *Builder.getMRI());
  MachineInstr *Load;
  MachineInstrBuilder Instr;
  // A CMP instruction can be used if we don't need the result, the overflow,
  // and the carry in is known to be set.
  if (!A && !V && CInSet) {
    if (!Instr && RConst) {
      assert(RConst->Value.getBitWidth() == 8);
      Instr =
          Builder.buildInstr(MOS::CMPImm, {MI.getOperand(1)},
                             {MI.getOperand(5), RConst->Value.getZExtValue()});
    }
    MachineOperand Addr =
        MachineOperand::CreateReg(MOS::NoRegister, /*isDef=*/false);
    if (!Instr &&
        mi_match(MI.getOperand(6).getReg(), MRI,
                 m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA)))) {
      Instr =
          Builder
              .buildInstr(MOS::CMPAbs, {MI.getOperand(1)}, {MI.getOperand(5)})
              .add(Addr)
              .cloneMemRefs(*Load);
    }
    Register Idx;
    bool ZP;
    if (!Instr && mi_match(MI.getOperand(6).getReg(), MRI,
                           m_all_of(m_MInstr(Load),
                                    m_FoldedLdIdx(MI, Addr, Idx, ZP, AA)))) {
      Instr = Builder
                  .buildInstr(ZP ? MOS::CMPZpIdx : MOS::CMPAbsIdx,
                              {MI.getOperand(1)}, {MI.getOperand(5)})
                  .add(Addr)
                  .addUse(Idx)
                  .cloneMemRefs(*Load);
    }
    Register RegAddr;
    if (!Instr &&
        mi_match(MI.getOperand(6).getReg(), MRI,
                 m_all_of(m_MInstr(Load), m_FoldedLdIndir(MI, RegAddr, AA)))) {
      Instr = Builder
                  .buildInstr(MOS::CMPIndir, {MI.getOperand(1)},
                              {MI.getOperand(5), RegAddr})
                  .cloneMemRefs(*Load);
    }
    if (!Instr &&
        mi_match(MI.getOperand(6).getReg(), MRI,
                 m_all_of(m_MInstr(Load),
                          m_FoldedLdIndirIdx(MI, RegAddr, Idx, AA)))) {
      Instr = Builder
                  .buildInstr(MOS::CMPIndirIdx, {MI.getOperand(1)},
                              {MI.getOperand(5), RegAddr, Idx})
                  .cloneMemRefs(*Load);
    }
    if (!Instr) {
      Instr = Builder.buildInstr(MOS::CMPImag8, {MI.getOperand(1)},
                                 {MI.getOperand(5), MI.getOperand(6)});
    }
  } else {
    if (!Instr && RConst) {
      assert(RConst->Value.getBitWidth() == 8);
      Instr = Builder.buildInstr(
          MOS::SBCImm, {MI.getOperand(0), MI.getOperand(1), MI.getOperand(3)},
          {MI.getOperand(5), RConst->Value.getZExtValue(), MI.getOperand(7)});
    }
    MachineOperand Addr =
        MachineOperand::CreateReg(MOS::NoRegister, /*isDef=*/false);
    if (!Instr &&
        mi_match(MI.getOperand(6).getReg(), MRI,
                 m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA)))) {
      Instr = Builder
                  .buildInstr(
                      MOS::SBCAbs,
                      {MI.getOperand(0), MI.getOperand(1), MI.getOperand(3)},
                      {MI.getOperand(5)})
                  .add(Addr)
                  .add(MI.getOperand(7))
                  .cloneMemRefs(*Load);
    }
    Register Idx;
    bool ZP;
    if (!Instr && mi_match(MI.getOperand(6).getReg(), MRI,
                           m_all_of(m_MInstr(Load),
                                    m_FoldedLdIdx(MI, Addr, Idx, ZP, AA)))) {
      Instr = Builder
                  .buildInstr(
                      ZP ? MOS::SBCZpIdx : MOS::SBCAbsIdx,
                      {MI.getOperand(0), MI.getOperand(1), MI.getOperand(3)},
                      {MI.getOperand(5)})
                  .add(Addr)
                  .addUse(Idx)
                  .add(MI.getOperand(7))
                  .cloneMemRefs(*Load);
    }
    Register RegAddr;
    if (!Instr &&
        mi_match(MI.getOperand(6).getReg(), MRI,
                 m_all_of(m_MInstr(Load), m_FoldedLdIndir(MI, RegAddr, AA)))) {
      Instr = Builder
                  .buildInstr(
                      MOS::SBCIndir,
                      {MI.getOperand(0), MI.getOperand(1), MI.getOperand(3)},
                      {MI.getOperand(5), RegAddr, MI.getOperand(7)})
                  .cloneMemRefs(*Load);
    }
    if (!Instr &&
        mi_match(MI.getOperand(6).getReg(), MRI,
                 m_all_of(m_MInstr(Load),
                          m_FoldedLdIndirIdx(MI, RegAddr, Idx, AA)))) {
      Instr = Builder
                  .buildInstr(
                      MOS::SBCIndirIdx,
                      {MI.getOperand(0), MI.getOperand(1), MI.getOperand(3)},
                      {MI.getOperand(5), RegAddr, Idx, MI.getOperand(7)})
                  .cloneMemRefs(*Load);
    }
    if (!Instr) {
      Instr = Builder.buildInstr(
          MOS::SBCImag8, {MI.getOperand(0), MI.getOperand(1), MI.getOperand(3)},
          {MI.getOperand(5), MI.getOperand(6), MI.getOperand(7)});
    }
  }
  if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
    return false;
  MI.eraseFromParent();
  return true;
}

bool MOSInstructionSelector::selectFrameIndex(MachineInstr &MI) {
  Register Dst = MI.getOperand(0).getReg();

  std::pair<Register, Register> LoHi = selectFrameIndexLoHi(MI);

  MachineIRBuilder Builder(MI);
  composePtr(Builder, Dst, LoHi.first, LoHi.second);
  MI.eraseFromParent();
  return true;
}

std::pair<Register, Register>
MOSInstructionSelector::selectFrameIndexLoHi(MachineInstr &MI) {
  const MachineFunction &MF = *MI.getMF();
  const MOSFrameLowering &TFL =
      *MF.getSubtarget<MOSSubtarget>().getFrameLowering();
  MachineIRBuilder Builder(MI);

  LLT S1 = LLT::scalar(1);
  LLT S8 = LLT::scalar(8);

  MachineInstrBuilder LoAddr;
  MachineInstrBuilder HiAddr;

  bool IsLocal =
      !MF.getFrameInfo().isFixedObjectIndex(MI.getOperand(1).getIndex());
  if (TFL.usesStaticStack(MF) && IsLocal)
    return selectAddrLoHi(MI);

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
    llvm_unreachable("Cannot constrain instruction.");
  if (!constrainSelectedInstRegOperands(*HiAddr, TII, TRI, RBI))
    llvm_unreachable("Cannot constrain instruction.");

  return {LoAddr.getReg(0), HiAddr.getReg(0)};
}

bool MOSInstructionSelector::selectAddr(MachineInstr &MI) {
  MachineIRBuilder Builder(MI);
  MachineOperand Op = MI.getOperand(1);
  if (Op.isCImm())
    Op.ChangeToImmediate(Op.getCImm()->getSExtValue());

  MachineInstrBuilder Instr = buildLdImm(Builder, MI.getOperand(0))
                                  .add(Op);
  if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
    return false;
  MI.eraseFromParent();
  return true;
}

std::pair<Register, Register>
MOSInstructionSelector::selectAddrLoHi(MachineInstr &MI) {
  MachineIRBuilder Builder(MI);
  LLT S8 = LLT::scalar(8);
  auto LoImm = buildLdImm(Builder, S8).add(MI.getOperand(1));
  LoImm->getOperand(1).setTargetFlags(MOS::MO_LO);
  if (!constrainSelectedInstRegOperands(*LoImm, TII, TRI, RBI))
    llvm_unreachable("Cannot constrain instruction.");
  auto HiImm = buildLdImm(Builder, S8).add(MI.getOperand(1));
  HiImm->getOperand(1).setTargetFlags(MOS::MO_HI);
  if (!constrainSelectedInstRegOperands(*HiImm, TII, TRI, RBI))
    llvm_unreachable("Cannot constrain instruction.");

  return {LoImm.getReg(0), HiImm.getReg(0)};
}

template <typename ADDR_P, typename CARRYIN_P> struct GShlE_match {
  Register &CarryOut;
  ADDR_P Addr;
  CARRYIN_P CarryIn;

  bool match(const MachineRegisterInfo &MRI, Register Reg) {
    const MachineInstr *GShlE = getOpcodeDef(MOS::G_SHLE, Reg, MRI);
    if (!GShlE)
      return false;
    CarryOut = GShlE->getOperand(1).getReg();
    return Addr.match(MRI, GShlE->getOperand(2).getReg()) &&
           CarryIn.match(MRI, GShlE->getOperand(3).getReg());
  }
};
template <typename ADDR_P, typename CARRYIN_P>
GShlE_match<ADDR_P, CARRYIN_P> m_GShlE(Register &CarryOut, const ADDR_P &Addr,
                                       const CARRYIN_P &CarryIn) {
  return {CarryOut, Addr, CarryIn};
}

template <typename ADDR_P, typename CARRYIN_P> struct GLshrE_match {
  Register &CarryOut;
  ADDR_P Addr;
  CARRYIN_P CarryIn;

  bool match(const MachineRegisterInfo &MRI, Register Reg) {
    const MachineInstr *GLshrE = getOpcodeDef(MOS::G_LSHRE, Reg, MRI);
    if (!GLshrE)
      return false;
    CarryOut = GLshrE->getOperand(1).getReg();
    return Addr.match(MRI, GLshrE->getOperand(2).getReg()) &&
           CarryIn.match(MRI, GLshrE->getOperand(3).getReg());
  }
};
template <typename ADDR_P, typename CARRYIN_P>
GLshrE_match<ADDR_P, CARRYIN_P> m_GLshrE(Register &CarryOut, const ADDR_P &Addr,
                                         const CARRYIN_P &CarryIn) {
  return {CarryOut, Addr, CarryIn};
}

// Replace all uses of a given virtual register after a given instruction with a
// new one. The given machine instruction must dominate all references outside
// the containing basic block. This allows folding a multi-def machine
// instruction into a later one in the same block by rewriting all later
// references to use new vregs.
static void replaceUsesAfter(MachineBasicBlock::iterator MI, Register From,
                             Register To, const MachineRegisterInfo &MRI) {
  for (MachineInstr &I : make_range(MI, MI->getParent()->end())) {
    for (MachineOperand &Op : I.all_uses())
      if (Op.getReg() == From)
        Op.setReg(To);
  }
  for (MachineOperand &MO : MRI.use_nodbg_operands(From))
    if (MO.getParent()->getParent() != MI->getParent())
      MO.setReg(To);
}

struct IncDecMBAbs_match {
  MachineInstr *&IncDec;
  MachineOperand &Addr;
  MachineInstr *&Load;
  AAResults *AA;

  bool match(const MachineRegisterInfo &MRI, Register Reg) {
    // Addressing modes destroy the vreg def for IncMB and DecMB.
    if (!MRI.hasOneNonDBGUse(Reg))
      return false;
    MachineInstr &StoreMI = *MRI.use_instr_nodbg_begin(Reg);

    IncDec = getDefIgnoringCopies(Reg, MRI);
    switch (IncDec->getOpcode()) {
    case MOS::G_INC:
    case MOS::G_DEC:
    case MOS::G_INC_TMP:
    case MOS::G_DEC_TMP:
      break;
    default:
      return false;
    }

    unsigned DstIdx = 0;
    unsigned SrcIdx = IncDec->getNumExplicitDefs();
    while (IncDec->getOperand(DstIdx).getReg() != Reg) {
      if (SrcIdx >= IncDec->getNumOperands())
        return false;
      if (IncDec->getOperand(SrcIdx).isReg())
        ++DstIdx;
      ++SrcIdx;
    }
    if (!mi_match(IncDec->getOperand(SrcIdx).getReg(), MRI,
                  m_all_of(m_MInstr(Load), m_FoldedLdAbs(StoreMI, Addr, AA))))
      return false;
    return Addr.isIdenticalTo(StoreMI.getOperand(1));
  }
};
IncDecMBAbs_match m_IncDecMBAbs(MachineInstr *&IncDec, MachineOperand &Addr,
                                MachineInstr *&Load, AAResults *AA) {
  return {IncDec, Addr, Load, AA};
}

bool MOSInstructionSelector::selectStore(MachineInstr &MI) {
  MachineIRBuilder Builder(MI);

  // Read-modify-write instruction patterns are rooted at store instructions, so
  // select one if possible. This can make an entire instruction sequence dead.
  if (!(*MI.memoperands_begin())->isVolatile())
    if (selectRMW(MI))
      return true;

  // If this isn't a STZ, emit a store pseudo.
  if (!STI.has65C02() ||
      !isOperandImmEqual(MI.getOperand(0), 0, *Builder.getMRI()))
    return selectGeneric(MI);

  // STZ

  unsigned Opcode;
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Unexpected opcode.");
  case MOS::G_STORE_ABS:
    Opcode = MOS::STZAbs;
    break;
  case MOS::G_STORE_ZP_IDX:
  case MOS::G_STORE_ABS_IDX:
    Opcode = MOS::STZIdx;
    break;
  }

  MI.setDesc(TII.get(Opcode));
  MI.removeOperand(0);
  if (!constrainSelectedInstRegOperands(MI, TII, TRI, RBI))
    return false;
  return true;
}

bool MOSInstructionSelector::selectRMW(MachineInstr &MI) {
  MachineIRBuilder Builder(MI);
  auto &MRI = *Builder.getMRI();

  Register Val = MI.getOperand(0).getReg();
  MachineInstr *Load;

  if (MI.getOpcode() == MOS::G_STORE_ABS) {
    MachineOperand Addr = MachineOperand::CreateReg(0, false);
    if (mi_match(Val, MRI,
                 m_GAdd(m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA)),
                        m_SpecificICst(1))) &&
        Addr.isIdenticalTo(MI.getOperand(1))) {
      Builder.buildInstr(MOS::INCAbs).add(Addr).cloneMergedMemRefs({&MI, Load});
      MI.eraseFromParent();
      return true;
    }
    if (mi_match(Val, MRI,
                 m_GAdd(m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA)),
                        m_SpecificICst(-1))) &&
        Addr.isIdenticalTo(MI.getOperand(1))) {
      Builder.buildInstr(MOS::DECAbs).add(Addr).cloneMergedMemRefs({&MI, Load});
      MI.eraseFromParent();
      return true;
    }

    MachineInstr *IncDec;
    if (mi_match(Val, MRI, m_IncDecMBAbs(IncDec, Addr, Load, AA))) {
      unsigned NumBytes = IncDec->getNumDefs();
      for (unsigned I = 0; I < NumBytes; ++I) {
        if (IncDec->getOperand(I).getReg() == Val) {
          MachineOperand NewAddr(Addr);
          // Remove the operand from use lists.
          IncDec->getOperand(I + NumBytes).ChangeToGA(nullptr, 0);
          IncDec->getOperand(I + NumBytes) = NewAddr;
          IncDec->removeOperand(I);
          IncDec->setDesc(TII.get(IncDec->getOpcode() == MOS::G_INC ||
                                          IncDec->getOpcode() == MOS::G_INC_TMP
                                      ? MOS::G_INC_TMP
                                      : MOS::G_DEC_TMP));
          IncDec->cloneMergedMemRefs(*MF, {&MI, Load});
          break;
        }
      }
      MI.eraseFromParent();
      return true;
    }

    Register CarryOut;
    if (mi_match(Val, MRI,
                 m_GShlE(CarryOut,
                         m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA)),
                         m_SpecificICst(0))) &&
        Addr.isIdenticalTo(MI.getOperand(1))) {
      auto Asl = Builder.buildInstr(MOS::ASLAbs, {&MOS::CcRegClass}, {})
                     .add(Addr)
                     .cloneMergedMemRefs({&MI, Load});
      replaceUsesAfter(Asl, CarryOut, Asl.getReg(0), MRI);
      if (!constrainSelectedInstRegOperands(*Asl, TII, TRI, RBI))
        return false;
      MI.eraseFromParent();
      return true;
    }
    if (mi_match(Val, MRI,
                 m_GLshrE(CarryOut,
                          m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA)),
                          m_SpecificICst(0))) &&
        Addr.isIdenticalTo(MI.getOperand(1))) {
      auto Lsr = Builder.buildInstr(MOS::LSRAbs, {&MOS::CcRegClass}, {})
                     .add(Addr)
                     .cloneMergedMemRefs({&MI, Load});
      replaceUsesAfter(Lsr, CarryOut, Lsr.getReg(0), MRI);
      if (!constrainSelectedInstRegOperands(*Lsr, TII, TRI, RBI))
        return false;
      MI.eraseFromParent();
      return true;
    }
    Register CarryIn;
    if (mi_match(Val, MRI,
                 m_GShlE(CarryOut,
                         m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA)),
                         m_Reg(CarryIn))) &&
        Addr.isIdenticalTo(MI.getOperand(1))) {
      auto Rol = Builder.buildInstr(MOS::ROLAbs, {&MOS::CcRegClass}, {})
                     .add(Addr)
                     .addUse(CarryIn)
                     .cloneMergedMemRefs({&MI, Load});
      replaceUsesAfter(Rol, CarryOut, Rol.getReg(0), MRI);
      if (!constrainSelectedInstRegOperands(*Rol, TII, TRI, RBI))
        return false;
      MI.eraseFromParent();
      return true;
    }
    if (mi_match(Val, MRI,
                 m_GLshrE(CarryOut,
                          m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA)),
                          m_Reg(CarryIn))) &&
        Addr.isIdenticalTo(MI.getOperand(1))) {
      auto Ror = Builder.buildInstr(MOS::RORAbs, {&MOS::CcRegClass}, {})
                     .add(Addr)
                     .addUse(CarryIn)
                     .cloneMergedMemRefs({&MI, Load});
      replaceUsesAfter(Ror, CarryOut, Ror.getReg(0), MRI);
      if (!constrainSelectedInstRegOperands(*Ror, TII, TRI, RBI))
        return false;
      MI.eraseFromParent();
      return true;
    }
  } else if (MI.getOpcode() == MOS::G_STORE_ABS_IDX) {
    MachineOperand Addr = MachineOperand::CreateReg(0, false);
    Register Idx;
    bool ZP;
    if (mi_match(Val, MRI,
                 m_GAdd(m_all_of(m_MInstr(Load),
                                 m_FoldedLdIdx(MI, Addr, Idx, ZP, AA)),
                        m_SpecificICst(1))) &&
        Addr.isIdenticalTo(MI.getOperand(1)) &&
        Idx == MI.getOperand(2).getReg()) {
      auto Inc = Builder.buildInstr(MOS::INCIdx)
                     .add(Addr)
                     .addUse(Idx)
                     .cloneMergedMemRefs({&MI, Load});
      if (!constrainSelectedInstRegOperands(*Inc, TII, TRI, RBI))
        return false;
      MI.eraseFromParent();
      return true;
    }
    if (mi_match(Val, MRI,
                 m_GAdd(m_all_of(m_MInstr(Load),
                                 m_FoldedLdIdx(MI, Addr, Idx, ZP, AA)),
                        m_SpecificICst(-1))) &&
        Addr.isIdenticalTo(MI.getOperand(1)) &&
        Idx == MI.getOperand(2).getReg()) {
      auto Inc = Builder.buildInstr(MOS::DECIdx)
                     .add(Addr)
                     .addUse(Idx)
                     .cloneMergedMemRefs({&MI, Load});
      if (!constrainSelectedInstRegOperands(*Inc, TII, TRI, RBI))
        return false;
      MI.eraseFromParent();
      return true;
    }
    Register CarryOut;
    if (mi_match(Val, MRI,
                 m_GShlE(CarryOut,
                         m_all_of(m_MInstr(Load),
                                  m_FoldedLdIdx(MI, Addr, Idx, ZP, AA)),
                         m_SpecificICst(0))) &&
        Addr.isIdenticalTo(MI.getOperand(1)) &&
        Idx == MI.getOperand(2).getReg()) {
      auto Asl = Builder.buildInstr(MOS::ASLIdx, {&MOS::CcRegClass}, {})
                     .add(Addr)
                     .addUse(Idx)
                     .cloneMergedMemRefs({&MI, Load});
      replaceUsesAfter(Asl, CarryOut, Asl.getReg(0), MRI);
      if (!constrainSelectedInstRegOperands(*Asl, TII, TRI, RBI))
        return false;
      MI.eraseFromParent();
      return true;
    }
    if (mi_match(Val, MRI,
                 m_GLshrE(CarryOut,
                          m_all_of(m_MInstr(Load),
                                   m_FoldedLdIdx(MI, Addr, Idx, ZP, AA)),
                          m_SpecificICst(0))) &&
        Addr.isIdenticalTo(MI.getOperand(1)) &&
        Idx == MI.getOperand(2).getReg()) {
      auto Lsr = Builder.buildInstr(MOS::LSRIdx, {&MOS::CcRegClass}, {})
                     .add(Addr)
                     .addUse(Idx)
                     .cloneMergedMemRefs({&MI, Load});
      replaceUsesAfter(Lsr, CarryOut, Lsr.getReg(0), MRI);
      if (!constrainSelectedInstRegOperands(*Lsr, TII, TRI, RBI))
        return false;
      MI.eraseFromParent();
      return true;
    }
    Register CarryIn;
    if (mi_match(Val, MRI,
                 m_GShlE(CarryOut,
                         m_all_of(m_MInstr(Load),
                                  m_FoldedLdIdx(MI, Addr, Idx, ZP, AA)),
                         m_Reg(CarryIn))) &&
        Addr.isIdenticalTo(MI.getOperand(1)) &&
        Idx == MI.getOperand(2).getReg()) {
      auto Rol = Builder.buildInstr(MOS::ROLIdx, {&MOS::CcRegClass}, {})
                     .add(Addr)
                     .addUse(Idx)
                     .addUse(CarryIn)
                     .cloneMergedMemRefs({&MI, Load});
      replaceUsesAfter(Rol, CarryOut, Rol.getReg(0), MRI);
      if (!constrainSelectedInstRegOperands(*Rol, TII, TRI, RBI))
        return false;
      MI.eraseFromParent();
      return true;
    }
    if (mi_match(Val, MRI,
                 m_GLshrE(CarryOut,
                          m_all_of(m_MInstr(Load),
                                   m_FoldedLdIdx(MI, Addr, Idx, ZP, AA)),
                          m_Reg(CarryIn))) &&
        Addr.isIdenticalTo(MI.getOperand(1)) &&
        Idx == MI.getOperand(2).getReg()) {
      auto Ror = Builder.buildInstr(MOS::RORIdx, {&MOS::CcRegClass}, {})
                     .add(Addr)
                     .addUse(Idx)
                     .addUse(CarryIn)
                     .cloneMergedMemRefs({&MI, Load});
      replaceUsesAfter(Ror, CarryOut, Ror.getReg(0), MRI);
      if (!constrainSelectedInstRegOperands(*Ror, TII, TRI, RBI))
        return false;
      MI.eraseFromParent();
      return true;
    }
  }
  return false;
}

bool MOSInstructionSelector::selectMergeValues(MachineInstr &MI) {
  MachineIRBuilder Builder(MI);
  const MachineRegisterInfo &MRI = *Builder.getMRI();

  auto [Dst, Lo, Hi] = MI.getFirst3Regs();

  auto LoConst = getIConstantVRegValWithLookThrough(Lo, MRI);
  auto HiConst = getIConstantVRegValWithLookThrough(Hi, MRI);
  if (LoConst && HiConst) {
    uint64_t Val =
        HiConst->Value.getZExtValue() << 8 | LoConst->Value.getZExtValue();
    auto Instr = STI.hasSPC700()
        ? Builder.buildInstr(MOS::LDImm16SPC700, {Dst}, {Val})
        : Builder.buildInstr(MOS::LDImm16, {Dst, &MOS::GPRRegClass}, {Val});
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      return false;
    MI.eraseFromParent();
    return true;
  }

  composePtr(Builder, Dst, Lo, Hi);
  MI.eraseFromParent();
  return true;
}

bool MOSInstructionSelector::selectLshrShlE(MachineInstr &MI) {
  auto [Dst, CarryOut, Src, CarryIn] = MI.getFirst4Regs();

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

  LLT S8 = LLT::scalar(8);

  Register From = MI.getOperand(1).getReg();

  assert(Builder.getMRI()->getType(From) == LLT::scalar(16));
  assert(Builder.getMRI()->getType(MI.getOperand(0).getReg()) ==
         LLT::scalar(1));

  MachineInstrSpan MIS(MI, MI.getParent());
  MI.getOperand(1).setReg(Builder.buildTrunc(S8, From).getReg(0));
  selectAll(MIS);
  return true;
}

bool MOSInstructionSelector::selectAddE(MachineInstr &MI) {
  auto &Add = cast<GAddSubCarryInOut>(MI);
  Register Result = Add.getDstReg();
  Register CarryOut = Add.getCarryOutReg();
  Register L = Add.getLHS().getReg();
  Register R = Add.getRHS().getReg();
  Register CarryIn = Add.getCarryInReg();

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
    Register Idx;
    bool ZP;
    if (mi_match(L, MRI, m_FoldedLdIdx(MI, Addr, Idx, ZP, AA)))
      std::swap(L, R);
    if (mi_match(R, MRI, m_FoldedLdIdx(MI, Addr, Idx, ZP, AA))) {
      return Builder.buildInstr(ZP ? MOS::ADCZpIdx : MOS::ADCAbsIdx)
          .addDef(Result)
          .addDef(CarryOut)
          .addDef(MRI.createGenericVirtualRegister(S1))
          .addUse(L)
          .add(Addr)
          .addUse(Idx)
          .addUse(CarryIn);
    }
    Register IndirAddr;
    if (mi_match(L, MRI, m_FoldedLdIndir(MI, IndirAddr, AA)))
      std::swap(L, R);
    if (mi_match(R, MRI, m_FoldedLdIndir(MI, IndirAddr, AA))) {
      return Builder.buildInstr(MOS::ADCIndir)
          .addDef(Result)
          .addDef(CarryOut)
          .addDef(MRI.createGenericVirtualRegister(S1))
          .addUse(L)
          .addUse(IndirAddr)
          .addUse(CarryIn);
    }
    if (mi_match(L, MRI, m_FoldedLdIndirIdx(MI, IndirAddr, Idx, AA)))
      std::swap(L, R);
    if (mi_match(R, MRI, m_FoldedLdIndirIdx(MI, IndirAddr, Idx, AA))) {
      return Builder.buildInstr(MOS::ADCIndirIdx)
          .addDef(Result)
          .addDef(CarryOut)
          .addDef(MRI.createGenericVirtualRegister(S1))
          .addUse(L)
          .addUse(IndirAddr)
          .addUse(Idx)
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

bool MOSInstructionSelector::selectIncDecMB(MachineInstr &MI) {
  unsigned Opcode;
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Bad opcode");
  case MOS::G_INC:
  case MOS::G_INC_TMP:
    Opcode = MOS::IncMB;
    break;
  case MOS::G_DEC:
  case MOS::G_DEC_TMP:
    Opcode = MOS::DecMB;
    if (STI.has6502X())
      Opcode = MOS::DecDcpMB;
    break;
  }

  MachineIRBuilder Builder(MI);
  if (MI.getOperand(0).isReg() &&
      Builder.getMRI()->getType(MI.getOperand(0).getReg()) ==
          LLT::pointer(0, 16)) {
    assert(MI.getOpcode() == MOS::G_INC || MI.getOpcode() == MOS::G_DEC);
    assert(MI.getNumDefs() == 1);
    auto Op = Opcode == MOS::IncMB
                  ? MOS::IncPtr
                  : (Opcode == MOS::DecMB ? MOS::DecPtr : MOS::DecDcpPtr);
    auto Instr = Builder.buildInstr(Op);
    if (Opcode == MOS::DecMB)
      Instr.addDef(Builder.getMRI()->createVirtualRegister(&MOS::GPRRegClass));
    else if (Opcode == MOS::DecDcpMB)
      Instr.addDef(Builder.getMRI()->createVirtualRegister(&MOS::AcRegClass));
    Instr.addDef(MI.getOperand(0).getReg()).addUse(MI.getOperand(1).getReg());
    MI.eraseFromParent();
    return constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
  }

  auto Instr = Builder.buildInstr(Opcode);
  if (Opcode == MOS::DecMB)
    Instr.addDef(Builder.getMRI()->createVirtualRegister(&MOS::GPRRegClass));
  else if (Opcode == MOS::DecDcpMB)
    Instr.addDef(Builder.getMRI()->createVirtualRegister(&MOS::AcRegClass));
  for (MachineOperand &MO : MI.operands())
    Instr.add(MO);
  for (MachineOperand &MO : Instr->explicit_operands())
    if (MO.isReg())
      constrainOperandRegClass(MO, MOS::Anyi8RegClass);

  unsigned DstIdx = Opcode == MOS::IncMB ? 0 : 1;
  unsigned SrcIdx = Instr->getNumExplicitDefs();
  while (SrcIdx != Instr->getNumExplicitOperands()) {
    if (Instr->getOperand(SrcIdx).isReg()) {
      Instr->tieOperands(DstIdx, SrcIdx);
      ++DstIdx;
    }
    ++SrcIdx;
  }
  MI.eraseFromParent();
  return true;
}

bool MOSInstructionSelector::selectUnMergeValues(MachineInstr &MI) {
  auto [Lo, Hi, Src] = MI.getFirst3Regs();

  MachineIRBuilder Builder(MI);

  MachineInstr *SrcMI = getDefIgnoringCopies(Src, *Builder.getMRI());
  std::optional<std::pair<Register, Register>> LoHi;
  switch (SrcMI->getOpcode()) {
  case MOS::G_FRAME_INDEX:
    LoHi = selectFrameIndexLoHi(*SrcMI);
    break;
  case MOS::G_BLOCK_ADDR:
  case MOS::G_GLOBAL_VALUE:
    LoHi = selectAddrLoHi(*SrcMI);
    break;
  }
  MachineInstrBuilder LoCopy;
  MachineInstrBuilder HiCopy;
  if (LoHi) {
    LoCopy = Builder.buildCopy(Lo, LoHi->first);
    HiCopy = Builder.buildCopy(Hi, LoHi->second);
  } else {
    LoCopy = Builder.buildCopy(Lo, Src);
    LoCopy->getOperand(1).setSubReg(MOS::sublo);
    HiCopy = Builder.buildCopy(Hi, Src);
    HiCopy->getOperand(1).setSubReg(MOS::subhi);
  }
  constrainGenericOp(*LoCopy);
  constrainGenericOp(*HiCopy);
  MI.eraseFromParent();
  return true;
}

bool MOSInstructionSelector::selectBrIndirect(MachineInstr &MI) {
  if (STI.hasSPC700()) {
    // SPC700 indirect jumps are indexed by X. Since G_BRINDIRECT does not
    // directly support indexed effective addresses, we simply make X zero.
    MachineIRBuilder Builder(MI);
    Register XZero = Builder.getMRI()->createVirtualRegister(&MOS::XcRegClass);
    Builder.buildInstr(MOS::LDImm, {XZero}, {INT64_C(0)});
    MI.addOperand(MachineOperand::CreateReg(XZero, false, true, true));
  }
  return selectGeneric(MI);
}

bool MOSInstructionSelector::selectGeneric(MachineInstr &MI) {
  unsigned Opcode;
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Unexpected opcode.");
  case MOS::G_BRINDIRECT:
    Opcode = MOS::JMPIndir;
    break;
  case MOS::G_BRINDIRECT_IDX:
    Opcode = MOS::JMPIdxIndir;
    break;
  case MOS::G_IMPLICIT_DEF:
    Opcode = MOS::IMPLICIT_DEF;
    break;
  case MOS::G_LOAD_ZP_IDX:
    Opcode = MOS::LDAZpIdx;
    break;
  case MOS::G_LOAD_ABS:
    Opcode = MOS::LDAbs;
    break;
  case MOS::G_LOAD_ABS_IDX:
    Opcode = MOS::LDAbsIdx;
    break;
  case MOS::G_LOAD_INDIR:
    Opcode = MOS::LDIndir;
    break;
  case MOS::G_LOAD_INDIR_IDX:
    Opcode = MOS::LDIndirIdx;
    break;
  case MOS::G_PHI:
    Opcode = MOS::PHI;
    break;
  case MOS::G_STORE_ZP_IDX:
    Opcode = MOS::STZpIdx;
    break;
  case MOS::G_STORE_ABS:
    Opcode = MOS::STAbs;
    break;
  case MOS::G_STORE_ABS_IDX:
    Opcode = MOS::STAbsIdx;
    break;
  case MOS::G_STORE_INDIR:
    Opcode = MOS::STIndir;
    break;
  case MOS::G_STORE_INDIR_IDX:
    Opcode = MOS::STIndirIdx;
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
}

// Ensures that any virtual registers defined by this operation are given a
// register class. Otherwise, it's possible for chains of generic operations
// (PHI, COPY, etc.) to circularly define virtual registers in such a way that
// they never actually receive a register class. Since every virtual register
// is defined exactly once, making sure definitions are constrained suffices.
void MOSInstructionSelector::constrainGenericOp(MachineInstr &MI) {
  MachineRegisterInfo &MRI = MI.getMF()->getRegInfo();
  for (MachineOperand &Op : MI.all_defs()) {
    if (Op.getReg().isPhysical() || MRI.getRegClassOrNull(Op.getReg()))
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
      MRI.setRegBank(Reg, RBI.getRegBank(MOS::AnyRegBankID));
    }

  // Select instructions in reverse block order.
  for (MachineInstr &MI : make_early_inc_range(mbb_reverse(MIS))) {
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
