//===-- CodeGenCommonISel.cpp ---------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines common utilies that are shared between SelectionDAG and
// GlobalISel frameworks.
//
//===----------------------------------------------------------------------===//

#include "llvm/CodeGen/CodeGenCommonISel.h"
#include "llvm/Analysis/BranchProbabilityInfo.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetOpcodes.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/DebugInfoMetadata.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/Metadata.h"
#include "llvm/Support/Casting.h"

#define DEBUG_TYPE "codegen-common"

using namespace llvm;

const MDNode *llvm::getMemCacheHintMetadata(const Instruction &I,
                                            unsigned OperandNo) {
  const MDNode *MD = I.getMetadata(LLVMContext::MD_mem_cache_hint);
  if (!MD)
    return nullptr;

  for (unsigned Idx = 0; Idx + 1 < MD->getNumOperands(); Idx += 2) {
    const auto *OpNoCI = mdconst::extract<ConstantInt>(MD->getOperand(Idx));
    const auto *Hint = cast<MDNode>(MD->getOperand(Idx + 1));
    if (OpNoCI->getZExtValue() == OperandNo)
      return Hint;
  }

  return nullptr;
}

/// Add a successor MBB to ParentMBB< creating a new MachineBB for BB if SuccMBB
/// is 0.
MachineBasicBlock *
StackProtectorDescriptor::addSuccessorMBB(
    const BasicBlock *BB, MachineBasicBlock *ParentMBB, bool IsLikely,
    MachineBasicBlock *SuccMBB) {
  // If SuccBB has not been created yet, create it.
  if (!SuccMBB) {
    MachineFunction *MF = ParentMBB->getParent();
    MachineFunction::iterator BBI(ParentMBB);
    SuccMBB = MF->CreateMachineBasicBlock(BB);
    MF->insert(++BBI, SuccMBB);
  }
  // Add it as a successor of ParentMBB.
  ParentMBB->addSuccessor(
      SuccMBB, BranchProbabilityInfo::getBranchProbStackProtector(IsLikely));
  return SuccMBB;
}

/// Given that the input MI is before a partial terminator sequence TSeq, return
/// true if M + TSeq also a partial terminator sequence.
///
/// A Terminator sequence is a sequence of MachineInstrs which at this point in
/// lowering copy vregs into physical registers, which are then passed into
/// terminator instructors so we can satisfy ABI constraints. A partial
/// terminator sequence is an improper subset of a terminator sequence (i.e. it
/// may be the whole terminator sequence).
static bool MIIsInTerminatorSequence(const MachineInstr &MI) {
  // If we do not have a copy or an implicit def, we return true if and only if
  // MI is a debug value.
  if (!MI.isCopy() && !MI.isImplicitDef()) {
    // Sometimes DBG_VALUE MI sneak in between the copies from the vregs to the
    // physical registers if there is debug info associated with the terminator
    // of our mbb. We want to include said debug info in our terminator
    // sequence, so we return true in that case.
    if (MI.isDebugInstr())
      return true;

    // For GlobalISel, we may have extension instructions for arguments within
    // copy sequences. Allow these.
    switch (MI.getOpcode()) {
    case TargetOpcode::G_TRUNC:
    case TargetOpcode::G_ZEXT:
    case TargetOpcode::G_ANYEXT:
    case TargetOpcode::G_SEXT:
    case TargetOpcode::G_MERGE_VALUES:
    case TargetOpcode::G_UNMERGE_VALUES:
    case TargetOpcode::G_CONCAT_VECTORS:
    case TargetOpcode::G_BUILD_VECTOR:
    case TargetOpcode::G_EXTRACT:
      return true;
    default:
      return false;
    }
  }

  // We have left the terminator sequence if we are not doing one of the
  // following:
  //
  // 1. Copying a vreg into a physical register.
  // 2. Copying a vreg into a vreg.
  // 3. Defining a register via an implicit def.

  // OPI should always be a register definition...
  MachineInstr::const_mop_iterator OPI = MI.operands_begin();
  if (!OPI->isReg() || !OPI->isDef())
    return false;

  // Defining any register via an implicit def is always ok.
  if (MI.isImplicitDef())
    return true;

  // Grab the copy source...
  MachineInstr::const_mop_iterator OPI2 = OPI;
  ++OPI2;
  assert(OPI2 != MI.operands_end()
         && "Should have a copy implying we should have 2 arguments.");

  // Make sure that the copy dest is not a vreg when the copy source is a
  // physical register.
  if (!OPI2->isReg() ||
      (!OPI->getReg().isPhysical() && OPI2->getReg().isPhysical()))
    return false;

  return true;
}

/// Find the split point at which to splice the end of BB into its success stack
/// protector check machine basic block.
///
/// On many platforms, due to ABI constraints, terminators, even before register
/// allocation, use physical registers. This creates an issue for us since
/// physical registers at this point can not travel across basic
/// blocks. Luckily, selectiondag always moves physical registers into vregs
/// when they enter functions and moves them through a sequence of copies back
/// into the physical registers right before the terminator creating a
/// ``Terminator Sequence''. This function is searching for the beginning of the
/// terminator sequence so that we can ensure that we splice off not just the
/// terminator, but additionally the copies that move the vregs into the
/// physical registers.
MachineBasicBlock::iterator
llvm::findSplitPointForStackProtector(MachineBasicBlock *BB,
                                      const TargetInstrInfo &TII) {
  MachineBasicBlock::iterator SplitPoint = BB->getFirstTerminator();
  if (SplitPoint == BB->begin())
    return SplitPoint;

  MachineBasicBlock::iterator Start = BB->begin();
  MachineBasicBlock::iterator Previous = SplitPoint;
  do {
    --Previous;
  } while (Previous != Start && Previous->isDebugInstr());

  if (TII.isTailCall(*SplitPoint) &&
      Previous->getOpcode() == TII.getCallFrameDestroyOpcode()) {
    // Call frames cannot be nested, so if this frame is describing the tail
    // call itself, then we must insert before the sequence even starts. For
    // example:
    //     <split point>
    //     ADJCALLSTACKDOWN ...
    //     <Moves>
    //     ADJCALLSTACKUP ...
    //     TAILJMP somewhere
    // On the other hand, it could be an unrelated call in which case this tail
    // call has no register moves of its own and should be the split point. For
    // example:
    //     ADJCALLSTACKDOWN
    //     CALL something_else
    //     ADJCALLSTACKUP
    //     <split point>
    //     TAILJMP somewhere
    do {
      --Previous;
      if (Previous->isCall())
        return SplitPoint;
    } while(Previous->getOpcode() != TII.getCallFrameSetupOpcode());

    return Previous;
  }

  while (MIIsInTerminatorSequence(*Previous)) {
    SplitPoint = Previous;
    if (Previous == Start)
      break;
    --Previous;
  }

  return SplitPoint;
}

FPClassTest llvm::invertFPClassTestIfSimpler(FPClassTest Test, bool UseFCmp) {
  FPClassTest InvertedTest = ~Test;

  // Pick the direction with fewer tests
  // TODO: Handle more combinations of cases that can be handled together
  switch (static_cast<unsigned>(InvertedTest)) {
  case fcNan:
  case fcSNan:
  case fcQNan:
  case fcInf:
  case fcPosInf:
  case fcNegInf:
  case fcNormal:
  case fcPosNormal:
  case fcNegNormal:
  case fcSubnormal:
  case fcPosSubnormal:
  case fcNegSubnormal:
  case fcZero:
  case fcPosZero:
  case fcNegZero:
  case fcFinite:
  case fcPosFinite:
  case fcNegFinite:
  case fcZero | fcNan:
  case fcSubnormal | fcZero:
  case fcSubnormal | fcZero | fcNan:
    return InvertedTest;
  case fcInf | fcNan:
  case fcPosInf | fcNan:
  case fcNegInf | fcNan:
    // If we're trying to use fcmp, we can take advantage of the nan check
    // behavior of the compare (but this is more instructions in the integer
    // expansion).
    return UseFCmp ? InvertedTest : fcNone;
  default:
    return fcNone;
  }

  llvm_unreachable("covered FPClassTest");
}

static MachineOperand *getSalvageOpsForCopy(const MachineRegisterInfo &MRI,
                                            MachineInstr &Copy) {
  assert(Copy.getOpcode() == TargetOpcode::COPY && "Must be a COPY");

  return &Copy.getOperand(1);
}

static MachineOperand *getSalvageOpsForTrunc(const MachineRegisterInfo &MRI,
                                            MachineInstr &Trunc,
                                            SmallVectorImpl<uint64_t> &Ops) {
  assert(Trunc.getOpcode() == TargetOpcode::G_TRUNC && "Must be a G_TRUNC");

  const auto FromLLT = MRI.getType(Trunc.getOperand(1).getReg());
  const auto ToLLT = MRI.getType(Trunc.defs().begin()->getReg());

  // TODO: Support non-scalar types.
  if (!FromLLT.isScalar()) {
    return nullptr;
  }

  auto ExtOps = DIExpression::getExtOps(FromLLT.getSizeInBits(),
                                        ToLLT.getSizeInBits(), false);
  Ops.append(ExtOps.begin(), ExtOps.end());
  return &Trunc.getOperand(1);
}

static MachineOperand *salvageDebugInfoImpl(const MachineRegisterInfo &MRI,
                                            MachineInstr &MI,
                                            SmallVectorImpl<uint64_t> &Ops) {
  switch (MI.getOpcode()) {
  case TargetOpcode::G_TRUNC:
    return getSalvageOpsForTrunc(MRI, MI, Ops);
  case TargetOpcode::COPY:
    return getSalvageOpsForCopy(MRI, MI);
  default:
    return nullptr;
  }
}

namespace {
/// One DBG_VALUE replacement produced by salvaging a to-be-erased instruction.
/// For most opcodes salvaging yields a single replacement (modifying an
/// existing DBG_VALUE in place); for opcodes like G_MERGE_VALUES it yields
/// one replacement per source piece, materialized as additional DBG_VALUEs
/// carrying DW_OP_LLVM_fragment sub-expressions.
struct SalvagedReplacement {
  Register Reg;
  unsigned SubReg;
  const DIExpression *Expr;
};
} // end anonymous namespace

/// Compute the set of DBG_VALUE replacements produced by salvaging \p MI with
/// respect to a DBG_VALUE currently carrying \p OrigExpr.
///
/// - Returns empty if the opcode isn't salvageable or if salvage failed
///   entirely (in which case the caller leaves the DBG_VALUE alone; it will
///   be orphaned when \p MI is erased, as with any un-salvageable case).
/// - Returns one element for single-replacement opcodes (G_TRUNC, COPY).
/// - Returns up to N elements for G_MERGE_VALUES with N sources, each with a
///   fragment expression describing one piece. Fewer than N is possible when
///   individual pieces can't be fragmented (e.g., expression carries ops that
///   don't survive fragmentation) — remaining pieces still salvage.
static SmallVector<SalvagedReplacement, 4>
computeSalvageReplacements(const MachineRegisterInfo &MRI, MachineInstr &MI,
                           const DIExpression *OrigExpr) {
  SmallVector<SalvagedReplacement, 4> Out;

  if (MI.getOpcode() == TargetOpcode::G_MERGE_VALUES) {
    const unsigned NumSrcs = MI.getNumOperands() - 1; // -1 for the def
    if (NumSrcs == 0)
      return Out;

    LLT SrcTy = MRI.getType(MI.getOperand(1).getReg());
    if (!SrcTy.isScalar())
      return Out; // Only handle scalar pieces for now.

    unsigned PieceSizeInBits = SrcTy.getSizeInBits();

    // If the original expression already carries a fragment, our sub-
    // fragments must fit within it. DIExpression::createFragmentExpression
    // asserts that OffsetInBits + SizeInBits <= existing fragment size; this
    // can happen during LTO when debug info fragments don't match actual
    // value sizes.
    if (auto ExistingFrag = OrigExpr->getFragmentInfo())
      if (NumSrcs * PieceSizeInBits > ExistingFrag->SizeInBits)
        return Out;

    for (unsigned I = 0; I < NumSrcs; ++I) {
      Register SrcReg = MI.getOperand(I + 1).getReg();
      unsigned OffsetInBits = I * PieceSizeInBits;
      auto FragExpr = DIExpression::createFragmentExpression(
          OrigExpr, OffsetInBits, PieceSizeInBits);
      if (!FragExpr)
        continue; // This piece is undescribable; other pieces may still salvage.
      Out.push_back({SrcReg, /*SubReg=*/0, *FragExpr});
    }
    return Out;
  }

  // Single-replacement salvage via salvageDebugInfoImpl (G_TRUNC, COPY, ...).
  // MaxExpressionSize caps the resulting DIExpression length for performance.
  const unsigned MaxExpressionSize = 128;
  SmallVector<uint64_t, 16> Ops;
  MachineOperand *Op0 = salvageDebugInfoImpl(MRI, MI, Ops);
  if (!Op0)
    return Out;
  const DIExpression *SalvagedExpr =
      DIExpression::appendOpsToArg(OrigExpr, Ops, 0, /*StackValue=*/true);
  if (SalvagedExpr->getNumElements() > MaxExpressionSize)
    return Out;
  Out.push_back({Op0->getReg(), Op0->getSubReg(), SalvagedExpr});
  return Out;
}

void llvm::salvageDebugInfoForDbgValue(const MachineRegisterInfo &MRI,
                                       MachineInstr &MI,
                                       ArrayRef<MachineOperand *> DbgUsers) {
  const TargetInstrInfo &TII = *MI.getMF()->getSubtarget().getInstrInfo();

  for (auto *DefMO : DbgUsers) {
    MachineInstr *DbgMI = DefMO->getParent();
    if (DbgMI->isIndirectDebugValue())
      continue;

    // TODO: Support DBG_VALUE_LIST.
    if (DbgMI->getOpcode() != TargetOpcode::DBG_VALUE) {
      assert(DbgMI->getOpcode() == TargetOpcode::DBG_VALUE_LIST &&
             "Must be either DBG_VALUE or DBG_VALUE_LIST");
      continue;
    }

    int UseMOIdx =
        DbgMI->findRegisterUseOperandIdx(DefMO->getReg(), /*TRI=*/nullptr);
    assert(UseMOIdx != -1 && DbgMI->hasDebugOperandForReg(DefMO->getReg()) &&
           "Must use salvaged instruction as its location");

    auto Reps = computeSalvageReplacements(MRI, MI, DbgMI->getDebugExpression());
    if (Reps.empty())
      continue;

    // First replacement modifies the existing DBG_VALUE in place.
    auto &UseMO = DbgMI->getOperand(UseMOIdx);
    UseMO.setReg(Reps[0].Reg);
    UseMO.setSubReg(Reps[0].SubReg);
    DbgMI->getDebugExpressionOp().setMetadata(Reps[0].Expr);
    LLVM_DEBUG(dbgs() << "SALVAGE: " << *DbgMI << '\n');

    // Additional replacements (produced by fragment-emitting opcodes like
    // G_MERGE_VALUES) materialize as new DBG_VALUEs inserted before the
    // original.
    if (Reps.size() > 1) {
      const DILocalVariable *Var = DbgMI->getDebugVariable();
      DebugLoc DL = DbgMI->getDebugLoc();
      MachineBasicBlock *MBB = DbgMI->getParent();
      for (unsigned I = 1, E = Reps.size(); I != E; ++I) {
        auto NewDbg = BuildMI(*MBB, DbgMI, DL, TII.get(TargetOpcode::DBG_VALUE),
                              /*IsIndirect=*/false, Reps[I].Reg, Var,
                              Reps[I].Expr);
        LLVM_DEBUG(dbgs() << "SALVAGE (piece " << I << "): " << *NewDbg
                          << '\n');
      }
    }
  }
}
