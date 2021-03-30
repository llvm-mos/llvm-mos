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
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/Constants.h"
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

  bool select(MachineInstr &MI) override;
  static const char *getName() { return DEBUG_TYPE; }

private:
  const MOSInstrInfo &TII;
  const MOSRegisterInfo &TRI;
  const MOSRegisterBankInfo &RBI;

  bool selectAddSub(MachineInstr &MI);
  bool selectCompareBranch(MachineInstr &MI);
  bool selectConstant(MachineInstr &MI);
  bool selectFrameIndex(MachineInstr &MI);
  bool selectGlobalValue(MachineInstr &MI);
  bool selectLoadStore(MachineInstr &MI);
  bool selectShlE(MachineInstr &MI);
  bool selectImplicitDef(MachineInstr &MI);
  bool selectIntToPtr(MachineInstr &MI);
  bool selectMergeValues(MachineInstr &MI);
  bool selectPhi(MachineInstr &MI);
  bool selectPtrAdd(MachineInstr &MI);
  bool selectPtrToInt(MachineInstr &MI);
  bool selectUAddSubE(MachineInstr &MI);
  bool selectUnMergeValues(MachineInstr &MI);

  void buildCopy(MachineIRBuilder &Builder, Register Dst, Register Src);

  void composePtr(MachineIRBuilder &Builder, Register Dst, Register Lo,
                  Register Hi);

  void constrainGenericOp(MachineInstr &MI);

  void constrainOperandRegClass(MachineOperand &RegMO,
                                const TargetRegisterClass &RegClass);

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
    // Copies can have generic VReg dests, so they must be constrained to a
    // register class.
    if (MI.isCopy())
      constrainGenericOp(MI);
    return true;
  }
  if (selectImpl(MI, *CoverageInfo))
    return true;

  switch (MI.getOpcode()) {
  default:
    return false;
  case MOS::G_ADD:
  case MOS::G_SUB:
    return selectAddSub(MI);
  case MOS::G_BRCOND:
    return selectCompareBranch(MI);
  case MOS::G_CONSTANT:
    return selectConstant(MI);
  case MOS::G_FRAME_INDEX:
    return selectFrameIndex(MI);
  case MOS::G_GLOBAL_VALUE:
    return selectGlobalValue(MI);
  case MOS::G_IMPLICIT_DEF:
    return selectImplicitDef(MI);
  case MOS::G_INTTOPTR:
    return selectIntToPtr(MI);
  case MOS::G_LOAD:
  case MOS::G_STORE:
    return selectLoadStore(MI);
  case MOS::G_SHLE:
    return selectShlE(MI);
  case MOS::G_MERGE_VALUES:
    return selectMergeValues(MI);
  case MOS::G_PHI:
    return selectPhi(MI);
  case MOS::G_PTR_ADD:
    return selectPtrAdd(MI);
  case MOS::G_PTRTOINT:
    return selectPtrToInt(MI);
  case MOS::G_UADDE:
  case MOS::G_USUBE:
    return selectUAddSubE(MI);
  case MOS::G_UNMERGE_VALUES:
    return selectUnMergeValues(MI);
  }
}

bool MOSInstructionSelector::selectAddSub(MachineInstr &MI) {
  MachineIRBuilder Builder(MI);

  unsigned Opcode;
  int64_t CarryInVal;
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Unexpected opcode.");
  case MOS::G_ADD:
    Opcode = MOS::G_UADDE;
    CarryInVal = 0;
    break;
  case MOS::G_SUB:
    Opcode = MOS::G_USUBE;
    CarryInVal = 1;
    break;
  }

  Register CarryIn =
      Builder.getMRI()->createGenericVirtualRegister(LLT::scalar(1));
  Builder.buildInstr(MOS::LDCimm).addDef(CarryIn).addImm(CarryInVal);
  auto Instr =
      Builder.buildInstr(Opcode, {MI.getOperand(0), LLT::scalar(1)},
                         {MI.getOperand(1), MI.getOperand(2), CarryIn});
  MI.eraseFromParent();
  if (!selectUAddSubE(*Instr))
    return false;
  return true;
}

bool MOSInstructionSelector::selectCompareBranch(MachineInstr &MI) {
  MachineRegisterInfo &MRI = MI.getMF()->getRegInfo();
  Register CondReg = MI.getOperand(0).getReg();
  MachineBasicBlock *Tgt = MI.getOperand(1).getMBB();

  CmpInst::Predicate Pred;
  Register LHS;
  int64_t RHS;
  if (!mi_match(CondReg, MRI, m_GICmp(m_Pred(Pred), m_Reg(LHS), m_ICst(RHS))))
    return false;

  MachineIRBuilder Builder(MI);

  auto Compare = Builder.buildInstr(MOS::CMPimm).addUse(LHS).addImm(RHS);
  if (!constrainSelectedInstRegOperands(*Compare, TII, TRI, RBI))
    return false;

  auto Br = Builder.buildInstr(MOS::BR).addMBB(Tgt);
  switch (Pred) {
  default:
    return false;
  case CmpInst::ICMP_EQ:
    Br.addUse(MOS::Z).addImm(1);
    break;
  case CmpInst::ICMP_NE:
    Br.addUse(MOS::Z).addImm(0);
    break;
  case CmpInst::ICMP_UGE:
    Br.addUse(MOS::N).addImm(0);
    break;
  case CmpInst::ICMP_ULT:
    Br.addUse(MOS::N).addImm(1);
    break;
  }
  MI.eraseFromParent();
  return true;
}

bool MOSInstructionSelector::selectConstant(MachineInstr &MI) {
  assert(MI.getOpcode() == MOS::G_CONSTANT);

  MachineIRBuilder Builder(MI);
  // s8 is handled by TableGen LDimm.
  assert(Builder.getMRI()->getType(MI.getOperand(0).getReg()) ==
         LLT::scalar(1));
  auto Ld = Builder.buildInstr(MOS::LDCimm)
                .addDef(MI.getOperand(0).getReg())
                .addImm(MI.getOperand(1).getCImm()->getZExtValue());
  if (!constrainSelectedInstRegOperands(*Ld, TII, TRI, RBI))
    return false;
  MI.eraseFromParent();
  return true;
}

bool MOSInstructionSelector::selectFrameIndex(MachineInstr &MI) {
  assert(MI.getOpcode() == MOS::G_FRAME_INDEX);

  MachineIRBuilder Builder(MI);
  MachineRegisterInfo &MRI = *Builder.getMRI();

  Register Dst = MI.getOperand(0).getReg();

  LLT S8 = LLT::scalar(8);
  Register Lo = MRI.createGenericVirtualRegister(S8);
  Register Hi = MRI.createGenericVirtualRegister(S8);

  MachineInstrBuilder LoAddr;
  MachineInstrBuilder HiAddr;
  if (MI.getMF()->getFunction().doesNotRecurse()) {
    LoAddr = Builder.buildInstr(MOS::LDimm).addDef(Lo).add(MI.getOperand(1));
    LoAddr->getOperand(1).setTargetFlags(MOS::MO_LO);
    HiAddr = Builder.buildInstr(MOS::LDimm).addDef(Hi).add(MI.getOperand(1));
    HiAddr->getOperand(1).setTargetFlags(MOS::MO_HI);
  } else {
    Register Carry = MRI.createGenericVirtualRegister(LLT::scalar(1));

    LoAddr = Builder.buildInstr(MOS::AddrLostk)
                 .addDef(Lo)
                 .addDef(Carry)
                 .add(MI.getOperand(1))
                 .addImm(0);

    HiAddr = Builder.buildInstr(MOS::AddrHistk)
                 .addDef(Hi)
                 .add(MI.getOperand(1))
                 .addImm(0)
                 .addUse(Carry);
  }

  if (!constrainSelectedInstRegOperands(*LoAddr, TII, TRI, RBI))
    return false;
  if (!constrainSelectedInstRegOperands(*HiAddr, TII, TRI, RBI))
    return false;
  composePtr(Builder, Dst, Lo, Hi);
  MI.eraseFromParent();
  return true;
}

bool MOSInstructionSelector::selectGlobalValue(MachineInstr &MI) {
  assert(MI.getOpcode() == MOS::G_GLOBAL_VALUE);

  Register Dst = MI.getOperand(0).getReg();
  const GlobalValue *Global = MI.getOperand(1).getGlobal();

  MachineIRBuilder Builder(MI);
  MachineRegisterInfo &MRI = *Builder.getMRI();
  LLT s8 = LLT::scalar(8);
  Register Lo = MRI.createGenericVirtualRegister(s8);
  auto LoImm = Builder.buildInstr(MOS::LDimm)
                   .addDef(Lo)
                   .addGlobalAddress(Global, 0, MOS::MO_LO);
  if (!constrainSelectedInstRegOperands(*LoImm, TII, TRI, RBI))
    return false;
  Register Hi = MRI.createGenericVirtualRegister(s8);
  auto HiImm = Builder.buildInstr(MOS::LDimm)
                   .addDef(Hi)
                   .addGlobalAddress(Global, 0, MOS::MO_HI);
  if (!constrainSelectedInstRegOperands(*HiImm, TII, TRI, RBI))
    return false;
  composePtr(Builder, Dst, Lo, Hi);
  MI.eraseFromParent();
  return true;
}

bool MOSInstructionSelector::selectImplicitDef(MachineInstr &MI) {
  assert(MI.getOpcode() == MOS::G_IMPLICIT_DEF);

  MachineIRBuilder Builder(MI);
  auto Def = Builder.buildInstr(MOS::IMPLICIT_DEF).add(MI.getOperand(0));
  constrainGenericOp(*Def);
  MI.eraseFromParent();
  return true;
}

bool MOSInstructionSelector::selectIntToPtr(MachineInstr &MI) {
  assert(MI.getOpcode() == MOS::G_INTTOPTR);

  MachineIRBuilder Builder(MI);
  buildCopy(Builder, MI.getOperand(0).getReg(), MI.getOperand(1).getReg());
  MI.eraseFromParent();
  return true;
}

// Determines if the memory address referenced by a load/store instruction
// is based on a constant value. Absolute or zero page addressing modes can
// be used under this condition.
static bool MatchConstantAddr(Register Addr, MachineOperand &BaseOut,
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
// mode. If so, sets BaseOut to the base operand and Offset to the value that
// should be in X/Y.
static bool MatchIndexed(Register Addr, MachineOperand &BaseOut,
                         MachineOperand &OffsetOut,
                         const MachineRegisterInfo &MRI) {
  MachineInstr *SumAddr = getOpcodeDef(MOS::G_PTR_ADD, Addr, MRI);
  if (!SumAddr)
    return false;

  Register Base = SumAddr->getOperand(1).getReg();
  Register Offset = SumAddr->getOperand(2).getReg();

  MachineInstr *BaseGlobal = getOpcodeDef(MOS::G_GLOBAL_VALUE, Base, MRI);
  if (!BaseGlobal)
    return false;

  BaseOut = BaseGlobal->getOperand(1);
  // Constant offsets should already have been folded into the base.
  OffsetOut.ChangeToRegister(Offset, /*isDef=*/false);

  return true;
}

// Determines whether Addr can be referenced using the indirect-indexed (addr),Y
// addressing mode. If so, sets BaseOut to the base operand and Offset to the
// value that should be in Y.
static void MatchIndirectIndexed(Register Addr, MachineOperand &BaseOut,
                                 MachineOperand &OffsetOut,
                                 const MachineRegisterInfo &MRI) {
  if (MachineInstr *DefMI = getDefIgnoringCopies(Addr, MRI)) {
    switch (DefMI->getOpcode()) {
    case MOS::G_PTR_ADD: {
      Register Base = DefMI->getOperand(1).getReg();
      Register Offset = DefMI->getOperand(2).getReg();

      BaseOut.ChangeToRegister(Base, /*isDef=*/false);
      auto ConstOffset = getConstantVRegValWithLookThrough(Offset, MRI);
      if (ConstOffset) {
        assert(ConstOffset->Value.getBitWidth() == 8);
        OffsetOut.ChangeToImmediate(ConstOffset->Value.getZExtValue());
      } else
        OffsetOut.ChangeToRegister(Offset, /*isDef=*/false);
      return;
    }
    case MOS::G_GLOBAL_VALUE: {
      Register Base = DefMI->getOperand(0).getReg();
      int64_t Offset = DefMI->getOperand(1).getOffset();

      BaseOut.ChangeToRegister(Base, /*isDef=*/false);
      OffsetOut.ChangeToImmediate(Offset);
      return;
    }
    default:
      break;
    }
  }

  BaseOut.ChangeToRegister(Addr, /*isDef=*/false);
  OffsetOut.ChangeToImmediate(0);
}

bool MOSInstructionSelector::selectLoadStore(MachineInstr &MI) {

  Register Val = MI.getOperand(0).getReg();
  Register Addr = MI.getOperand(1).getReg();
  MachineIRBuilder Builder(MI);
  MachineRegisterInfo &MRI = *Builder.getMRI();

  MachineOperand ValOp = MachineOperand::CreateReg(Val, /*isDef=*/false);
  unsigned AbsOpcode;
  unsigned IdxOpcode;
  unsigned YIndirOpcode;
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Unexpected opcode.");
  case MOS::G_LOAD:
    ValOp.setIsDef();
    AbsOpcode = MOS::LDabs;
    IdxOpcode = MOS::LDidx;
    YIndirOpcode = MOS::LDyindir;
    break;
  case MOS::G_STORE:
    AbsOpcode = MOS::STabs;
    IdxOpcode = MOS::STidx;
    YIndirOpcode = MOS::STyindir;
    break;
  }

  MachineOperand Base = MachineOperand::CreateImm(0);
  MachineOperand Offset = MachineOperand::CreateImm(0);

  if (MatchConstantAddr(Addr, Base, MRI)) {
    auto Instr =
        Builder.buildInstr(AbsOpcode).add(ValOp).add(Base).cloneMemRefs(MI);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      return false;
    MI.eraseFromParent();
    return true;
  }

  if (MatchIndexed(Addr, Base, Offset, MRI)) {
    auto Instr = Builder.buildInstr(IdxOpcode)
                     .add(ValOp)
                     .add(Base)
                     .add(Offset)
                     .cloneMemRefs(MI);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      return false;
    MI.eraseFromParent();
    return true;
  }

  MatchIndirectIndexed(Addr, Base, Offset, MRI);

  Register OffsetReg = MRI.createGenericVirtualRegister(LLT::scalar(8));
  if (Offset.isImm())
    Builder.buildInstr(MOS::LDimm).addDef(OffsetReg).add(Offset);
  else
    buildCopy(Builder, OffsetReg, Offset.getReg());

  auto Instr = Builder.buildInstr(YIndirOpcode)
                   .add(ValOp)
                   .addUse(Base.getReg())
                   .addUse(OffsetReg)
                   .cloneMemRefs(MI);
  if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
    return false;

  MI.eraseFromParent();
  return true;
}

bool MOSInstructionSelector::selectShlE(MachineInstr &MI) {
  assert(MI.getOpcode() == MOS::G_SHLE);

  Register Dst = MI.getOperand(0).getReg();
  Register CarryOut = MI.getOperand(1).getReg();
  Register Src = MI.getOperand(2).getReg();
  Register CarryIn = MI.getOperand(3).getReg();

  MachineIRBuilder Builder(MI);
  auto ConstCarryIn =
      getConstantVRegValWithLookThrough(CarryIn, *Builder.getMRI());
  if (ConstCarryIn) {
    auto Asl =
        Builder.buildInstr(MOS::ASL).addDef(Dst).addDef(CarryOut).addUse(Src);
    if (!constrainSelectedInstRegOperands(*Asl, TII, TRI, RBI))
      return false;
  } else {
    auto Rol = Builder.buildInstr(MOS::ROL)
                   .addDef(Dst)
                   .addDef(CarryOut)
                   .addUse(Src)
                   .addUse(CarryIn);
    if (!constrainSelectedInstRegOperands(*Rol, TII, TRI, RBI))
      return false;
  }
  MI.eraseFromParent();
  return true;
}

bool MOSInstructionSelector::selectMergeValues(MachineInstr &MI) {
  assert(MI.getOpcode() == MOS::G_MERGE_VALUES);

  Register Dst = MI.getOperand(0).getReg();
  Register Lo = MI.getOperand(1).getReg();
  Register Hi = MI.getOperand(2).getReg();

  MachineIRBuilder Builder(MI);
  composePtr(Builder, Dst, Lo, Hi);
  MI.eraseFromParent();
  return true;
}

bool MOSInstructionSelector::selectPhi(MachineInstr &MI) {
  assert(MI.getOpcode() == MOS::G_PHI);

  MachineIRBuilder Builder(MI);

  auto Phi = Builder.buildInstr(MOS::PHI);
  for (MachineOperand &Op : MI.operands())
    Phi.add(Op);
  constrainGenericOp(*Phi);
  MI.eraseFromParent();
  return true;
}

bool MOSInstructionSelector::selectPtrAdd(MachineInstr &MI) {
  assert(MI.getOpcode() == MOS::G_PTR_ADD);

  Register Dst = MI.getOperand(0).getReg();
  Register Base = MI.getOperand(1).getReg();
  Register Offset = MI.getOperand(2).getReg();

  MachineIRBuilder Builder(MI);

  // All legal G_PTR_ADDs have a constant 8-bit offset, but the address
  // still may need to be materialized if used outside of a G_LOAD or
  // G_STORE context. Reaching this function indicates that this is the
  // case, since otherwise the G_PTR_ADD would have been removed already,
  // since all uses have already been selected.
  auto ConstOffset =
      getConstantVRegValWithLookThrough(Offset, *Builder.getMRI());
  if (!ConstOffset)
    return false;

  LLT s1 = LLT::scalar(1);
  LLT s8 = LLT::scalar(8);

  auto BaseLoCopy = Builder.buildCopy(&MOS::AcRegClass, Base);
  BaseLoCopy->getOperand(1).setSubReg(MOS::sublo);
  Register BaseLo = BaseLoCopy.getReg(0);

  auto BaseHiCopy = Builder.buildCopy(&MOS::AcRegClass, Base);
  BaseHiCopy->getOperand(1).setSubReg(MOS::subhi);
  Register BaseHi = BaseHiCopy.getReg(0);

  Register Carry =
      Builder.buildInstr(MOS::LDCimm, {s1}, {uint64_t(0)}).getReg(0);

  auto AddLo =
      Builder.buildInstr(MOS::ADCimm, {s8, s1},
                         {BaseLo, ConstOffset->Value.getSExtValue(), Carry});
  if (!constrainSelectedInstRegOperands(*AddLo, TII, TRI, RBI))
    return false;
  Register AddrLo = AddLo.getReg(0);
  Carry = AddLo.getReg(1);

  auto AddHi =
      Builder.buildInstr(MOS::ADCimm, {s8, s1}, {BaseHi, int64_t(0), Carry});
  if (!constrainSelectedInstRegOperands(*AddHi, TII, TRI, RBI))
    return false;
  Register AddrHi = AddHi.getReg(0);

  composePtr(Builder, Dst, AddrLo, AddrHi);

  MI.eraseFromParent();
  return true;
}

bool MOSInstructionSelector::selectPtrToInt(MachineInstr &MI) {
  assert(MI.getOpcode() == MOS::G_PTRTOINT);

  MachineIRBuilder Builder(MI);
  buildCopy(Builder, MI.getOperand(0).getReg(), MI.getOperand(1).getReg());
  MI.eraseFromParent();
  return true;
}

bool MOSInstructionSelector::selectUAddSubE(MachineInstr &MI) {
  unsigned ImmOpcode;
  unsigned Imag8Opcode;
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Unexpected opcode.");
  case MOS::G_UADDE:
    ImmOpcode = MOS::ADCimm;
    Imag8Opcode = MOS::ADCimag8;
    break;
  case MOS::G_USUBE:
    ImmOpcode = MOS::SBCimm;
    Imag8Opcode = MOS::SBCimag8;
    break;
  }

  Register Result = MI.getOperand(0).getReg();
  Register CarryOut = MI.getOperand(1).getReg();
  Register L = MI.getOperand(2).getReg();
  Register R = MI.getOperand(3).getReg();
  Register CarryIn = MI.getOperand(4).getReg();

  MachineIRBuilder Builder(MI);

  auto RConst = getConstantVRegValWithLookThrough(R, *Builder.getMRI());
  MachineInstrBuilder Instr;
  if (RConst) {
    assert(RConst->Value.getBitWidth() == 8);
    Instr = Builder.buildInstr(ImmOpcode)
                .addDef(Result)
                .addDef(CarryOut)
                .addUse(L)
                .addImm(RConst->Value.getZExtValue())
                .addUse(CarryIn);
  } else {
    Instr = Builder.buildInstr(Imag8Opcode)
                .addDef(Result)
                .addDef(CarryOut)
                .addUse(L)
                .addUse(R)
                .addUse(CarryIn);
  }
  if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
    return false;

  MI.eraseFromParent();
  return true;
}

bool MOSInstructionSelector::selectUnMergeValues(MachineInstr &MI) {
  assert(MI.getOpcode() == MOS::G_UNMERGE_VALUES);

  Register Lo = MI.getOperand(0).getReg();
  Register Hi = MI.getOperand(1).getReg();
  Register Src = MI.getOperand(2).getReg();

  MachineIRBuilder Builder(MI);

  auto LoCopy =
      Builder.buildInstr(MOS::COPY).addDef(Lo).addUse(Src, 0, MOS::sublo);
  constrainGenericOp(*LoCopy);
  auto HiCopy =
      Builder.buildInstr(MOS::COPY).addDef(Hi).addUse(Src, 0, MOS::subhi);
  constrainGenericOp(*HiCopy);
  MI.eraseFromParent();
  return true;
}

void MOSInstructionSelector::buildCopy(MachineIRBuilder &Builder, Register Dst,
                                       Register Src) {
  auto Copy = Builder.buildCopy(Dst, Src);
  constrainGenericOp(*Copy);
}

void MOSInstructionSelector::composePtr(MachineIRBuilder &Builder, Register Dst,
                                        Register Lo, Register Hi) {
  auto RegSeq = Builder.buildInstr(MOS::REG_SEQUENCE)
                    .addDef(Dst)
                    .addUse(Lo)
                    .addImm(MOS::sublo)
                    .addUse(Hi)
                    .addImm(MOS::subhi);
  constrainGenericOp(*RegSeq);

  // Propagate Lo and Hi to uses, hopefully killing the REG_SEQUENCE and
  // unconstraining the register classes of Lo and Hi.
  std::set<Register> WorkList = {Dst};
  std::vector<MachineOperand *> Uses;
  while (!WorkList.empty()) {
    Register Reg = *WorkList.begin();
    WorkList.erase(Reg);
    for (MachineOperand &MO : Builder.getMRI()->use_nodbg_operands(Reg)) {
      if (MO.getSubReg())
        Uses.push_back(&MO);
      else if (MO.getParent()->isCopy())
        WorkList.insert(MO.getParent()->getOperand(0).getReg());
    }
  }

  for (MachineOperand *MO : Uses) {
    if (MO->getSubReg() == MOS::sublo) {
      MO->setReg(Lo);
    } else {
      assert(MO->getSubReg() == MOS::subhi);
      MO->setReg(Hi);
    }
    MO->setSubReg(0);
  }
}

void MOSInstructionSelector::constrainGenericOp(MachineInstr &MI) {
  MachineRegisterInfo &MRI = MI.getMF()->getRegInfo();
  for (MachineOperand &Op : MI.operands()) {
    if (!Op.isReg() || !Op.isDef() || Op.getReg().isPhysical())
      continue;
    LLT Ty = MRI.getType(Op.getReg());
    if (Ty.isPointer()) {
      Ty = LLT::scalar(16);
      MRI.setType(Op.getReg(), Ty);
    }
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

InstructionSelector *llvm::createMOSInstructionSelector(
    const MOSTargetMachine &TM, MOSSubtarget &STI, MOSRegisterBankInfo &RBI) {
  return new MOSInstructionSelector(TM, STI, RBI);
}
