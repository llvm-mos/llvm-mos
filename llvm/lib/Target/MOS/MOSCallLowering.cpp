//===-- MOSCallLowering.cpp - MOS Call lowering -----------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the lowering of LLVM calls to machine code calls for
// GlobalISel.
//
//===----------------------------------------------------------------------===//

#include "MOSCallLowering.h"
#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOSCallingConv.h"
#include "MOSMachineFunctionInfo.h"

#include "llvm/CodeGen/Analysis.h"
#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGen/FunctionLoweringInfo.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/TargetCallingConv.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/Target/TargetMachine.h"
#include <memory>

using namespace llvm;

#define DEBUG_TYPE "mos-call-lowering"

namespace {

/// Handler to pass values outward to calls and return statements.
struct MOSOutgoingValueHandler : CallLowering::OutgoingValueHandler {
  /// Function used to assign locations to variable arguments.
  CCAssignFn *AssignFnVarArg;

  /// The instruction causing control flow to leave the current function. This
  /// instruction will be annotated with implicit use operands to record
  /// registers used to pass arguments.
  MachineInstrBuilder &MIB;

  /// Cached copy of the reserved register set for the current fn.
  BitVector Reserved;

  /// A VReg containing the value of the stack pointer right before control flow
  /// leaves the current function.
  Register SPReg = 0;

  /// Size in bytes of the stack region used to pass arguments.
  uint64_t StackSize = 0;

  MOSOutgoingValueHandler(MachineIRBuilder &MIRBuilder,
                          MachineInstrBuilder &MIB, MachineRegisterInfo &MRI,
                          CCAssignFn *AssignFn, CCAssignFn *AssignFnVarArg)
      : OutgoingValueHandler(MIRBuilder, MRI, AssignFn),
        AssignFnVarArg(AssignFnVarArg), MIB(MIB) {
    Reserved = MRI.getTargetRegisterInfo()->getReservedRegs(MIRBuilder.getMF());
  }

  Register getStackAddress(uint64_t Size, int64_t Offset,
                           MachinePointerInfo &MPO) override {
    assert(1 <= Size && Size < 65536);
    assert(0 <= Offset && Offset < 65536);
    MachineFunction &MF = MIRBuilder.getMF();
    LLT p0 = LLT::pointer(0, 16);
    LLT s16 = LLT::scalar(16);

    if (!SPReg)
      SPReg = MIRBuilder.buildCopy(p0, Register(MOS::RS0)).getReg(0);

    auto OffsetReg = MIRBuilder.buildConstant(s16, Offset);

    auto AddrReg = MIRBuilder.buildPtrAdd(p0, SPReg, OffsetReg);

    MPO = MachinePointerInfo::getStack(MF, Offset);
    return AddrReg.getReg(0);
  }

  void assignValueToReg(Register ValVReg, Register PhysReg,
                        CCValAssign &VA) override {
    switch (VA.getLocVT().getSizeInBits()) {
    default:
      llvm_unreachable("Bad register size.");
    case 8:
    case 16:
      break;
    }

    // Ensure that the physical remains alive until control flow leaves.
    MIB.addUse(PhysReg, RegState::Implicit);
    MIRBuilder.buildCopy(PhysReg, ValVReg);
  }

  void assignValueToAddress(Register ValVReg, Register Addr, uint64_t Size,
                            MachinePointerInfo &MPO, CCValAssign &VA) override {
    MachineFunction &MF = MIRBuilder.getMF();
    auto MMO = MF.getMachineMemOperand(MPO, MachineMemOperand::MOStore, Size,
                                       inferAlignFromPtrInfo(MF, MPO));
    MIRBuilder.buildStore(ValVReg, Addr, *MMO);
  }

  bool assignArg(unsigned ValNo, MVT ValVT, MVT LocVT,
                 CCValAssign::LocInfo LocInfo,
                 const llvm::CallLowering::ArgInfo &Info, ISD::ArgFlagsTy Flags,
                 CCState &State) override {
    // Ensure that reserved registers are not used in calling convention by
    // marking them as already allocated.
    for (Register R : Reserved.set_bits())
      State.AllocateReg(R);

    bool Res;
    if (Info.IsFixed)
      Res = AssignFn(ValNo, ValVT, LocVT, LocInfo, Flags, State);
    else
      Res = AssignFnVarArg(ValNo, ValVT, LocVT, LocInfo, Flags, State);
    StackSize = State.getNextStackOffset();
    return Res;
  }
};

/// Handler to receive values from formal arguments and call returns.
struct MOSIncomingValueHandler : CallLowering::IncomingValueHandler {
  /// Function to mark the given register as live-in. These will be
  /// function-level live-ins or implicit defs for formal arguments and call
  /// statements, respectively.
  std::function<void(Register Reg)> MakeLive;

  /// Cache of the reserved registers for the current function.
  BitVector Reserved;

  /// Size of the stack region used to pass arguments.
  uint64_t StackSize = 0;

  MOSIncomingValueHandler(MachineIRBuilder &MIRBuilder,
                          MachineRegisterInfo &MRI, CCAssignFn *AssignFn,
                          std::function<void(Register Reg)> MakeLive)
      : IncomingValueHandler(MIRBuilder, MRI, AssignFn), MakeLive(MakeLive) {
    Reserved = MRI.getTargetRegisterInfo()->getReservedRegs(MIRBuilder.getMF());
  }

  Register getStackAddress(uint64_t Size, int64_t Offset,
                           MachinePointerInfo &MPO) override {
    auto &MFI = MIRBuilder.getMF().getFrameInfo();
    int FI = MFI.CreateFixedObject(Size, Offset, true);
    MPO = MachinePointerInfo::getFixedStack(MIRBuilder.getMF(), FI);
    auto AddrReg = MIRBuilder.buildFrameIndex(LLT::pointer(0, 16), FI);
    return AddrReg.getReg(0);
  }

  void assignValueToReg(Register ValVReg, Register PhysReg,
                        CCValAssign &VA) override {
    switch (VA.getLocVT().getSizeInBits()) {
    default:
      llvm_unreachable("Bad register size.");
    case 8:
    case 16:
      break;
    }

    MakeLive(PhysReg);
    MIRBuilder.buildCopy(ValVReg, PhysReg);
  }

  void assignValueToAddress(Register ValVReg, Register Addr, uint64_t MemSize,
                            MachinePointerInfo &MPO, CCValAssign &VA) override {
    MachineFunction &MF = MIRBuilder.getMF();
    auto MMO = MF.getMachineMemOperand(
        MPO, MachineMemOperand::MOLoad | MachineMemOperand::MOInvariant,
        MemSize, inferAlignFromPtrInfo(MF, MPO));
    MIRBuilder.buildLoad(ValVReg, Addr, *MMO);
  }

  bool assignArg(unsigned ValNo, MVT ValVT, MVT LocVT,
                 CCValAssign::LocInfo LocInfo,
                 const llvm::CallLowering::ArgInfo &Info, ISD::ArgFlagsTy Flags,
                 CCState &State) override {
    // Ensure that reserved registers are not used in calling convention by
    // marking them as already allocated.
    for (Register R : Reserved.set_bits())
      State.AllocateReg(R);

    bool Res = AssignFn(ValNo, ValVT, LocVT, LocInfo, Flags, State);
    StackSize = State.getNextStackOffset();
    return Res;
  }
};

// Add missing pointer information from the LLT to the argument flags for the
// corresponding MVT. The MVT doesn't contain pointer information, so this would
// otherwise be unavailable for use by the calling convention (i.e., CCIfPtr).
void adjustArgFlags(CallLowering::ArgInfo &Arg, LLT Ty) {
  if (!Ty.isPointer())
    return;

  auto &Flags = Arg.Flags[0];
  Flags.setPointer();
  Flags.setPointerAddrSpace(Ty.getAddressSpace());
}

} // namespace

bool MOSCallLowering::lowerReturn(MachineIRBuilder &MIRBuilder,
                                  const Value *Val, ArrayRef<Register> VRegs,
                                  FunctionLoweringInfo &FLI) const {
  auto Return = MIRBuilder.buildInstrNoInsert(MOS::RTS_Implied);

  if (Val) {
    MachineFunction &MF = MIRBuilder.getMF();
    MachineRegisterInfo &MRI = MF.getRegInfo();
    const TargetLowering &TLI = *getTLI();
    const DataLayout &DL = MF.getDataLayout();
    LLVMContext &Ctx = Val->getContext();
    const Function &F = MF.getFunction();

    // Initialize data structures for TableGen calling convention compatibility
    // layer.
    SmallVector<EVT> ValueVTs;
    ComputeValueVTs(TLI, DL, Val->getType(), ValueVTs);
    // The LLTs here are mostly redundant, except they contain information
    // missing from the VTs about whether or not the argument is a pointer. This
    // information is added to the arg flags via adjustArgFlags below.
    SmallVector<LLT> ValueLLTs;
    computeValueLLTs(DL, *Val->getType(), ValueLLTs);
    assert(ValueVTs.size() == VRegs.size() && "Need one MVT for each VReg.");
    assert(ValueLLTs.size() == VRegs.size() && "Need one LLT for each VReg.");
    SmallVector<ArgInfo> Args;
    for (size_t Idx = 0; Idx < VRegs.size(); ++Idx) {
      Args.emplace_back(VRegs[Idx], ValueVTs[Idx].getTypeForEVT(Ctx));
      setArgFlags(Args.back(), AttributeList::ReturnIndex, DL, F);
      adjustArgFlags(Args.back(), ValueLLTs[Idx]);
    }

    // Invoke TableGen compatibility layer. The return instruction will be
    // annotated with implicit uses of any live variables out of the function.
    MOSOutgoingValueHandler Handler(MIRBuilder, Return, MRI, CC_MOS,
                                    CC_MOS_VarArgs);
    if (!handleAssignments(MIRBuilder, Args, Handler, F.getCallingConv(),
                           F.isVarArg()))
      return false;
  }

  // Insert the final return once the return values are in place.
  MIRBuilder.insertInstr(Return);
  return true;
}

bool MOSCallLowering::lowerFormalArguments(MachineIRBuilder &MIRBuilder,
                                           const Function &F,
                                           ArrayRef<ArrayRef<Register>> VRegs,
                                           FunctionLoweringInfo &FLI) const {
  MachineFunction &MF = MIRBuilder.getMF();
  const DataLayout &DL = MF.getDataLayout();
  MachineRegisterInfo &MRI = MF.getRegInfo();

  // Initialize data structures for TableGen calling convention compatibility
  // layer.
  SmallVector<ArgInfo> SplitArgs;
  unsigned i = 0;
  for (auto &Arg : F.args()) {
    if (DL.getTypeStoreSize(Arg.getType()).isZero())
      continue;

    ArgInfo OrigArg{VRegs[i], Arg.getType()};
    setArgFlags(OrigArg, i + AttributeList::FirstArgIndex, DL, F);
    splitToValueTypes(OrigArg, SplitArgs, DL);
    ++i;
  }

  // Invoke TableGen compatibility layer.
  const auto MakeLive = [&](Register PhysReg) {
    MIRBuilder.getMBB().addLiveIn(PhysReg);
  };
  MOSIncomingValueHandler Handler(MIRBuilder, MRI, CC_MOS, MakeLive);
  if (!handleAssignments(MIRBuilder, SplitArgs, Handler, F.getCallingConv(),
                         F.isVarArg()))
    return false;

  // Record the beginning of the varargs region of the stack by creating a fake
  // stack argument at that location. The varargs instructions are lowered by
  // walking a pointer forward from that memory location.
  if (F.isVarArg()) {
    auto *FuncInfo = MF.getInfo<MOSFunctionInfo>();
    FuncInfo->setVarArgsStackIndex(MF.getFrameInfo().CreateFixedObject(
        1, Handler.StackSize, /*IsImmutable=*/true));
  }

  return true;
}

bool MOSCallLowering::lowerCall(MachineIRBuilder &MIRBuilder,
                                CallLoweringInfo &Info) const {
  if (!Info.Callee.isGlobal() && !Info.Callee.isSymbol())
    report_fatal_error("Callee type not yet implemented.");
  if (Info.IsMustTailCall)
    report_fatal_error("Musttail calls not supported.");

  MachineFunction &MF = MIRBuilder.getMF();
  MachineRegisterInfo &MRI = MF.getRegInfo();
  const DataLayout &DL = MF.getDataLayout();
  const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();

  auto CallSeqStart = MIRBuilder.buildInstr(MOS::ADJCALLSTACKDOWN);

  auto Call = MIRBuilder.buildInstrNoInsert(MOS::JSR_Absolute)
                  .add(Info.Callee)
                  .addRegMask(TRI.getCallPreservedMask(
                      MF, MF.getFunction().getCallingConv()));

  SmallVector<ArgInfo, 8> OutArgs;
  for (auto &OrigArg : Info.OrigArgs) {
    splitToValueTypes(OrigArg, OutArgs, DL);
  }

  SmallVector<ArgInfo, 8> InArgs;
  if (!Info.OrigRet.Ty->isVoidTy())
    splitToValueTypes(Info.OrigRet, InArgs, DL);

  // Invoke TableGen compatibility layer for outgoing arguments. The call
  // instruction will be annotated with implicit uses of any live variables out
  // of the function.
  MOSOutgoingValueHandler ArgsHandler(MIRBuilder, Call, MRI, CC_MOS,
                                      CC_MOS_VarArgs);
  if (!handleAssignments(MIRBuilder, OutArgs, ArgsHandler, Info.CallConv,
                         Info.IsVarArg))
    return false;

  // Insert the call once the outgoing arguments are in place.
  MIRBuilder.insertInstr(Call);

  if (!Info.OrigRet.Ty->isVoidTy()) {
    // Invoke TableGen compatibility layer for return value.
    const auto MakeLive = [&](Register PhysReg) {
      Call.addDef(PhysReg, RegState::Implicit);
    };
    MOSIncomingValueHandler RetHandler(MIRBuilder, MRI, CC_MOS, MakeLive);
    if (!handleAssignments(MIRBuilder, InArgs, RetHandler, Info.CallConv,
                           Info.IsVarArg))
      return false;
  }

  // Now that the size of the argument stack region is known, the SP adjustment
  // intrinsics can be given their arguments.
  CallSeqStart.addImm(ArgsHandler.StackSize).addImm(0);
  MIRBuilder.buildInstr(MOS::ADJCALLSTACKUP)
      .addImm(ArgsHandler.StackSize)
      .addImm(0);
  return true;
}

/// Split an aggregate type into its individual components, allowing each to be
/// passed separately. This only handles aggregate types; large primitives (e.g.
/// i32) are handled via a splitting mechanism in CallLowering itself. C
/// aggregates are passed by pointer, so this code may not actually be needed,
/// but eventually small aggregates (struct { char a, b;}) should be passed by
/// value, so it's nice to have around.
void MOSCallLowering::splitToValueTypes(const ArgInfo &OrigArg,
                                        SmallVectorImpl<ArgInfo> &SplitArgs,
                                        const DataLayout &DL) const {
  LLVMContext &Ctx = OrigArg.Ty->getContext();

  SmallVector<EVT> SplitVTs;
  ComputeValueVTs(*getTLI(), DL, OrigArg.Ty, SplitVTs);
  SmallVector<LLT> SplitLLTs;
  computeValueLLTs(DL, *OrigArg.Ty, SplitLLTs);
  assert(SplitVTs.size() == SplitLLTs.size());

  if (SplitVTs.size() == 0)
    return;

  if (SplitVTs.size() == 1) {
    // No splitting to do, but we want to replace the original type (e.g. [1 x
    // double] -> double).
    SplitArgs.emplace_back(OrigArg.Regs[0], SplitVTs[0].getTypeForEVT(Ctx),
                           OrigArg.Flags[0], OrigArg.IsFixed);
    adjustArgFlags(SplitArgs.back(), SplitLLTs[0]);
    return;
  }

  // Create one ArgInfo for each virtual register in the original ArgInfo.
  assert(OrigArg.Regs.size() == SplitVTs.size() && "Regs / types mismatch");

  for (unsigned i = 0, e = SplitVTs.size(); i < e; ++i) {
    Type *SplitTy = SplitVTs[i].getTypeForEVT(Ctx);
    SplitArgs.emplace_back(OrigArg.Regs[i], SplitTy, OrigArg.Flags[0],
                           OrigArg.IsFixed);
    adjustArgFlags(SplitArgs.back(), SplitLLTs[i]);
  }
}
