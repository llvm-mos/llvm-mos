//===-- MOSCallLowering.cpp - MOS Call lowering -----------------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
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
#include "MOSRegisterInfo.h"

#include "llvm/CodeGen/Analysis.h"
#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGen/FunctionLoweringInfo.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/TargetCallingConv.h"
#include "llvm/CodeGen/TargetOpcodes.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/IR/CallingConv.h"
#include "llvm/Target/TargetMachine.h"
#include <memory>

using namespace llvm;

#define DEBUG_TYPE "mos-call-lowering"

namespace {

/// Handler to pass values outward to calls and return statements.
struct MOSOutgoingValueHandler : CallLowering::OutgoingValueHandler {
  /// The instruction causing control flow to leave the current function. This
  /// instruction will be annotated with implicit use operands to record
  /// registers used to pass arguments.
  MachineInstrBuilder &MIB;

  /// Cached copy of the reserved register set for the current fn. These
  /// registers must be avoided when selecting registers for arguments.
  BitVector Reserved;

  /// A VReg containing the value of the stack pointer right before control flow
  /// leaves the current function. The VReg is cached to avoid generating it
  /// more than once.
  Register SPReg = 0;

  /// Size in bytes of the stack region used to pass arguments. Records space to
  /// be allocated for those arguments via call frame pseudos.
  uint64_t StackSize = 0;

  MOSOutgoingValueHandler(MachineIRBuilder &MIRBuilder,
                          MachineInstrBuilder &MIB, MachineRegisterInfo &MRI)
      : OutgoingValueHandler(MIRBuilder, MRI, nullptr), MIB(MIB) {
    Reserved = MRI.getTargetRegisterInfo()->getReservedRegs(MIRBuilder.getMF());
  }

  Register getStackAddress(uint64_t Size, int64_t Offset,
                           MachinePointerInfo &MPO,
                           ISD::ArgFlagsTy Flags) override {
    assert(1 <= Size && Size < 65536);
    assert(0 <= Offset && Offset < 65536);

    MPO = MachinePointerInfo::getStack(MIRBuilder.getMF(), Offset);

    LLT P = LLT::pointer(0, 16);

    // Cache the SP virtual register to avoid generating it more than once.
    if (!SPReg)
      SPReg = MIRBuilder.buildCopy(P, Register(MOS::RS0)).getReg(0);

    auto OffsetReg =
        MIRBuilder.buildConstant(LLT::scalar(16), Offset).getReg(0);
    return MIRBuilder.buildPtrAdd(P, SPReg, OffsetReg).getReg(0);
  }

  void assignValueToReg(Register ValVReg, Register PhysReg,
                        CCValAssign &VA) override {
    switch (VA.getLocVT().getSizeInBits()) {
    default:
      report_fatal_error("Not yet implemented.");
    case 8:
    case 16:
      break;
    }

    // Ensure that the physical remains alive until control flow leaves the
    // current function.
    MIB.addUse(PhysReg, RegState::Implicit);

    MIRBuilder.buildCopy(PhysReg, ValVReg);
  }

  void assignValueToAddress(Register ValVReg, Register Addr, uint64_t Size,
                            MachinePointerInfo &MPO, CCValAssign &VA) override {
    MachineFunction &MF = MIRBuilder.getMF();
    auto *MMO = MF.getMachineMemOperand(MPO, MachineMemOperand::MOStore, Size,
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

    // Use TableGen-ereted code to assign the argument to a location.
    bool Res;
    if (Info.IsFixed) {
      Res = CC_MOS(ValNo, ValVT, LocVT, LocInfo, Flags, State);
    } else {
      // The variable portion of vararg calls are always passed through the
      // stack.
      Res = CC_MOS_VarArgs(ValNo, ValVT, LocVT, LocInfo, Flags, State);
    }

    // Record current max stack size so that space can be allocated for the
    // outgoing arguments via call frame pseudos.
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

  /// Size of the stack region used to pass fixed arguments. This indicates the
  /// beginning of the stack region used for the varargs portion of calls.
  uint64_t StackSize = 0;

  MOSIncomingValueHandler(MachineIRBuilder &MIRBuilder,
                          MachineRegisterInfo &MRI,
                          std::function<void(Register Reg)> MakeLive)
      : IncomingValueHandler(MIRBuilder, MRI, nullptr), MakeLive(MakeLive) {
    Reserved = MRI.getTargetRegisterInfo()->getReservedRegs(MIRBuilder.getMF());
  }

  Register getStackAddress(uint64_t Size, int64_t Offset,
                           MachinePointerInfo &MPO,
                           ISD::ArgFlagsTy Flags) override {
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
      report_fatal_error("Not yet implemented.");
    case 8:
    case 16:
      break;
    }

    // Ensure that the physical register is considerd live at the point control
    // flow (re)enters to the current function.
    MakeLive(PhysReg);

    MIRBuilder.buildCopy(ValVReg, PhysReg);
  }

  void assignValueToAddress(Register ValVReg, Register Addr, uint64_t MemSize,
                            MachinePointerInfo &MPO, CCValAssign &VA) override {
    MachineFunction &MF = MIRBuilder.getMF();
    // All such loads are invariant: if the values are later spilled, they'll be
    // spilled to spill slots, not the original incoming argument slots.
    auto *MMO = MF.getMachineMemOperand(
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

    // Use TableGen-ereted code to assign the argument to a location.
    bool Res = CC_MOS(ValNo, ValVT, LocVT, LocInfo, Flags, State);

    // Record current max stack size so that space can be allocated for the
    // outgoing arguments via call frame pseudos.
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
  auto Return = MIRBuilder.buildInstrNoInsert(MOS::RTS);

  if (Val) {
    MachineFunction &MF = MIRBuilder.getMF();
    MachineRegisterInfo &MRI = MF.getRegInfo();
    const TargetLowering &TLI = *getTLI();
    const DataLayout &DL = MF.getDataLayout();
    LLVMContext &Ctx = Val->getContext();
    const Function &F = MF.getFunction();

    SmallVector<EVT> ValueVTs;
    ComputeValueVTs(TLI, DL, Val->getType(), ValueVTs);

    // The LLTs here are mostly redundant, except they contain information
    // missing from the VTs about whether or not the argument is a pointer. This
    // information is added to the arg flags via adjustArgFlags below.
    SmallVector<LLT> ValueLLTs;
    computeValueLLTs(DL, *Val->getType(), ValueLLTs);
    assert(ValueVTs.size() == VRegs.size() && "Need one MVT for each VReg.");
    assert(ValueLLTs.size() == VRegs.size() && "Need one LLT for each VReg.");

    // Copy flags from the instruction definition over to the return value
    // description for TableGen compatibility layer.
    SmallVector<ArgInfo> Args;
    for (size_t Idx = 0; Idx < VRegs.size(); ++Idx) {
      Args.emplace_back(VRegs[Idx], ValueVTs[Idx].getTypeForEVT(Ctx));
      setArgFlags(Args.back(), AttributeList::ReturnIndex, DL, F);
      adjustArgFlags(Args.back(), ValueLLTs[Idx]);
    }

    // Invoke TableGen compatibility layer. This will generate copies and stores
    // from the return value virtual register to physical and stack locations.
    MOSOutgoingValueHandler Handler(MIRBuilder, Return, MRI);
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

  SmallVector<ArgInfo> SplitArgs;
  unsigned Idx = 0;
  for (auto &Arg : F.args()) {
    if (DL.getTypeStoreSize(Arg.getType()).isZero())
      continue;

    // Copy flag information over from the function to the argument descriptors.
    ArgInfo OrigArg{VRegs[Idx], Arg.getType()};
    setArgFlags(OrigArg, Idx + AttributeList::FirstArgIndex, DL, F);
    splitToValueTypes(OrigArg, SplitArgs, DL);
    ++Idx;
  }

  const auto MakeLive = [&](Register PhysReg) {
    MIRBuilder.getMBB().addLiveIn(PhysReg);
  };
  MOSIncomingValueHandler Handler(MIRBuilder, MRI, MakeLive);
  // Invoke TableGen compatibility layer to create loads and copies from the
  // formal argument physical and stack locations to virtual registers.
  if (!handleAssignments(MIRBuilder, SplitArgs, Handler, F.getCallingConv(),
                         F.isVarArg()))
    return false;

  // Record the beginning of the varargs region of the stack by creating a fake
  // stack argument at that location. The varargs instructions are lowered by
  // walking a pointer forward from that memory location.
  if (F.isVarArg()) {
    auto *FuncInfo = MF.getInfo<MOSFunctionInfo>();
    FuncInfo->setVarArgsStackIndex(MF.getFrameInfo().CreateFixedObject(
        /*Size=*/1, Handler.StackSize, /*IsImmutable=*/true));
  }

  return true;
}

bool MOSCallLowering::lowerCall(MachineIRBuilder &MIRBuilder,
                                CallLoweringInfo &Info) const {
  if (Info.IsMustTailCall)
    report_fatal_error("Musttail calls not supported.");

  MachineFunction &MF = MIRBuilder.getMF();
  MachineRegisterInfo &MRI = MF.getRegInfo();
  const DataLayout &DL = MF.getDataLayout();
  const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();

  // Indirect call
  if (Info.Callee.isReg()) {
    // Store the callee in __call_indir_target.
    // Doing this before argument lowering gives additional freedom to
    // instruction scheduling. This just needs to happen some time before the
    // call, and no specific registers or stack pointer state are required.
    auto Unmerge =
        MIRBuilder.buildUnmerge({LLT::scalar(8), LLT::scalar(8)}, Info.Callee);
    Register Lo = Unmerge.getReg(0);
    Register Hi = Unmerge.getReg(1);

    // Do a bit of instruction selection here, since generic opcodes don't
    // really have a mechanism for using external symbols as store destinations.
    Register LoGPR = MIRBuilder.buildCopy(&MOS::GPRRegClass, Lo).getReg(0);
    Register HiGPR = MIRBuilder.buildCopy(&MOS::GPRRegClass, Hi).getReg(0);
    MIRBuilder.buildInstr(MOS::STabs, {}, {LoGPR})
        .addExternalSymbol("__call_indir_target");
    auto HiST = MIRBuilder.buildInstr(MOS::STabs, {}, {HiGPR})
                    .addExternalSymbol("__call_indir_target");
    HiST->getOperand(1).setOffset(1);

    // Call __call_indir to execute the indirect call.
    Info.Callee.ChangeToES("__call_indir");
  }

  // Generate the setup call frame pseudo instruction. This will record the size
  // of the outgoing stack frame once it's known. Usually, all such pseudos can
  // be folded into the prolog/epilog of the function without emitting any
  // additional code.
  auto CallSeqStart = MIRBuilder.buildInstr(MOS::ADJCALLSTACKDOWN);

  auto Call = MIRBuilder.buildInstrNoInsert(MOS::JSR)
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

  // Copy arguments from virtual registers to their real physical locations.
  MOSOutgoingValueHandler ArgsHandler(MIRBuilder, Call, MRI);
  if (!handleAssignments(MIRBuilder, OutArgs, ArgsHandler, Info.CallConv,
                         Info.IsVarArg))
    return false;

  // Insert the call once the outgoing arguments are in place.
  MIRBuilder.insertInstr(Call);

  if (!Info.OrigRet.Ty->isVoidTy()) {
    const auto MakeLive = [&](Register PhysReg) {
      Call.addDef(PhysReg, RegState::Implicit);
    };
    // Copy the return value from its physical location into a virtual register.
    MOSIncomingValueHandler RetHandler(MIRBuilder, MRI, MakeLive);
    if (!handleAssignments(MIRBuilder, InArgs, RetHandler, Info.CallConv,
                           Info.IsVarArg))
      return false;
  }

  // Now that the size of the argument stack region is known, the setup call
  // frame pseudo can be given its arguments.
  CallSeqStart.addImm(ArgsHandler.StackSize).addImm(0);

  // Generate the call frame destroy pseudo with the correct sizes.
  MIRBuilder.buildInstr(MOS::ADJCALLSTACKUP)
      .addImm(ArgsHandler.StackSize)
      .addImm(0);
  return true;
}

void MOSCallLowering::splitToValueTypes(const ArgInfo &OrigArg,
                                        SmallVectorImpl<ArgInfo> &SplitArgs,
                                        const DataLayout &DL) const {
  size_t OldSize = SplitArgs.size();
  CallLowering::splitToValueTypes(OrigArg, SplitArgs, DL, CallingConv::C);

  // Transfer is-pointer information from LLTs to argument flags.
  SmallVector<LLT> SplitLLTs;
  computeValueLLTs(DL, *OrigArg.Ty, SplitLLTs);
  assert(SplitArgs.size() - OldSize == SplitLLTs.size());
  for (unsigned Idx = 0, End = SplitLLTs.size(); Idx < End; ++Idx)
    adjustArgFlags(SplitArgs[OldSize + Idx], SplitLLTs[Idx]);
}
