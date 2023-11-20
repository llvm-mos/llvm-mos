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
#include "MOSFrameLowering.h"
#include "MOSMachineFunctionInfo.h"
#include "MOSRegisterInfo.h"
#include "MOSSubtarget.h"

#include "llvm/CodeGen/Analysis.h"
#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGen/FunctionLoweringInfo.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/TargetCallingConv.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetOpcodes.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/IR/CallingConv.h"
#include "llvm/Target/TargetMachine.h"
#include <memory>

using namespace llvm;

#define DEBUG_TYPE "mos-call-lowering"

namespace {

struct MOSValueAssigner : CallLowering::ValueAssigner {

  /// Cached copy of the reserved register set for the current fn. These
  /// registers must be avoided when selecting registers for arguments.
  BitVector Reserved;

  MOSValueAssigner(bool IsIncoming, MachineRegisterInfo &MRI,
                   const MachineFunction &MF)
      : CallLowering::ValueAssigner(IsIncoming, CC_MOS, CC_MOS_VarArgs) {
    Reserved = MRI.getTargetRegisterInfo()->getReservedRegs(MF);
  }

  bool assignArg(unsigned ValNo, EVT OrigVT, MVT ValVT, MVT LocVT,
                 CCValAssign::LocInfo LocInfo,
                 const CallLowering::ArgInfo &Info, ISD::ArgFlagsTy Flags,
                 CCState &State) override {
    // Ensure that reserved registers are not used in calling convention by
    // marking them as already allocated.
    for (Register R : Reserved.set_bits())
      State.AllocateReg(R);

    if (getAssignFn(!Info.IsFixed)(ValNo, ValVT, LocVT, LocInfo, Flags, State))
      return true;
    StackSize = State.getStackSize();
    return false;
  }
};

/// Handler to pass values outward to calls and return statements.
struct MOSOutgoingValueHandler : CallLowering::OutgoingValueHandler {
  /// The instruction causing control flow to leave the current function. This
  /// instruction will be annotated with implicit use operands to record
  /// registers used to pass arguments.
  MachineInstrBuilder &MIB;

  MOSOutgoingValueHandler(MachineIRBuilder &MIRBuilder,
                          MachineInstrBuilder &MIB, MachineRegisterInfo &MRI)
      : OutgoingValueHandler(MIRBuilder, MRI), MIB(MIB) {}

  void assignValueToReg(Register ValVReg, Register PhysReg,
                        const CCValAssign &VA) override {
    // Ensure that the physical remains alive until control flow leaves the
    // current function.
    MIB.addUse(PhysReg, RegState::Implicit);
    Register ExtReg = extendRegister(ValVReg, VA);
    MIRBuilder.buildCopy(PhysReg, ExtReg);
  }

  void assignValueToAddress(Register ValVReg, Register Addr, LLT MemTy,
                            const MachinePointerInfo &MPO,
                            const CCValAssign &VA) override {
    MachineFunction &MF = MIRBuilder.getMF();
    auto *MMO = MF.getMachineMemOperand(MPO, MachineMemOperand::MOStore, MemTy,
                                        inferAlignFromPtrInfo(MF, MPO));
    Register ExtReg = extendRegister(ValVReg, VA);
    MIRBuilder.buildStore(ExtReg, Addr, *MMO);
  }
};

struct MOSOutgoingArgsHandler : MOSOutgoingValueHandler {
  /// A VReg containing the value of the stack pointer right before control flow
  /// leaves the current function. The VReg is cached to avoid generating it
  /// more than once.
  Register SPReg = 0;

  MOSOutgoingArgsHandler(MachineIRBuilder &MIRBuilder, MachineInstrBuilder &MIB,
                         MachineRegisterInfo &MRI)
      : MOSOutgoingValueHandler(MIRBuilder, MIB, MRI) {}

  Register getStackAddress(uint64_t Size, int64_t Offset,
                           MachinePointerInfo &MPO,
                           ISD::ArgFlagsTy Flags) override {
    MPO = MachinePointerInfo::getStack(MIRBuilder.getMF(), Offset);

    LLT P = LLT::pointer(0, 16);

    // Cache the SP virtual register to avoid generating it more than once.
    if (!SPReg)
      SPReg = MIRBuilder.buildCopy(P, Register(MOS::RS0)).getReg(0);

    auto OffsetReg =
        MIRBuilder.buildConstant(LLT::scalar(16), Offset).getReg(0);
    return MIRBuilder.buildPtrAdd(P, SPReg, OffsetReg).getReg(0);
  }
};

struct MOSOutgoingReturnHandler : MOSOutgoingValueHandler {
  MOSOutgoingReturnHandler(MachineIRBuilder &MIRBuilder,
                           MachineInstrBuilder &MIB, MachineRegisterInfo &MRI)
      : MOSOutgoingValueHandler(MIRBuilder, MIB, MRI) {}

  Register getStackAddress(uint64_t Size, int64_t Offset,
                           MachinePointerInfo &MPO,
                           ISD::ArgFlagsTy Flags) override {
    auto &MFI = MIRBuilder.getMF().getFrameInfo();
    int FI = MFI.CreateFixedObject(Size, Offset, false);
    MPO = MachinePointerInfo::getFixedStack(MIRBuilder.getMF(), FI);
    auto AddrReg = MIRBuilder.buildFrameIndex(LLT::pointer(0, 16), FI);
    return AddrReg.getReg(0);
  }
};

/// Handler to receive values from formal arguments and call returns.
struct MOSIncomingValueHandler : CallLowering::IncomingValueHandler {
  MOSIncomingValueHandler(MachineIRBuilder &MIRBuilder,
                          MachineRegisterInfo &MRI)
      : IncomingValueHandler(MIRBuilder, MRI) {}

  void assignValueToReg(Register ValVReg, Register PhysReg,
                        const CCValAssign &VA) override {
    switch (VA.getLocVT().getSizeInBits()) {
    default:
      report_fatal_error("Not yet implemented.");
    case 8:
    case 16:
      break;
    }

    // Ensure that the physical register is considerd live at the point control
    // flow (re)enters to the current function.
    makeLive(PhysReg);

    MIRBuilder.buildCopy(ValVReg, PhysReg);
  }

  /// Mark the given register as live-in. These will be function-level live-ins
  /// or implicit defs for formal arguments or call statements, respectively.
  virtual void makeLive(Register PhysReg) = 0;
};

struct MOSIncomingArgsHandler : public MOSIncomingValueHandler {
  MOSIncomingArgsHandler(MachineIRBuilder &MIRBuilder, MachineRegisterInfo &MRI)
      : MOSIncomingValueHandler(MIRBuilder, MRI) {}

  Register getStackAddress(uint64_t Size, int64_t Offset,
                           MachinePointerInfo &MPO,
                           ISD::ArgFlagsTy Flags) override {
    auto &MFI = MIRBuilder.getMF().getFrameInfo();
    int FI = MFI.CreateFixedObject(Size, Offset, true);
    MPO = MachinePointerInfo::getFixedStack(MIRBuilder.getMF(), FI);
    auto AddrReg = MIRBuilder.buildFrameIndex(LLT::pointer(0, 16), FI);
    return AddrReg.getReg(0);
  }

  void assignValueToAddress(Register ValVReg, Register Addr, LLT MemTy,
                            const MachinePointerInfo &MPO,
                            const CCValAssign &VA) override {
    MachineFunction &MF = MIRBuilder.getMF();
    // All such loads are invariant: if the values are later spilled, they'll be
    // spilled to spill slots, not the original incoming argument slots.
    auto *MMO = MF.getMachineMemOperand(
        MPO, MachineMemOperand::MOLoad | MachineMemOperand::MOInvariant, MemTy,
        inferAlignFromPtrInfo(MF, MPO));
    MIRBuilder.buildLoad(ValVReg, Addr, *MMO);
  }

  void makeLive(Register PhysReg) override {
    MIRBuilder.getMBB().addLiveIn(PhysReg);
  }
};

struct MOSIncomingReturnHandler : public MOSIncomingValueHandler {
  MachineInstrBuilder &Call;

  /// A VReg containing the value of the stack pointer right after control flow
  /// returned to the current function. The VReg is cached to avoid generating
  /// it more than once.
  Register SPReg = 0;

  MOSIncomingReturnHandler(MachineIRBuilder &MIRBuilder,
                           MachineRegisterInfo &MRI, MachineInstrBuilder &Call)
      : MOSIncomingValueHandler(MIRBuilder, MRI), Call(Call) {}

  Register getStackAddress(uint64_t Size, int64_t Offset,
                           MachinePointerInfo &MPO,
                           ISD::ArgFlagsTy Flags) override {
    MPO = MachinePointerInfo::getStack(MIRBuilder.getMF(), Offset);

    LLT P = LLT::pointer(0, 16);

    // Cache the SP virtual register to avoid generating it more than once.
    if (!SPReg)
      SPReg = MIRBuilder.buildCopy(P, Register(MOS::RS0)).getReg(0);

    auto OffsetReg =
        MIRBuilder.buildConstant(LLT::scalar(16), Offset).getReg(0);
    return MIRBuilder.buildPtrAdd(P, SPReg, OffsetReg).getReg(0);
  }

  void assignValueToAddress(Register ValVReg, Register Addr, LLT MemTy,
                            const MachinePointerInfo &MPO,
                            const CCValAssign &VA) override {
    MachineFunction &MF = MIRBuilder.getMF();
    // Such loads are not invariant; the same stack region may be reused for
    // many different calls.
    auto *MMO = MF.getMachineMemOperand(MPO, MachineMemOperand::MOLoad, MemTy,
                                        inferAlignFromPtrInfo(MF, MPO));
    MIRBuilder.buildLoad(ValVReg, Addr, *MMO);
  }

  void makeLive(Register PhysReg) override {
    Call.addDef(PhysReg, RegState::Implicit);
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
  MachineFunction &MF = MIRBuilder.getMF();
  const auto &TFI = static_cast<const MOSFrameLowering &>(
      *MF.getSubtarget().getFrameLowering());
  const Function &F = MF.getFunction();

  auto Return =
      MIRBuilder.buildInstrNoInsert(TFI.isISR(MF) ? MOS::RTI : MOS::RTS);

  if (Val) {
    MachineRegisterInfo &MRI = MF.getRegInfo();
    const TargetLowering &TLI = *getTLI();
    const DataLayout &DL = MF.getDataLayout();
    LLVMContext &Ctx = Val->getContext();

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
    for (const auto &[VReg, ValueVT, ValueLLT] :
         zip(VRegs, ValueVTs, ValueLLTs)) {
      Args.emplace_back(VReg, ValueVT.getTypeForEVT(Ctx), 0);
      setArgFlags(Args.back(), AttributeList::ReturnIndex, DL, F);
      adjustArgFlags(Args.back(), ValueLLT);
    }

    // Invoke TableGen compatibility layer. This will generate copies and stores
    // from the return value virtual register to physical and stack locations.
    MOSOutgoingReturnHandler Handler(MIRBuilder, Return, MRI);
    MOSValueAssigner Assigner(/*IsIncoming=*/false, MRI, MF);
    if (!determineAndHandleAssignments(Handler, Assigner, Args, MIRBuilder,
                                       F.getCallingConv(), F.isVarArg()))
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
  const auto &TFI = static_cast<const MOSFrameLowering &>(
      *MF.getSubtarget().getFrameLowering());

  // The Decimal Flag is undefined upon interrupt and must be cleared.
  if (TFI.isISR(MF))
    MIRBuilder.buildInstr(MOS::CLD_Implied);

  SmallVector<ArgInfo> SplitArgs;
  unsigned Idx = 0;
  for (auto &Arg : F.args()) {
    if (DL.getTypeStoreSize(Arg.getType()).isZero())
      continue;

    // Copy flag information over from the function to the argument descriptors.
    ArgInfo OrigArg{VRegs[Idx], Arg.getType(), Idx};
    setArgFlags(OrigArg, Idx + AttributeList::FirstArgIndex, DL, F);
    splitToValueTypes(OrigArg, SplitArgs, DL);
    ++Idx;
  }

  MOSIncomingArgsHandler Handler(MIRBuilder, MRI);
  MOSValueAssigner Assigner(/*IsIncoming=*/true, MRI, MF);
  // Invoke TableGen compatibility layer to create loads and copies from the
  // formal argument physical and stack locations to virtual registers.
  if (!determineAndHandleAssignments(Handler, Assigner, SplitArgs, MIRBuilder,
                                     F.getCallingConv(), F.isVarArg()))
    return false;

  // Record the beginning of the varargs region of the stack by creating a fake
  // stack argument a4 that location. The varargs instructions are lowered by
  // walking a pointer forward from that memory location.
  if (F.isVarArg()) {
    auto *FuncInfo = MF.getInfo<MOSFunctionInfo>();
    FuncInfo->VarArgsStackIndex = MF.getFrameInfo().CreateFixedObject(
        /*Size=*/1, Assigner.StackSize, /*IsImmutable=*/true);
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
  const MOSSubtarget &STI = MF.getSubtarget<MOSSubtarget>();
  const TargetRegisterInfo &TRI = *STI.getRegisterInfo();

  bool IsIndirect = Info.Callee.isReg();
  if (IsIndirect) {
    // Store the callee in RS9 (used by the libcall).
    // Doing this before argument lowering gives additional freedom to
    // instruction scheduling. This just needs to happen some time before the
    // call, and no specific arguments or stack pointer state are required.
    MIRBuilder.buildCopy(MOS::RS9, Info.Callee);

    if (STI.hasSPC700()) {
      // SPC700 lacks a non-indexed indirect jump, yet we must preserve the X
      // register to meet the requirements of the calling convention.
      //
      // Instead, we use this sneaky approach of writing the opcode for an
      // absolute JMP in RC17 (immediately before the address in RS9). Now
      // it is a simple matter of JSR to RC17 which serves as an indirect thunk.
      MIRBuilder.buildCopy(MOS::RC17,
                           MIRBuilder.buildConstant(LLT::scalar(8), 0x5F));

      // Call __rc17 to execute the indirect call.
      Info.Callee.ChangeToES("__rc17");
    } else {
      // Call __call_indir to execute the indirect call.
      Info.Callee.ChangeToES("__call_indir");
    }
  }

  // Generate the setup call frame pseudo instruction. This will record the size
  // of the outgoing stack frame once it's known. Usually, all such pseudos can
  // be folded into the prolog/epilog of the function without emitting any
  // additional code.
  auto CallSeqStart = MIRBuilder.buildInstr(MOS::ADJCALLSTACKDOWN);

  auto Call = MIRBuilder.buildInstrNoInsert(MOS::JSR)
                  .add(Info.Callee)
                  .addRegMask(TRI.getCallPreservedMask(MF, Info.CallConv));

  // Indirect calls store the callee in RS9.
  if (IsIndirect) {
    Call.addUse(MOS::RS9, RegState::Implicit);
    if (STI.hasSPC700())
      Call.addUse(MOS::RC17, RegState::Implicit);
  }

  SmallVector<ArgInfo, 8> OutArgs;
  for (auto &OrigArg : Info.OrigArgs) {
    splitToValueTypes(OrigArg, OutArgs, DL);
  }

  SmallVector<ArgInfo, 8> InArgs;
  if (!Info.OrigRet.Ty->isVoidTy())
    splitToValueTypes(Info.OrigRet, InArgs, DL);

  // Copy arguments from virtual registers to their real physical locations.
  MOSOutgoingArgsHandler ArgsHandler(MIRBuilder, Call, MRI);
  MOSValueAssigner ArgsAssigner(/*IsIncoming=*/false, MRI, MF);
  if (!determineAndHandleAssignments(ArgsHandler, ArgsAssigner, OutArgs,
                                     MIRBuilder, Info.CallConv, Info.IsVarArg))
    return false;

  // Insert the call once the outgoing arguments are in place.
  MIRBuilder.insertInstr(Call);

  uint64_t StackSize = ArgsAssigner.StackSize;

  if (!Info.OrigRet.Ty->isVoidTy()) {
    // Copy the return value from its physical location into a virtual register.
    MOSIncomingReturnHandler RetHandler(MIRBuilder, MRI, Call);
    MOSValueAssigner RetAssigner(/*IsIncoming=*/true, MRI, MF);
    if (!determineAndHandleAssignments(RetHandler, RetAssigner, InArgs,
                                       MIRBuilder, Info.CallConv,
                                       Info.IsVarArg))
      return false;
    StackSize = std::max(StackSize, RetAssigner.StackSize);
  }

  // Now that the size of the argument stack region is known, the setup call
  // frame pseudo can be given its arguments.
  CallSeqStart.addImm(StackSize).addImm(0);

  // Generate the call frame destroy pseudo with the correct sizes.
  MIRBuilder.buildInstr(MOS::ADJCALLSTACKUP).addImm(StackSize).addImm(0);
  return true;
}

void MOSCallLowering::splitToValueTypes(const ArgInfo &OrigArg,
                                        SmallVectorImpl<ArgInfo> &SplitArgs,
                                        const DataLayout &DL) const {
  size_t OldSize = SplitArgs.size();
  CallLowering::splitToValueTypes(OrigArg, SplitArgs, DL, CallingConv::C);
  auto NewArgs = make_range(SplitArgs.begin() + OldSize, SplitArgs.end());

  // Transfer is-pointer information from LLTs to argument flags.
  SmallVector<LLT> SplitLLTs;
  computeValueLLTs(DL, *OrigArg.Ty, SplitLLTs);
  assert((size_t)size(NewArgs) == SplitLLTs.size());
  for (const auto &I : zip(NewArgs, SplitLLTs))
    std::apply(adjustArgFlags, I);
}
