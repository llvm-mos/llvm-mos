//===-- MOSLegalizerInfo.h - MOS Legalizer ----------------------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the interface that MOS uses to legalize generic MIR.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOSLEGALIZERINFO_H
#define LLVM_LIB_TARGET_MOS_MOSLEGALIZERINFO_H

#include "llvm/ADT/IndexedMap.h"
#include "llvm/CodeGen/GlobalISel/GenericMachineInstrs.h"
#include "llvm/CodeGen/GlobalISel/LegalizerInfo.h"
#include "llvm/CodeGen/GlobalISel/LostDebugLocObserver.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RuntimeLibcalls.h"
#include "llvm/IR/Instructions.h"

namespace llvm {

class MOSSubtarget;

class MOSLegalizerInfo : public LegalizerInfo {
public:
  MOSLegalizerInfo(const MOSSubtarget &STI);

  bool legalizeIntrinsic(LegalizerHelper &Helper,
                         MachineInstr &MI) const override;

  bool legalizeCustom(LegalizerHelper &Helper, MachineInstr &MI,
                      LostDebugLocObserver &LocObserver) const override;

  bool legalizeLshrEShlE(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                         MachineInstr &MI) const;

private:
  // Integer Extension and Truncation
  bool legalizeAnyExt(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                      MachineInstr &MI) const;
  bool legalizeSExt(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                    MachineInstr &MI) const;
  bool legalizeZExt(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                    MachineInstr &MI) const;

  // Integer Operations
  bool legalizeAddSub(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                      MachineInstr &MI) const;
  bool legalizeDivRem(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                      MachineInstr &MI, LostDebugLocObserver &LocObserver) const;
  bool legalizeXor(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                   MachineInstr &MI) const;
  bool legalizeShiftRotate(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                           MachineInstr &MI, LostDebugLocObserver &LocObserver) const;
  bool shiftRotateLibcall(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                          MachineInstr &MI, LostDebugLocObserver &LocObserver) const;
  bool legalizeICmp(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                    MachineInstr &MI) const;
  bool legalizeSelect(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                      MachineInstr &MI) const;
  bool legalizeAbs(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                   MachineInstr &MI) const;
  bool legalizePtrAdd(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                      MachineInstr &MI) const;
  bool legalizePtrMask(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                       MachineInstr &MI) const;
  bool legalizeAddrSpaceCast(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                             MachineInstr &MI) const;
  bool legalizeAddSubO(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                       MachineInstr &MI) const;
  bool legalizeSubE(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                    MachineInstr &MI) const;

  // Memory Operations
  bool legalizeLoad(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                    GAnyLoad &MI) const;
  bool legalizeStore(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                     GStore &MI) const;
  bool selectAddressingMode(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                            GLoadStore &MI) const;
  std::optional<MachineOperand>
  matchAbsoluteAddressing(MachineRegisterInfo &MRI, Register Addr) const;
  bool tryAbsoluteAddressing(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                             GLoadStore &MI, bool ZP) const;
  bool tryAbsoluteIndexedAddressing(LegalizerHelper &Helper,
                                    MachineRegisterInfo &MRI, GLoadStore &MI,
                                    bool ZP) const;
  bool selectIndirectAddressing(LegalizerHelper &Helper,
                                MachineRegisterInfo &MRI, GLoadStore &MI) const;
  bool selectZeroIndexedAddressing(LegalizerHelper &Helper,
                                   MachineRegisterInfo &MRI,
                                   GLoadStore &MI) const;
  bool legalizeMemOp(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                     MachineInstr &MI, LostDebugLocObserver &LocObserver) const;
  bool tryHuCBlockCopy(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                       MachineInstr &MI) const;

  // Control Flow
  bool legalizeBrCond(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                      MachineInstr &MI) const;

  bool legalizeBrJt(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                    MachineInstr &MI) const;

  // Variadic Arguments
  bool legalizeVAArg(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                     MachineInstr &MI) const;
  bool legalizeVAStart(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                       MachineInstr &MI) const;

  // Floating Point Operations
  bool legalizeFAbs(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                    MachineInstr &MI) const;
  bool legalizeFCmp(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                    MachineInstr &MI, LostDebugLocObserver &LocObserver) const;
  bool legalizeFConst(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                      MachineInstr &MI) const;

  // Other Operations
  bool legalizeDynStackAlloc(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                             MachineInstr &MI) const;
  bool legalizeFreeze(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                      MachineInstr &MI) const;

  bool legalizeToCopy(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                      MachineInstr &MI) const;

  bool preferZext() const override { return true; }

  struct FCmpLibcallInfo {
    // Which libcall this is.
    RTLIB::Libcall LibcallID;

    // The predicate to be used when comparing the value returned by the
    // function with a relevant constant (currently hard-coded to zero). This is
    // necessary because often the libcall will return e.g. a value greater than
    // 0 to represent 'true' and anything negative to represent 'false', or
    // maybe 0 to represent 'true' and non-zero for 'false'. If no comparison is
    // needed, this should be CmpInst::BAD_ICMP_PREDICATE.
    CmpInst::Predicate Predicate;
  };
  using FCmpLibcallsList = SmallVector<FCmpLibcallInfo, 2>;

  // Map from each FCmp predicate to the corresponding libcall infos. A FCmp
  // instruction may be lowered to one or two libcalls, which is why we need a
  // list. If two libcalls are needed, their results will be OR'ed.
  using FCmpLibcallsMapTy = IndexedMap<FCmpLibcallsList>;

  FCmpLibcallsMapTy FCmp32Libcalls;
  FCmpLibcallsMapTy FCmp64Libcalls;

  // Set up the floating-point comparison libcalls (GNU-compatible).
  void setFCmpLibcallsGNU();

  // Get the libcall(s) corresponding to \p Predicate for operands of \p Size
  // bits.
  FCmpLibcallsList getFCmpLibcalls(CmpInst::Predicate, unsigned Size) const;
};

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSLEGALIZERINFO_H
