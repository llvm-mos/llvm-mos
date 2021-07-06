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

#include "llvm/CodeGen/GlobalISel/LegalizerInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"

namespace llvm {

class MOSLegalizerInfo : public LegalizerInfo {
public:
  MOSLegalizerInfo();

  bool legalizeIntrinsic(LegalizerHelper &Helper,
                         MachineInstr &MI) const override;

  bool legalizeCustom(LegalizerHelper &Helper, MachineInstr &MI) const override;

private:
  // Integer Extension and Truncation
  bool legalizeSExt(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                    MachineInstr &MI) const;
  bool legalizeZExt(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                    MachineInstr &MI) const;

  // Scalar Operations
  bool legalizeBSwap(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                     MachineInstr &MI) const;

  // Integer Operations
  bool legalizeXor(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                   MachineInstr &MI) const;
  bool legalizeLshrShl(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                       MachineInstr &MI) const;
  bool shiftLibcall(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                    MachineInstr &MI) const;
  bool legalizeRotl(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                    MachineInstr &MI) const;
  bool legalizeRotr(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                    MachineInstr &MI) const;
  bool legalizeICmp(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                    MachineInstr &MI) const;
  bool legalizePtrAdd(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                      MachineInstr &MI) const;
  bool legalizeAddSubO(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                       MachineInstr &MI) const;
  bool legalizeSubE(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                    MachineInstr &MI) const;

  // Memory Operations
  bool legalizeLoad(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                    MachineInstr &MI) const;
  bool legalizeStore(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                     MachineInstr &MI) const;

  // Control Flow
  bool legalizeBrCond(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                      MachineInstr &MI) const;

  // Variadic Arguments
  bool legalizeVAArg(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                     MachineInstr &MI) const;
  bool legalizeVAStart(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                       MachineInstr &MI) const;
};

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSLEGALIZERINFO_H
