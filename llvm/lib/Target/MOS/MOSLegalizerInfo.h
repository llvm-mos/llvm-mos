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

  bool legalizeCustom(LegalizerHelper &Helper, MachineInstr &MI) const override;

private:
  bool legalizeBrCond(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                      MachineInstr &MI) const;
  bool legalizeICmp(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                    MachineInstr &MI) const;
  bool legalizeLoad(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                    MachineInstr &MI) const;
  bool legalizePtrAdd(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                      MachineInstr &MI) const;
  bool legalizeShl(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                   MachineInstr &MI) const;
  bool legalizeStore(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                     MachineInstr &MI) const;
  bool legalizeUAddSubO(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                        MachineInstr &MI) const;
  bool legalizeVAArg(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                     MachineInstr &MI) const;
  bool legalizeVAStart(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                       MachineInstr &MI) const;
  bool legalizeXOR(LegalizerHelper &Helper, MachineRegisterInfo &MRI,
                   MachineInstr &MI) const;
};

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSLEGALIZERINFO_H
