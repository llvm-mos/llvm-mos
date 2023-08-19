//===- MOSRegisterBankInfo.h ---------------------------------*- C++ -*----===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the targeting of the RegisterBankInfo class for MOS.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOSREGISTERBANKINFO_H
#define LLVM_LIB_TARGET_MOS_MOSREGISTERBANKINFO_H

#include "llvm/CodeGen/RegisterBankInfo.h"

#define GET_REGBANK_DECLARATIONS
#include "MOSGenRegisterBank.inc"

namespace llvm {

class MOSGenRegisterBankInfo : public RegisterBankInfo {
protected:
#define GET_TARGET_REGBANK_CLASS
#include "MOSGenRegisterBank.inc"
};

class MOSRegisterBankInfo final : public MOSGenRegisterBankInfo {
public:
  const InstructionMapping &
  getInstrMapping(const MachineInstr &MI) const override;

  void applyMappingImpl(MachineIRBuilder &Builder,
                        const OperandsMapper &OpdMapper) const override;

  const RegisterBank &getRegBankFromRegClass(const TargetRegisterClass &RC,
                                             LLT Ty) const override;
};

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSREGISTERBANKINFO_H
