//===-- MOSMCTargetDesc.h - MOS Target Descriptions -------------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file provides MOS specific target descriptions.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MOS_MCTARGET_DESC_H
#define LLVM_MOS_MCTARGET_DESC_H

#include "llvm/ADT/Sequence.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/Support/DataTypes.h"

#include <memory>

namespace llvm {

class FeatureBitset;
class MCAsmBackend;
class MCCodeEmitter;
class MCContext;
class MCInstrInfo;
class MCObjectTargetWriter;
class MCRegisterInfo;
class MCSubtargetInfo;
class MCTargetOptions;
class StringRef;
class Target;
class Triple;
class raw_pwrite_stream;

Target &getTheMOSTarget();

MCInstrInfo *createMOSMCInstrInfo();

/// Creates a machine code emitter for MOS.
MCCodeEmitter *createMOSMCCodeEmitter(const MCInstrInfo &MCII, MCContext &Ctx);

/// Creates an assembly backend for MOS.
MCAsmBackend *createMOSAsmBackend(const Target &T, const MCSubtargetInfo &STI,
                                  const MCRegisterInfo &MRI,
                                  const llvm::MCTargetOptions &TO);

/// Creates an ELF object writer for MOS.
std::unique_ptr<MCObjectTargetWriter> createMOSELFObjectWriter(uint8_t OSABI);

namespace MOS_MC {
/// Makes an e_flags value based on subtarget features.
unsigned makeEFlags(const FeatureBitset &Features);
} // namespace MOS_MC

} // end namespace llvm

#define GET_REGINFO_ENUM
#include "MOSGenRegisterInfo.inc"

#define GET_INSTRINFO_ENUM
#include "MOSGenInstrInfo.inc"

#define GET_SUBTARGETINFO_ENUM
#include "MOSGenSubtargetInfo.inc"

namespace llvm {
template <> struct enum_iteration_traits<decltype(MOS::NoRegister)> {
  static constexpr bool is_iterable = true;
};

namespace MOSOp {

enum OperandType : unsigned {
  OPERAND_IMM8 = MCOI::OPERAND_FIRST_TARGET,
  OPERAND_ADDR8,
  OPERAND_ADDR16,
  OPERAND_IMM16,
  OPERAND_IMM3,
  OPERAND_ADDR24,
  OPERAND_IMM24,
  OPERAND_ADDR13,
  OPERAND_IMM4
};

} // namespace MOSOp

namespace MOS {
bool isZeroPageSectionName(StringRef Name);
} // namespace MOS
} // namespace llvm

#endif // LLVM_MOS_MCTARGET_DESC_H
