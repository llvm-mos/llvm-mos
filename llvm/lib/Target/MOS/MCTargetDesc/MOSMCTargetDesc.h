//===-- MOSMCTargetDesc.h - MOS Target Descriptions -------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides MOS specific target descriptions.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MOS_MCTARGET_DESC_H
#define LLVM_MOS_MCTARGET_DESC_H

#include "llvm/Support/DataTypes.h"

#include <memory>

namespace llvm {

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
MCCodeEmitter *createMOSMCCodeEmitter(const MCInstrInfo &MCII,
                                      const MCRegisterInfo &MRI,
                                      MCContext &Ctx);

/// Creates an assembly backend for MOS.
MCAsmBackend *createMOSAsmBackend(const Target &T, const MCSubtargetInfo &STI,
                                  const MCRegisterInfo &MRI,
                                  const llvm::MCTargetOptions &TO);

/// Creates an ELF object writer for MOS.
std::unique_ptr<MCObjectTargetWriter> createMOSELFObjectWriter(uint8_t OSABI);

} // end namespace llvm

#define GET_REGINFO_ENUM
#include "MOSGenRegisterInfo.inc"

#define GET_INSTRINFO_ENUM
#include "MOSGenInstrInfo.inc"

#define GET_SUBTARGETINFO_ENUM
#include "MOSGenSubtargetInfo.inc"

#endif // LLVM_MOS_MCTARGET_DESC_H
