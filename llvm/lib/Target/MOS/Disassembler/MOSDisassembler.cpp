//===- MOSDisassembler.cpp - Disassembler for MOS ---------------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file is part of the MOS Disassembler.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "MOSRegisterInfo.h"
#include "MOSSubtarget.h"

#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDisassembler/MCDisassembler.h"
#include "llvm/MC/MCFixedLenDisassembler.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

#define DEBUG_TYPE "mos-disassembler"

using DecodeStatus = MCDisassembler::DecodeStatus;

namespace {
/// A disassembler class for MOS.
class MOSDisassembler : public MCDisassembler {
public:
  MOSDisassembler(const MCSubtargetInfo &STI, MCContext &Ctx)
      : MCDisassembler(STI, Ctx) {}
  DecodeStatus getInstruction(MCInst &Instr, uint64_t &Size,
                              ArrayRef<uint8_t> Bytes, uint64_t Address,
                              raw_ostream &CStream) const override;
};
} // namespace

MCDisassembler *createMOSDisassembler(const Target &T,
                                      const MCSubtargetInfo &STI,
                                      MCContext &Ctx) {
  return new MOSDisassembler(STI, Ctx);
}

extern "C" void LLVM_EXTERNAL_VISIBILITY LLVMInitializeMOSDisassembler() {
  // Register the disassembler.
  TargetRegistry::RegisterMCDisassembler(getTheMOSTarget(),
                                         createMOSDisassembler);
}

#include "MOSGenDisassemblerTables.inc"

const uint8_t *getDecoderTable(size_t Size) {
  switch (Size) {
  case 1:
    return DecoderTableMOS8;
  case 2:
    return DecoderTableMOS16;
  case 3:
    return DecoderTableMOS24;
  default:
    llvm_unreachable("instruction size must be between 1 and 3 bytes");
  }
}

DecodeStatus MOSDisassembler::getInstruction(MCInst &Instr, uint64_t &Size,
                                             ArrayRef<uint8_t> Bytes,
                                             uint64_t Address,
                                             raw_ostream &CStream) const {
  Size = 0;
  for (size_t InsnSize : seq_inclusive(1, 3)) {
    uint32_t Insn = 0;
    DecodeStatus Result;
    if (Bytes.size() < InsnSize) {
      return MCDisassembler::Fail;
    }
    for (size_t Byte : seq((size_t)0, InsnSize)) {
      Insn |= Bytes[Byte] << (8 * Byte);
    }
    Result = decodeInstruction(getDecoderTable(InsnSize), Instr, Insn, Address,
                               this, STI);
    if (Result != MCDisassembler::Fail)
    {
      Size = InsnSize;
      return Result;
    }
  }
  return MCDisassembler::Fail;
}

using DecodeFunc = DecodeStatus (*)(MCInst &, unsigned int, uint64_t,
                                    const void *);
