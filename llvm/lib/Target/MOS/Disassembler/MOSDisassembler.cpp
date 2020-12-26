//===- MOSDisassembler.cpp - Disassembler for MOS ---------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
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
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"

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
}; // namespace

MCDisassembler *createMOSDisassembler(const Target &T,
                                      const MCSubtargetInfo &STI,
                                      MCContext &Ctx) {
  return new MOSDisassembler(STI, Ctx);
}

extern "C" void LLVMInitializeMOSDisassembler() {
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
  for (size_t InsnSize = 1; InsnSize <= 3; InsnSize++) {
    uint32_t Insn = 0;
    DecodeStatus Result;
    if (Bytes.size() < InsnSize) {
      return MCDisassembler::Fail;
    }
    for (size_t Byte = 0; Byte < InsnSize; Byte++) {
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
};

using DecodeFunc = DecodeStatus (*)(MCInst &, unsigned int, uint64_t,
                                    const void *);
