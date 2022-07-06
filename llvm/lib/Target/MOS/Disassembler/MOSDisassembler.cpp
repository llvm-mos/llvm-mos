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

#include "MCTargetDesc/MOSMCExpr.h"
#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "MOSRegisterInfo.h"
#include "MOSSubtarget.h"

#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDecoderOps.h"
#include "llvm/MC/MCDisassembler/MCDisassembler.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

#define DEBUG_TYPE "mos-disassembler"

using DecodeStatus = MCDisassembler::DecodeStatus;

namespace {
/// A disassembler class for MOS.
class MOSDisassembler : public MCDisassembler {
  bool Has65816 = false;
  mutable bool MLow = false;
  mutable bool XLow = false;

public:
  MOSDisassembler(const MCSubtargetInfo &STI, MCContext &Ctx)
      : MCDisassembler(STI, Ctx), Has65816(STI.hasFeature(MOS::FeatureW65816)) {
  }
  Optional<MCDisassembler::DecodeStatus>
  onSymbolStart(SymbolInfoTy &Symbol, uint64_t &Size, ArrayRef<uint8_t> Bytes,
                uint64_t Address, raw_ostream &CStream) const override;
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
  case 4:
    return DecoderTableMOS32;
  default:
    llvm_unreachable("instruction size must be between 1 and 3 bytes");
  }
}

const uint8_t *getDecoderTable65CE02(size_t Size) {
  switch (Size) {
  case 1:
    return DecoderTable65ce028;
  case 2:
    return DecoderTable65ce0216;
  case 3:
    return DecoderTable65ce0224;
  default:
    llvm_unreachable("instruction size must be between 1 and 3 bytes");
  }
}

Optional<MCDisassembler::DecodeStatus>
MOSDisassembler::onSymbolStart(SymbolInfoTy &Symbol, uint64_t &Size,
                               ArrayRef<uint8_t> Bytes, uint64_t Address,
                               raw_ostream &CStream) const {
  // 16-bit flags for decoding immediates are set based on the occurrence of
  // mapping symbols $ml, $mh, $xl, $xh.
  if (Has65816) {
    if (Symbol.Name.startswith("$ml"))
      MLow = true;
    else if (Symbol.Name.startswith("$mh"))
      MLow = false;
    else if (Symbol.Name.startswith("$xl"))
      XLow = true;
    else if (Symbol.Name.startswith("$xh"))
      XLow = false;
  }
  return None;
}

DecodeStatus MOSDisassembler::getInstruction(MCInst &Instr, uint64_t &Size,
                                             ArrayRef<uint8_t> Bytes,
                                             uint64_t Address,
                                             raw_ostream &CStream) const {
  Size = 0;

  // First attempt to decode 16-bit immediate 65816 instructions.
  if (Has65816) {
    if ((MLow || XLow) && Bytes.size() >= 3) {
      uint32_t Insn = 0;
      DecodeStatus Result = MCDisassembler::Fail;
      for (size_t Byte : seq(0, 3)) {
        Insn |= Bytes[Byte] << (8 * Byte);
      }
      if (MLow) {
        Result = decodeInstruction(DecoderTableMOSMLow24, Instr, Insn, Address,
                                   this, STI);
      }
      if (Result == MCDisassembler::Fail && XLow) {
        Result = decodeInstruction(DecoderTableMOSXLow24, Instr, Insn, Address,
                                   this, STI);
      }
      if (Result != MCDisassembler::Fail) {
        MCOperand &Op = Instr.getOperand(0);
        if ((uint64_t)Op.getImm() <= UCHAR_MAX) {
          // Add mos16 modifier if necessary to retain 16-bit width.
          Op = MCOperand::createExpr(MOSMCExpr::create(
              MOSMCExpr::VK_MOS_IMM16,
              MCConstantExpr::create(Op.getImm(), getContext()), false,
              getContext()));
        }
        Size = 3;
        return Result;
      }
    }
  }

  // Otherwise decode from the normal tables.
  for (size_t InsnSize : seq_inclusive(1, 4)) {
    uint32_t Insn = 0;
    DecodeStatus Result = MCDisassembler::Fail;
    if (Bytes.size() < InsnSize) {
      return MCDisassembler::Fail;
    }
    for (size_t Byte : seq((size_t)0, InsnSize)) {
      Insn |= Bytes[Byte] << (8 * Byte);
    }
    if (STI.getFeatureBits()[MOS::Feature65CE02]) {
      Result = decodeInstruction(getDecoderTable65CE02(InsnSize), Instr, Insn,
                                 Address, this, STI);
    }
    if (Result == MCDisassembler::Fail) {
      Result = decodeInstruction(getDecoderTable(InsnSize), Instr, Insn,
                                 Address, this, STI);
    }
    if (Result != MCDisassembler::Fail) {
      Size = InsnSize;
      return Result;
    }
  }

  return MCDisassembler::Fail;
}

using DecodeFunc = DecodeStatus (*)(MCInst &, unsigned int, uint64_t,
                                    const void *);
