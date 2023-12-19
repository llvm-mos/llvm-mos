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
#include <optional>

using namespace llvm;

#define DEBUG_TYPE "mos-disassembler"

using DecodeStatus = MCDisassembler::DecodeStatus;

namespace {
/// A disassembler class for MOS.
class MOSDisassembler : public MCDisassembler {
  bool Has65816RegisterWidths = false;
  mutable bool MLow = false;
  mutable bool XLow = false;

public:
  uint16_t ZeroPageOffset = 0;

  MOSDisassembler(const MCSubtargetInfo &STI, MCContext &Ctx)
      : MCDisassembler(STI, Ctx),
        Has65816RegisterWidths(STI.hasFeature(MOS::FeatureW65816) ||
                               STI.hasFeature(MOS::Feature65EL02)),
        ZeroPageOffset(STI.hasFeature(MOS::FeatureHUC6280) ? 0x2000 : 0) {}
  std::optional<MCDisassembler::DecodeStatus>
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

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeMOSDisassembler() {
  // Register the disassembler.
  TargetRegistry::RegisterMCDisassembler(getTheMOSTarget(),
                                         createMOSDisassembler);
}

static DecodeStatus decodeAddr8Operand(MCInst &Inst, uint64_t Imm,
                                       int64_t Address,
                                       const MCDisassembler *Decoder) {
  auto *MD = static_cast<const MOSDisassembler *>(Decoder);
  if (!isUInt<8>(Imm))
    return MCDisassembler::Fail;
  Inst.addOperand(MCOperand::createImm(Imm + MD->ZeroPageOffset));
  return MCDisassembler::Success;
}

template <unsigned N>
static DecodeStatus decodeUImmOperand(MCInst &Inst, uint64_t Imm,
                                      int64_t Address,
                                      const MCDisassembler *Decoder) {
  if (!isUInt<N>(Imm))
    return MCDisassembler::Fail;
  Inst.addOperand(MCOperand::createImm(Imm));
  return MCDisassembler::Success;
}

template <unsigned N>
static DecodeStatus decodeSImmOperand(MCInst &Inst, uint64_t Imm,
                                      int64_t Address,
                                      const MCDisassembler *Decoder) {
  if (!isUInt<N>(Imm))
    return MCDisassembler::Fail;
  Inst.addOperand(MCOperand::createImm(SignExtend64<N>(Imm)));
  return MCDisassembler::Success;
}

#include "MOSGenDisassemblerTables.inc"

std::optional<const uint8_t *> getDecoderTable(size_t Size) {
  // Suppress unused variable warning for generated function.
  (void)Check;

  switch (Size) {
  case 1:
    return DecoderTableMOS8;
  case 2:
    return DecoderTableMOS16;
  case 3:
    return DecoderTableMOS24;
  default:
    return std::nullopt;
  }
}

std::optional<const uint8_t *> getDecoderTable6502X(size_t Size) {
  switch (Size) {
  case 2:
    return DecoderTable6502x16;
  case 3:
    return DecoderTable6502x24;
  default:
    return std::nullopt;
  }
}

std::optional<const uint8_t *> getDecoderTable65DTV02(size_t Size) {
  // Suppress unused variable warning for generated function.
  (void)Check;

  switch (Size) {
  case 2:
    return DecoderTable65dtv0216;
  default:
    return std::nullopt;
  }
}

std::optional<const uint8_t *> getDecoderTableR65C02(size_t Size) {
  switch (Size) {
  case 2:
    return DecoderTabler65c0216;
  case 3:
    return DecoderTabler65c0224;
  default:
    return std::nullopt;
  }
}

std::optional<const uint8_t *> getDecoderTable65CE02(size_t Size) {
  switch (Size) {
  case 1:
    return DecoderTable65ce028;
  case 2:
    return DecoderTable65ce0216;
  case 3:
    return DecoderTable65ce0224;
  default:
    return std::nullopt;
  }
}

std::optional<const uint8_t *> getDecoderTable45GS02(size_t Size) {
  switch (Size) {
  case 3:
    return DecoderTable45gs0224;
  case 4:
    return DecoderTable45gs0232;
  case 5:
    return DecoderTable45gs0240;
  default:
    return std::nullopt;
  }
}

std::optional<const uint8_t *> getDecoderTableHUC6280(size_t Size) {
  // Suppress unused variable warning for generated function.
  (void)Check;

  switch (Size) {
  case 1:
    return DecoderTablehuc62808;
  case 2:
    return DecoderTablehuc628016;
  case 3:
    return DecoderTablehuc628024;
  case 4:
    return DecoderTablehuc628032;
  case 7:
    return DecoderTablehuc628056;
  default:
    return std::nullopt;
  }
}

std::optional<const uint8_t *> getDecoderTableW65816(size_t Size) {
  // Suppress unused variable warning for generated function.
  (void)Check;

  switch (Size) {
  case 1:
    return DecoderTablew658168;
  case 2:
    return DecoderTablew6581616;
  case 3:
    return DecoderTablew6581624;
  case 4:
    return DecoderTablew6581632;
  default:
    return std::nullopt;
  }
}

std::optional<const uint8_t *> getDecoderTable65EL02(size_t Size) {
  // Suppress unused variable warning for generated function.
  (void)Check;

  switch (Size) {
  case 1:
    return DecoderTable65el028;
  case 2:
    return DecoderTable65el0216;
  case 3:
    return DecoderTable65el0224;
  default:
    return std::nullopt;
  }
}

std::optional<const uint8_t *> getDecoderTableSPC700(size_t Size) {
  switch (Size) {
  case 1:
    return DecoderTablespc7008;
  case 2:
    return DecoderTablespc70016;
  case 3:
    return DecoderTablespc70024;
  default:
    return std::nullopt;
  }
}

template <typename InsnType>
static DecodeStatus
decodeInstruction(std::optional<const uint8_t *> DecodeTable, MCInst &MI,
                  InsnType insn, uint64_t Address, const MCDisassembler *DisAsm,
                  const MCSubtargetInfo &STI) {
  if (!DecodeTable.has_value())
    return MCDisassembler::Fail;

  return decodeInstruction(DecodeTable.value(), MI, insn, Address, DisAsm, STI);
}

std::optional<MCDisassembler::DecodeStatus>
MOSDisassembler::onSymbolStart(SymbolInfoTy &Symbol, uint64_t &Size,
                               ArrayRef<uint8_t> Bytes, uint64_t Address,
                               raw_ostream &CStream) const {
  // 16-bit flags for decoding immediates are set based on the occurrence of
  // mapping symbols $ml, $mh, $xl, $xh.
  if (Has65816RegisterWidths) {
    if (Symbol.Name.starts_with("$ml"))
      MLow = true;
    else if (Symbol.Name.starts_with("$mh"))
      MLow = false;
    else if (Symbol.Name.starts_with("$xl"))
      XLow = true;
    else if (Symbol.Name.starts_with("$xh"))
      XLow = false;
  }
  return std::nullopt;
}

DecodeStatus MOSDisassembler::getInstruction(MCInst &Instr, uint64_t &Size,
                                             ArrayRef<uint8_t> Bytes,
                                             uint64_t Address,
                                             raw_ostream &CStream) const {
  Size = 0;

  // If on SPC700, process the SPC700 table only.
  if (STI.getFeatureBits()[MOS::FeatureSPC700]) {
    for (size_t InsnSize : seq_inclusive(1, 3)) {
      if (Bytes.size() < InsnSize) {
        return MCDisassembler::Fail;
      }
      uint64_t Insn = 0;
      for (size_t Byte : seq((size_t)0, InsnSize)) {
        Insn |= ((uint64_t)Bytes[Byte]) << (8 * Byte);
      }
      DecodeStatus Result = decodeInstruction(getDecoderTableSPC700(InsnSize),
                                              Instr, Insn, Address, this, STI);
      if (Result != MCDisassembler::Fail) {
        Size = InsnSize;
        return Result;
      }
    }
    return MCDisassembler::Fail;
  }

  // Check for 45GS02 extended mnemonics (5, 4, or 3 bytes)
  if (STI.getFeatureBits()[MOS::Feature45GS02]) {
    for (size_t i : seq_inclusive(0, 2)) {
      size_t InsnSize = 5 - i;
      uint64_t Insn = 0;
      if (Bytes.size() < InsnSize) {
        continue;
      }
      for (size_t Byte : seq((size_t)0, InsnSize)) {
        Insn |= ((uint64_t)Bytes[Byte]) << (8 * Byte);
      }
      DecodeStatus Result = decodeInstruction(getDecoderTable45GS02(InsnSize),
                                              Instr, Insn, Address, this, STI);
      if (Result != MCDisassembler::Fail) {
        Size = InsnSize;
        return Result;
      }
    }
  }

  // Attempt to decode 16-bit immediate 65816 instructions.
  if (Has65816RegisterWidths) {
    if ((MLow || XLow) && Bytes.size() >= 3) {
      uint64_t Insn = 0;
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
  for (size_t InsnSize : seq_inclusive(1, 7)) {
    uint64_t Insn = 0;
    DecodeStatus Result = MCDisassembler::Fail;
    if (Bytes.size() < InsnSize) {
      return MCDisassembler::Fail;
    }
    for (size_t Byte : seq((size_t)0, InsnSize)) {
      Insn |= ((uint64_t)Bytes[Byte]) << (8 * Byte);
    }
    if (Result == MCDisassembler::Fail) {
      if (STI.getFeatureBits()[MOS::Feature65CE02]) {
        Result = decodeInstruction(getDecoderTable65CE02(InsnSize), Instr, Insn,
                                   Address, this, STI);
      }
    }
    if (Result == MCDisassembler::Fail) {
      if (STI.getFeatureBits()[MOS::Feature65EL02]) {
        Result = decodeInstruction(getDecoderTable65EL02(InsnSize), Instr, Insn,
                                   Address, this, STI);
      }
    }
    if (Result == MCDisassembler::Fail) {
      if (STI.getFeatureBits()[MOS::FeatureW65816]) {
        Result = decodeInstruction(getDecoderTableW65816(InsnSize), Instr, Insn,
                                   Address, this, STI);
      }
    }
    if (Result == MCDisassembler::Fail) {
      if (STI.getFeatureBits()[MOS::FeatureHUC6280]) {
        Result = decodeInstruction(getDecoderTableHUC6280(InsnSize), Instr,
                                   Insn, Address, this, STI);
      }
    }
    if (Result == MCDisassembler::Fail) {
      if (STI.getFeatureBits()[MOS::FeatureR65C02]) {
        Result = decodeInstruction(getDecoderTableR65C02(InsnSize), Instr, Insn,
                                   Address, this, STI);
      }
    }
    if (Result == MCDisassembler::Fail) {
      if (STI.getFeatureBits()[MOS::Feature65DTV02]) {
        Result = decodeInstruction(getDecoderTable65DTV02(InsnSize), Instr,
                                   Insn, Address, this, STI);
      }
    }
    if (Result == MCDisassembler::Fail) {
      if (STI.getFeatureBits()[MOS::Feature6502X]) {
        Result = decodeInstruction(getDecoderTable6502X(InsnSize), Instr, Insn,
                                   Address, this, STI);
      }
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
