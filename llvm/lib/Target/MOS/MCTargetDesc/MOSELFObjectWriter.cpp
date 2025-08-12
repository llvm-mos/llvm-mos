//===-- MOSELFObjectWriter.cpp - MOS ELF Writer ---------------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MOSELFObjectWriter.h"

#include "MCTargetDesc/MOSFixupKinds.h"
#include "MCTargetDesc/MOSMCExpr.h"
#include "MCTargetDesc/MOSMCTargetDesc.h"

#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCFixup.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSection.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Support/ErrorHandling.h"

namespace llvm {

MOSELFObjectWriter::MOSELFObjectWriter(uint8_t OSABI)
    : MCELFObjectTargetWriter(false, OSABI, ELF::EM_MOS, true) {}

unsigned MOSELFObjectWriter::getRelocType(const MCFixup &Fixup,
                                          const MCValue &Target,
                                          bool IsPCRel) const {
  unsigned Kind = Fixup.getKind();
  auto Specifier = static_cast<MOSMCExpr::VariantKind>(Target.getSpecifier());
  switch (Kind) {
  case FK_Data_1:
    switch (Specifier) {
    default:
      llvm_unreachable("Unsupported Specifier");
    case MOSMCExpr::VK_NONE:
    case MOSMCExpr::VK_ADDR8:
      return ELF::R_MOS_ADDR8;
    case MOSMCExpr::VK_ADDR16_LO:
      return ELF::R_MOS_ADDR16_LO;
    case MOSMCExpr::VK_ADDR16_HI:
      return ELF::R_MOS_ADDR16_HI;
    case MOSMCExpr::VK_ADDR24_BANK:
      return ELF::R_MOS_ADDR24_BANK;
    case MOSMCExpr::VK_ADDR24_SEGMENT_LO:
      return ELF::R_MOS_ADDR24_SEGMENT_LO;
    case MOSMCExpr::VK_ADDR24_SEGMENT_HI:
      return ELF::R_MOS_ADDR24_SEGMENT_HI;
    case MOSMCExpr::VK_ADDR13:
      return ELF::R_MOS_ADDR13;
    }
  case FK_Data_2:
    switch (Specifier) {
    default:
      llvm_unreachable("Unsupported Specifier");
    case MOSMCExpr::VK_NONE:
    case MOSMCExpr::VK_ADDR16:
      return ELF::R_MOS_ADDR16;
    case MOSMCExpr::VK_ADDR13:
      return ELF::R_MOS_ADDR13;
    case MOSMCExpr::VK_ADDR24_SEGMENT:
      return ELF::R_MOS_ADDR24_SEGMENT;
    }

  case MOS::Imm8:
    return ELF::R_MOS_IMM8;
  case MOS::Addr8:
    return ELF::R_MOS_ADDR8;
  case MOS::Addr16:
    return ELF::R_MOS_ADDR16;
  case MOS::Addr16_Low:
    return ELF::R_MOS_ADDR16_LO;
  case MOS::Addr16_High:
    return ELF::R_MOS_ADDR16_HI;
  case MOS::Addr24:
    return ELF::R_MOS_ADDR24;
  case MOS::Addr24_Bank:
    return ELF::R_MOS_ADDR24_BANK;
  case MOS::Addr24_Segment:
    return ELF::R_MOS_ADDR24_SEGMENT;
  case MOS::Addr24_Segment_Low:
    return ELF::R_MOS_ADDR24_SEGMENT_LO;
  case MOS::Addr24_Segment_High:
    return ELF::R_MOS_ADDR24_SEGMENT_HI;
  case MOS::PCRel8:
    return ELF::R_MOS_PCREL_8;
  case MOS::PCRel16:
    return ELF::R_MOS_PCREL_16;
  case FK_Data_4:
    return ELF::R_MOS_FK_DATA_4;
  case FK_Data_8:
    return ELF::R_MOS_FK_DATA_8;
  case MOS::AddrAsciz:
    return ELF::R_MOS_ADDR_ASCIZ;
  case MOS::Imm16:
    return ELF::R_MOS_IMM16;
  case MOS::Addr13:
    return ELF::R_MOS_ADDR13;

  default:
    llvm_unreachable("invalid fixup kind!");
  }
}

std::unique_ptr<MCObjectTargetWriter> createMOSELFObjectWriter(uint8_t OSABI) {
  return std::make_unique<MOSELFObjectWriter>(OSABI);
}

} // end of namespace llvm
