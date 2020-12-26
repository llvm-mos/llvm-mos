//===-- MOSELFObjectWriter.cpp - MOS ELF Writer ---------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/MOSFixupKinds.h"
#include "MCTargetDesc/MOSMCTargetDesc.h"

#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSection.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Support/ErrorHandling.h"

namespace llvm {

/// Writes MOS machine code into an ELF32 object file.
class MOSELFObjectWriter : public MCELFObjectTargetWriter {
public:
  MOSELFObjectWriter(uint8_t OSABI);
  virtual ~MOSELFObjectWriter() {}

  unsigned getRelocType(MCContext &Ctx,
                        const MCValue &Target,
                        const MCFixup &Fixup,
                        bool IsPCRel) const override;
};

MOSELFObjectWriter::MOSELFObjectWriter(uint8_t OSABI)
    : MCELFObjectTargetWriter(false, OSABI, ELF::EM_MOS, true) {}

unsigned MOSELFObjectWriter::getRelocType(MCContext &Ctx,
                                          const MCValue &Target,
                                          const MCFixup &Fixup,
                                          bool IsPCRel) const {
  MCSymbolRefExpr::VariantKind Modifier = Target.getAccessVariant();
  switch ((unsigned) Fixup.getKind()) {
  case FK_Data_1:
    switch (Modifier) {
    default:
      llvm_unreachable("Unsupported Modifier");
    case MCSymbolRefExpr::VK_None:
      return ELF::R_AVR_8;
    case MCSymbolRefExpr::VK_AVR_DIFF8:
      return ELF::R_AVR_DIFF8;
    case MCSymbolRefExpr::VK_AVR_LO8:
      return ELF::R_AVR_8_LO8;
    case MCSymbolRefExpr::VK_AVR_HI8:
      return ELF::R_AVR_8_HI8;
    case MCSymbolRefExpr::VK_AVR_HLO8:
      return ELF::R_AVR_8_HLO8;
    }
  case FK_Data_4:
    switch (Modifier) {
    default:
      llvm_unreachable("Unsupported Modifier");
    case MCSymbolRefExpr::VK_None:
      return ELF::R_AVR_32;
    case MCSymbolRefExpr::VK_AVR_DIFF32:
      return ELF::R_AVR_DIFF32;
    }
  case FK_Data_2:
    switch (Modifier) {
    default:
      llvm_unreachable("Unsupported Modifier");
    case MCSymbolRefExpr::VK_None:
      return ELF::R_AVR_16;
    case MCSymbolRefExpr::VK_AVR_NONE:
      return ELF::R_AVR_16_PM;
    case MCSymbolRefExpr::VK_AVR_DIFF16:
      return ELF::R_AVR_DIFF16;
    }
  case MOS::Imm8:
    return ELF::R_MOS_IMM8;
  case MOS::Addr8:
    return ELF::R_MOS_ADDR8;
  case MOS::Addr16:
    return ELF::R_MOS_ADDR16;
  case MOS::Addr16_Low:
    return ELF::R_MOS_ADDR16_LOW;
  case MOS::Addr16_High:
    return ELF::R_MOS_ADDR16_HIGH;
  case MOS::Addr24:
    return ELF::R_MOS_ADDR24;
  case MOS::Addr24_Segment:
    return ELF::R_MOS_ADDR24_SEGMENT;
  case MOS::PCRel8:
    return ELF::R_MOS_PCREL_8;
  default:
    llvm_unreachable("invalid fixup kind!");
  }
}

std::unique_ptr<MCObjectTargetWriter> createMOSELFObjectWriter(uint8_t OSABI) {
  return std::make_unique<MOSELFObjectWriter>(OSABI);
}

} // end of namespace llvm

