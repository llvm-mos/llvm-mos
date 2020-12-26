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
      return ELF::R_MOS_8;
    case MCSymbolRefExpr::VK_MOS_DIFF8:
      return ELF::R_MOS_DIFF8;
    case MCSymbolRefExpr::VK_MOS_LO8:
      return ELF::R_MOS_8_LO8;
    case MCSymbolRefExpr::VK_MOS_HI8:
      return ELF::R_MOS_8_HI8;
    case MCSymbolRefExpr::VK_MOS_HLO8:
      return ELF::R_MOS_8_HLO8;
    }
  case FK_Data_4:
    switch (Modifier) {
    default:
      llvm_unreachable("Unsupported Modifier");
    case MCSymbolRefExpr::VK_None:
      return ELF::R_MOS_32;
    case MCSymbolRefExpr::VK_MOS_DIFF32:
      return ELF::R_MOS_DIFF32;
    }
  case FK_Data_2:
    switch (Modifier) {
    default:
      llvm_unreachable("Unsupported Modifier");
    case MCSymbolRefExpr::VK_None:
      return ELF::R_MOS_16;
    case MCSymbolRefExpr::VK_MOS_NONE:
      return ELF::R_MOS_16_PM;
    case MCSymbolRefExpr::VK_MOS_DIFF16:
      return ELF::R_MOS_DIFF16;
    }
  case MOS::fixup_32:
    return ELF::R_MOS_32;
  case MOS::fixup_7_pcrel:
    return ELF::R_MOS_7_PCREL;
  case MOS::fixup_13_pcrel:
    return ELF::R_MOS_13_PCREL;
  case MOS::fixup_16:
    return ELF::R_MOS_16;
  case MOS::fixup_16_pm:
    return ELF::R_MOS_16_PM;
  case MOS::fixup_lo8_ldi:
    return ELF::R_MOS_LO8_LDI;
  case MOS::fixup_hi8_ldi:
    return ELF::R_MOS_HI8_LDI;
  case MOS::fixup_hh8_ldi:
    return ELF::R_MOS_HH8_LDI;
  case MOS::fixup_lo8_ldi_neg:
    return ELF::R_MOS_LO8_LDI_NEG;
  case MOS::fixup_hi8_ldi_neg:
    return ELF::R_MOS_HI8_LDI_NEG;
  case MOS::fixup_hh8_ldi_neg:
    return ELF::R_MOS_HH8_LDI_NEG;
  case MOS::fixup_lo8_ldi_pm:
    return ELF::R_MOS_LO8_LDI_PM;
  case MOS::fixup_hi8_ldi_pm:
    return ELF::R_MOS_HI8_LDI_PM;
  case MOS::fixup_hh8_ldi_pm:
    return ELF::R_MOS_HH8_LDI_PM;
  case MOS::fixup_lo8_ldi_pm_neg:
    return ELF::R_MOS_LO8_LDI_PM_NEG;
  case MOS::fixup_hi8_ldi_pm_neg:
    return ELF::R_MOS_HI8_LDI_PM_NEG;
  case MOS::fixup_hh8_ldi_pm_neg:
    return ELF::R_MOS_HH8_LDI_PM_NEG;
  case MOS::fixup_call:
    return ELF::R_MOS_CALL;
  case MOS::fixup_ldi:
    return ELF::R_MOS_LDI;
  case MOS::fixup_6:
    return ELF::R_MOS_6;
  case MOS::fixup_6_adiw:
    return ELF::R_MOS_6_ADIW;
  case MOS::fixup_ms8_ldi:
    return ELF::R_MOS_MS8_LDI;
  case MOS::fixup_ms8_ldi_neg:
    return ELF::R_MOS_MS8_LDI_NEG;
  case MOS::fixup_lo8_ldi_gs:
    return ELF::R_MOS_LO8_LDI_GS;
  case MOS::fixup_hi8_ldi_gs:
    return ELF::R_MOS_HI8_LDI_GS;
  case MOS::fixup_8:
    return ELF::R_MOS_8;
  case MOS::fixup_8_lo8:
    return ELF::R_MOS_8_LO8;
  case MOS::fixup_8_hi8:
    return ELF::R_MOS_8_HI8;
  case MOS::fixup_8_hlo8:
    return ELF::R_MOS_8_HLO8;
  case MOS::fixup_diff8:
    return ELF::R_MOS_DIFF8;
  case MOS::fixup_diff16:
    return ELF::R_MOS_DIFF16;
  case MOS::fixup_diff32:
    return ELF::R_MOS_DIFF32;
  case MOS::fixup_lds_sts_16:
    return ELF::R_MOS_LDS_STS_16;
  case MOS::fixup_port6:
    return ELF::R_MOS_PORT6;
  case MOS::fixup_port5:
    return ELF::R_MOS_PORT5;
  default:
    llvm_unreachable("invalid fixup kind!");
  }
}

std::unique_ptr<MCObjectTargetWriter> createMOSELFObjectWriter(uint8_t OSABI) {
  return make_unique<MOSELFObjectWriter>(OSABI);
}

} // end of namespace llvm

