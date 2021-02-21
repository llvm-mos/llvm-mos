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
#include "llvm/MC/MCFixup.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSection.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Support/ErrorHandling.h"

namespace llvm {

/// Writes MOS machine code into an ELF32 object file.
class MOSELFObjectWriter : public MCELFObjectTargetWriter {
public:
  explicit MOSELFObjectWriter(uint8_t OSABI);

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
  switch ((unsigned) Fixup.getKind()) {
  case MOS::Imm8:
    return ELF::R_MOS_IMM8;
  case MOS::Addr8:
  case FK_Data_1:
    return ELF::R_MOS_ADDR8;
  case FK_Data_2:
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
  default:
    llvm_unreachable("invalid fixup kind!");
  }
}

std::unique_ptr<MCObjectTargetWriter> createMOSELFObjectWriter(uint8_t OSABI) {
  return std::make_unique<MOSELFObjectWriter>(OSABI);
}

} // end of namespace llvm

