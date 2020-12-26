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
    }
  case FK_Data_4:
    switch (Modifier) {
    default:
      llvm_unreachable("Unsupported Modifier");
    }
  case FK_Data_2:
    switch (Modifier) {
    default:
      llvm_unreachable("Unsupported Modifier");
    }
  default:
    llvm_unreachable("invalid fixup kind!");
  }
}

std::unique_ptr<MCObjectTargetWriter> createMOSELFObjectWriter(uint8_t OSABI) {
  return make_unique<MOSELFObjectWriter>(OSABI);
}

} // end of namespace llvm

