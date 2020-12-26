//===-- MOSAsmBackend.cpp - MOS Asm Backend  ------------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the MOSAsmBackend class.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/MOSAsmBackend.h"
#include "MCTargetDesc/MOSFixupKinds.h"
#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MCTargetDesc/MOSELFObjectWriter.h"

#include "llvm/BinaryFormat/ELF.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDirectives.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCFixupKindInfo.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/raw_ostream.h"

#include <memory>

namespace llvm {
MCAsmBackend *createMOSAsmBackend(const Target &T, const MCSubtargetInfo &STI,
                                  const MCRegisterInfo &MRI,
                                  const llvm::MCTargetOptions &TO) {
  return new MOSAsmBackend(STI.getTargetTriple().getOS());
}

void MOSAsmBackend::relaxInstruction(const MCInst &Inst,
                                     const MCSubtargetInfo &STI,
                                     MCInst &Res) const {
  Res = Inst;
}

MCFixupKindInfo const &MOSAsmBackend::getFixupKindInfo(MCFixupKind Kind) const {
  // NOTE: Many AVR fixups work on sets of non-contignous bits. We work around
  // this by saying that the fixup is the size of the entire instruction.
  const static MCFixupKindInfo Infos[MOS::NumTargetFixupKinds] = {
      // This table *must* be in same the order of fixup_* kinds in
      // MOSFixupKinds.h.
      //
      // name, offset, bits, flags
      {"Imm8", 0, 8, 0}, // An 8 bit immediate value.
      {"Addr8", 0, 8, 0}, // An 8 bit zero page address.
      {"Addr16", 0, 16, 0}, // A 16-bit address.
      {"Addr16_Low", 0, 8, 0}, // The low byte of a 16-bit address.
      {"Addr16_High", 0, 8, 0}, // The high byte of a 16-bit address.
      {"Addr24", 0, 24, 0}, // A 24-bit 65816 address.
      {"Addr24_Segment", 0, 8, 0}, // The segment byte of a 24-bit address.
      {"PCRel8", 0, 8, MCFixupKindInfo::FKF_IsPCRel}
  };

  if (Kind < FirstTargetFixupKind)
    return MCAsmBackend::getFixupKindInfo(Kind);

  assert(unsigned(Kind - FirstTargetFixupKind) < getNumFixupKinds() &&
         "Invalid kind!");

  return Infos[Kind - FirstTargetFixupKind];
}


bool MOSAsmBackend::writeNopData(raw_ostream &OS, uint64_t Count) const {
  // todo: fix for virtual targets

  while ((Count--) > 0)
  {
    OS << 0xEA; // Sports. It's in the game.  Knowing the 6502 hexadecimal
                // representation of a NOP on 6502, used to be an interview
                // question at Electronic Arts.
  }
  return true;
}

bool MOSAsmBackend::mayNeedRelaxation(const MCInst &Inst,
                                      const MCSubtargetInfo &STI) const {
  return false;
}

unsigned MOSAsmBackend::getNumFixupKinds() const {
  return MOS::Fixups::NumTargetFixupKinds;
}

void MOSAsmBackend::applyFixup(const MCAssembler &Asm, const MCFixup &Fixup,
                               const MCValue &Target,
                               MutableArrayRef<char> Data, uint64_t Value,
                               bool IsResolved,
                               const MCSubtargetInfo *STI) const {
  unsigned int Bytes = 0;
  unsigned int Kind = Fixup.getKind();
  switch( Kind )
  {
    case MOS::Imm8:
    case MOS::Addr8:
    case MOS::Addr16_High:
    case MOS::Addr16_Low:
    case MOS::Addr24_Segment:
    case MOS::PCRel8:
      Bytes = 1;
      break;
    case MOS::Addr16:
      Bytes = 2;
      break;
    case MOS::Addr24:
      Bytes = 3;
      break;
    default:
      llvm_unreachable("unknown fixup kind");
      return;
  }

  auto Offset = Fixup.getOffset();

  for (unsigned int T = Offset; T < (Bytes + Offset); T++)
  {
    Data[T] = Value & 0xff;
    Value = Value >> 8;
  }
}

bool MOSAsmBackend::fixupNeedsRelaxation(const MCFixup &Fixup, uint64_t Value,
                                         const MCRelaxableFragment *DF,
                                         const MCAsmLayout &Layout) const {
  return false;
}

std::unique_ptr<llvm::MCObjectTargetWriter>
MOSAsmBackend::createObjectTargetWriter() const {
  return std::make_unique<MOSELFObjectWriter>(OSType);
}

} // namespace llvm