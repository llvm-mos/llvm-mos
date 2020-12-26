//===--------- MOSMCELFStreamer.cpp - MOS subclass of MCELFStreamer -------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file is a stub that parses a MCInst bundle and passes the
// instructions on to the real streamer.
//
//===----------------------------------------------------------------------===//
#include "llvm/BinaryFormat/ELF.h"
#define DEBUG_TYPE "mosmcelfstreamer"

#include "MCTargetDesc/MOSMCELFStreamer.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCObjectFileInfo.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSymbol.h"

using namespace llvm;

namespace llvm {

void MOSMCELFStreamer::InitSections(bool NoExecStack) {
  MCContext &Ctx = getContext();
  SwitchSection(Ctx.getObjectFileInfo()->getTextSection());
  emitCodeAlignment(1);

  if (NoExecStack)
    SwitchSection(Ctx.getAsmInfo()->getNonexecutableStackSection(Ctx));
}

void MOSMCELFStreamer::emitValueForModiferKind(
    const MCSymbol *Sym, unsigned SizeInBytes, SMLoc Loc,
    MOSMCExpr::VariantKind ModifierKind) {
  MCSymbolRefExpr::VariantKind Kind = MCSymbolRefExpr::VK_Invalid;
  if (ModifierKind == MOSMCExpr::VK_MOS_ADDR16_LO) {
    Kind = MCSymbolRefExpr::VK_MOS_ADDR16_LO;
  } else if (ModifierKind == MOSMCExpr::VK_MOS_ADDR16_HI) {
    Kind = MCSymbolRefExpr::VK_MOS_ADDR16_HI;
  } else if (ModifierKind == MOSMCExpr::VK_MOS_ADDR24_BANK) {
    Kind = MCSymbolRefExpr::VK_MOS_ADDR24_BANK;
  } else if (ModifierKind == MOSMCExpr::VK_MOS_ADDR24_SEGMENT) {
    Kind = MCSymbolRefExpr::VK_MOS_ADDR24_SEGMENT;
  } else if (ModifierKind == MOSMCExpr::VK_MOS_ADDR24_SEGMENT_LO) {
    Kind = MCSymbolRefExpr::VK_MOS_ADDR24_SEGMENT_LO;
  } else if (ModifierKind == MOSMCExpr::VK_MOS_ADDR24_SEGMENT_HI) {
    Kind = MCSymbolRefExpr::VK_MOS_ADDR24_SEGMENT_HI;
  }
  MCELFStreamer::emitValue(MCSymbolRefExpr::create(Sym, Kind, getContext()),
                           SizeInBytes, Loc);
}

MCStreamer *createMOSMCELFStreamer(const Triple & /*T*/, MCContext &Ctx,
                                   std::unique_ptr<MCAsmBackend> &&TAB,
                                   std::unique_ptr<MCObjectWriter> &&OW,
                                   std::unique_ptr<MCCodeEmitter> &&Emitter,
                                   bool RelaxAll) {
  auto *S = new MOSMCELFStreamer(Ctx, std::move(TAB), std::move(OW),
                                 std::move(Emitter));
  if (RelaxAll) {
    S->getAssembler().setRelaxAll(true);
  }
  return S;
}

} // end namespace llvm
