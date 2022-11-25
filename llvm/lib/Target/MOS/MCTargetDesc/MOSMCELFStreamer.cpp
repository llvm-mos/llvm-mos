//===--------- MOSMCELFStreamer.cpp - MOS subclass of MCELFStreamer -------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file is a stub that parses a MCInst bundle and passes the
// instructions on to the real streamer.
//
//===----------------------------------------------------------------------===//
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCELFStreamer.h"
#define DEBUG_TYPE "mosmcelfstreamer"

#include "MCTargetDesc/MOSMCELFStreamer.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCObjectFileInfo.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/Casting.h"

using namespace llvm;

namespace llvm {

void MOSMCELFStreamer::initSections(bool NoExecStack,
                                    const MCSubtargetInfo &STI) {
  MCContext &Ctx = getContext();
  switchSection(Ctx.getObjectFileInfo()->getTextSection());
  emitCodeAlignment(1, &STI);

  if (NoExecStack)
    switchSection(Ctx.getAsmInfo()->getNonexecutableStackSection(Ctx));
}

static bool HasPrefix(StringRef Name, StringRef Prefix) {
  SmallString<32> PrefixDot = Prefix;
  PrefixDot += ".";
  return Name == Prefix || Name.startswith(PrefixDot);
}

void MOSMCELFStreamer::changeSection(MCSection *Section, const MCExpr *Subsection) {
  MCELFStreamer::changeSection(Section, Subsection);
  HasBSS |= HasPrefix(Section->getName(), ".bss");
  HasZPBSS |= HasPrefix(Section->getName(), ".zp.bss");
  HasData |= HasPrefix(Section->getName(), ".data");
  HasZPData |= HasPrefix(Section->getName(), ".zp.data");
  HasZPData |= HasPrefix(Section->getName(), ".zp.rodata");
  HasInitArray |= HasPrefix(Section->getName(), ".init_array");
  HasFiniArray |= HasPrefix(Section->getName(), ".fini_array");
}

void MOSMCELFStreamer::emitValueImpl(const MCExpr *Value, unsigned Size,
                                     SMLoc Loc) {
  if (const auto *MME = dyn_cast<MOSMCExpr>(Value)) {
    if (MME->getKind() == MOSMCExpr::VK_MOS_ADDR_ASCIZ) {
      emitMosAddrAsciz(MME->getSubExpr(), Size, Loc);
      return;
    }
  }
  MCELFStreamer::emitValueImpl(Value, Size, Loc);
}

void MOSMCELFStreamer::emitMosAddrAsciz(const MCExpr *Value, unsigned Size,
                                        SMLoc Loc) {
  visitUsedExpr(*Value);
  MCDwarfLineEntry::make(this, getCurrentSectionOnly());
  MCDataFragment *DF = getOrCreateDataFragment();
  flushPendingLabels(DF, DF->getContents().size());

  DF->getFixups().push_back(
      MCFixup::create(DF->getContents().size(), Value,
                      (MCFixupKind)MOS::AddrAsciz, Loc));
  DF->getContents().resize(DF->getContents().size() + Size, 0);
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
