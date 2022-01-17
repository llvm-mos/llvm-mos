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

void MOSMCELFStreamer::initSections(bool NoExecStack,
                                    const MCSubtargetInfo &STI) {
  MCContext &Ctx = getContext();
  SwitchSection(Ctx.getObjectFileInfo()->getTextSection());
  emitCodeAlignment(1, &STI);

  if (NoExecStack)
    SwitchSection(Ctx.getAsmInfo()->getNonexecutableStackSection(Ctx));
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
