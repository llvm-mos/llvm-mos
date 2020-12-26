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
#define DEBUG_TYPE "mosmcelfstreamer"

#include "MCTargetDesc/MOSMCELFStreamer.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSymbol.h"

using namespace llvm;

namespace llvm {
MCStreamer *createMOSMCELFStreamer(
    const Triple & /*T*/, MCContext &Ctx, std::unique_ptr<MCAsmBackend> &&TAB,
    std::unique_ptr<MCObjectWriter> &&OW,
    std::unique_ptr<MCCodeEmitter> &&Emitter, bool RelaxAll) {
  auto *S = new MOSMCELFStreamer(Ctx, std::move(TAB), std::move(OW),
                                 std::move(Emitter));
  if (RelaxAll) {
    S->getAssembler().setRelaxAll(true);
  }
  return S;
}

} // end namespace llvm
