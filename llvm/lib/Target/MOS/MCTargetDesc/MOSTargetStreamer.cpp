//===-- MOSTargetStreamer.cpp - MOS Target Streamer Methods ---------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file provides MOS specific target streamer methods.
//
//===----------------------------------------------------------------------===//

#include "MOSTargetStreamer.h"

#include "MOSMCELFStreamer.h"

#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSymbolELF.h"

namespace llvm {

MOSTargetStreamer::MOSTargetStreamer(MCStreamer &S) : MCTargetStreamer(S) {}

void MOSTargetStreamer::finish() {
  MCStreamer &OS = getStreamer();
  MCContext &Context = OS.getContext();

  if (hasInitArray()) {
    MCSymbol *Init = Context.getOrCreateSymbol("_init");
    OS.emitRawComment("Declaring this symbol tells the CRT that there are");
    OS.emitRawComment("initialization routines to be run in .init_array");
    stronglyReference(Init);
  }

  if (hasFiniArray()) {
    MCSymbol *Fini = Context.getOrCreateSymbol("_fini");

    OS.emitRawComment("Declaring this symbol tells the CRT that there are");
    OS.emitRawComment("finalization routines to be run in .fini_array");
    stronglyReference(Fini);
    OS.emitSymbolAttribute(Fini, MCSA_Global);
  }
}

MOSTargetAsmStreamer::MOSTargetAsmStreamer(MCStreamer &S)
    : MOSTargetStreamer(S) {}
void MOSTargetAsmStreamer::changeSection(const MCSection *CurSection,
                                         MCSection *Section,
                                         const MCExpr *SubSection,
                                         raw_ostream &OS) {
  MCTargetStreamer::changeSection(CurSection, Section, SubSection, OS);
  HasInitArray |= Section->getName().startswith(".init_array");
  HasFiniArray |= Section->getName().startswith(".fini_array");
}

void MOSTargetAsmStreamer::stronglyReference(MCSymbol *Sym) {
  getStreamer().emitSymbolAttribute(Sym, MCSA_Global);
}

MOSTargetELFStreamer::MOSTargetELFStreamer(MCStreamer &S,
                                           const MCSubtargetInfo &STI)
    : MOSTargetStreamer(S) {}

bool MOSTargetELFStreamer::hasInitArray() {
  return static_cast<MOSMCELFStreamer &>(getStreamer()).hasInitArray();
}
bool MOSTargetELFStreamer::hasFiniArray() {
  return static_cast<MOSMCELFStreamer &>(getStreamer()).hasFiniArray();
}
void MOSTargetELFStreamer::stronglyReference(MCSymbol *Sym) {
  auto *ES = cast<MCSymbolELF>(Sym);
  // There's an explicit check in emitSymbolAttribute to avoid accidentally
  // overriding weak->global due to a GCC corner case, but it should always be
  // safe for symbols under complete compiler control.
  if (ES->isBindingSet())
    ES->setBinding(ELF::STB_GLOBAL);
  else
    getStreamer().emitSymbolAttribute(Sym, MCSA_Global);
}

} // end namespace llvm
