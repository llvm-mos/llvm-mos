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

  if (hasInitArray())
    stronglyReference("__do_init_array",
                      "Declaring this symbol tells the CRT that there are "
                      "initialization routines to be run in .init_array");

  if (hasFiniArray())
    stronglyReference("__do_fini_array",
                      "Declaring this symbol tells the CRT that there are "
                      "finalization routines to be run in .fini_array");

  for (const auto &TableEntry : Context.getSymbols()) {
    if (TableEntry.getKey() != "__rc0" && TableEntry.getKey() != "__rc1")
      continue;
    stronglyReference("__do_init_stack",
                      "Declaring this symbol tells the CRT that the stack "
                      "pointer needs to be initialized.");
  }
}

void MOSTargetStreamer::stronglyReference(StringRef Name, StringRef Comment) {
  MCStreamer &OS = getStreamer();
  MCContext &Context = OS.getContext();
  MCSymbol *InitStack = Context.getOrCreateSymbol(Name);
  OS.emitRawComment(Comment);
  stronglyReference(InitStack);
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
