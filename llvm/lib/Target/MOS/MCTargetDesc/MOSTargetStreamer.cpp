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

#include "llvm/MC/MCContext.h"

namespace llvm {

MOSTargetStreamer::MOSTargetStreamer(MCStreamer &S) : MCTargetStreamer(S) {}

MOSTargetAsmStreamer::MOSTargetAsmStreamer(MCStreamer &S)
    : MOSTargetStreamer(S) {}

void MOSTargetStreamer::finish() {
#ifdef LLVM_MOS_USE_MOS_SYMBOLS_ON_MC
  MCStreamer &OS = getStreamer();
  MCContext &Context = OS.getContext();

  MCSymbol *DoCopyData = Context.getOrCreateSymbol("__do_copy_data");
  MCSymbol *DoClearBss = Context.getOrCreateSymbol("__do_clear_bss");

  // FIXME: We can disable __do_copy_data if there are no static RAM variables.

  OS.emitRawComment(" Declaring this symbol tells the CRT that it should");
  OS.emitRawComment("copy all variables from program memory to RAM on startup");
  OS.EmitSymbolAttribute(DoCopyData, MCSA_Global);

  OS.emitRawComment(" Declaring this symbol tells the CRT that it should");
  OS.emitRawComment("clear the zeroed data section on startup");
  OS.EmitSymbolAttribute(DoClearBss, MCSA_Global);
  #endif //  LLVM_MOS_USE_AVR_SYMBOLS_ON_MC
}

} // end namespace llvm

