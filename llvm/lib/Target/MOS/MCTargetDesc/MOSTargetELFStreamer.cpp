//===----- MOSTargetELFStreamer.cpp - MOS Target ELF Streamer -------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MOSTargetELFStreamer.h"

#include "llvm/MC/MCSubtargetInfo.h"

using namespace llvm;

MOSTargetELFStreamer::MOSTargetELFStreamer(MCStreamer &S,
                                           const MCSubtargetInfo &STI)
    : MOSTargetStreamer(S) {}
