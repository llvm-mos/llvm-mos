//===----- MOSELFStreamer.cpp - MOS Target Streamer -----------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MOSELFStreamer.h"

#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/SubtargetFeature.h"
#include "llvm/Support/FormattedStream.h"

#include "MOSMCTargetDesc.h"

namespace llvm {

MOSELFStreamer::MOSELFStreamer(MCStreamer &S,
                               const MCSubtargetInfo &STI)
    : MOSTargetStreamer(S) {
}

} // end namespace llvm
