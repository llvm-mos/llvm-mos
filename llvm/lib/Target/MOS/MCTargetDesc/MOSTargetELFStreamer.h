//===----- MOSTargetELFStreamer.h - MOS Target ELF Streamer ----*- C++ -*--===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MOS_TARGET_ELF_STREAMER_H
#define LLVM_MOS_TARGET_ELF_STREAMER_H

#include "MOSTargetStreamer.h"

namespace llvm {

/// A target streamer for an MOS ELF object file.
class MOSTargetELFStreamer : public MOSTargetStreamer {
public:
  MOSTargetELFStreamer(MCStreamer &S, const MCSubtargetInfo &STI);
};

} // end namespace llvm

#endif
