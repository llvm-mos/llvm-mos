//===-- MOSTargetStreamer.h - MOS Target Streamer --------------*- C++ -*--===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MOS_TARGET_STREAMER_H
#define LLVM_MOS_TARGET_STREAMER_H

#include "llvm/MC/MCELFStreamer.h"

namespace llvm {
class MCStreamer;

/// A generic MOS target output stream.
class MOSTargetStreamer : public MCTargetStreamer {
public:
  explicit MOSTargetStreamer(MCStreamer &S);

  void finish() override;
};

/// A target streamer for textual MOS assembly code.
class MOSTargetAsmStreamer : public MOSTargetStreamer {
public:
  explicit MOSTargetAsmStreamer(MCStreamer &S);
};

} // end namespace llvm

#endif // LLVM_MOS_TARGET_STREAMER_H
