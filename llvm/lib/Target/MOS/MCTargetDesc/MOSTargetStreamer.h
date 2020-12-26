//===-- MOSTargetStreamer.h - MOS Target Streamer --------------*- C++ -*--===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
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
