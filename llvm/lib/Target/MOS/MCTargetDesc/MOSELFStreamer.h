//===----- MOSELFStreamer.h - MOS Target Streamer --------------*- C++ -*--===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MOS_ELF_STREAMER_H
#define LLVM_MOS_ELF_STREAMER_H

#include "MOSTargetStreamer.h"

namespace llvm {

/// A target streamer for an MOS ELF object file.
class MOSELFStreamer : public MOSTargetStreamer {
public:
  MOSELFStreamer(MCStreamer &S, const MCSubtargetInfo &STI);

  MCELFStreamer &getStreamer() {
    return static_cast<MCELFStreamer &>(Streamer);
  }
};

} // end namespace llvm

#endif
