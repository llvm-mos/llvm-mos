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
