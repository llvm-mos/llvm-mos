#include "MOSELFStreamer.h"

#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/SubtargetFeature.h"
#include "llvm/Support/FormattedStream.h"

#include "MOSMCTargetDesc.h"

namespace llvm {

static unsigned getEFlagsForFeatureSet(const FeatureBitset &Features) {
  unsigned EFlags = 0;

  // Set architecture
  if (Features[MOS::ELFArchMOS1])
    EFlags |= ELF::EF_MOS_ARCH_MOS1;
  else if (Features[MOS::ELFArchMOS2])
    EFlags |= ELF::EF_MOS_ARCH_MOS2;
  else if (Features[MOS::ELFArchMOS25])
    EFlags |= ELF::EF_MOS_ARCH_MOS25;
  else if (Features[MOS::ELFArchMOS3])
    EFlags |= ELF::EF_MOS_ARCH_MOS3;
  else if (Features[MOS::ELFArchMOS31])
    EFlags |= ELF::EF_MOS_ARCH_MOS31;
  else if (Features[MOS::ELFArchMOS35])
    EFlags |= ELF::EF_MOS_ARCH_MOS35;
  else if (Features[MOS::ELFArchMOS4])
    EFlags |= ELF::EF_MOS_ARCH_MOS4;
  else if (Features[MOS::ELFArchMOS5])
    EFlags |= ELF::EF_MOS_ARCH_MOS5;
  else if (Features[MOS::ELFArchMOS51])
    EFlags |= ELF::EF_MOS_ARCH_MOS51;
  else if (Features[MOS::ELFArchMOS6])
    EFlags |= ELF::EF_MOS_ARCH_MOS6;
  else if (Features[MOS::ELFArchTiny])
    EFlags |= ELF::EF_MOS_ARCH_MOSTINY;
  else if (Features[MOS::ELFArchXMEGA1])
    EFlags |= ELF::EF_MOS_ARCH_XMEGA1;
  else if (Features[MOS::ELFArchXMEGA2])
    EFlags |= ELF::EF_MOS_ARCH_XMEGA2;
  else if (Features[MOS::ELFArchXMEGA3])
    EFlags |= ELF::EF_MOS_ARCH_XMEGA3;
  else if (Features[MOS::ELFArchXMEGA4])
    EFlags |= ELF::EF_MOS_ARCH_XMEGA4;
  else if (Features[MOS::ELFArchXMEGA5])
    EFlags |= ELF::EF_MOS_ARCH_XMEGA5;
  else if (Features[MOS::ELFArchXMEGA6])
    EFlags |= ELF::EF_MOS_ARCH_XMEGA6;
  else if (Features[MOS::ELFArchXMEGA7])
    EFlags |= ELF::EF_MOS_ARCH_XMEGA7;

  return EFlags;
}

MOSELFStreamer::MOSELFStreamer(MCStreamer &S,
                               const MCSubtargetInfo &STI)
    : MOSTargetStreamer(S) {

  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned EFlags = MCA.getELFHeaderEFlags();

  EFlags |= getEFlagsForFeatureSet(STI.getFeatureBits());

  MCA.setELFHeaderEFlags(EFlags);
}

} // end namespace llvm
