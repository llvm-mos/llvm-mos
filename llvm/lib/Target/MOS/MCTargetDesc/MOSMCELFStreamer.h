//===--------- MOSMCELFStreamer.h - MOS subclass of MCELFStreamer ---------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MCTARGETDESC_MOSMCELFSTREAMER_H
#define LLVM_LIB_TARGET_MOS_MCTARGETDESC_MOSMCELFSTREAMER_H

#include "MCTargetDesc/MOSMCExpr.h"
#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCELFStreamer.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCObjectWriter.h"

namespace llvm {

class MOSMCELFStreamer : public MCELFStreamer {
  std::unique_ptr<MCInstrInfo> MCII;

public:
  MOSMCELFStreamer(MCContext &Context, std::unique_ptr<MCAsmBackend> TAB,
                   std::unique_ptr<MCObjectWriter> OW,
                   std::unique_ptr<MCCodeEmitter> Emitter)
      : MCELFStreamer(Context, std::move(TAB), std::move(OW),
                      std::move(Emitter)),
        MCII(createMOSMCInstrInfo()) {}

  void initSections(bool NoExecStack, const MCSubtargetInfo &STI) override;
  void changeSection(MCSection *Section, const MCExpr *Subsection) override;

  void emitValueImpl(const MCExpr *Value, unsigned Size,
                     SMLoc Loc = SMLoc()) override;

  void emitMosAddrAsciz(const MCExpr *Value, unsigned Size,
                        SMLoc Loc = SMLoc());

  bool hasBSS() const { return HasBSS; }
  bool hasZPBSS() const { return HasZPBSS; }
  bool hasData() const { return HasData; }
  bool hasZPData() const { return HasZPData; }
  bool hasInitArray() const { return HasInitArray; }
  bool hasFiniArray() const { return HasFiniArray; }

private:
  bool HasBSS = false;
  bool HasZPBSS = false;
  bool HasData = false;
  bool HasZPData = false;
  bool HasInitArray = false;
  bool HasFiniArray = false;
};

MCStreamer *createMOSMCELFStreamer(const Triple &T, MCContext &Ctx,
                                   std::unique_ptr<MCAsmBackend> &&TAB,
                                   std::unique_ptr<MCObjectWriter> &&OW,
                                   std::unique_ptr<MCCodeEmitter> &&Emitter,
                                   bool RelaxAll);

} // end namespace llvm

#endif // LLVM_LIB_TARGET_MOS_MCTARGETDESC_MOSMCELFSTREAMER_H
