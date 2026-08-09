# REQUIRES: mos
# RUN: llvm-mc -filetype=obj -triple=mos -mcpu=mosw65816 %s -o %t816.o
# RUN: llvm-mc -filetype=obj -triple=mos -mcpu=mos65ce02 %s -o %tce02.o
# RUN: not ld.lld %t816.o %tce02.o -o /dev/null 2>&1 | FileCheck %s

## The two CPUs disagree on the base address of a 16-bit branch, and the linker
## resolves R_MOS_PCREL_16 from the merged output flags, which cannot tell one
## input apart from the other. Reject the combination rather than silently
## mis-resolving one of them. The lineages are disjoint — no CPU implies both
## FeatureW65816 and Feature65CE02 — so nothing legitimate is refused.

# CHECK:      error: Input file
# CHECK-SAME: uses bad MOS feature combination
# CHECK:      Input file: Flags
# CHECK:      EF_MOS_ARCH_65CE02
# CHECK:      Output file: Flags
# CHECK:      EF_MOS_ARCH_W65816

  nop
