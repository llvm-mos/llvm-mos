//===-- MOSCombine.td - Describe MOS combiner -------------*- tablegen -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===---------------------------------------------------------------------===//
//
// This file describes which combine operations are included in the MOS
// combiner.
//
//===---------------------------------------------------------------------===//

def fold_global_offset_matchdata : GIDefMatchData<"std::pair<const MachineOperand*, int64_t>">;
def fold_global_offset : GICombineRule<
  (defs root:$root, fold_global_offset_matchdata:$matchinfo),
  (match (wip_match_opcode G_PTR_ADD):$root,
          [{ return matchFoldGlobalOffset(*${root}, MRI, ${matchinfo}); }]),
  (apply [{  return applyFoldGlobalOffset(*${root}, MRI, B, Observer, ${matchinfo});}])>;

def sbc_equal : GICombineRule<
  (defs root:$root),
  (match (wip_match_opcode G_SBC):$root,
          [{ return matchSBCEqual(*${root}, MRI); }]),
  (apply [{  return applySBCEqual(*${root}, MRI, B, Observer);}])>;

def MOSCombinerHelper: GICombinerHelper<
  "MOSGenCombinerHelper", [all_combines, fold_global_offset, sbc_equal]> {
  let DisableRuleOption = "moscombiner-disable-rule";
  let StateClass = "MOSCombinerHelperState";
  let AdditionalArguments = [];
}
