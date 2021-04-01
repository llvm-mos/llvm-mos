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

def MOSCombinerHelper: GICombinerHelper<
  "MOSGenCombinerHelper", [all_combines]> {
  let DisableRuleOption = "moscombiner-disable-rule";
  let StateClass = "MOSCombinerHelperState";
  let AdditionalArguments = [];
}
