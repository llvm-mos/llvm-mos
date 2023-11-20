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
          [{ return matchFoldGlobalOffset(*${root}, ${matchinfo}); }]),
  (apply [{  return applyFoldGlobalOffset(*${root}, ${matchinfo});}])>;

def sbc_equal : GICombineRule<
  (defs root:$root),
  (match (wip_match_opcode G_SBC):$root,
          [{ return matchSBCEqual(*${root}); }]),
  (apply [{  return applySBCEqual(*${root});}])>;

def extract_low_bit_matchdata : GIDefMatchData<"MachineInstr*">;
def extract_low_bit : GICombineRule<
  (defs root:$root, extract_low_bit_matchdata:$matchinfo),
  (match (wip_match_opcode G_TRUNC, G_ICMP):$root,
          [{ return matchExtractLowBit(*${root}, ${matchinfo}); }]),
  (apply [{  return applyExtractLowBit(*${root}, ${matchinfo});}])>;

def uaddo_1 : GICombineRule<
  (defs root:$root),
  (match (wip_match_opcode G_UADDO):$root,
          [{ return matchUAddO1(*${root}); }]),
  (apply [{  return applyUAddO1(*${root});}])>;

def cmpz_zero_matchdata : GIDefMatchData<"MachineOperand*">;
def cmpz_zero : GICombineRule<
  (defs root:$root, cmpz_zero_matchdata:$matchinfo),
  (match (wip_match_opcode G_CMPZ):$root,
          [{ return matchCMPZZero(*${root}, ${matchinfo}); }]),
  (apply [{  return applyCMPZZero(*${root}, ${matchinfo});}])>;

def load_store_to_memcpy_matchdata : GIDefMatchData<"GLoad*">;
def load_store_to_memcpy : GICombineRule<
  (defs root:$root, load_store_to_memcpy_matchdata:$matchinfo),
  (match (wip_match_opcode G_STORE):$root,
          [{ return matchLoadStoreToMemcpy(*${root}, ${matchinfo}); }]),
  (apply [{  return applyLoadStoreToMemcpy(*${root}, ${matchinfo});}])>;

def store_to_memset_matchdata : GIDefMatchData<"uint8_t">;
def store_to_memset : GICombineRule<
  (defs root:$root, store_to_memset_matchdata:$matchinfo),
  (match (wip_match_opcode G_STORE):$root,
          [{ return matchStoreToMemset(*${root}, ${matchinfo}); }]),
  (apply [{  return applyStoreToMemset(*${root}, ${matchinfo});}])>;

def fold_adde : GICombineRule<
  (defs root:$root, build_fn_matchinfo:$matchinfo),
  (match (wip_match_opcode G_UADDE, G_SADDE):$root,
          [{ return matchFoldAddE(*${root}, ${matchinfo}); }]),
  (apply [{ return Helper.applyBuildFn(*${root}, ${matchinfo}); }])>;

def fold_sbc : GICombineRule<
  (defs root:$root, build_fn_matchinfo:$matchinfo),
  (match (wip_match_opcode G_SBC):$root,
          [{ return matchFoldShift(*${root}, ${matchinfo}); }]),
  (apply [{ return Helper.applyBuildFn(*${root}, ${matchinfo}); }])>;

def fold_shift : GICombineRule<
  (defs root:$root, build_fn_matchinfo:$matchinfo),
  (match (wip_match_opcode G_SHLE, G_LSHRE):$root,
          [{ return matchFoldShift(*${root}, ${matchinfo}); }]),
  (apply [{ return Helper.applyBuildFn(*${root}, ${matchinfo}); }])>;

def shift_unused_carry_in : GICombineRule<
  (defs root:$root, build_fn_matchinfo:$matchinfo),
  (match (wip_match_opcode G_SHLE, G_LSHRE):$root,
          [{ return matchShiftUnusedCarryIn(*${root}, ${matchinfo}); }]),
  (apply [{ return Helper.applyBuildFnNoErase(*${root}, ${matchinfo}); }])>;

def mul_to_shift_and_add : GICombineRule<
  (defs root:$root, build_fn_matchinfo:$matchinfo),
  (match (wip_match_opcode G_MUL):$root,
          [{ return matchMulToShiftAndAdd(*${root}, ${matchinfo}); }]),
  (apply [{ return Helper.applyBuildFnNoErase(*${root}, ${matchinfo}); }])>;

def MOSCombiner : GICombiner<"MOSCombinerImpl", [
  all_combines,
  fold_global_offset,
  extract_low_bit,
  uaddo_1,
  cmpz_zero,
  load_store_to_memcpy,
  store_to_memset,
  fold_adde,
  fold_sbc,
  sbc_equal,
  fold_shift,
  shift_unused_carry_in,
  mul_to_shift_and_add,
]>;
