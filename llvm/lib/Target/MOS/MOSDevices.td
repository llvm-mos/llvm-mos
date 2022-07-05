//===-- MOSDevices.td - MOS Devices -----------------------*- tablegen -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===---------------------------------------------------------------------===//
// MOS Device Definitions
//===---------------------------------------------------------------------===//

def Feature6502
    : SubtargetFeature<"mos-insns-6502", "Has6502Insns", "true",
                       "The original documented 6502 instruction set">;

def Feature6502BCD
    : SubtargetFeature<"mos-insns-6502bcd", "Has6502BCDInsns", "true",
                       "BCD instruction support, including SED and CLD (most "
                       "6502 series CPUs support this)">;

def Feature6502X
    : SubtargetFeature<"mos-insns-6502x", "Has6502XInsns", "true",
                       "The 'illegal' opcodes present on some early variants of "
                       "the original 6502 processor">;

def Feature65C02
    : SubtargetFeature<"mos-insns-65c02", "Has65C02Insns", "true",
                       "The new and modified instructions present on the generic "
                       "65c02 and variants">;

def FeatureR65C02
    : SubtargetFeature<"mos-insns-r65c02", "HasR65C02Insns", "true",
                       "The new and modified instructions present on Rockwell "
                       "and WDC 65c02">;

def FeatureW65C02
    : SubtargetFeature<"mos-insns-w65c02", "HasW65C02Insns", "true",
                       "The new and modified instructions present on "
                       "WDC 65c02">;

def FeatureW65816
    : SubtargetFeature<"mos-insns-w65816", "HasW65816Insns", "true",
                       "The new and modified instructions present on "
                       "WDC 65816">;

def Feature65EL02
    : SubtargetFeature<"mos-insns-65el02", "Has65EL02Insns", "true",
                       "The new and modified instructions present on "
                       "65EL02">;

def Feature65CE02
    : SubtargetFeature<"mos-insns-65ce02", "Has65CE02Insns", "true",
                       "The new and modified instructions present on "
                       "65ce02 and variants">;

def FeatureSWEET16
    : SubtargetFeature<"mos-insns-sweet16", "HasSWEET16Insns", "true",
                        "The SWEET16 instruction set">;

// A feature set aggregates features, grouping them. We don't want to create a
// new member in MOSSubtarget (to store a value) for each set because we do not
// care if the set is supported, only the subfeatures inside the set. We fix
// this by simply setting the same dummy member for all feature sets, which is
// then ignored.
class FeatureSet<string name, string desc, list<SubtargetFeature> i>
  : SubtargetFeature<name, "FeatureSetDummy", "true", desc, i>;

// A family of microcontrollers, defining a set of supported features.
class Family<string name, string desc, list<SubtargetFeature> i>
  : FeatureSet<name, desc, i>;


def Family6502
    : Family<
      "mos6502",
      "Generic MOS 6502, with support for BCD instructions",
      [Feature6502, Feature6502BCD, FeatureStaticStack]>;

def Family6502X
    : Family<"mos6502x",
      "NMOS 6502, with illegal opcode support",
      Family6502.Implies#[Feature6502X]>;

def Family65C02
    : Family<"mos65c02",
      "Generic MOS 65C02",
      Family6502.Implies#[Feature65C02]>;

def FamilyR65C02
    : Family<"mosr65c02",
      "Rockwell 65C02",
      Family65C02.Implies#[FeatureR65C02]>;

def FamilyW65C02
    : Family<"mosw65c02",
      "WDC 65C02",
      FamilyR65C02.Implies#[FeatureW65C02]>;

def FamilyW65816
    : Family<"mosw65816",
      "WDC 65816",
      FamilyW65C02.Implies#[FeatureW65816]>;

def Family65EL02
    : Family<"mos65el02",
      "65EL02 virtual machine",
      FamilyW65C02.Implies#[Feature65EL02]>;

def Family65CE02
    : Family<"mos65ce02",
      "Commodore 65CE02",
      FamilyW65C02.Implies#[Feature65CE02]>;

def FamilySWEET16
    : Family<"mossweet16",
      "MOS 6502 compatible with SWEET16 virtual machine support",
      [FeatureSWEET16]>;

//===---------------------------------------------------------------------===//
// MOS Families
//===---------------------------------------------------------------------===//

def MOSSchedModel : SchedMachineModel {
  // Prevent instructions from being added to the pending queue, since latency
  // doesn't matter. The remaining latency concerns are disabled in the C++
  // scheduling strategy.
  let MicroOpBufferSize = 1;
  let CompleteModel = false;
}

class Device<string Name, Family Fam,
             list<SubtargetFeature> ExtraFeatures = []>
  : ProcessorModel<Name, MOSSchedModel, !listconcat([Fam],ExtraFeatures)>;

def : Device<"mos6502",  Family6502>;
def : Device<"mos6502x", Family6502X>;
def : Device<"mos65c02", Family65C02>;
def : Device<"mos65ce02", Family65CE02>;
def : Device<"mosr65c02", FamilyR65C02>;
def : Device<"mosw65c02", FamilyW65C02>;
def : Device<"mosw65816", FamilyW65816>;
def : Device<"mosw65el02", Family65EL02>;
def : Device<"mossweet16", FamilySWEET16>;
