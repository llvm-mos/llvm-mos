//===-- MOS.td - Describe the MOS Target Machine ----------*- tablegen -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===---------------------------------------------------------------------===//
// This is the top level entry point for the MOS target.
//===---------------------------------------------------------------------===//

//===---------------------------------------------------------------------===//
// Target-independent interfaces which we are implementing
//===---------------------------------------------------------------------===//

include "llvm/Target/Target.td"
include "llvm/Target/GlobalISel/Combine.td"

//===---------------------------------------------------------------------===//
// MOS Device Definitions
//===---------------------------------------------------------------------===//

include "MOSFeatures.td"
include "MOSDevices.td"
include "MOSRegisterInfo.td"
include "MOSRegisterBanks.td"
include "MOSInstrInfo.td"
include "MOSCombine.td"

def MOSInstrInfo : InstrInfo {
  let guessInstructionProperties = 0;
}

include "MOSCallingConv.td"

def MOSAsmWriter : AsmWriter {
 string AsmWriterClassName = "InstPrinter";
 bit isMCAsmWriter = 1;
}

def MOSAsmParser : AsmParser {
  let ShouldEmitMatchRegisterAltName = 1;
  let ReportMultipleNearMisses = 0;
  string RegisterPrefix = "";
  string TokenizingCharacters = "#(),[]*!";
}

def MOSAsmParserVariantDefault : AsmParserVariant {
  int Variant = 0;
  let Name = "generic";
}

def MOSAsmParserVariantCA65 : AsmParserVariant {
  int Variant = 1;
  let Name = "ca65";
}

def MOSAsmParserVariantXA65 : AsmParserVariant {
  int Variant = 2;
  let Name = "xa65";
}

//===---------------------------------------------------------------------===//
// Target Declaration
//===---------------------------------------------------------------------===//

def MOS : Target {
  let InstructionSet         = MOSInstrInfo;
  let AssemblyWriters        = [MOSAsmWriter];
  let AllowRegisterRenaming = 1;

  let AssemblyParsers        = [MOSAsmParser];
  let AssemblyParserVariants = [MOSAsmParserVariantDefault,
                                MOSAsmParserVariantCA65,
                                MOSAsmParserVariantXA65];
}
