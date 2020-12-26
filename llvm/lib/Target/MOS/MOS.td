//===-- MOS.td - Describe the MOS Target Machine ----------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===---------------------------------------------------------------------===//
// This is the top level entry point for the MOS target.
//===---------------------------------------------------------------------===//

//===---------------------------------------------------------------------===//
// Target-independent interfaces which we are implementing
//===---------------------------------------------------------------------===//

include "llvm/Target/Target.td"

//===---------------------------------------------------------------------===//
// MOS Device Definitions
//===---------------------------------------------------------------------===//

include "MOSDevices.td"
include "MOSFeatures.td"
include "MOSRegisterInfo.td"
include "MOSInstrInfo.td"

def MOSInstrInfo : InstrInfo;

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

  let AssemblyParsers        = [MOSAsmParser];
  let AssemblyParserVariants = [MOSAsmParserVariantDefault, 
                                MOSAsmParserVariantCA65, 
                                MOSAsmParserVariantXA65];
}

