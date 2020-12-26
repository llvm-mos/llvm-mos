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

//===---------------------------------------------------------------------===//
// Register File Description
//===---------------------------------------------------------------------===//

include "MOSRegisterInfo.td"

//===---------------------------------------------------------------------===//
// Instruction Descriptions
//===---------------------------------------------------------------------===//

include "MOSInstrInfo.td"

def MOSInstrInfo : InstrInfo;

//===---------------------------------------------------------------------===//
// Calling Conventions
//===---------------------------------------------------------------------===//

include "MOSCallingConv.td"

//===---------------------------------------------------------------------===//
// Assembly Printers
//===---------------------------------------------------------------------===//

def MOSAsmWriter : AsmWriter {
 string AsmWriterClassName = "InstPrinter";
 bit isMCAsmWriter = 1;
}

//===---------------------------------------------------------------------===//
// Assembly Parsers
//===---------------------------------------------------------------------===//

def MOSAsmParser : AsmParser {
  let ShouldEmitMatchRegisterAltName = 1;
  let ReportMultipleNearMisses = 0;
  string RegisterPrefix = "";
  string TokenizingCharacters = "(),[]*!";
}

def MOSAsmParserVariant : AsmParserVariant {
  int Variant = 0;
  // Recognize hard coded registers.
}

def MOSAsmParserVariantCA65 : AsmParserVariant {
  int Variant = 1;
  let Name = "ca65";
  string RegisterPrefix = "";
}


//===---------------------------------------------------------------------===//
// Target Declaration
//===---------------------------------------------------------------------===//

def MOS : Target {
  let InstructionSet         = MOSInstrInfo;
  let AssemblyWriters        = [MOSAsmWriter];

  let AssemblyParsers        = [MOSAsmParser];
  let AssemblyParserVariants = [MOSAsmParserVariant, MOSAsmParserVariantCA65];
}

