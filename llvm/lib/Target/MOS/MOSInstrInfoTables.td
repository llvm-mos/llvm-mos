//===-- MOSInstrInfoTables.td - MOS Instruction tables -----*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file describes MOS constant tables in TableGen format.
//
//===----------------------------------------------------------------------===//

include "llvm/TableGen/SearchableTable.td"

/// ---------------------------------------------------------------------------
/// A MOS zero page instruction relaxation entry.  Used to relax 8 bit
/// instructions that access zero page, to 16 bit equivalents, if the 
/// target addresses do not necessarily reside in 8 bit memory.
class ZeroPageInstructionRelaxationEntry< InstAddressMode from, 
                                          InstAddressMode to>
{
	InstAddressMode From = from;
	InstAddressMode To = to;
}

class ZPIRE< InstAddressMode from, InstAddressMode to> :
	ZeroPageInstructionRelaxationEntry< from, to >;

def : ZPIRE< ADC_ZeroPage, ADC_Absolute >;
def : ZPIRE< ADC_ZeroPageX, ADC_AbsoluteX >;

def : ZPIRE< AND_ZeroPage, AND_Absolute >;
def : ZPIRE< AND_ZeroPageX, AND_AbsoluteX >;

def : ZPIRE< ASL_ZeroPage, ASL_Absolute >;
def : ZPIRE< ASL_ZeroPageX, ASL_AbsoluteX >;

def : ZPIRE< BIT_ZeroPage, BIT_Absolute >;

def : ZPIRE< CMP_ZeroPage, CMP_Absolute >;
def : ZPIRE< CMP_ZeroPageX, CMP_AbsoluteX >;

def : ZPIRE< CPX_ZeroPage, CPX_Absolute >;

def : ZPIRE< CPY_ZeroPage, CPY_Absolute >;

def : ZPIRE< DEC_ZeroPage, DEC_Absolute >;
def : ZPIRE< DEC_ZeroPageX, DEC_AbsoluteX >;

def : ZPIRE< EOR_ZeroPage, EOR_Absolute >;
def : ZPIRE< EOR_ZeroPageX, EOR_AbsoluteX >;

def : ZPIRE< INC_ZeroPage, INC_Absolute >;
def : ZPIRE< INC_ZeroPageX, INC_AbsoluteX >;

def : ZPIRE< LDA_ZeroPage, LDA_Absolute >;
def : ZPIRE< LDA_ZeroPageX, LDA_AbsoluteX >;

def : ZPIRE< LDX_ZeroPage, LDX_Absolute >;
def : ZPIRE< LDX_ZeroPageY, LDX_AbsoluteY >;

def : ZPIRE< LDY_ZeroPage, LDY_Absolute >;
def : ZPIRE< LDY_ZeroPageX, LDY_AbsoluteX >;

def : ZPIRE< LSR_ZeroPage, LSR_Absolute >;
def : ZPIRE< LSR_ZeroPageX, LSR_AbsoluteX >;

def : ZPIRE< ORA_ZeroPage, ORA_Absolute >;
def : ZPIRE< ORA_ZeroPageX, ORA_AbsoluteX >;

def : ZPIRE< ROL_ZeroPage, ROL_Absolute >;
def : ZPIRE< ROL_ZeroPageX, ROL_AbsoluteX >;

def : ZPIRE< ROR_ZeroPage, ROR_Absolute >;
def : ZPIRE< ROR_ZeroPageX, ROR_AbsoluteX >;

def : ZPIRE< SBC_ZeroPage, SBC_Absolute >;
def : ZPIRE< SBC_ZeroPageX, SBC_AbsoluteX >;

def : ZPIRE< STA_ZeroPage, STA_Absolute >;
def : ZPIRE< STA_ZeroPageX, STA_AbsoluteX >;

def : ZPIRE< STX_ZeroPage, STX_Absolute >;

def : ZPIRE< STY_ZeroPage, STY_Absolute >;

/// ---------------------------------------------------------------------------
/// The MOS zero page instruction lowering table.
def MOSZeroPageInstructionRelaxationTable : GenericTable {
  let FilterClass = "ZeroPageInstructionRelaxationEntry";
  let Fields = ["From", "To"];

  let PrimaryKey = ["From"];
  let PrimaryKeyName = "getZeroPageInstructionRelaxationEntry";
}

def MOSZeroPageInstructionLoweringIndex : SearchIndex {
  let Table = MOSZeroPageInstructionRelaxationTable;
  let Key = ["From"];
}

/// Zero page section entry.  If a symbol exists in one of these sections,
/// it belongs in zero page.
class MOSZeroPageSectionEntry< string name >
{
	string Name = name;
}

class ZPSE< string name > : MOSZeroPageSectionEntry< name >;

def : ZPSE<".zp">;
def : ZPSE<".zeropage">;
def : ZPSE<".directpage">;

def MOSZeroPageSectionTable : GenericTable {
	let FilterClass = "MOSZeroPageSectionEntry";
	let Fields = ["Name"];
}
