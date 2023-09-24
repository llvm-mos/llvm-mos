//===-- MOSInstrInfoTables.td - MOS Instruction tables -----*- tablegen -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
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
/// target addresses do not necessarily reside in 8 bit memory. The same
/// pattern is used to further relax 16 bit zero bank instructions into
/// 24 bit equivalents for 65816.
class MOSRelaxationEntry< InstAddressMode from, 
                          InstAddressMode to> {
  InstAddressMode From = from;
  InstAddressMode To = to;
}

class ZeroPageInstructionRelaxationEntry< InstAddressMode from, 
                                          InstAddressMode to> :
                                          MOSRelaxationEntry<from, to>;

class ZeroBankInstructionRelaxationEntry< InstAddressMode from, 
                                          InstAddressMode to> :
                                          MOSRelaxationEntry<from, to>;


class ZPIRE< InstAddressMode from, InstAddressMode to> :
	ZeroPageInstructionRelaxationEntry< from, to >;

class ZBIRE< InstAddressMode from, InstAddressMode to> :
	ZeroBankInstructionRelaxationEntry< from, to >;

def : ZPIRE< ADC_ZeroPage, ADC_Absolute >;
def : ZBIRE< ADC_Absolute, ADC_AbsoluteLong >;
def : ZPIRE< ADC_ZeroPageX, ADC_AbsoluteX >;
def : ZBIRE< ADC_AbsoluteX, ADC_AbsoluteXLong >;

def : ZPIRE< AND_ZeroPage, AND_Absolute >;
def : ZBIRE< AND_Absolute, AND_AbsoluteLong >;
def : ZPIRE< AND_ZeroPageX, AND_AbsoluteX >;
def : ZBIRE< AND_AbsoluteX, AND_AbsoluteXLong >;

def : ZPIRE< ASL_ZeroPage, ASL_Absolute >;
def : ZPIRE< ASL_ZeroPageX, ASL_AbsoluteX >;

def : ZPIRE< BIT_ZeroPage, BIT_Absolute >;
def : ZPIRE< BIT_ZeroPageX, BIT_AbsoluteX >;

def : ZPIRE< CMP_ZeroPage, CMP_Absolute >;
def : ZBIRE< CMP_Absolute, CMP_AbsoluteLong >;
def : ZPIRE< CMP_ZeroPageX, CMP_AbsoluteX >;
def : ZBIRE< CMP_AbsoluteX, CMP_AbsoluteXLong >;

def : ZPIRE< CPX_ZeroPage, CPX_Absolute >;

def : ZPIRE< CPY_ZeroPage, CPY_Absolute >;

def : ZPIRE< CPZ_ZeroPage, CPZ_Absolute >;

def : ZPIRE< DCP_ZeroPage, DCP_Absolute >;
def : ZPIRE< DCP_ZeroPageX, DCP_AbsoluteX >;

def : ZPIRE< DEC_ZeroPage, DEC_Absolute >;
def : ZPIRE< DEC_ZeroPageX, DEC_AbsoluteX >;

def : ZPIRE< EOR_ZeroPage, EOR_Absolute >;
def : ZBIRE< EOR_Absolute, EOR_AbsoluteLong >;
def : ZPIRE< EOR_ZeroPageX, EOR_AbsoluteX >;
def : ZBIRE< EOR_AbsoluteX, EOR_AbsoluteXLong >;

def : ZPIRE< INC_ZeroPage, INC_Absolute >;
def : ZPIRE< INC_ZeroPageX, INC_AbsoluteX >;

def : ZPIRE< ISC_ZeroPage, ISC_Absolute >;
def : ZPIRE< ISC_ZeroPageX, ISC_AbsoluteX >;

def : ZPIRE< LDA_ZeroPage, LDA_Absolute >;
def : ZBIRE< LDA_Absolute, LDA_AbsoluteLong >;
def : ZPIRE< LDA_ZeroPageX, LDA_AbsoluteX >;
def : ZBIRE< LDA_AbsoluteX, LDA_AbsoluteXLong >;

def : ZPIRE< LDX_ZeroPage, LDX_Absolute >;
def : ZPIRE< LDX_ZeroPageY, LDX_AbsoluteY >;

def : ZPIRE< LDY_ZeroPage, LDY_Absolute >;
def : ZPIRE< LDY_ZeroPageX, LDY_AbsoluteX >;

def : ZPIRE< LSR_ZeroPage, LSR_Absolute >;
def : ZPIRE< LSR_ZeroPageX, LSR_AbsoluteX >;

def : ZPIRE< ORA_ZeroPage, ORA_Absolute >;
def : ZBIRE< ORA_Absolute, ORA_AbsoluteLong >;
def : ZPIRE< ORA_ZeroPageX, ORA_AbsoluteX >;
def : ZBIRE< ORA_AbsoluteX, ORA_AbsoluteXLong >;

def : ZPIRE< RLA_ZeroPage, RLA_Absolute >;
def : ZPIRE< RLA_ZeroPageX, RLA_AbsoluteX >;

def : ZPIRE< ROL_ZeroPage, ROL_Absolute >;
def : ZPIRE< ROL_ZeroPageX, ROL_AbsoluteX >;

def : ZPIRE< ROR_ZeroPage, ROR_Absolute >;
def : ZPIRE< ROR_ZeroPageX, ROR_AbsoluteX >;

def : ZPIRE< RRA_ZeroPage, RRA_Absolute >;
def : ZPIRE< RRA_ZeroPageX, RRA_AbsoluteX >;

def : ZPIRE< SBC_ZeroPage, SBC_Absolute >;
def : ZBIRE< SBC_Absolute, SBC_AbsoluteLong >;
def : ZPIRE< SBC_ZeroPageX, SBC_AbsoluteX >;
def : ZBIRE< SBC_AbsoluteX, SBC_AbsoluteXLong >;

def : ZPIRE< SLO_ZeroPage, SLO_Absolute >;
def : ZPIRE< SLO_ZeroPageX, SLO_AbsoluteX >;

def : ZPIRE< SRE_ZeroPage, SRE_Absolute >;
def : ZPIRE< SRE_ZeroPageX, SRE_AbsoluteX >;

def : ZPIRE< STA_ZeroPage, STA_Absolute >;
def : ZBIRE< STA_Absolute, STA_AbsoluteLong >;
def : ZPIRE< STA_ZeroPageX, STA_AbsoluteX >;
def : ZBIRE< STA_AbsoluteX, STA_AbsoluteXLong >;

def : ZPIRE< STX_ZeroPage, STX_Absolute >;
def : ZPIRE< STX_ZeroPageY, STX_AbsoluteY >;

def : ZPIRE< STY_ZeroPage, STY_Absolute >;
def : ZPIRE< STY_ZeroPageX, STY_AbsoluteX >;

def : ZPIRE< STZ_ZeroPage, STZ_Absolute >;
def : ZPIRE< STZ_ZeroPageX, STZ_AbsoluteX >;

def : ZPIRE< TRB_ZeroPage, TRB_Absolute >;

def : ZPIRE< TSB_ZeroPage, TSB_Absolute >;

def : ZPIRE< TST_ZeroPage, TST_Absolute >;
def : ZPIRE< TST_ZeroPageX, TST_AbsoluteX >;

def : ZPIRE< MUL_ZeroPage_EL02, MUL_Absolute_EL02 >;
def : ZPIRE< MUL_ZeroPageX_EL02, MUL_AbsoluteX_EL02 >;
def : ZPIRE< DIV_ZeroPage_EL02, DIV_Absolute_EL02 >;
def : ZPIRE< DIV_ZeroPageX_EL02, DIV_AbsoluteX_EL02 >;

/// ---------------------------------------------------------------------------
/// The MOS zero page instruction lowering table.
class MOSRelaxationTable : GenericTable {
  let FilterClass = NAME # "Entry";
  let CppTypeName = "InstructionRelaxationEntry";
  let Fields = ["From", "To"];

  let PrimaryKey = ["From"];
  let PrimaryKeyName = "get" # NAME # "Entry";
}

def ZeroPageInstructionRelaxation : MOSRelaxationTable;
def ZeroBankInstructionRelaxation : MOSRelaxationTable;
