//===---------------------------------------------------------------------===//
// MOS Device Definitions
//===---------------------------------------------------------------------===//

def Feature6502
    : SubtargetFeature<"mos-insns-6502", "m_Has6502Insns", "false",
                       "The original documented 6502 instruction set">;

def Feature6502BCD
    : SubtargetFeature<"mos-insns-6502-bcd", "m_Has6502BCDInsns", "false",
                       "BCD instruction support, including SED and CLD (most "
                       "6502 series CPUs support this)">;

def Feature6502X
    : SubtargetFeature<"mos-insns-6502x", "m_Has6502XInsns", "false",
                       "The 'illegal' opcodes present on some early variants of "
                       "the original 6502 processor">;

def Feature65C02
    : SubtargetFeature<"mos-insns-65c02", "m_Has65C02Insns", "false",
                       "The new and modified instructions present on the generic "
                       "65c02 and variants">;

def FeatureR65C02
    : SubtargetFeature<"mos-insns-r65c02", "m_HasR65C02Insns", "false",
                       "The new and modified instructions present on Rockwell "
                       "and WDC 65c02">;

def FeatureW65C02
    : SubtargetFeature<"mos-insns-w65c02", "m_HasW65C02Insns", "false",
                       "The new and modified instructions present on "
                       "WDC 65c02">;

def FeatureW65816
    : SubtargetFeature<"mos-insns-w65816", "m_HasW65816Insns", "false",
                       "The new and modified instructions present on "
                       "WDC 65816">;

def Feature65EL02
    : SubtargetFeature<"mos-insns-65el02", "m_Has65EL02Insns", "false",
                       "The new and modified instructions present on "
                       "65EL02">;

def Feature65CE02
    : SubtargetFeature<"mos-insns-65ce02", "m_Has65CE02Insns", "false",
                       "The new and modified instructions present on "
                       "WDC 65CE02">;

def FeatureSWEET16
    : SubtargetFeature<"mos-insns-sweet16", "m_HasSWEET16Insns", "false",
                        "The SWEET16 instruction set">;

// A feature set aggregates features, grouping them. We don't want to create a
// new member in MOSSubtarget (to store a value) for each set because we do not
// care if the set is supported, only the subfeatures inside the set. We fix
// this by simply setting the same dummy member for all feature sets, which is
// then ignored.
class FeatureSet<string name, string desc, list<SubtargetFeature> i>
  : SubtargetFeature<name, "m_FeatureSetDummy", "true", desc, i>;

// A family of microcontrollers, defining a set of supported features.
class Family<string name, string desc, list<SubtargetFeature> i>
  : FeatureSet<name, desc, i>;


def Family6502 
    : Family<
      "mos6502",
      "Generic MOS 6502, with support for BCD instructions",
      [Feature6502, Feature6502BCD]>;

def Family6502X
    : Family<"mos6502x", 
      "NMOS 6502, with illegal opcode support", 
      [Feature6502, Feature6502BCD, Feature6502X]>;

def Family65C02
    : Family<"mos65c02",  
      "Generic MOS 65C02",
      [Feature6502, Feature6502BCD, Feature65C02]>;

def FamilyR65C02
    : Family<"mosr65c02",  
      "Rockwell 65C02",
      [Feature6502, Feature65C02, FeatureR65C02]>;

def FamilyW65C02
    : Family<"mosw65c02",  
      "WDC 65C02",
      [Feature6502, Feature65C02, FeatureR65C02, FeatureW65C02]>;

def FamilyW65816
    : Family<"mosw65816",  
      "WDC 65816",
      [Feature6502, Feature65C02, FeatureR65C02, FeatureW65C02,
      FeatureW65816]>;

def Family65EL02
    : Family<"mos65el02",  
      "65EL02 virtual machine",
      [Feature6502, Feature65C02, FeatureR65C02, FeatureW65C02,
      Feature65EL02]>;

def Family65CE02
    : Family<"mos65ce02",  
      "Commodore 65CE02",
      [Feature6502, Feature65C02, FeatureR65C02, FeatureW65C02,
      Feature65CE02]>;

def FamilySWEET16
    : Family<"mossweet16",
      "MOS 6502 compatible with SWEET16 virtual machine support", 
      [FeatureSWEET16]>;

class ELFArch<string name>  : SubtargetFeature<"", "ELFArch",
                                    !strconcat("ELF::",name), "">;
// ELF e_flags architecture values
def ELFArchMOS6502 : ELFArch<"EM_MOS_ARCH_6502">;
def ELFArchMOS6502BCD : ELFArch<"EM_MOS_ARCH_6502_BCD">;
def ELFArchMOS6502X : ELFArch<"EM_MOS_ARCH_6502X">;
def ELFArchMOS65C02 : ELFArch<"EM_MOS_ARCH_65C02">;
def ELFArchMOSR65C02 : ELFArch<"EM_MOS_ARCH_R65C02">;
def ELFArchMOSW65C02 : ELFArch<"EM_MOS_ARCH_W65C02">;
def ELFArchMOSW65816 : ELFArch<"EM_MOS_ARCH_W65816">;
def ELFArchMOS65EL02 : ELFArch<"EM_MOS_ARCH_65EL02">;
def ELFArchMOS65CE02 : ELFArch<"EM_MOS_ARCH_65CE02">;
def ELFArchMOSSWEET16 : ELFArch<"EM_MOS_ARCH_SWEET16">;

//===---------------------------------------------------------------------===//
// MOS Families
//===---------------------------------------------------------------------===//

class Device<string Name, Family Fam, ELFArch Arch,
             list<SubtargetFeature> ExtraFeatures = []>
  : Processor<Name, NoItineraries, !listconcat([Fam,Arch],ExtraFeatures)>;

def : Device<"mos6502",  Family6502, ELFArchMOS6502>;
def : Device<"mos6502x", Family6502X, ELFArchMOS6502X>;
def : Device<"mos65c02", Family65C02, ELFArchMOS65C02>;
def : Device<"mosr65c02", FamilyR65C02, ELFArchMOSR65C02>;
def : Device<"mosw65c02", FamilyR65C02, ELFArchMOSW65C02>;
def : Device<"mosw65816", FamilyW65816, ELFArchMOSW65816>;
def : Device<"mosw65el02", Family65EL02, ELFArchMOS65EL02>;
def : Device<"mosw65ce02", Family65CE02, ELFArchMOS65CE02>;
def : Device<"mossweet16", FamilySWEET16, ELFArchMOSSWEET16>;

