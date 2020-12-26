//===---------------------------------------------------------------------===//
// MOS Device Definitions
//===---------------------------------------------------------------------===//

def Feature6502
    : SubtargetFeature<"mos-insns-6502", "m_has6502Insns", "true",
                       "The original documented 6502 instruction set">;

def Feature6502X
    : SubtargetFeature<"mos-insns-6502x", "m_has6502XInsns", "false",
                       "The 'illegal' opcodes present on some early variants of "
                       "the original 6502 processor">;

def Feature65SC02
    : SubtargetFeature<"mos-insns-65sc02", "m_has65SC02Insns", "false",
                       "The new and modified instructions present on the 65sc02 "
                       "and variants">;

def Feature65C02
    : SubtargetFeature<"mos-insns-65c02", "m_has65C02Insns", "false",
                       "The new and modified instructions present on the 65c02 "
                       "and variants">;

def FeatureSWEET16
    : SubtargetFeature<"mos-insns-sweet16", "m_hasSweet16Insns", "false",
                        "The SWEET16 instruction set">;

// A feature set aggregates features, grouping them. We don't want to create a
// new member in AVRSubtarget (to store a value) for each set because we do not
// care if the set is supported, only the subfeatures inside the set. We fix
// this by simply setting the same dummy member for all feature sets, which is
// then ignored.
class FeatureSet<string name, string desc, list<SubtargetFeature> i>
  : SubtargetFeature<name, "m_FeatureSetDummy", "true", desc, i>;

// A family of microcontrollers, defining a set of supported features.
class Family<string name, string desc, list<SubtargetFeature> i>
  : FeatureSet<name, desc, i>;


def Family6502 
    : Family<"mos-6502", "The device is a member of the original MOS 6502 "
             "family", [Feature6502]>;

def Family6502X
    : Family<"mos-6502x", 
      "The device is a member of the original MOS 6502 family, with illegal "
      "opcode support", [Feature6502, Feature6502X]>;

def Family65SC02
    : Family<"mos-65sc02",  
      "The device is a member of the MOS 65SC02 family",
      [Feature6502, Feature65SC02]>;

def Family65C02
    : Family<"mos-65c02",  
      "The device is a member of the MOS 65C02 family",
      [Feature6502, Feature65SC02, Feature65C02]>;

def FamilySWEET16
    : Family<"mos-sweet16",
      "The device is a member of the MOS 6502 family, with SWEET16 virtual "
      "machine support", [Feature6502, FeatureSWEET16]>;

class ELFArch<string name>  : SubtargetFeature<"", "ELFArch",
                                    !strconcat("ELF::",name), "">;
// ELF e_flags architecture values
def ELFArchMOS6502 : ELFArch<"EF_MOS_ARCH_6502">;
def ELFArchMOS6502X : ELFArch<"EF_MOS_ARCH_6502X">;
def ELFArchMOS65SC02 : ELFArch<"EF_MOS_ARCH_65SC02">;
def ELFArchMOS65C02 : ELFArch<"EF_MOS_ARCH_65C02">;
def ELFArchMOSSweet16 : ELFArch<"EF_MOS_ARCH_SWEET16">;

//===---------------------------------------------------------------------===//
// MOS Families
//===---------------------------------------------------------------------===//

class Device<string Name, Family Fam, ELFArch Arch,
             list<SubtargetFeature> ExtraFeatures = []>
  : Processor<Name, NoItineraries, !listconcat([Fam,Arch],ExtraFeatures)>;

def : Device<"mos-6502",  Family6502, ELFArchMOS6502>;
def : Device<"mos-6502x", Family6502X, ELFArchMOS6502X>;
def : Device<"mos-65c02", Family65C02, ELFArchMOS65C02>;
def : Device<"mos-65sc02", Family65SC02, ELFArchMOS65SC02>;
def : Device<"mos-sweet16", FamilySWEET16, ELFArchMOSSweet16>;

