//===---------------------------------------------------------------------===//
// MOS Features
//===---------------------------------------------------------------------===//

def FeatureAltRegisterNamesOnly
    : SubtargetFeature<"mos-long-register-names", "m_LongRegisterNames", 
                        "true",
                        "Requires llvm_mos_* prefixes to all registers. "#
                        "Useful if your code has variable names that conflict "#
                        "with llvm-mos register names">;

                    
