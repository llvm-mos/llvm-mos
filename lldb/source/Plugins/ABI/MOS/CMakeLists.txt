add_lldb_library(lldbPluginArchitectureMOS PLUGIN
  ABISysV_mos.cpp
  MOSImaginaryRegisters.cpp
  MOSRegisterContext.cpp

  LINK_LIBS
    lldbCore
    lldbSymbol
    lldbTarget
    lldbValueObject
    lldbUtility
  LINK_COMPONENTS
    Support
    TargetParser
  )
