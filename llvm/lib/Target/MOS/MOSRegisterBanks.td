//===- MOSRegisterBanks.td - MOS Register Bank Description -*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file describes the GlobalISel Register Banks for MOS.
//
//===----------------------------------------------------------------------===//

// Register banks are unused, so place everything in one big bank.
def AnyRegBank : RegisterBank<"Any", [Anyi1, Anyi8, Imag16, Fakei32, Fakei64]>;
