#!/usr/bin/env python3

from pathlib import Path
from shutil import copyfile
from sys import argv

assert(argv[1] == "-C")
cfgPath = Path(argv[2])

assert(argv[3] == "-o")
outPath = Path(argv[4])

assert(argv[6] == "-m")
mapPath = Path(argv[7])

argPath = Path(argv[8])

copyfile(cfgPath, argPath.with_name("ld65.cfg"))
copyfile(argPath.with_name("ld65.map"), mapPath)
with open(argPath.with_name("ld65.hex"), "r") as h, open(outPath, "wb") as o:
    o.write(bytes.fromhex(h.read()))
