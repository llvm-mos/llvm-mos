#!/usr/bin/env python3

from sys import argv, stderr
from pathlib import Path

assert(argv[1] == "--dump-all")

path = Path(argv[2])
with open(path.with_suffix(".od65"), "r") as f:
  print(f.read())
