#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2024 Tom Noble.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Tom Noble

import sys
import logging
from typing import List, Tuple
from pathlib import Path


class NoIncludeGuard(Exception):
    ERROR = "No include guard found in {}.hpp. Unable to generate pretext."

    def __init__(self, file: Path):
        super().__init__(self.ERROR.format(file))


class NoIncludeDirectory(Exception):
    ERROR = "No include directory found for {}.hpp. Unable to generate relative .hpp include"

    def __init__(self, file: Path):
        super().__init__(self.ERROR.format(file))


class HppFile:
    def __init__(self, path: Path):
        self.path = path
        self.guard = "#pragma once"
        self.pretext = self.pretext()
        self.include = self.include()

    def drop_data_after(self, data: str, match: str):
        return data[: data.find(match) + len(match)]

    def read(self) -> str:
        data = open(self.path, "r").read()
        contains_guard = self.guard in data
        if not contains_guard:
            raise NoIncludeGuard(self.path)
        return data

    def pretext(self) -> str:
        data = self.read()
        return self.drop_data_after(data, self.guard)

    def include(self) -> str:
        ends_with_include = lambda p: str(p).endswith("include")
        include_paths = [p for p in self.path.parents if ends_with_include(p)]
        if not include_paths:
            raise NoIncludeDirectory(self.path)
        relative_import = self.path.relative_to(include_paths[0])
        return f"#include <{relative_import}>"


class DeprecatedHeader:
    def __init__(self, hpp: HppFile):
        self.hpp = hpp
        self.path = hpp.path.with_suffix(".h")
        self.prefix = f"/* This file was autogenerated by {Path(__file__).name} */"
        self.warn = '#pragma message(".h header is obsolete. Please use the .hpp header instead.")'
        self.contents = self.contents()

    def contents(self) -> str:
        items = [self.prefix, self.hpp.pretext, self.warn, self.hpp.include]
        return "\n\n".join(items)


def parse_args(args: List) -> bool:
    n_args = len(sys.argv)
    if n_args > 2:
        sys.exit("Usage: ./create_deprecated_headers [--apply]")
    apply = "--apply" == sys.argv[1] if n_args == 2 else False
    return apply


if __name__ == "__main__":
    apply = parse_args(sys.argv)
    hpp_paths = list(Path.cwd().rglob("*.hpp"))
    print(f"Generating deprecated .h files for {len(hpp_paths)} .hpp files...")
    processed = []
    bad = []
    for hpp_path in hpp_paths:
        try:
            processed.append(HppFile(hpp_path))
        except NoIncludeDirectory as e:
            bad.append(str(hpp_path))
        except NoIncludeGuard as e:
            bad.append(str(hpp_path))
    print(f"Summary: Can generate {len(processed)} .h files")
    if bad:
        print(f"Cannot generate .h files for the following files:")
        print("\n".join(bad))
    if apply and bad:
        answer = input("Proceed to generate? (y/n): ")
        apply = answer.lower() == "y"
    if apply:
        print(f"Proceeding to generate {len(processed)} .h files...")
        to_generate = [DeprecatedHeader(hpp) for hpp in processed]
        _ = [open(h.path, "w").write(h.contents) for h in to_generate]
        print("Done. (You may need to rerun formatting)")
