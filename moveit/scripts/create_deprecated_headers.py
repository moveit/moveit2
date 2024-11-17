#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2021 PickNik Inc.
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
#    * Neither the name of the PickNik Inc. nor the names of its
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

import sys
from pathlib import Path

# TODO: This doesn't work for chomp and pilz
MOVEIT_HPPS = "**/moveit*/**/*.hpp"
USAGE = "Usage: ./generate_deprecated_headers <install_directory>"
DEPRECATION = "{0}.h imports are deprecated. Consider using {0}.hpp"


def create_c_header(hpp_path: str) -> None:
    header_directory = hpp_path.parent
    file = hpp_path.name.replace(".hpp", "")
    with open(header_directory / f"{file}.h", "w") as h_file:
        h_file.write(f"#warning {DEPRECATION.format(file)}\n")
        h_file.write(f"#include <{file}.hpp>\n")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(USAGE)
    install_directory = sys.argv[1]
    for hpp_path in Path(install_directory).rglob(MOVEIT_HPPS):
        create_c_header(hpp_path)
