#!/usr/bin/env python3

import sys
import os

if len(sys.argv) < 2:
    print("USAGE: log_to_vector_generator.py <log_file>")
    exit(1)

log_name = os.path.splitext(sys.argv[1])[0]
cpp_file = open(log_name + '.h', 'w')
with open(sys.argv[1], mode='rb') as log_file:
    cpp_file.write(R'''#pragma once
#include <cstdint>
#include <vector>

const std::vector<uint8_t> {}{{
'''.format(log_name))
    pos = 0
    while True:
        byte = log_file.read(1)
        if not byte:
            break
        cpp_file.write("0x{},".format(byte.hex()))
        pos += 1
        if pos > 10:
            cpp_file.write('\n')
            pos = 0
        else:
            cpp_file.write(' ')
    cpp_file.write('};\n')
    cpp_file.close()
