#!/usr/bin/env python
# -*- coding:utf-8 -*-

import os
import os.path
import subprocess


def file_name(file_dir):
    target_files = []
    for dirpath, dirnames, filenames in os.walk(file_dir):
        for file in filenames:
            # print(file)
            suffix = os.path.splitext(file)[1]
            # print(suffix)
            if '.cc' == suffix or '.cpp' == suffix or '.h' == suffix or '.hpp' == suffix:
                target_files.append(os.path.join(dirpath, file))
    return target_files


target_dir = ['./demo', './src', './include']
files = []

for dir in target_dir:
    files += file_name(dir)

for file in files:
    if "mpark/variant.hpp" in file:
        continue
    cmd = ["clang-format", "-i", file]
    print(cmd)
    print(subprocess.check_call(cmd))
