#!/bin/python
# Convert a box library from xar_version 3 to new format version 4

import os
import sys
import xar_converter


def list_files(rootPath):
    """ return a list of all files found from rootPath
        the resulting file pathes are relative to rootPath
    """
    result = []
    for root, dirs, files in os.walk(rootPath):
        for filename in files:
            fullpath = os.path.join(root, filename)
            result.append(os.path.relpath(fullpath, rootPath))
    return result


def main():
    """ Entry point of the box library converter
    """
    param = []
    if len(sys.argv) not in range(2, 4):
        sys.stderr.write("Incorrect number of arguments" + os.linesep)
        sys.exit(1)
    param.append(sys.argv[1])
    if (len(sys.argv) == 3):
        param.append(sys.argv[2])
    else:
        param.append("objects")

    abspath = os.path.abspath(param[0])
    dest_dir = os.path.abspath(param[1])

    files = list_files(abspath)
    for file in files:
        filepath, filename = os.path.split(file)
        if(filename in ["box.xar", "behavior.xar"]):
            print("converting " + str(file) + ":\n")
            try:
                fullinputpath = os.path.join(abspath, file)
                fulloutputpath = os.path.join(dest_dir, filepath)

                folders = filepath.split(os.sep)
                xar_converter.convert_boxlibrary(fullinputpath,
                                                 fulloutputpath,
                                                 folders[len(folders) - 1])
            except BaseException as error:
                print("Could not convert: " + str(error))


if __name__ == "__main__":
    main()
