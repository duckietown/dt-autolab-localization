#!/usr/bin/env python

if __name__ == '__main__':
    """
    Fixes bug in Boost.NumPy library
    """

    FILE_NAME = 'Boost.NumPy/CMakeLists.txt'
    with open(FILE_NAME, 'r') as f:
        lines = f.readlines()
    with open(FILE_NAME, 'w+') as f:
        for line in lines:
            if line.strip() != 'find_package(Boost COMPONENTS python3 REQUIRED)':
                f.write(line)
            else:
                f.write('find_package(Boost COMPONENTS python REQUIRED)\n')
