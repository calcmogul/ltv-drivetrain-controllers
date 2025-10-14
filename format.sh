#!/bin/bash

find src -type f \( -name '*.cpp' -o -name '*.h' \) -exec clang-format -i {} \;
python -m gersemi -i CMakeLists.txt
