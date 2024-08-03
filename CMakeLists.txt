# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
include_directories($ENV{IDF_PATH}/tools/riscv32-esp-elf/esp-13.2.0_20230928/riscv32-esp-elf/riscv32-esp-elf/include/c++/13.2.0)
project(i2c-simple)
