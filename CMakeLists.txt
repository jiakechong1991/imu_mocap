# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

#if("${IDF_TARGET}" STREQUAL "linux")
set(EXTRA_COMPONENT_DIRS "D:/dev/dev_tools/Espressif/frameworks/esp-idf-v5.2.2/examples/common_components/protocol_examples_common")
#set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/frameworks/esp-idf-v5.2.2/examples/common_components/protocol_examples_common)
# set(COMPONENTS main esp_driver_i2c  esp_netif lwip protocol_examples_tapif_io startup esp_hw_support esp_system nvs_flash)
#endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(i2c-simple)
