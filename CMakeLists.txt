cmake_minimum_required(VERSION 3.15)

project(OS LANGUAGES C CXX ASM)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(TOOLCHAIN_PATH /usr)
set(CMAKE_C_COMPILER ${TOOLCHAIN_PATH}/bin/arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PATH}/bin/arm-none-eabi-g++)
set(CMAKE_OBJCOPY ${TOOLCHAIN_PATH}/bin/arm-none-eabi-objcopy)
set(CMAKE_SIZE ${TOOLCHAIN_PATH}/bin/arm-none-eabi-size)

set(PLATFORM_CONFIG_FILE "" CACHE STRING "Path to the platform-specific configuration file")
if (PLATFORM_CONFIG_FILE STREQUAL "")
    message(FATAL_ERROR "You must specify a platform configuration file using -DPLATFORM_CONFIG_FILE=<path>")
endif()
include(${PLATFORM_CONFIG_FILE})

set(TARGET_NAME SROS)

file(GLOB C_SOURCES ${CMAKE_SOURCE_DIR}/src/*.c ${HAL_FILES_C} ${CMAKE_SOURCE_DIR}/src/krnl/*.c ${CMAKE_SOURCE_DIR}/src/hal/*.c ${CMAKE_SOURCE_DIR}/src/communication/*.c ${CMAKE_SOURCE_DIR}/src/math/*.c)
file(GLOB CPP_SOURCES ${CMAKE_SOURCE_DIR}/src/*.cpp ${HAL_FILES_CPP} ${CMAKE_SOURCE_DIR}/src/krnl/*.cpp ${CMAKE_SOURCE_DIR}/src/hal/*.cpp ${CMAKE_SOURCE_DIR}/src/communication/*.cpp ${CMAKE_SOURCE_DIR}/src/math/*.cpp)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES} ${OUTHER_SOURCES})

add_executable(${TARGET_NAME}.elf ${SOURCES})

add_custom_command(
    TARGET ${TARGET_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${TARGET_NAME}.elf ${TARGET_NAME}.bin
    COMMAND ${CMAKE_SIZE} ${TARGET_NAME}.elf
)

add_custom_target(flash
    COMMAND st-flash write ${TARGET_NAME}.bin ${FLASH_LOCATION}
    DEPENDS ${TARGET_NAME}.bin
)

set_property(DIRECTORY PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
    "${CMAKE_SOURCE_DIR}/src/*.o"
)
