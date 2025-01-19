cmake_minimum_required(VERSION 3.15)

project(OS LANGUAGES C CXX ASM)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(TOOLCHAIN_PATH /usr)
set(CMAKE_C_COMPILER ${TOOLCHAIN_PATH}/bin/arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PATH}/bin/arm-none-eabi-g++)
set(CMAKE_OBJCOPY ${TOOLCHAIN_PATH}/bin/arm-none-eabi-objcopy)
set(CMAKE_SIZE ${TOOLCHAIN_PATH}/bin/arm-none-eabi-size)

set(TARGET_NAME OS)

set(MCU_SPEC cortex-m3)
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/linker_script.ld)

set(CMAKE_C_FLAGS "-mcpu=${MCU_SPEC} -DSTM32F103xB -mthumb -g")
set(CMAKE_CXX_FLAGS "-mcpu=${MCU_SPEC} -DSTM32F103xB -mthumb -g -std=c++11 -fno-exceptions -fno-rtti")
set(CMAKE_EXE_LINKER_FLAGS "-T${LINKER_SCRIPT} -mcpu=${MCU_SPEC} --specs=nano.specs -Wl,--gc-sections")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmessage-length=0 -fno-common -ffunction-sections -fdata-sections")

include_directories(
    cmsis/Device/ST/STM32F1/Include
    cmsis/CMSIS/Core/Include
)

set(STM32F103_FILES_C ${CMAKE_SOURCE_DIR}/src/hal/stm32f103*.c)
set(STM32F103_FILES_CPP ${CMAKE_SOURCE_DIR}/src/hal/stm32f103*.cpp)

file(GLOB_RECURSE C_SOURCES ${CMAKE_SOURCE_DIR}/src/*.c ${STM32F103_FILES_C} ${CMAKE_SOURCE_DIR}/src/krnl/*.c ${CMAKE_SOURCE_DIR}/src/hal/*.c ${CMAKE_SOURCE_DIR}/src/communication*.c)
file(GLOB_RECURSE CPP_SOURCES ${CMAKE_SOURCE_DIR}/src/*.cpp ${STM32F103_FILES_CPP} ${CMAKE_SOURCE_DIR}/src/krnl/*.cpp ${CMAKE_SOURCE_DIR}/src/hal/*.cpp ${CMAKE_SOURCE_DIR}/src/communication*.cpp)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES} cmsis/Device/ST/STM32F1/Source/Templates/system_stm32f1xx.c)

add_executable(${TARGET_NAME}.elf ${SOURCES})

add_custom_command(
    TARGET ${TARGET_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${TARGET_NAME}.elf ${TARGET_NAME}.bin
    COMMAND ${CMAKE_SIZE} ${TARGET_NAME}.elf
)

add_custom_target(flash
    COMMAND st-flash write ${TARGET_NAME}.bin 0x8000000
    DEPENDS ${TARGET_NAME}.bin
)

set_property(DIRECTORY PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
    "${CMAKE_SOURCE_DIR}/src/*.o"
)
