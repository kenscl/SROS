set(MCU_SPEC "cortex-m3")
set(MCU_DEFINE "STM32F103xB")

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/src/hal/stm32f103/linker_script.ld)

set(CMAKE_C_FLAGS "-mcpu=${MCU_SPEC} -DSTM32F103xB -mthumb -g")
set(CMAKE_CXX_FLAGS "-mcpu=${MCU_SPEC} -DSTM32F103xB -mthumb -g -std=c++11 -fno-exceptions -fno-rtti")
set(CMAKE_EXE_LINKER_FLAGS "-T${LINKER_SCRIPT} -mcpu=${MCU_SPEC} --specs=nano.specs -Wl,--gc-sections")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmessage-length=0 -fno-common -ffunction-sections -fdata-sections")

include_directories(
    cmsis/Device/ST/STM32F1/Include
    cmsis/CMSIS/Core/Include
)

set(HAL_FILES_C ${CMAKE_SOURCE_DIR}/src/hal/stm32f103*.c)
set(HAL_FILES_CPP ${CMAKE_SOURCE_DIR}/src/hal/stm32f103*.cpp)

set(OTHER_SOURCES cmsis/Device/ST/STM32F1/Source/Templates/system_stm32f1xx.c)

set(FLASH_LOCATION 0x8000000)