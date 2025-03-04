set(MCU_SPEC "cortex-m4")
set(MCU_DEFINE "STM32F407G")

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/src/hal/stm32f407/linker_script.ld)

set(CMAKE_C_FLAGS "-mcpu=${MCU_SPEC} -DSTM32F407xx -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g -O0 -Wno-write-strings -fno-builtin -fno-lto")
set(CMAKE_CXX_FLAGS "-mcpu=${MCU_SPEC} -DSTM32F407xx -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g -std=c++11 -fno-exceptions -fno-rtti -O0 -Wno-write-strings -Wno-endif-labels -fno-builtin -fno-lto")
set(CMAKE_EXE_LINKER_FLAGS "-T${LINKER_SCRIPT} -mcpu=${MCU_SPEC} --specs=nano.specs -Wl,--gc-sections -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmessage-length=0 -fno-common -ffunction-sections -fdata-sections")

include_directories(
    cmsis/Device/ST/STM32F4/Include
    cmsis/CMSIS/Core/Include
)

set(HAL_FILES_C ${CMAKE_SOURCE_DIR}/src/hal/stm32f407/*.c ${CMAKE_SOURCE_DIR}/src/hal/stm32f407/startup.c)
set(HAL_FILES_CPP ${CMAKE_SOURCE_DIR}/src/hal/stm32f407/*.cpp)

set(OTHER_SOURCES cmsis/Device/ST/STM32F1/Source/Templates/system_stm32f4xx.c)

set(FLASH_LOCATION 0x8000000)