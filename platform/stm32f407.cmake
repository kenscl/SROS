set(MCU_SPEC "cortex-m4")
set(MCU_DEFINE "STM32F407G")

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/src/hal/stm32f407/linker_script.ld)

set(CMAKE_C_FLAGS "-mcpu=${MCU_SPEC} -D${MCU_DEFINE} -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g -Os -Wno-write-strings -fno-builtin -fno-lto -DSTM32F407xx")
set(CMAKE_CXX_FLAGS "-mcpu=${MCU_SPEC} -D${MCU_DEFINE} -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g -fno-exceptions -fno-rtti -Os -Wno-write-strings -Wno-endif-labels -fno-builtin -fno-lto -DSTM32F407xx")
set(CMAKE_EXE_LINKER_FLAGS "-T${LINKER_SCRIPT} -mcpu=${MCU_SPEC} --specs=nano.specs -Wl,--gc-sections -Os")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmessage-length=0 -fno-common -ffunction-sections -fdata-sections")

include_directories(
    ${CMAKE_SOURCE_DIR}/src/hal/stm32f407/Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_SOURCE_DIR}/src/hal/stm32f407/Drivers/CMSIS/Core/Include
    ${CMAKE_SOURCE_DIR}/src/hal/stm32f407/Drivers/STM32F4xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/src/hal/stm32f407/Core/Inc
    ${CMAKE_SOURCE_DIR}/src/hal/stm32f407
)

file(GLOB HAL_SRC
    ${CMAKE_SOURCE_DIR}/src/hal/stm32f407/Drivers/STM32F4xx_HAL_Driver/Src/*.cpp
    ${CMAKE_SOURCE_DIR}/src/hal/stm32f407/Core/Src/*.cpp
    ${CMAKE_SOURCE_DIR}/src/hal/stm32f407/Drivers/STM32F4xx_HAL_Driver/Src/*.c
    ${CMAKE_SOURCE_DIR}/src/hal/stm32f407/Core/Src/*.c
)

file(GLOB STARTUP_SRC
    ${CMAKE_SOURCE_DIR}/src/hal/stm32f407/*.c
    ${CMAKE_SOURCE_DIR}/src/hal/stm32f407/*.cpp
)

set(HAL_FILES_C ${STARTUP_SRC} ${HAL_SRC})


set(FLASH_LOCATION 0x8000000)