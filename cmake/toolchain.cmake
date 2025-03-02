set(CMAKE_SYSTEM_NAME Generic) # 指定目标系统
set(CMAKE_SYSTEM_PROCESSOR arm) # 指定目标处理器

set(TOOLCHAIN_PREFIX "arm-none-eabi-") # 工具链前缀

set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc) # 指定C编译器
set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy) # 指定目标文件转换工具
set(CMAKE_SIZE ${TOOLCHAIN_PREFIX}size) # 指定目标文件大小工具
set(CMAKE_DUMP ${TOOLCHAIN_PREFIX}objdump) # 指定目标文件反汇编工具
set(CMAKE_AS_COMPILER ${TOOLCHAIN_PREFIX}as) # 指定汇编编译器
set(CMAKE_LINKER ${TOOLCHAIN_PREFIX}ld) # 指定链接器

#set(CMAKE_C_DEFS "-DSTM32F40_41xxx -DUSE_STDPERIPH_DRIVER") # 定义编译宏


set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)  # 禁用测试编译（嵌入式系统无OS）

set(MCU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

set(CMAKE_C_FLAGS "${MCU_FLAGS} -ffunction-sections -fdata-sections")
set(CMAKE_EXE_LINKER_FLAGS "${MCU_FLAGS} -Wl,--gc-sections -Wl,-Map=${PROJECT_NAME}.map -T ${CMAKE_SOURCE_DIR}/STM32F401RETx_FLASH.ld")
