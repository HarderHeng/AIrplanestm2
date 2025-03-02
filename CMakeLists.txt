if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(Airplane VERSION 0.1.0 LANGUAGES C) # CMake version
set(CMAKE_C_STANDARD 99) # C standard

add_compile_options(-Wall -Wextra -pedantic) # Compiler options
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pipe -std=c99") # C flags
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g -O0") # Debug flags
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3") # Release flags

# Define macros
add_definitions(-DUSE_STDPERIPH_DRIVER)
add_definitions(-DSTM32F401xx)

include_directories(Driver/Inc) # Include directories
include_directories(OS/Inc)
include_directories(STM32F40xLib/Inc)

include_directories(USR)
# 添加源文件
file(GLOB_RECURSE SOURCES
    OS/Src/*.c
    startup_stm32f401xe.s
    OS/*.s
    OS/*.asm
    Drivers/Src/*.c
    STM32F40xLib/Src/*.c
    USR/*.c
)
add_executable(${PROJECT_NAME} ${SOURCES}) # Add executable

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME} ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME} ${PROJECT_NAME}.bin
    COMMENT "Generating HEX and BIN files"
)
