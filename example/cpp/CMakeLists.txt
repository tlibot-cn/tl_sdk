cmake_minimum_required(VERSION 3.10)
project(TL_Robot_Arm_SDK_Examples LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 项目根目录
set(PROJ_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

# 头文件路径
set(TL_INCLUDE_DIR ${PROJ_ROOT}/include)

# 动态库路径（不同平台路径不同）
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
        set(TL_LIB_DIR ${PROJ_ROOT}/lib/linux/arm64)
    else()
        set(TL_LIB_DIR ${PROJ_ROOT}/lib/linux/x86)
    endif()
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set(TL_LIB_DIR ${PROJ_ROOT}/lib/windows)
else()
    set(TL_LIB_DIR ${PROJ_ROOT}/lib/linux/x86)
endif()

# 导入 libtl_host.so 共享库（注意：_tl_host.so 是 Python SWIG 扩展模块，不可用于 C++ 链接）
add_library(tl_host SHARED IMPORTED)
if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set_target_properties(tl_host PROPERTIES
        IMPORTED_LOCATION ${TL_LIB_DIR}/tl_host.dll
        IMPORTED_IMPLIB ${TL_LIB_DIR}/tl_host.lib
    )
else()
    set_target_properties(tl_host PROPERTIES
        IMPORTED_LOCATION ${TL_LIB_DIR}/libtl_host.so
    )
endif()

# 自动扫描所有 .cpp 源文件生成可执行文件
file(GLOB EXAMPLE_SOURCES "*.cpp")
foreach(SOURCE_FILE ${EXAMPLE_SOURCES})
    get_filename_component(EXEC_NAME ${SOURCE_FILE} NAME_WE)
    add_executable(${EXEC_NAME} ${SOURCE_FILE})
    target_include_directories(${EXEC_NAME} PRIVATE ${TL_INCLUDE_DIR})
    target_link_libraries(${EXEC_NAME} PRIVATE tl_host pthread)
    # 嵌入 RPATH，避免运行时设置 LD_LIBRARY_PATH
    set_target_properties(${EXEC_NAME} PROPERTIES
        INSTALL_RPATH ${TL_LIB_DIR}
        BUILD_RPATH ${TL_LIB_DIR}
    )
endforeach()
