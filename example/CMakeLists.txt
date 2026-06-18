cmake_minimum_required(VERSION 3.10)
project(TL_Robot_Arm_SDK_Examples LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 项目根目录（即 _tl_host.so 所在目录）
set(PROJ_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

# 头文件路径
set(TL_INCLUDE_DIR ${PROJ_ROOT}/include)

# 将 _tl_host.so 作为导入的共享库
add_library(tl_host SHARED IMPORTED)
set_target_properties(tl_host PROPERTIES
    IMPORTED_LOCATION ${PROJ_ROOT}/_tl_host.so
)

# 自动扫描所有 .cpp 源文件生成可执行文件
file(GLOB EXAMPLE_SOURCES "*.cpp")
foreach(SOURCE_FILE ${EXAMPLE_SOURCES})
    get_filename_component(EXEC_NAME ${SOURCE_FILE} NAME_WE)
    add_executable(${EXEC_NAME} ${SOURCE_FILE})
    target_include_directories(${EXEC_NAME} PRIVATE ${TL_INCLUDE_DIR})
    target_link_libraries(${EXEC_NAME} PRIVATE tl_host pthread)
    # 嵌入 RPATH，避免运行时设置 LD_LIBRARY_PATH
    set_target_properties(${EXEC_NAME} PROPERTIES
        INSTALL_RPATH ${PROJ_ROOT}
        BUILD_RPATH ${PROJ_ROOT}
    )
endforeach()
