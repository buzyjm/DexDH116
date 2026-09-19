cmake_minimum_required(VERSION 3.14)
project(LHandProLib_CANFD_Test LANGUAGES CXX)

# 定位 SDK 根目录
set(SDK_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../..)
message(STATUS "SDK Root Directory: ${SDK_ROOT_DIR}")

if(NOT DEFINED SDK_DIST_NAME)
    file(GLOB SDK_INCLUDE_CANDIDATES LIST_DIRECTORIES true "${SDK_ROOT_DIR}/include/*")
    list(LENGTH SDK_INCLUDE_CANDIDATES SDK_INCLUDE_COUNT)
    if(SDK_INCLUDE_COUNT EQUAL 1)
        list(GET SDK_INCLUDE_CANDIDATES 0 SDK_INCLUDE_DIR)
        get_filename_component(SDK_DIST_NAME "${SDK_INCLUDE_DIR}" NAME)
    else()
        set(SDK_DIST_NAME "LHandProLib")
    endif()
endif()
set(SDK_LIBRARY_NAME "${SDK_DIST_NAME}" CACHE STRING "SDK library base name")
set(SDK_INCLUDE_SUBDIR "${SDK_DIST_NAME}" CACHE STRING "SDK include subdirectory")
set(SDK_DLL_NAME "${SDK_LIBRARY_NAME}.dll")
set(SDK_CPP_HEADER_NAME "${SDK_DIST_NAME}.hpp" CACHE STRING "SDK C++ header file name")
set(SDK_C_HEADER_NAME "${SDK_DIST_NAME}.h" CACHE STRING "SDK C header file name")
set(SDK_EXAMPLE_NAME "${SDK_DIST_NAME}_CANFD_Test" CACHE STRING "CANFD example executable name")
add_compile_definitions(
    SDK_CPP_HEADER="${SDK_CPP_HEADER_NAME}"
    SDK_C_HEADER="${SDK_C_HEADER_NAME}"
)

option(LHANDPRO_USE_LIBCANBUS "Use libcanbus backend on Linux (lower priority than CANFDMaster.h macro)" OFF)
# 输出路径统一配置
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELWITHDEBINFO ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_MINSIZEREL ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_MINSIZEREL ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# 添加头文件目录
include_directories(${SDK_ROOT_DIR}/include/${SDK_INCLUDE_SUBDIR})

# 设置第三方库路径（根据架构选择 x64 或 x86）
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(THIRDPARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/x64)
else()
    set(THIRDPARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/x86)
endif()

# 构建测试程序
add_executable(LHandProLib_CANFD_Test main.cpp CANFDMaster.cpp)
set_target_properties(LHandProLib_CANFD_Test PROPERTIES
    OUTPUT_NAME "${SDK_EXAMPLE_NAME}"
)

# 查找 LHandProLib
find_library(SDK_LIB NAMES ${SDK_LIBRARY_NAME} LHandProLib PATHS ${SDK_ROOT_DIR}/lib REQUIRED)

if(WIN32)
    find_library(HCANBUS_LIBRARY NAMES HCanbus PATHS ${THIRDPARTY_DIR}/lib REQUIRED)
    target_link_libraries(LHandProLib_CANFD_Test PRIVATE
        ${SDK_LIB}
        ${HCANBUS_LIBRARY}
    )
else()
    target_link_libraries(LHandProLib_CANFD_Test PRIVATE
        ${SDK_LIB}
        dl
    )
endif()

if(LHANDPRO_USE_LIBCANBUS)
    target_compile_definitions(LHandProLib_CANFD_Test PRIVATE LHANDPRO_USE_LIBCANBUS=1)
    message(STATUS "LHANDPRO_USE_LIBCANBUS is ON (effective only when CANFDMaster.h macro is commented out)")
endif()

# 添加头文件
target_include_directories(LHandProLib_CANFD_Test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${THIRDPARTY_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Windows: 构建后拷贝 DLL 到输出目录
if(WIN32)
    set(CANFD_DLL_PATHS
        ${SDK_ROOT_DIR}/bin/${SDK_DLL_NAME}
        ${THIRDPARTY_DIR}/bin/HCanbus.dll
    )

    foreach(dll_path ${CANFD_DLL_PATHS})
        get_filename_component(dll_name "${dll_path}" NAME)
        add_custom_command(TARGET LHandProLib_CANFD_Test POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${dll_path}"
                "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${dll_name}"
        )
    endforeach()
endif()

# 安装测试程序
install(TARGETS LHandProLib_CANFD_Test
    RUNTIME DESTINATION bin
)

# Linux: 同时安装 LHandProLib 动态库
if(UNIX AND NOT APPLE)
    get_filename_component(LIB_SOURCE_PATH "${SDK_ROOT_DIR}/lib" ABSOLUTE)
    message(STATUS "Absolute lib path: ${LIB_SOURCE_PATH}")
    install(DIRECTORY
        ${LIB_SOURCE_PATH}/
        DESTINATION bin
    )
endif()

# Windows: 安装 DLL
if(WIN32)
    install(FILES
        ${CANFD_DLL_PATHS}
        DESTINATION bin
    )
endif()
