cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(LHandProLib_RS485_Test LANGUAGES CXX)

# 相对路径 SDK 目录
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
set(SDK_EXAMPLE_NAME "${SDK_DIST_NAME}_RS485_Test" CACHE STRING "RS485 example executable name")
set(SDK_DUAL_EXAMPLE_NAME "${SDK_DIST_NAME}_RS485_DualNode_Test" CACHE STRING "RS485 dual-node example executable name")

# 设置输出路径统一管理
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
# 分别设置不同编译器的输出目录
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

# 包含头文件目录
include_directories(${SDK_ROOT_DIR}/include/${SDK_INCLUDE_SUBDIR})

# 查找库文件
find_library(SDK_LIB NAMES ${SDK_LIBRARY_NAME} LHandProLib PATHS ${SDK_ROOT_DIR}/lib REQUIRED)

# C++测试工程
add_executable(LHandProLib_RS485_Test main.cpp SerialPort.cpp)
target_compile_definitions(LHandProLib_RS485_Test PRIVATE
    SDK_CPP_HEADER="${SDK_CPP_HEADER_NAME}"
)
set_target_properties(LHandProLib_RS485_Test PROPERTIES
    OUTPUT_NAME "${SDK_EXAMPLE_NAME}"
)

# 单串口双节点测试工程
add_executable(LHandProLib_RS485_DualNode_Test main_dual_node.cpp SerialPort.cpp)
target_compile_definitions(LHandProLib_RS485_DualNode_Test PRIVATE
    SDK_CPP_HEADER="${SDK_CPP_HEADER_NAME}"
)
set_target_properties(LHandProLib_RS485_DualNode_Test PROPERTIES
    OUTPUT_NAME "${SDK_DUAL_EXAMPLE_NAME}"
)

# 链接库
target_link_libraries(LHandProLib_RS485_Test PRIVATE ${SDK_LIB})
target_link_libraries(LHandProLib_RS485_DualNode_Test PRIVATE ${SDK_LIB})

# 设置头文件为公开目录
target_include_directories(LHandProLib_RS485_Test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# 设置双节点测试头文件目录
target_include_directories(LHandProLib_RS485_DualNode_Test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# 安装
install(TARGETS LHandProLib_RS485_Test
    RUNTIME DESTINATION bin
)

install(TARGETS LHandProLib_RS485_DualNode_Test
    RUNTIME DESTINATION bin
)

if(WIN32)
    # Windows (x64)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        message(STATUS "Building for 64-bit (x64)")

        # 复制 DLL
        set(DLL_FILES
            ${SDK_ROOT_DIR}/bin/${SDK_DLL_NAME}
        )

        foreach(example_target
            LHandProLib_RS485_Test
            LHandProLib_RS485_DualNode_Test
        )
            foreach(dll_path ${DLL_FILES})
                get_filename_component(dll_name "${dll_path}" NAME)
                set(dst_path "${CMAKE_BINARY_DIR}/bin/${dll_name}")

                add_custom_command(
                    TARGET ${example_target} POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        "${dll_path}" "${dst_path}"
                    COMMENT "Copying ${dll_name} to output directory"
                    VERBATIM
                )
            endforeach()
        endforeach()

        install(FILES
            ${DLL_FILES}
            DESTINATION bin
        )
    endif()
endif()

if(UNIX AND NOT APPLE)
    # Linux: 设置库路径
    get_filename_component(LIB_SOURCE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../../lib" ABSOLUTE)
    message(STATUS "Absolute lib path: ${LIB_SOURCE_PATH}")
    install(DIRECTORY
        ${LIB_SOURCE_PATH}/
        DESTINATION bin
    )
endif()
