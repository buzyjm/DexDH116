cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(LHandProLib_EtherCAT_Test LANGUAGES CXX)

# 定位到 SDK 根目录
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
set(SDK_EXAMPLE_NAME "${SDK_DIST_NAME}_EtherCAT_Test" CACHE STRING "EtherCAT example executable name")
set(SDK_MULTI_EXAMPLE_NAME "${SDK_DIST_NAME}_EtherCAT_Test_Multi" CACHE STRING "EtherCAT multi example executable name")
set(SDK_GLOVE_EXAMPLE_NAME "${SDK_DIST_NAME}_EtherCAT_Test_GLOVE" CACHE STRING "EtherCAT glove example executable name")
add_compile_definitions(
    SDK_CPP_HEADER="${SDK_CPP_HEADER_NAME}"
    SDK_C_HEADER="${SDK_C_HEADER_NAME}"
)
# 设置输出路径统一管理
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
# 覆盖各配置的输出路径，忽略 Debug/Release 子目录
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

option(ETHERCAT_DEBUG "Enable EtherCAT print of test module LHandProLib_EtherCAT_Test" OFF)
add_compile_definitions(ETHERCAT_DEBUG=$<BOOL:${ETHERCAT_DEBUG}>)

# 构建SOEM库
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/SOEM)

# 查找主库
find_library(SDK_LIB NAMES ${SDK_LIBRARY_NAME} LHandProLib PATHS ${SDK_ROOT_DIR}/lib REQUIRED)

# 构建第一个测试程序
add_executable(LHandProLib_EtherCAT_Test main.cpp EthercatMaster.cpp)
set_target_properties(LHandProLib_EtherCAT_Test PROPERTIES
    OUTPUT_NAME "${SDK_EXAMPLE_NAME}"
)

# 链接主库
target_link_libraries(LHandProLib_EtherCAT_Test PRIVATE
    ${SDK_LIB}
    soem
)

# 导出头文件（为第一个目标）
target_include_directories(LHandProLib_EtherCAT_Test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# 构建多从站测试程序
add_executable(LHandProLib_EtherCAT_Test_Multi main_multi.cpp EthercatMaster.cpp)
set_target_properties(LHandProLib_EtherCAT_Test_Multi PROPERTIES
    OUTPUT_NAME "${SDK_MULTI_EXAMPLE_NAME}"
)

# 链接主库
target_link_libraries(LHandProLib_EtherCAT_Test_Multi PRIVATE
    ${SDK_LIB}
    soem
)

# 导出头文件（为多从站目标）
target_include_directories(LHandProLib_EtherCAT_Test_Multi PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)


if(WIN32)
# 构建第二个测试程序（手套版本）
add_executable(LHandProLib_EtherCAT_Test_GLOVE main_glove.cpp EthercatMaster.cpp GloveSDK.cpp)
set_target_properties(LHandProLib_EtherCAT_Test_GLOVE PROPERTIES
    OUTPUT_NAME "${SDK_GLOVE_EXAMPLE_NAME}"
)

# 链接主库
target_link_libraries(LHandProLib_EtherCAT_Test_GLOVE PRIVATE
    ${SDK_LIB}
    soem
    ws2_32
)

# 导出头文件（为第二个目标）
target_include_directories(LHandProLib_EtherCAT_Test_GLOVE PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)
endif()


# 区分 Windows 和 Linux 平台
if(WIN32)
    # Windows (x64)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        # 64-bit (x64)
        message(STATUS "Building for 64-bit (x64)")
    
        # 安装对应的 DLL   
        set(NPCAP_DLLS
            ${CMAKE_CURRENT_SOURCE_DIR}/x64/Packet.dll
            ${CMAKE_CURRENT_SOURCE_DIR}/x64/wpcap.dll
            ${SDK_ROOT_DIR}/bin/${SDK_DLL_NAME}
        )
        
        # 遍历每个 DLL 文件
        foreach(dll_path ${NPCAP_DLLS})
            # 获取文件名（如 Packet.dll）
            get_filename_component(dll_name "${dll_path}" NAME)

            # 目标路径
            set(dst_path "${CMAKE_BINARY_DIR}/bin/${dll_name}")

            # 添加复制命令：构建目标后复制文件（如果不同则复制）
            add_custom_command(
                TARGET LHandProLib_EtherCAT_Test POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${dll_path}" "${dst_path}"
                COMMENT "Copying ${dll_name} to output directory"
                VERBATIM
            )
        endforeach()
        
        install(FILES 
            ${NPCAP_DLLS}
            DESTINATION bin
        )
    else()
        message(FATAL_ERROR "Other architecture: ${CMAKE_SIZEOF_VOID_P}")
    endif()
    
elseif(UNIX AND NOT APPLE)
    # Linux: 链接系统安装
endif()


# 安装测试程序
install(TARGETS LHandProLib_EtherCAT_Test
    RUNTIME DESTINATION bin
)

# 安装多从站测试程序
install(TARGETS LHandProLib_EtherCAT_Test_Multi
    RUNTIME DESTINATION bin
)

if(WIN32)
# 安装手套测试程序
install(TARGETS LHandProLib_EtherCAT_Test_GLOVE
    RUNTIME DESTINATION bin
)
endif()

if(UNIX AND NOT APPLE)
    # 计算绝对路径
    get_filename_component(LIB_SOURCE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../../lib" ABSOLUTE)
    message(STATUS "Absolute lib path: ${LIB_SOURCE_PATH}")
    install(DIRECTORY 
        ${LIB_SOURCE_PATH}/
        DESTINATION bin
    )
endif()
