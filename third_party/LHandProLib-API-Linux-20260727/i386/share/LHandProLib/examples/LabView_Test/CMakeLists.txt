cmake_minimum_required(VERSION 3.16)
project(LHandProLib_LabView_Test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
set(SDK_LV_LIBRARY_NAME "${SDK_DIST_NAME}_LV" CACHE STRING "LabView wrapper library base name")
set(SDK_INCLUDE_SUBDIR "${SDK_DIST_NAME}" CACHE STRING "SDK include subdirectory")
set(SDK_DLL_NAME "${SDK_LIBRARY_NAME}.dll")
set(SDK_LV_DLL_NAME "${SDK_LV_LIBRARY_NAME}.dll")

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(THIRDPARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/x64)
else()
    set(THIRDPARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/x86)
endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
foreach(cfg DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${cfg} ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${cfg} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${cfg} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
endforeach()

include_directories(${SDK_ROOT_DIR}/include/${SDK_INCLUDE_SUBDIR})
include_directories(${THIRDPARTY_DIR}/include)

find_library(SDK_LIB NAMES ${SDK_LIBRARY_NAME} LHandProLib PATHS ${SDK_ROOT_DIR}/lib REQUIRED)
find_library(SDK_LV_LIB NAMES ${SDK_LV_LIBRARY_NAME} LHandProLib_LV PATHS ${SDK_ROOT_DIR}/lib REQUIRED)

set(COMMON_LINK_LIBS ${SDK_LIB} ${SDK_LV_LIB})
if(WIN32)
    find_library(HCANBUS_LIBRARY NAMES HCanbus PATHS ${THIRDPARTY_DIR}/lib REQUIRED)
    list(APPEND COMMON_LINK_LIBS ${HCANBUS_LIBRARY})
endif()

add_executable(LHandProLib_LabView_Test_CANFD main_canfd.cpp)
set_target_properties(LHandProLib_LabView_Test_CANFD PROPERTIES
    OUTPUT_NAME "${SDK_DIST_NAME}_LabView_Test_CANFD"
)
target_link_libraries(LHandProLib_LabView_Test_CANFD PRIVATE ${COMMON_LINK_LIBS})

add_executable(LHandProLib_LabView_Test_EtherCAT main_ethercat.cpp)
set_target_properties(LHandProLib_LabView_Test_EtherCAT PROPERTIES
    OUTPUT_NAME "${SDK_DIST_NAME}_LabView_Test_EtherCAT"
)
target_link_libraries(LHandProLib_LabView_Test_EtherCAT PRIVATE ${COMMON_LINK_LIBS})

add_executable(LHandProLib_LabView_Test_RS485 main_rs485.cpp)
set_target_properties(LHandProLib_LabView_Test_RS485 PROPERTIES
    OUTPUT_NAME "${SDK_DIST_NAME}_LabView_Test_RS485"
)
target_link_libraries(LHandProLib_LabView_Test_RS485 PRIVATE ${COMMON_LINK_LIBS})

if(WIN32)
    set(TEST_DLLS
        ${SDK_DLL_NAME}
        ${SDK_LV_DLL_NAME}
        HCanbus.dll
    )

    foreach(target LHandProLib_LabView_Test_CANFD LHandProLib_LabView_Test_EtherCAT LHandProLib_LabView_Test_RS485)
        foreach(dll_name ${TEST_DLLS})
            set(src_path "${SDK_ROOT_DIR}/bin/${dll_name}")
            set(dst_path "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${dll_name}")
            add_custom_command(TARGET ${target} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${src_path}" "${dst_path}"
            )
        endforeach()
    endforeach()
endif()
