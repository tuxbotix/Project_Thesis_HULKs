cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 14)

set(NAO_PATH "${CMAKE_CURRENT_LIST_DIR}/../nao")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${NAO_PATH}/CMake")

# generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
# for some weird compiler bug
set(RTTR_ENABLE_WERROR 0)

set(FIND_CMAKE_SCRIPT_DIR "${NAO_PATH}/CMake")

include("${FIND_CMAKE_SCRIPT_DIR}/settings.cmake")
include("${FIND_CMAKE_SCRIPT_DIR}/dependencies.cmake")



# Find packages
find_package(tuhhSDK REQUIRED)

#target_include_directories(${PROJECT_NAME} PUBLIC ${TUHHSDK_INCLUDE_DIRS})

set(TUHHSDK_HOME ${NAO_PATH}/src/tuhhsdk)

add_subdirectory(${TUHHSDK_HOME} "${CMAKE_CURRENT_BINARY_DIR}/tuhhsdk")

message("lalal" ${TUHHSDK_HOME})
include(tuhhSDKInfo.cmake)

# Execs

set(target1 test1)
add_executable(${target1} test1.cpp ${TUHHSDK_SOURCES} ${TUHHSDK_HEADERS})
target_include_directories(${target1} SYSTEM PUBLIC ${TUHHSDK_HOME} ${TUHH_DEPS_INCLUDE_DIRECTORIES})

target_link_libraries(${target1} ${TUHH_DEPS_LIBRARIES} ${TUHHSDK_LIBRARIES})
