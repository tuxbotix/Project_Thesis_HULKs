cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 14)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(NAO_PATH "${CMAKE_CURRENT_LIST_DIR}/../nao")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${NAO_PATH}/CMake")

# generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
# for some weird compiler bug
set(RTTR_ENABLE_WERROR 0)

add_subdirectory(lib)
# Execs
#poseGen

set(target1 test1)
add_executable(${target1} test1.cpp ${TUHHSDK_SOURCES} ${TUHHSDK_HEADERS})
target_link_libraries(${target1} utils naoPose tuhh)

set(target2 poseGen)
add_executable(${target2} poseGenerator.cpp ${TUHHSDK_SOURCES} ${TUHHSDK_HEADERS})
target_link_libraries(${target2} observability utils naoPose tuhh)

# Get sensitivities by digesting the poses :P
set(target3 sensitivityExtractor)
add_executable(${target3} SensitivityExtractor.cpp ${TUHHSDK_SOURCES} ${TUHHSDK_HEADERS})
target_link_libraries(${target3} observability utils naoPose tuhh)

# Filter poses by sensitivity, etc..

set(target4 poseFilter)
add_executable(${target4} PoseFilter.cpp ${TUHHSDK_SOURCES} ${TUHHSDK_HEADERS})
target_link_libraries(${target4} observability utils naoPose tuhh)

# other tests
set(target5 tests)
add_executable(${target5} tests.cpp ${TUHHSDK_SOURCES} ${TUHHSDK_HEADERS})
target_link_libraries(${target5} observability utils naoPose tuhh)