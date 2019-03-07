cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 14)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(NAO_PATH "${CMAKE_CURRENT_LIST_DIR}/../nao" CACHE PATH "Nao path")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${NAO_PATH}/CMake")

# generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
# for some weird compiler bug
set(RTTR_ENABLE_WERROR 0)

find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)

add_subdirectory(lib)

### Main Programs

# old test1
#set(target1 test1)
#add_executable(${target1} tests/test1.cpp ${TUHHSDK_SOURCES} ${TUHHSDK_HEADERS})
#target_link_libraries(${target1} utils naoPose tuhh)

# poseGen
set(target2 poseGen)
add_executable(${target2} poseGenerator.cpp lib/inc/MiniConfigHandle.hpp ${TUHHSDK_SOURCES} ${TUHHSDK_HEADERS})
target_link_libraries(${target2} observability utils naoPose tuhh)

# Get sensitivities by digesting the poses :P
set(target3 sensitivityExtractor)
add_executable(${target3} SensitivityExtractor.cpp ${TUHHSDK_SOURCES} ${TUHHSDK_HEADERS})
target_link_libraries(${target3} observability utils naoPose tuhh)

# Filter poses by sensitivity, etc..
set(target4 poseFilter)
add_executable(${target4} PoseFilter.cpp ${TUHHSDK_SOURCES} ${TUHHSDK_HEADERS})
target_link_libraries(${target4} observability utils naoPose tuhh)

## Filter poses by sensitivity, etc..
#set(target5 calibServer)
#add_executable(${target5} calibServer.cpp ${TUHHSDK_SOURCES} ${TUHHSDK_HEADERS})
#target_link_libraries(${target5} observability utils naoPose tuhh zmq)

### Evaluations

# 1st Simulation test
set(eval1 simWholeGridTest)
add_executable(${eval1} tests/simTest.cpp ${TUHHSDK_SOURCES} ${TUHHSDK_HEADERS})
target_link_libraries(${eval1} observability utils naoPose tuhh)

### Tests

# other tests
set(test1 tests)
add_executable(${test1} tests/tests.cpp lib/inc/MiniConfigHandle.hpp ${TUHHSDK_SOURCES} ${TUHHSDK_HEADERS})
target_link_libraries(${test1} observability utils naoPose tuhh)

# Pose2d test [ sort of ICP ]
set(test2 icpTest)
add_executable(${test2} tests/icpTest.cpp ${TUHHSDK_SOURCES} ${TUHHSDK_HEADERS})
target_link_libraries(${test2} observability utils naoPose tuhh)

# Obs Tests
set(test3 calibFeatureTest)
add_executable(${test3} tests/calibFeatureTest.cpp ${TUHHSDK_SOURCES} ${TUHHSDK_HEADERS})
target_link_libraries(${test3} observability utils naoPose tuhh)

# Obs Tests
#set(test4 calibSolverTest)
#add_executable(${test4} tests/JointCalibSolverTest.cpp ${TUHHSDK_SOURCES} ${TUHHSDK_HEADERS})
#target_link_libraries(${test4} observability utils naoPose tuhh)

# Joint error data generator
set(dataGen jointErrorGen)
add_executable(${dataGen} tests/JointErrorGeneration.cpp ${TUHHSDK_SOURCES} ${TUHHSDK_HEADERS})
target_link_libraries(${dataGen} observability utils naoPose tuhh)

### Post build

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/configuration)
configure_file(configuration/limits_d.json configuration/)
configure_file(configuration/limits_l.json configuration/)
configure_file(configuration/limits_r.json configuration/)
configure_file(configuration/simGridEvalConf.json configuration/)
configure_file(configuration/cameraObsModelConf.json configuration/)
configure_file(configuration/poseFilterConf.json configuration/)
# test data
configure_file(tests/test_data/l_1000_jointErrors.txt test_data/)
