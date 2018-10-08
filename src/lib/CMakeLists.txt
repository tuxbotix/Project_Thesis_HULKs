cmake_minimum_required(VERSION 3.10)

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

include_directories(SYSTEM PUBLIC inc)
# LibTUHH
set(libtuhh tuhh)
add_library(${libtuhh} ${TUHHSDK_SOURCES} ${TUHHSDK_HEADERS})
target_include_directories(${libtuhh} SYSTEM PUBLIC ${TUHHSDK_HOME} ${TUHH_DEPS_INCLUDE_DIRECTORIES})
target_link_libraries(${libtuhh} ${TUHH_DEPS_LIBRARIES} ${TUHHSDK_LIBRARIES})

add_library(naoPose src/NaoCollision.cpp src/NaoProjectionDataProvider.cpp src/Sensors.cpp inc/NaoStability.hpp inc/NaoCollision.hpp inc/NaoTorsoPose.hpp inc/NaoProjectionDataProvider.hpp inc/NaoPoseInfo.hpp inc/Sensors.hpp)
target_include_directories(naoPose SYSTEM PUBLIC inc external)
target_link_libraries(naoPose tuhh)


add_library(utils src/utils.cpp inc/utils.hpp)
target_include_directories(utils SYSTEM PUBLIC inc)
target_link_libraries(utils naoPose tuhh)

set(libObs observability)
set(OBS_HEADERS 
    inc/ObservationSensitivityProvider.hpp
    inc/CameraObservationModel.hpp
    inc/Solvers.hpp
    inc/ObservationModel.hpp
)

set(OBS_SOURCES
    src/CameraObservationModel.cpp 
)

add_library(${libObs} ${OBS_SOURCES} ${OBS_HEADERS})
target_include_directories(${libObs} SYSTEM PUBLIC inc)
target_link_libraries(${libObs} naoPose tuhh)

# target_include_directories(${targetLib1} SYSTEM PUBLIC ${TUHHSDK_HOME} ${TUHH_DEPS_INCLUDE_DIRECTORIES})
# target_link_libraries(${targetLib1} ${TUHH_DEPS_LIBRARIES} ${TUHHSDK_LIBRARIES})

# # add_library(nao_pose inc/NaoStability.hpp inc/NaoTorsoPose.hpp inc/NaoProjectionDataProvider.hpp inc/NaoPoseInfo.hpp)
# # add_library(nao_pose inc/NaoStability.hpp inc/NaoTorsoPose.hpp inc/NaoProjectionDataProvider.hpp src/NaoPoseInfo.cpp)
