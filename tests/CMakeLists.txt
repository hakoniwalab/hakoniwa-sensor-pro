cmake_minimum_required(VERSION 3.16)

# Tests of the Hakoniwa sensor implementation, built against a binary set
# (dist/<platform>/, see tools/build-binaries.py or the release assets), its public
# headers and the public interface hakoniwa-sensor-adapter (MIT) only. The test
# sources are synchronised from the implementation's own repository by tools/sync.py.
project(hakoniwa_sensor_pro_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
set(HAKO_SENSOR_PRO_DIST "" CACHE PATH "Binary set directory (contains lib/ and include/)")
if(NOT EXISTS "${HAKO_SENSOR_PRO_DIST}/MANIFEST.json")
    message(FATAL_ERROR "Set HAKO_SENSOR_PRO_DIST to a binary set (dist/<platform>): ${HAKO_SENSOR_PRO_DIST}")
endif()

# The public interface, at the revision the binaries were built with (config/sources.json).
set(HAKONIWA_SENSOR_ADAPTER_ROOT "${REPO_ROOT}/../hakoniwa-sensor-adapter" CACHE PATH "hakoniwa-sensor-adapter checkout")
file(READ "${REPO_ROOT}/config/sources.json" _sources)
string(JSON _adapter_revision GET "${_sources}" sources sensor-adapter revision)
execute_process(COMMAND git -C "${HAKONIWA_SENSOR_ADAPTER_ROOT}" rev-parse HEAD
    OUTPUT_VARIABLE _adapter_head OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
if(NOT _adapter_head STREQUAL _adapter_revision)
    message(FATAL_ERROR "hakoniwa-sensor-adapter at ${HAKONIWA_SENSOR_ADAPTER_ROOT} is ${_adapter_head}; "
                        "the binaries were built with ${_adapter_revision}")
endif()
add_subdirectory("${HAKONIWA_SENSOR_ADAPTER_ROOT}" sensor-adapter)

include(FetchContent)
find_package(nlohmann_json 3.11 QUIET)
if(NOT nlohmann_json_FOUND)
    FetchContent_Declare(json
        URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    FetchContent_MakeAvailable(json)
endif()
find_package(GTest REQUIRED)
include(GoogleTest)
enable_testing()

# The binaries. The IMU, magnetometer, barometer and GPS models are in their own two
# libraries, which the adapters library needs.
set(_lib "${HAKO_SENSOR_PRO_DIST}/lib")
add_library(hakoniwa_drone_pro_sensor_models STATIC IMPORTED)
set_target_properties(hakoniwa_drone_pro_sensor_models PROPERTIES
    IMPORTED_LOCATION "${_lib}/libhakoniwa_imu_gnss_sensor_models.a"
    INTERFACE_LINK_LIBRARIES nlohmann_json::nlohmann_json)
add_library(hakoniwa_drone_pro_sensor_adapters STATIC IMPORTED)
set_target_properties(hakoniwa_drone_pro_sensor_adapters PROPERTIES
    IMPORTED_LOCATION "${_lib}/libhakoniwa_imu_gnss_sensor_adapters.a"
    INTERFACE_LINK_LIBRARIES "hakoniwa_sensor_adapter;hakoniwa_drone_pro_sensor_models")
add_library(hakoniwa_mujoco_sensors_adapters STATIC IMPORTED)
set_target_properties(hakoniwa_mujoco_sensors_adapters PROPERTIES
    IMPORTED_LOCATION "${_lib}/libhakoniwa_mujoco_sensors_adapters.a"
    INTERFACE_INCLUDE_DIRECTORIES "${HAKO_SENSOR_PRO_DIST}/include"
    INTERFACE_LINK_LIBRARIES "hakoniwa_drone_pro_sensor_adapters;hakoniwa_sensor_adapter;nlohmann_json::nlohmann_json")

set(_s "${CMAKE_CURRENT_SOURCE_DIR}/sensors")
set(_config "HAKO_SENSORS_CONFIG_DIR=\"${_s}/config\"")
function(sensor_gtest name)
    add_executable(${name} ${_s}/test/${name}.cpp)
    target_link_libraries(${name} PRIVATE ${ARGN} GTest::gtest_main)
    target_compile_definitions(${name} PRIVATE ${_config})
    gtest_discover_tests(${name})
endfunction()
function(sensor_test name test)
    add_executable(${name} ${_s}/test/${name}.cpp)
    target_link_libraries(${name} PRIVATE ${ARGN})
    target_compile_definitions(${name} PRIVATE ${_config})
    add_test(NAME ${test} COMMAND ${name})
endfunction()

# --- The binary set itself ---
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_test(NAME no_physics_engine
         COMMAND Python3::Interpreter ${CMAKE_CURRENT_SOURCE_DIR}/binary/no_physics_engine.py ${HAKO_SENSOR_PRO_DIST})

# --- The sensor models behind the public interface ---
foreach(name test_rangefinder test_lidar test_depth_camera test_rgb_camera
             test_force_torque test_joint_sensor test_implementation_descriptor)
    sensor_gtest(${name} hakoniwa_mujoco_sensors_adapters)
endforeach()
foreach(name test_force_torque_spec test_joint_spec)
    add_executable(${name} public/${name}.cpp)
    target_link_libraries(${name} PRIVATE hakoniwa_mujoco_sensors_adapters GTest::gtest_main)
    target_compile_definitions(${name} PRIVATE ${_config})
    gtest_discover_tests(${name})
endforeach()
target_compile_definitions(test_implementation_descriptor PRIVATE HAKO_SENSORS_WITH_DRONE_PRO=1)
foreach(sensor rangefinder lidar depth_camera rgb_camera)
    sensor_test(${sensor}_adapter_test ${sensor}_adapter hakoniwa_mujoco_sensors_adapters)
endforeach()

# --- IMU, magnetometer, barometer and GNSS ---
# These tests use the model classes directly, through the public headers in include/hakoniwa/imu_gnss_sensors.
set(_d "${CMAKE_CURRENT_SOURCE_DIR}/imu-gnss")
set_property(TARGET hakoniwa_drone_pro_sensor_models APPEND PROPERTY
    INTERFACE_INCLUDE_DIRECTORIES "${HAKO_SENSOR_PRO_DIST}/include")
set_property(TARGET hakoniwa_drone_pro_sensor_models APPEND PROPERTY
    INTERFACE_COMPILE_DEFINITIONS HAKO_ENABLE_SENSOR_MODEL_V2)
add_executable(test_sensor_models ${_d}/src/test/test_sensor_models.cpp)
target_link_libraries(test_sensor_models PRIVATE hakoniwa_drone_pro_sensor_models GTest::gtest_main)
target_compile_definitions(test_sensor_models PRIVATE HAKO_TEST_PROJECT_ROOT="${_d}")
gtest_discover_tests(test_sensor_models)
foreach(name test_hakoniwa_imu_sensor_adapter test_hakoniwa_mag_baro_gps_sensor_adapters)
    add_executable(${name} ${_d}/sensors/test/${name}.cpp)
    target_link_libraries(${name} PRIVATE hakoniwa_drone_pro_sensor_adapters hakoniwa_drone_pro_sensor_models)
    target_compile_definitions(${name} PRIVATE HAKO_SENSORS_CONFIG_DIR="${_d}/sensors/config")
    string(REPLACE "test_" "" _test ${name})
    add_test(NAME ${_test} COMMAND ${name})
endforeach()
add_executable(test_imu_gnss_spec public/test_imu_gnss_spec.cpp)
target_link_libraries(test_imu_gnss_spec PRIVATE
    hakoniwa_drone_pro_sensor_adapters hakoniwa_drone_pro_sensor_models GTest::gtest_main)
target_compile_definitions(test_imu_gnss_spec PRIVATE HAKO_SENSORS_CONFIG_DIR="${_d}/sensors/config")
gtest_discover_tests(test_imu_gnss_spec)

# --- The sensors on a MuJoCo model (rays, forces, joints, rendering) ---
set(HAKO_MUJOCO_ROOT "" CACHE PATH "MuJoCo release directory (include/, lib/) the binaries were built with")
set(HAKO_MUJOCO_LIBRARY "" CACHE FILEPATH "MuJoCo library")
if(HAKO_MUJOCO_ROOT AND HAKO_MUJOCO_LIBRARY)
    add_library(hakoniwa_mujoco_sensors_mujoco STATIC IMPORTED)
    set_target_properties(hakoniwa_mujoco_sensors_mujoco PROPERTIES
        IMPORTED_LOCATION "${_lib}/libhakoniwa_mujoco_sensors_mujoco.a"
        INTERFACE_INCLUDE_DIRECTORIES "${HAKO_SENSOR_PRO_DIST}/include;${HAKO_MUJOCO_ROOT}/include"
        INTERFACE_LINK_LIBRARIES "hakoniwa_sensor_adapter;${HAKO_MUJOCO_LIBRARY}")
    if(APPLE)
        set_property(TARGET hakoniwa_mujoco_sensors_mujoco APPEND PROPERTY
            INTERFACE_LINK_OPTIONS "-Wl,-rpath,${HAKO_MUJOCO_ROOT}")
    endif()
    find_package(OpenGL REQUIRED)
    find_package(glfw3 3.3 REQUIRED)
    add_library(hakoniwa_mujoco_sensors_renderer STATIC IMPORTED)
    set_target_properties(hakoniwa_mujoco_sensors_renderer PROPERTIES
        IMPORTED_LOCATION "${_lib}/libhakoniwa_mujoco_sensors_renderer.a"
        INTERFACE_LINK_LIBRARIES "hakoniwa_mujoco_sensors_mujoco;OpenGL::GL;glfw")

    sensor_gtest(test_mujoco_ray_caster hakoniwa_mujoco_sensors_mujoco)
    sensor_gtest(test_mujoco_mount hakoniwa_mujoco_sensors_renderer)
    target_compile_definitions(test_mujoco_mount PRIVATE HAKO_TEST_WITH_RENDERER)
    foreach(name test_mujoco_rangefinder test_mujoco_lidar test_mujoco_depth_camera
                 test_mujoco_force_torque test_mujoco_joint_sensor)
        sensor_gtest(${name} hakoniwa_mujoco_sensors_adapters hakoniwa_mujoco_sensors_mujoco)
    endforeach()
    sensor_test(mujoco_rangefinder_test mujoco_rangefinder hakoniwa_mujoco_sensors_adapters hakoniwa_mujoco_sensors_mujoco)
    sensor_test(mujoco_lidar_test mujoco_lidar hakoniwa_mujoco_sensors_adapters hakoniwa_mujoco_sensors_mujoco)
    sensor_test(mujoco_depth_camera_test mujoco_depth_camera hakoniwa_mujoco_sensors_adapters hakoniwa_mujoco_sensors_mujoco)
    sensor_gtest(test_mujoco_image_renderer hakoniwa_mujoco_sensors_renderer)
    sensor_gtest(test_mujoco_depth_render hakoniwa_mujoco_sensors_adapters hakoniwa_mujoco_sensors_renderer)
    sensor_test(mujoco_rgb_camera_test mujoco_rgb_camera hakoniwa_mujoco_sensors_adapters hakoniwa_mujoco_sensors_renderer)
endif()
