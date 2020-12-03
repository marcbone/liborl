cmake_minimum_required(VERSION 3.5.1)

project(liborl VERSION 0.3.0)
include(CTest)

add_compile_options(-fPIC)

add_compile_options(-Wall -Wextra)
find_package(Eigen3 REQUIRED)

find_package(Franka REQUIRED)
message("libfranka version: ${Franka_VERSION}")

add_library(orl SHARED
        src/Robot.cpp
        3rdparty/libfranka/examples_common.cpp
        src/Pose.cpp src/geometry_math.cpp include/liborl/geometry_math.h src/PoseGenerator.cpp src/Payload.cpp src/StopCondition.cpp include/liborl/StopCondition.h src/SpeedProfile.cpp include/liborl/SpeedProfile.h include/liborl/Frame.h src/SCurve.cpp include/liborl/SCurve.h include/liborl/enums.h)
set_target_properties(orl PROPERTIES VERSION ${PROJECT_VERSION}
        )
target_include_directories(orl
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/libfranka/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        )

target_link_libraries(orl PUBLIC
        Franka::Franka
        )
include(GNUInstallDirs)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/orl)
install(DIRECTORY include/liborl/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/liborl
        FILES_MATCHING PATTERN "*.h*"
        )

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(TARGETS orl
        EXPORT orlTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
        )
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
        orlConfigVersion.cmake
        VERSION ${PACKAGE_VERSION}
        COMPATIBILITY AnyNewerVersion
)
configure_package_config_file(cmake/orlConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/orlConfig.cmake
        INSTALL_DESTINATION ${INSTALL_CONFIGDIR})
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/orlConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/orlConfigVersion.cmake"
        DESTINATION lib/cmake/orl
        )
install(EXPORT orlTargets
        FILE orlTargets.cmake
        NAMESPACE orl::
        DESTINATION lib/cmake/orl
        )
add_subdirectory(examples)
if (BUILD_TESTING)
    enable_testing()
    add_subdirectory(test)
endif ()
