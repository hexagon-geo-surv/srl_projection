# This CMake configuration is based on
# https://github.com/pabloariasal/modern-cmake-sample

cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)
# Adding the C language is needed for CMake 3.0.0 compatibility
project(srl-projection VERSION 1.0.0 LANGUAGES C CXX)



# Options
option(SRL_PROJECTION_BUILD_TESTS "Enable testing" ON)



# Make sure that custom modules are found
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set the build type and warn if other than Release
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()
if (NOT "${CMAKE_BUILD_TYPE}" STREQUAL "Release")
  message(WARNING "CMAKE_BUILD_TYPE set to ${CMAKE_BUILD_TYPE}. Performance may be terrible.")
else()
  message(STATUS "Building with build type '${CMAKE_BUILD_TYPE}'")
endif()

##############################################
# Declare dependencies
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS highgui imgproc core features2d)

##############################################
# Create target and set properties

add_library(${PROJECT_NAME} INTERFACE)

# Add an alias so that the library can be used inside the build tree, e.g. when
# testing
add_library(SRL::Projection ALIAS ${PROJECT_NAME})

# Set target properties
target_include_directories(${PROJECT_NAME} BEFORE
  INTERFACE
		$<INSTALL_INTERFACE:include>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${EIGEN3_INCLUDE_DIR}
		${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  INTERFACE
		${OpenCV_LIBS}
)



##############################################
# Installation instructions

include(GNUInstallDirs)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/SRL)

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# This is required so that the exported target has the name Projection and not
# srl-projection
set_target_properties(${PROJECT_NAME} PROPERTIES EXPORT_NAME Projection)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Export the targets to a script
install(EXPORT ${PROJECT_NAME}-targets
	FILE
		SRLProjectionTargets.cmake
	NAMESPACE
		SRL::
	DESTINATION
		${INSTALL_CONFIGDIR}
)

# Create a ConfigVersion.cmake file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
	${CMAKE_CURRENT_BINARY_DIR}/SRLProjectionConfigVersion.cmake
	VERSION ${PROJECT_VERSION}
	COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/SRLProjectionConfig.cmake.in
	${CMAKE_CURRENT_BINARY_DIR}/SRLProjectionConfig.cmake
	INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

# Install the Config and ConfigVersion files
install(FILES
	${CMAKE_CURRENT_BINARY_DIR}/SRLProjectionConfig.cmake
	${CMAKE_CURRENT_BINARY_DIR}/SRLProjectionConfigVersion.cmake
	DESTINATION ${INSTALL_CONFIGDIR}
)

##############################################
## Exporting from the build tree
export(EXPORT ${PROJECT_NAME}-targets
	FILE ${CMAKE_CURRENT_BINARY_DIR}/SRLProjectionTargets.cmake
	NAMESPACE SRL::
)

# Register package in user's package registry
export(PACKAGE SRLProjection)

##############################################
## Add tests
if(SRL_PROJECTION_BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()

