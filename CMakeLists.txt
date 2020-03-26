cmake_minimum_required(VERSION 2.8.11)

project(srl_projection)

# project options
option (BUILD_TESTS "Builds all gtests" ON)

# require Eigen
find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR}) 

# require OpenCV
find_package( OpenCV REQUIRED COMPONENTS highgui imgproc core features2d)
include_directories(BEFORE ${OpenCV_INCLUDE_DIRS})

# build the library
set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -march=native -Wall -pedantic -Wextra -Werror -std=c++11 -fPIC")
include_directories(include/)
add_library(${PROJECT_NAME} STATIC 
  include/srl/projection/ProjectionBase.hpp
  include/srl/projection/implementation/ProjectionBase.hpp
  src/ProjectionBase.cpp
  include/srl/projection/PinholeCamera.hpp
  include/srl/projection/implementation/PinholeCamera.hpp
  include/srl/projection/EquidistantDistortion.hpp
  include/srl/projection/implementation/EquidistantDistortion.hpp
  include/srl/projection/RadialTangentialDistortion.hpp
  include/srl/projection/implementation/RadialTangentialDistortion.hpp
  include/srl/projection/RadialTangentialDistortion8.hpp
  include/srl/projection/implementation/RadialTangentialDistortion8.hpp
  include/srl/projection/OusterLidar.hpp
  include/srl/projection/implementation/OusterLidar.hpp
)

# and link it
target_link_libraries(${PROJECT_NAME}  
  ${OpenCV_LIBS}
)

# testing
if(BUILD_TESTS)
  if(APPLE)
    add_definitions(-DGTEST_HAS_TR1_TUPLE=1)
  else()
    add_definitions(-DGTEST_HAS_TR1_TUPLE=0)
  endif(APPLE)
  enable_testing()
  set(PROJECT_TEST_NAME ${PROJECT_NAME}_test)
  add_executable(${PROJECT_TEST_NAME} 
    test/runTests.cpp
    test/TestPinholeCamera.cpp
    test/TestOusterLidar.cpp
  )
  target_link_libraries(${PROJECT_TEST_NAME} 
    ${PROJECT_NAME} 
    gtest
    ${OpenCV_LIBS}
    pthread)
  add_test(test ${PROJECT_TEST_NAME})
endif()
