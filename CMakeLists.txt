cmake_minimum_required(VERSION 3.5.1)
project(finger)
set (CMAKE_CXX_STANDARD 14)
add_compile_options(-fPIC)
add_compile_options(-Wdeprecated-declarations)
SET(CMAKE_BUILD_TYPE Release CACHE STRING
	"Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
#	FORCE
	)

find_package(DART REQUIRED COMPONENTS gui optimizer-ipopt collision-bullet collision-ode CONFIG)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

add_subdirectory(FastMassSpring)

add_compile_options(-DHAVE_CSTDDEF)
include_directories(${DART_INCLUDE_DIRS})
include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${EIGEN_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})
file(GLOB srcs "*.h" "*.cpp")

add_executable(run ${srcs})
target_link_libraries(run ${DART_LIBRARIES} GL GLU glut sim)

