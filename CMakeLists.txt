cmake_minimum_required(VERSION 3.20)

project(voxel_mesher
        DESCRIPTION "voxel structure based surface mesher from a 3D point cloud"
        LANGUAGES C CXX)
message(STATUS "Buiding: ${PROJECT_NAME}")


# cmake -B build/ -DCMAKE_TOOLCHAIN_FILE:PATH=C:/DevTools/vcpkg/scripts/buildsystems/vcpkg.cmake -DCMAKE_BUILD_TYPE=Release (Windows)
# cmake -B build/ -DCMAKE_TOOLCHAIN_FILE:PATH=/home/xzhao/workdir/vcpkg/scripts/buildsystems/vcpkg.cmake -DCMAKE_BUILD_TYPE=Release (Linux)
# cmake --build build/ --config Release

# Let's nicely support folders in IDEs
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# set debug level
if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
	message(STATUS "Linux debug build: System: ${CMAKE_SYSTEM_NAME}. BuildType: ${CMAKE_BUILD_TYPE}")
	if(${CMAKE_BUILD_TYPE} MATCHES "Debug")	# gcc debug
		set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Og -g3 -ggdb -Wall -Wextra")
		set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g3 -ggdb -Wall -Wextra")
		# -- PCL_COMPILE_OPTIONS="-msse4.2;-mfpmath=sse;-march=native"
	else()	# gcc optimization
		# -- PCL_COMPILE_OPTIONS="-msse4.2;-mfpmath=sse;-march=native" : x86 Options
		set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3")
		set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
	endif()
else()
	message(STATUS "System: ${CMAKE_SYSTEM_NAME}. BuildType: ${CMAKE_BUILD_TYPE}")	
endif()


set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads)  # If not installed, use: sudo apt-get install
find_package(OpenMP)   # some PCL and OpenGR functions use OMP

## find PCL
find_package(PCL CONFIG REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## find boost
find_package(Boost REQUIRED)
IF (Boost_FOUND)
  message(STATUS "Boost found: ${Boost_INCLUDE_DIR}")
  INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
  ADD_DEFINITIONS( "-DHAS_BOOST" )
ENDIF()

## find CGAL
find_package(CGAL REQUIRED)
if(CGAL_FOUND)
  message(STATUS "CGAL found: ${CGAL_INCLUDE_DIRS}")
  message(STATUS "CGAL_DEFINITIONS: ${CGAL_DEFINITIONS}")
  include_directories(${CGAL_INCLUDE_DIRS})
  add_definitions(${CGAL_DEFINITIONS})
endif()



add_executable(${PROJECT_NAME} "${CMAKE_SOURCE_DIR}/src/main.cpp")

# subdirectories
add_subdirectory("${CMAKE_SOURCE_DIR}/src")