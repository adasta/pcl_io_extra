cmake_minimum_required (VERSION 2.6)
project(pcl_io_extra)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_LIST_DIR}/cmake /usr/local/E57RefImpl)

SET(CMAKE_BUILD_TYPE "DEBUG")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
link_directories(${CMAKE_BINARY_DIR}/lib)

find_package(PCL 1.6 REQUIRED COMPONENTS  io )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

FIND_PACKAGE( Boost 1.40 COMPONENTS 
					program_options filesystem thread
					serialization python 
					REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
include_directories( include)

FIND_PACKAGE(LIBLAS)
include_directories(${LIBLAS_INCLUDE_DIRS})

#FIX E57Cmake !!!
#FIND_PACKAGE(E57RefImpl)
find_package(Xerces)
set( E57RefImpl_INCLUDE_DIRS /usr/local/e57/include ${Xerces_INCLUDE_DIRS})
include_directories(${E57RefImpl_INCLUDE_DIRS})
set( E57RefImpl_LIBRARY_DIRS /usr/local/e57/lib ${Xerces_LIBRARY_DIRS})

link_directories(${E57RefImpl_LIBRARY_DIRS})

set( E57RefImpl_LIBRARIES E57RefImpl xerces-c)


add_library( pcl_e57 src/e57_io.cpp)
target_link_libraries(pcl_e57 ${Boost_LIBRARIES}  ${PCL_LIBRARIES} ${E57RefImpl_LIBRARIES})


add_library(pcl_io_extra src/cloud_io.cpp)
target_link_libraries(pcl_io_extra ${Boost_LIBRARIES}  ${PCL_LIBRARIES})

add_library(pcl_las src/las_io.cpp)
target_link_libraries(pcl_las ${Boost_LIBRARIES}  ${PCL_LIBRARIES} ${LIBLAS_LIBRARIES})

add_executable(pcl_las2pcd tools/pcl_las2pcd.cpp)
target_link_libraries(pcl_las2pcd pcl_las ${Boost_LIBRARIES}  ${PCL_LIBRARIES} ${LIBLAS_LIBRARIES})


add_executable(cvt2pcd tools/cvt2pcd.cpp)
target_link_libraries(cvt2pcd pcl_las pcl_e57 pcl_io_extra ${Boost_LIBRARIES}  ${PCL_LIBRARIES} ${LIBLAS_LIBRARIES})
