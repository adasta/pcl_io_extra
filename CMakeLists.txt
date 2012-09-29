cmake_minimum_required (VERSION 2.6)
project(pcl_io_extra)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_LIST_DIR}/cmake /usr/local/E57RefImpl)

set(CMAKE_BUILD_TYPE "RELEASE")

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
					serialization python  system
					REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
include_directories( include)


add_library(pcl_io_extra src/cloud_io.cpp 
						 src/ptx_io.cpp 
						 src/pts_io.cpp
						 src/xyz_io.cpp
						 src/vtx_io.cpp
						 src/xya_io.cpp
						 src/ascii_io.cpp
						 src/cloud_collection.cpp
						 src/double_utils.cpp
						 )
target_link_libraries(pcl_io_extra ${Boost_LIBRARIES}  ${PCL_LIBRARIES})




#FIX E57Cmake !!!
FIND_PACKAGE(E57RefImpl)

if (E57RefImpl_FOUND)
	message(STATUS "Adding E57 Support")
	include_directories(${E57RefImpl_INCLUDE_DIRS})
	link_directories(${E57RefImpl_LIBRARY_DIRS} )
	add_library( pcl_e57 src/e57_io.cpp)
	target_link_libraries(pcl_e57 ${Boost_LIBRARIES}  ${PCL_LIBRARIES} ${E57RefImpl_LIBRARIES} )
	add_definitions(-DE57)
endif()


FIND_PACKAGE(LIBLAS)
if (LIBLAS_FOUND)
	add_library(pcl_las src/las_io.cpp)
	target_link_libraries(pcl_las ${Boost_LIBRARIES}  ${PCL_LIBRARIES} ${LIBLAS_LIBRARIES})
	include_directories(${LIBLAS_INCLUDE_DIRS})
	add_definitions(-DLAS)
endif()

add_executable(cvt2pcd tools/cvt2pcd.cpp)
target_link_libraries(cvt2pcd pcl_io_extra ${Boost_LIBRARIES}  ${PCL_LIBRARIES})

if (E57RefImpl_FOUND)
target_link_libraries(cvt2pcd pcl_e57)
INSTALL(TARGETS pcl_e57  
  ARCHIVE DESTINATION lib
)
endif()

if (LIBLAS_FOUND)
target_link_libraries(cvt2pcd pcl_las ${LIBLAS_LIBRARIES})
INSTALL(TARGETS pcl_las  
  LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
endif()



INSTALL(TARGETS pcl_io_extra cvt2pcd
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

INSTALL(DIRECTORY include/pcl  DESTINATION include)
