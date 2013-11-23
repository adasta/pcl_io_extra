  
include(FindPackageHandleStandardArgs)

find_path(PCL_IO_EXTRA_ROOT
			include/pcl/io/cloud_io.h
			/usr/local/include
)

MESSAGE( "IO ROOT IS " ${PCL_IO_EXTRA_ROOT}) 

set(PCL_IO_EXTRA_INCLUDE_DIRS ${PCL_IO_EXTRA_ROOT}/include)
set(PCL_IO_EXTRA_LIBRARY_DIRS  ${PCL_IO_EXTRA_ROOT}/lib)
set(PCL_IO_EXTRA_LIBRARIES   pcl_io_extra )

FIND_PACKAGE(E57RefImpl)

if (E57RefImpl_FOUND)
	message(STATUS "Adding E57 Support")
	set(PCL_IO_EXTRA_INCLUDE_DIRS ${PCL_IO_EXTRA_INCLUDE_DIRS} ${E57RefImpl_INCLUDE_DIRS})
	set( PCL_IO_EXTRA_LIBRARY_DIRS ${PCL_IO_EXTRA_LIBRARY_DIRS} ${E57RefImpl_LIBRARY_DIRS} )
	set(PCL_IO_EXTRA_LIBRARIES  ${PCL_IO_EXTRA_LIBRARIES} pcl_e57 )
	add_definitions(-DE57)
endif()


FIND_PACKAGE(LIBLAS)
if (LIBLAS_FOUND)
	set(PCL_IO_EXTRA_INCLUDE_DIRS ${PCL_IO_EXTRA_INCLUDE_DIRS} ${LIBLAS_INCLUDE_DIRS} )
	set( PCL_IO_EXTRA_LIBRARY_DIRS ${PCL_IO_EXTRA_LIBRARY_DIRS} ${LIBLAS_LIBRARIES} )
	set(PCL_IO_EXTRA_LIBRARIES  ${PCL_IO_EXTRA_LIBRARIES} pcl_las )
 	add_definitions(-DLAS)
endif()

find_package_handle_standard_args(PCL_IO_EXTRA  DEFAULT_MSG
                                   PCL_IO_EXTRA_ROOT  
                                   PCL_IO_EXTRA_INCLUDE_DIRS 
                                   PCL_IO_EXTRA_LIBRARY_DIRS)

