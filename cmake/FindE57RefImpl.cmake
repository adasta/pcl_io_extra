# $Id: FindE57RefImpl.cmake   $

if (E57RefImpl_FIND_VERSION)
    message(WARNING "Finding a specific version of E57RefImpl is not supported.")
endif (E57RefImpl_FIND_VERSION)


find_package(Xerces)

if(Xerces_FOUND)

	find_path(E57RefImpl_ROOT
			NAMES E57RefImplConfig.cmake
			PATHS /usr/local/e57 /usr/E57RefImpl
		)


	if (E57RefImpl_ROOT)
	set(E57RefImpl_LIBRARY_DIRS "${E57RefImpl_ROOT}/lib" 
								${Xerces_LIBRARY_DIRS})
	set(E57RefImpl_INCLUDE_DIRS "${E57RefImpl_ROOT}/include" 
								${Xerces_INCLUDE_DIRS})
	set(E57RefImpl_LIBRARIES E57RefImpl ${Xerces_LIBRARIES} ${Xerces_LIBRARY} )
	endif()


	find_package_handle_standard_args(E57RefImpl  DEFAULT_MSG
									E57RefImpl_LIBRARY_DIRS 
									E57RefImpl_INCLUDE_DIRS 
									E57RefImpl_LIBRARIES)
else()

message(STATUS "Cannot use E57.  Xerces was not found")
endif()
