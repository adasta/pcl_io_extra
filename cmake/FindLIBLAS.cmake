  
include(FindPackageHandleStandardArgs)

find_program(LIBLAS_CONFIG_PROG
			liblas-config
			/usr/local/bin
)

if (LIBLAS_CONFIG_PROG)

exec_program(${LIBLAS_CONFIG_PROG}  ARGS  --libs OUTPUT_VARIABLE
               LIBLAS_LIBRARIES)

set(LIBLAS_LIBRARIES ${LIBLAS_LIBRARIES} las)
exec_program(${LIBLAS_CONFIG_PROG} ARGS --includes OUTPUT_VARIABLE
               LIBLAS_INCLUDE_DIRS)

exec_program(${LIBLAS_CONFIG_PROG} ARGS --defines OUTPUT_VARIABLE
               LIBLAS_DEFINES)

exec_program(${LIBLAS_CONFIG_PROG} ARGS --cxxflags OUTPUT_VARIABLE
               LIBLAS_CXXFLAGS)

else()
	MESSAGE(FATAL_ERROR, "FindLIBAS.cmaek.  liblas-config not found. liblas-config=${LIBLAS_CONFIG_PROG}")
endif(LIBLAS_CONFIG_PROG)


find_package_handle_standard_args(LIBLAS  DEFAULT_MSG
                                   LIBLAS_INCLUDE_DIRS LIBLAS_LIBRARIES)

