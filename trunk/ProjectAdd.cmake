# this is the template for creation of new project
# based on the template for a OpenCV module 
macro(define_new_project name)
	
	project(SfM_${name})
	add_definitions(-DCVAPI_EXPORTS)

	include_directories("${CMAKE_CURRENT_SOURCE_DIR}"
	                    "${CMAKE_CURRENT_BINARY_DIR}"
	                    "${CMAKE_BINARY_DIR}")

	file(GLOB bin_srcs "*.cpp")
	file(GLOB bin_int_hdrs "*.h*")
	source_group("Src" FILES ${bin_srcs})
	source_group("Include" FILES ${bin_int_hdrs} ${CMAKE_BINARY_DIR}/config.h)

	#file(GLOB bin_hdrs "include/${name}/*.h*")
	#source_group("Include" FILES ${bin_hdrs})

	set(the_target "SfM_${name}")

	add_executable(${the_target} ${bin_srcs} ${bin_hdrs} ${bin_int_hdrs} ${CMAKE_BINARY_DIR}/config.h)

	# For dynamic link numbering convenions
	set_target_properties(${the_target} PROPERTIES
	    VERSION ${SfM_lib_VERSION}
	    SOVERSION ${SfM_lib_VERSION}
	    OUTPUT_NAME "${the_target}${SfM_lib_DLLVERSION}"
	    )

	# Additional target properties
	set_target_properties(${the_target} PROPERTIES
	    DEBUG_POSTFIX "${SfM_bin_DEBUG_POSTFIX}"
	    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/"
	    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/"
	    INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
	    )

	# Add the required libraries for linking:
	target_link_libraries(${the_target} ${SFM_LINKER_LIBS} ${ARGN})
	if(MSVC)
	    if(CMAKE_CROSSCOMPILING)
	        set_target_properties(${the_target} PROPERTIES
	            LINK_FLAGS "/NODEFAULTLIB:secchk"
	            )
	    endif()
	    set_target_properties(${the_target} PROPERTIES
	        LINK_FLAGS "/NODEFAULTLIB:libc"
	        )
	endif()

	# Dependencies of this target:
	if(ARGN)
		add_dependencies(${the_target} ${ARGN})
	endif()

	install(TARGETS ${the_target}
	    RUNTIME DESTINATION bin COMPONENT main
	    LIBRARY DESTINATION lib COMPONENT main
	    ARCHIVE DESTINATION lib COMPONENT main)

	install(FILES ${bin_hdrs}
	    DESTINATION include/${name}
	    COMPONENT main)

endmacro()
