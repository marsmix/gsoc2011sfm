
macro(begin_new_project name)

	project(SfM_${name})
	add_definitions(-DCVAPI_EXPORTS)

	include_directories("${CMAKE_CURRENT_SOURCE_DIR}"
	                    "${CMAKE_CURRENT_BINARY_DIR}"
	                    "${CMAKE_BINARY_DIR}")

	file(GLOB files_srcs "*.cpp")
	file(GLOB files_int_hdrs "*.h*")
	source_group("Src" FILES ${files_srcs})
	source_group("Include" FILES ${files_int_hdrs} ${CMAKE_BINARY_DIR}/config.h)

	#file(GLOB bin_hdrs "include/${name}/*.h*")
	#source_group("Include" FILES ${bin_hdrs})

	set(the_target "SfM_${name}")

set(file_list_ ${files_srcs} ${files_int_hdrs} ${CMAKE_BINARY_DIR}/config_SFM.h)
endmacro()

macro(end_new_project name)

	# For dynamic link numbering convenions
	set_target_properties(${the_target} PROPERTIES
	    VERSION ${SfM_lib_VERSION}
	    SOVERSION ${SfM_lib_VERSION}
	    OUTPUT_NAME "${the_target}${SfM_lib_DLLVERSION}"
	    )
	# Additional target properties
	set_target_properties(${the_target} PROPERTIES
	    DEBUG_POSTFIX "${SfM_lib_DEBUG_POSTFIX}"
	    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/"
	    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/"
	    INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
	    )

	if(MSVC)
	    if(CMAKE_CROSSCOMPILING)
	        set_target_properties(${the_target} PROPERTIES
	            LINK_FLAGS "/NODEFAULTLIB:secchk"
	            )
	    endif()
	    set_target_properties(${the_target} PROPERTIES
	        LINK_FLAGS "/NODEFAULTLIB:libc;libcmt"
	        )
	endif()

	# Dependencies of this target:
	if(ARGN)
		add_dependencies(${the_target} ${ARGN})
	endif()
	
	# Add the required libraries for linking:
	target_link_libraries(${the_target} ${SFM_LINKER_LIBS} ${ARGN})

	install(TARGETS ${the_target}
	    RUNTIME DESTINATION bin COMPONENT main
	    LIBRARY DESTINATION lib COMPONENT main
	    ARCHIVE DESTINATION lib COMPONENT main)

	install(FILES ${files_int_hdrs}
	    DESTINATION include/${name}
	    COMPONENT main)

endmacro()


macro(new_library name)

	begin_new_project(${name})
	
	add_library(${the_target} ${LIB_TYPE} ${file_list_})
	SET_TARGET_PROPERTIES (${the_target} PROPERTIES DEFINE_SYMBOL "SFM_API_EXPORTS")
	
	end_new_project(${name} ${ARGN})
endmacro()


macro(new_executable name)

	begin_new_project(${name})
	
	add_executable	(${the_target} ${file_list_})
	
	end_new_project(${name} ${ARGN})
	
endmacro()
