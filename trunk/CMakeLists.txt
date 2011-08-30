cmake_minimum_required ( VERSION 2.6 )


set( CMAKE_VERBOSE OFF CACHE BOOL "Verbose mode" )

if ( CMAKE_VERBOSE )
        set ( CMAKE_VERBOSE_MAKEFILE 1 )
endif( )

if( CMAKE_CACHE_MAJOR_VERSION>=2 AND CMAKE_CACHE_MINOR_VERSION>6 )
	cmake_policy( SET CMP0015 OLD )
endif( )

project ( opencvSfM )
add_definitions( -DSFM_API_EXPORTS )

if( MSVC )
    set( CMAKE_USE_RELATIVE_PATHS ON CACHE INTERNAL "" FORCE )
endif( )
# --------------------------------------------------------------
# Indicate CMake 2.7 and above that we don't want to mix relative
#  and absolute paths in linker lib lists.
# Run "cmake --help-policy CMP0003" for more information.
# --------------------------------------------------------------
if( COMMAND cmake_policy )
    cmake_policy( SET CMP0003 NEW )
endif( )


# ----------------------------------------------------------------------------
#  Current version number:
# ----------------------------------------------------------------------------
set( SfM_lib_VERSION "0.1.0" )

string( REGEX MATCHALL "[ 0-9 ]" SfM_lib_VERSION_PARTS "${SfM_lib_VERSION}" )

list( GET SfM_lib_VERSION_PARTS 0 SfM_lib_VERSION_MAJOR )
list( GET SfM_lib_VERSION_PARTS 1 SfM_lib_VERSION_MINOR )
list( GET SfM_lib_VERSION_PARTS 2 SfM_lib_VERSION_PATCH )
set( SfM_lib_SOVERSION "${SfM_lib_VERSION_MAJOR}.${SfM_lib_VERSION_MINOR}" )

if( WIN32 )
    # Postfix of DLLs:
    set( SfM_lib_DLLVERSION "${SfM_lib_VERSION_MAJOR}${SfM_lib_VERSION_MINOR}${SfM_lib_VERSION_PATCH}" )
    set( SfM_lib_DEBUG_POSTFIX d )
else( )
    # Postfix of so's:
    #set( SfM_lib_DLLVERSION "${SfM_lib_VERSION_MAJOR}${SfM_lib_VERSION_MINOR}${SfM_lib_VERSION_PATCH}" )
    set( SfM_lib_DLLVERSION "" )
    set( SfM_lib_DEBUG_POSTFIX )
endif( )

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules/)#because some "find...cmake" are in CMakeModules folder...

include( ProjectAdd.cmake REQUIRED )

FIND_PACKAGE( OpenCV REQUIRED )
set( SFM_LINKER_LIBS ${OpenCV_LIBS} )


# Unfortunately boost puts the version number into the
# actual filename for the libraries, so this variable will certainly be needed
# in the future when new Boost versions are released.
SET( Boost_ADDITIONAL_VERSIONS "1.43" "1.43.0" "1.44" "1.44.0" "1.45" "1.45.0" "1.46.1" "1.46.0" "1.46" "1.47" )

include( CheckIncludeFiles )
check_include_files( "pthread.h" HAVE_PTHREAD_H )

set( BOOST_needed_list filesystem system thread )
if( ${HAVE_PTHREAD_H} )
	find_package( Threads )
	set( SFM_LINKER_LIBS ${SFM_LINKER_LIBS} ${CMAKE_THREAD_LIBS_INIT} )
	set( BOOST_needed_list ${BOOST_needed_list} thread )
endif( )

	
find_package( Boost COMPONENTS ${BOOST_needed_list} )
if( Boost_FOUND )
	add_definitions( -DHAVE_BOOST )
	LINK_DIRECTORIES( ${Boost_LIBRARY_DIRS} )
	include_directories( ${Boost_INCLUDE_DIR} )
else( )
	find_path( BOOST_ROOT "boost"
	DOC "The path to Boost root directory ( before boost_1_XX_X )" )
	message( STATUS " ** Warning: Boost library Could NOT be found, you will not be able to process automatically a folder full of images /!\\" )
endif( )


add_definitions("-DEIGEN2_SUPPORT")

find_package( PCL 1.1 )
set( SFM_LINKER_LIBS ${SFM_LINKER_LIBS} ${PCL_LIBRARIES} )
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

find_package( Libmv COMPONENTS multiview numeric )
if( LIBMV_FOUND )
	include_directories( ${LIBMV_INCLUDE_DIRS} )
	link_directories( ${LIBMV_LIBRARY_DIR} )
	#as we don't want all libraries, we don't use ${LIBMV_LIBRARIES}
	if( WIN32 )
	SET ( LIBMV_LIB debug "multiview_d" "numeric_d" optimized "multiview" "numeric" )
	else( )
	SET ( LIBMV_LIB "libmultiview.a" "libnumeric.a" "libtools.a" "libV3D.a" "libldl.a" "libcolamd.a" "libglog.a" "libgflags.a" "libgtest.a" )
	endif( )
	SET( SFM_LINKER_LIBS ${SFM_LINKER_LIBS} ${LIBMV_LIB})
else( )
  find_path( LIBMV_ROOT camera/camera.h
	PATHS "/usr/local/include/" "/opt/include/" "/usr/include/"
	PATH_SUFFIXES "inlude/libmv/"
	DOC "The path to libmv's directory" )
endif( )


SET ( BLAS_NAME_LIB "blas" )
SET ( CLAPACK_NAME_LIB "lapack" )

#sba library (first dependencies):
find_library( CLAPACK_LIBRARIES NAMES ${CLAPACK_NAME_LIB} DOC "The path to CLAPACK library" )
SET( SFM_LINKER_LIBS ${SFM_LINKER_LIBS} ${CLAPACK_LIBRARIES} )

FIND_PACKAGE(F2C REQUIRED)
#not needed. Moreover, seems to have some conflicts on windows systems...
#include_directories( ${F2C_INCLUDE_DIR} )
SET( SFM_LINKER_LIBS ${SFM_LINKER_LIBS} ${F2C_LIBRARY} )

GET_FILENAME_COMPONENT(CLAPACK_DIR ${CLAPACK_LIBRARIES} PATH)

find_library( BLAS_LIBRARIES NAMES ${BLAS_NAME_LIB} HINTS ${CLAPACK_DIR} PATH_SUFFIXES "BLAS/SRC/" "../BLAS/SRC/" DOC "The path to BLAS library" )
SET( SFM_LINKER_LIBS ${SFM_LINKER_LIBS} ${BLAS_LIBRARIES} )

find_path(SBA_INCLUDE_DIR sba.h
    PATH_SUFFIXES
    sba )
include_directories( ${SBA_INCLUDE_DIR} )
find_library( SBA_LIBRARIES NAMES sba DOC "The path to SBA library" )

SET( SFM_LINKER_LIBS ${SFM_LINKER_LIBS} ${SBA_LIBRARIES} )

# Allow the developer to select if Dynamic or Static libraries are built
OPTION ( BUILD_STATIC_LIBS "Build Static Libraries" OFF )
# Set the LIB_TYPE variable to SHARED
SET ( LIB_TYPE SHARED )
IF ( BUILD_STATIC_LIBS )
  # User wants to build Static Libraries, so change the LIB_TYPE variable to CMake keyword 'STATIC'
  SET ( LIB_TYPE STATIC )
ENDIF ( BUILD_STATIC_LIBS )


configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/config_SFM.h.in ${CMAKE_BINARY_DIR}/config_SFM.h IMMEDIATE )
add_subdirectory( src )
add_subdirectory( tutorials )
add_subdirectory( YAML_loader )
add_subdirectory( EuclideanReconstruction )
#add_subdirectory( documents )
