#----------------------------------------------------------------------------------------------------------------------------------------#
#///////// General //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////#
#----------------------------------------------------------------------------------------------------------------------------------------#
set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Debug;Release" FORCE)

# CMake
cmake_minimum_required(VERSION 3.25)
cmake_policy(SET CMP0048 NEW)

# # Git
# find_package(Git)

# execute_process(COMMAND "${GIT_EXECUTABLE}" branch
#                 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
#                 OUTPUT_VARIABLE GIT_BRANCH
#                 OUTPUT_STRIP_TRAILING_WHITESPACE)
# string(REGEX MATCHALL "\\* ([^\n]+)" GIT_BRANCH ${GIT_BRANCH})
# string(REGEX REPLACE "\\* " "" GIT_BRANCH ${GIT_BRANCH})

# execute_process(COMMAND "${GIT_EXECUTABLE}" describe --tags --long --dirty
#                 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
#                 OUTPUT_VARIABLE GIT_COMMIT
#                 OUTPUT_STRIP_TRAILING_WHITESPACE)

# execute_process(COMMAND "${GIT_EXECUTABLE}" describe --tags --long
#                 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
#                 OUTPUT_VARIABLE GIT_VERSION_TAG
#                 OUTPUT_STRIP_TRAILING_WHITESPACE)
# string(REGEX REPLACE "v([0-9]+.[0-9]+)-([0-9]+)-.+" "\\1.\\2" GIT_VERSION_TAG ${GIT_VERSION_TAG})

# Project
project(Smart_Sliding_Gate
        # VERSION ${GIT_VERSION_TAG}
        HOMEPAGE_URL "https://github.com/ZBHorizon/Smart_Sliding_Gate"
        LANGUAGES CXX)



#----------------------------------------------------------------------------------------------------------------------------------------#
#///////// Flags / Options //////////////////////////////////////////////////////////////////////////////////////////////////////////////#
#----------------------------------------------------------------------------------------------------------------------------------------#
# Flags #
#----------------------------------------------------------------------------------------------------------------------------------------#,
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)

set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_CXX_MODULE_STD ON)

if (CMAKE_SYSTEM_NAME STREQUAL Windows)
  set(CMAKE_AUTOUIC ON)
  set(CMAKE_AUTOMOC ON)
  set(CMAKE_AUTORCC ON)
endif()

# clang-tidy #
#----------------------------------------------------------------------------------------------------------------------------------------#
find_program(CLANG_TIDY "clang-tidy")

if (NOT CLANG_TIDY)
  if (CMAKE_SYSTEM_NAME STREQUAL Windows)
    set(CLANG_TIDY "C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/Llvm/x64/bin/clang-tidy.exe")
  endif()
endif()

if (CLANG_TIDY)
  # set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY};-config-file=${PROJECT_SOURCE_DIR}/.clang-tidy") #--extra-arg=/EHsc
  # set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY};--config-file=${PROJECT_SOURCE_DIR}/.clang-tidy;-p")
endif()


# Miscellaneous #
#----------------------------------------------------------------------------------------------------------------------------------------#
set(CMAKE_BUILD_WITH_INSTALL_RPATH OFF)
set(CMAKE_SKIP_RPATH               OFF)
set(CMAKE_INSTALL_RPATH "$ORIGIN/../../../lib/${CMAKE_LIBRARY_ARCHITECTURE}/noah")

set(CMAKE_DEBUG_POSTFIX d)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)



#----------------------------------------------------------------------------------------------------------------------------------------#
#///////// Modules & Packages ///////////////////////////////////////////////////////////////////////////////////////////////////////////#
# #----------------------------------------------------------------------------------------------------------------------------------------#
# # Catch2 #
# #----------------------------------------------------------------------------------------------------------------------------------------#
# include(FetchContent)

# FetchContent_Declare(
#   Catch2
#   GIT_REPOSITORY https://github.com/catchorg/Catch2.git
#   GIT_TAG        v3.5.4)

# FetchContent_MakeAvailable(Catch2)

# #----------------------------------------------------------------------------------------------------------------------------------------#
# # Qt #
# #----------------------------------------------------------------------------------------------------------------------------------------#
if (CMAKE_SYSTEM_NAME STREQUAL Windows)
  #set(QT_DIR "C:/Qt/6.8.3/mingw_64")
  list(APPEND CMAKE_PREFIX_PATH "C:/Qt/6.8.3/msvc2022_64/lib/cmake")

  if(MSVC) 
  string(REPLACE "/source-charset:.28605" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}") 
  endif()
  
  #set(CMAKE_INCLUDE_PATH "C:/Qt/6.5.2/msvc2019_64/include")
  find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
  find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)
endif()

# #----------------------------------------------------------------------------------------------------------------------------------------#
# # WiringPi #
# #----------------------------------------------------------------------------------------------------------------------------------------#
include(FetchContent)

FetchContent_Declare(
    WiringPi
    GIT_REPOSITORY https://github.com/WiringPi/WiringPi.git
    GIT_TAG master
)

FetchContent_MakeAvailable(WiringPi)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(STATUS "Building and installing WiringPi...")
    
    execute_process(
        COMMAND chmod +x ./build
        WORKING_DIRECTORY ${wiringpi_SOURCE_DIR}
    )

    execute_process(
        COMMAND ./build
        WORKING_DIRECTORY ${wiringpi_SOURCE_DIR}
        RESULT_VARIABLE build_result
    )

    if(NOT build_result EQUAL 0)
        message(FATAL_ERROR "WiringPi installation failed with error code ${build_result}")
    endif()
endif()

# GNUInstallDirs #
#----------------------------------------------------------------------------------------------------------------------------------------#
if (CMAKE_SYSTEM_NAME STREQUAL Linux)
  include(GNUInstallDirs) # In 3.26 still defect for multiarch CMAKE_INSTALL_LIBDIR - https://gitlab.kitware.com/cmake/cmake/-/issues/20565
  set(CMAKE_INSTALL_LIBDIR "lib/${CMAKE_LIBRARY_ARCHITECTURE}/Smart_Sliding_Gate")
endif()


# # OpenAL #
# #----------------------------------------------------------------------------------------------------------------------------------------#
# if (CMAKE_SYSTEM_NAME STREQUAL Android)
#   set(OPENAL_INCLUDE_DIR "$ENV{OPENALDIR}/include/AL")
#   set(OPENAL_LIBRARY     "$ENV{OPENALDIR}/lib/libopenal.so")
# else()
#   find_package(OpenAL) # TODO - 3.25: OpenAL::OpenAL
# endif()

# if (NOT EXISTS ${OPENAL_INCLUDE_DIR})
#   message(WARNING "OpenAL not found")
# endif()


# # SVT-AV1 #
# #----------------------------------------------------------------------------------------------------------------------------------------#
# if (CMAKE_SYSTEM_NAME STREQUAL Linux)
#   find_library(SVT_AV1_Decoder SvtAv1Dec)
#   find_library(SVT_AV1_Encoder SvtAv1Enc)
#   if (SVT_AV1_Decoder)
#     set(SVT_AV1_INCLUDE "/usr/include/svt-av1")
#   endif()
# elseif (CMAKE_SYSTEM_NAME STREQUAL Windows)
#   find_package(SVT_AV1 1.7.0)
# endif()

# if (NOT EXISTS ${SVT_AV1_INCLUDE})
#   message(WARNING "SVT-AV1 not found")
# endif()


# # X11 #
# #----------------------------------------------------------------------------------------------------------------------------------------#
# if (CMAKE_SYSTEM_NAME STREQUAL Linux)
#   find_package(X11 1.7.5 COMPONENTS Xcursor)
#   if (NOT TARGET X11::X11)
#     message(WARNING "X11 not found")
#   endif()
# endif()



#----------------------------------------------------------------------------------------------------------------------------------------#
#///////// Targets //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////#
#----------------------------------------------------------------------------------------------------------------------------------------#
add_subdirectory(src/SlidingGate)

if (CMAKE_SYSTEM_NAME STREQUAL Linux)
  add_subdirectory(src)
endif()

if (CMAKE_SYSTEM_NAME STREQUAL Windows)
  add_subdirectory(test)
endif()


# Documentation #
#----------------------------------------------------------------------------------------------------------------------------------------#
option(BUILD_DOCUMENTATION "Build documentation." OFF)
if (BUILD_DOCUMENTATION)
  # Complete Configuration with CMake Variables
  configure_file("Doxygen/Doxyfile.config" ${PROJECT_BINARY_DIR}/Doxyfile @ONLY)

  add_custom_target(Documentation
    COMMAND ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/Doxyfile
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen")

  install(CODE "execute_process(COMMAND ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/Doxyfile
                                WORKING_DIRECTORY ${PROJECT_BINARY_DIR})")
  install(DIRECTORY ${PROJECT_BINARY_DIR}/documentation/html DESTINATION "documentation")
endif()


# Examples #
#----------------------------------------------------------------------------------------------------------------------------------------#
# option(BUILD_EXAMPLES "Build examples." OFF)
# if (BUILD_EXAMPLES)
#   add_subdirectory("Examples" "examples")
# endif()


# Tests #
#----------------------------------------------------------------------------------------------------------------------------------------#
# option(BUILD_TESTS "Build tests." OFF)
# if (BUILD_TESTS)
#   set_property(GLOBAL PROPERTY CTEST_TARGETS_ADDED 1)
#   enable_testing()

#   add_subdirectory("Tests" "tests")
# endif()



#----------------------------------------------------------------------------------------------------------------------------------------#
#///////// Console //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////#
#----------------------------------------------------------------------------------------------------------------------------------------#
message(STATUS " Toolchain: ${CMAKE_TOOLCHAIN_FILE}"                     )
message(STATUS ""                                                        )
message(STATUS " Summary:"                                               )
message(STATUS "  System Processor: ${CMAKE_SYSTEM_PROCESSOR}"           )
message(STATUS "  System Name:      ${CMAKE_SYSTEM_NAME}"                )
message(STATUS ""                                                        )
message(STATUS "  CMake Version:        ${CMAKE_VERSION}"                )
message(STATUS "  CMake Crosscompiling: ${CMAKE_CROSSCOMPILING}"         )
message(STATUS ""                                                        )
message(STATUS "  Clang-Tidy: ${CLANG_TIDY}"                             )
message(STATUS ""                                                        )
message(STATUS "  Git Branch:      ${GIT_BRANCH}"                        )
message(STATUS "  Git Commit:      ${GIT_COMMIT}"                        )
message(STATUS "  Git Version Tag: ${GIT_VERSION_TAG}"                   )
message(STATUS ""                                                        )
message(STATUS "  Compiler Name:    ${CMAKE_CXX_COMPILER_ID}"            )
message(STATUS "  Compiler Version: ${CMAKE_CXX_COMPILER_VERSION}"       )
message(STATUS ""                                                        )
message(STATUS "  Library Architecture: ${CMAKE_LIBRARY_ARCHITECTURE}"   )
message(STATUS ""                                                        )
message(STATUS "  Install Prefix:            ${CMAKE_INSTALL_PREFIX}"    )
message(STATUS "  Install Include Directory: ${CMAKE_INSTALL_INCLUDEDIR}")
message(STATUS "  Install Library Directory: ${CMAKE_INSTALL_LIBDIR}"    )
message(STATUS ""                                                        )
