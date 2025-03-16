add_library(LibSlidingGate STATIC)

set_target_properties(LibSlidingGate PROPERTIES OUTPUT_NAME sliding-gate)


# Files #
#----------------------------------------------------------------------------------------------------------------------------------------#
include(CMakeFiles.txt)

target_sources(LibSlidingGate PRIVATE
    ${TARGET_ASSETS_FILES} 
    ${TARGET_INCLUDE_FILES} 
    ${TARGET_SOURCE_FILES})


# Includes #
#----------------------------------------------------------------------------------------------------------------------------------------#
target_include_directories(LibSlidingGate PUBLIC    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
target_include_directories(LibSlidingGate INTERFACE $<INSTALL_INTERFACE:include>)


# Compiler #
#----------------------------------------------------------------------------------------------------------------------------------------#
target_compile_features(LibSlidingGate PUBLIC cxx_std_20)


# Linker #
#----------------------------------------------------------------------------------------------------------------------------------------#
target_link_libraries(LibSlidingGate PRIVATE wiringPi)

# Installation #
#----------------------------------------------------------------------------------------------------------------------------------------#
install(TARGETS  LibSlidingGate
        EXPORT   ${PROJECT_EXPORT_NAME}
        ARCHIVE  COMPONENT devel
        LIBRARY  COMPONENT core
        RUNTIME  COMPONENT core)
        