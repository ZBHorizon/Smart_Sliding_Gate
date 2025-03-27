add_executable(SlidingGate)

set_target_properties(SlidingGate PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})

# Includes #
#----------------------------------------------------------------------------------------------------------------------------------------#
target_include_directories(SlidingGate PRIVATE 
    ${PROJECT_SOURCE_DIR}/include 
    ${TARGET_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/Build/_deps/wiringpi-src/wiringPi
)
# Files #
#----------------------------------------------------------------------------------------------------------------------------------------#
target_sources(SlidingGate PRIVATE "${PROJECT_SOURCE_DIR}/src/main.cpp" "${PROJECT_SOURCE_DIR}/src/wiringPi_IO.cpp")

# Linker #
#----------------------------------------------------------------------------------------------------------------------------------------#
target_link_libraries(SlidingGate PRIVATE LibSlidingGate wiringPi)