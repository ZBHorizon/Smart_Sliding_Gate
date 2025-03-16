add_executable(SlidingGate)

set_target_properties(SlidingGate PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})


# Files #
#----------------------------------------------------------------------------------------------------------------------------------------#
target_sources(SlidingGate PRIVATE "${PROJECT_SOURCE_DIR}/src/main.cpp")


# Linker #
#----------------------------------------------------------------------------------------------------------------------------------------#
target_link_libraries(SlidingGate PRIVATE LibSlidingGate)