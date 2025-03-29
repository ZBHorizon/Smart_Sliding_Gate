include(CMakeFiles.txt)

qt_add_executable(IO_Test ${TARGET_SOURCE_FILES} ${TARGET_ASSETS_FILES})
set_target_properties(IO_Test PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX} WIN32_EXECUTABLE TRUE)

# Setze AUTOUIC_SEARCH_PATHS, damit uic in assets sucht
set_target_properties(IO_Test PROPERTIES AUTOUIC_SEARCH_PATHS "${CMAKE_CURRENT_LIST_DIR}/assets")

# Files #
#----------------------------------------------------------------------------------------------------------------------------------------#
#include(CMakeFiles.txt)

#target_sources(IO_Test PRIVATE ${TARGET_ASSETS_FILES} ${TARGET_SOURCE_FILES})

# Includes #
#----------------------------------------------------------------------------------------------------------------------------------------#
target_include_directories(IO_Test PRIVATE ${TARGET_INCLUDE_DIR})

# Preprocessor #
#----------------------------------------------------------------------------------------------------------------------------------------#
#target_compile_definitions(IO_Test PRIVATE ASSETS_PATH="${CMAKE_CURRENT_LIST_DIR}/Assets")

target_compile_options(IO_Test PRIVATE /Zc:__cplusplus)

# Linker #
#----------------------------------------------------------------------------------------------------------------------------------------#
target_link_libraries(IO_Test PRIVATE LibSlidingGate Qt${QT_VERSION_MAJOR}::Widgets)

#qt_finalize_executable(IO_Test)

if(WIN32)
  # Deploy required Qt DLLs using windeployqt.
  add_custom_command(TARGET IO_Test POST_BUILD
    COMMAND "C:/Qt/6.8.3/msvc2022_64/bin/windeployqt" $<TARGET_FILE:IO_Test>
    COMMENT "Deploying Qt DLLs using windeployqt"
  )
endif()

# Execution #
#----------------------------------------------------------------------------------------------------------------------------------------#
add_test(NAME IO_CTest
         COMMAND  ${CMAKE_CIO_TEST_COMMAND}
                  --build-and-test    "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}"
                  --build-target      IO_Test
                  --build-nocmake
                  --build-run-dir     ${CMAKE_CURRENT_BINARY_DIR}
                  --build-exe-dir     ${CMAKE_CURRENT_BINARY_DIR}
                  --build-generator   ${CMAKE_GENERATOR}
                  --build-project     Smart_Sliding_Gate
                  --build-makeprogram ${CMAKE_MAKE_PROGRAM}
                  --build-noclean
                  --test-command      $<TARGET_FILE:IO_Test>)

# if (CMAKE_SYSTEM_NAME STREQUAL Linux)
#   set_IO_tests_properties(UtilitiesCIO_Test PROPERTIES ENVIRONMENT "LD_LIBRARY_PATH=$<TARGET_FILE_DIR:Utilities>")
# elseif (CMAKE_SYSTEM_NAME STREQUAL Windows)
#   set_IO_tests_properties(UtilitiesCIO_Test PROPERTIES ENVIRONMENT "PATH=$<TARGET_FILE_DIR:Utilities>\\;${TARGET_ENVIRONMENT_PATH}")
# endif()