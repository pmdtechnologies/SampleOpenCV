cmake_minimum_required (VERSION 3.21)

set (royale_DIR CACHE PATH "Path to the share folder inside the Royale package")

find_package (OpenCV QUIET)

if (NOT OpenCV_FOUND)
    message ("OpenCV example will not be build as no OpenCV was found!")
    return ()
endif ()

project (sampleOpenCV)

set_target_properties (${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)
set_target_properties (${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_MINSIZEREL RELEASE)

find_package (royale REQUIRED)

link_directories (${royale_LIB_DIR})

include_directories (${royale_INCLUDE_DIRS} .)

add_executable (sampleOpenCV sampleOpenCV.cpp)

if (WIN32)
    if (OpenCV_STATIC)
        target_compile_options (sampleOpenCV PRIVATE "/MT$<$<CONFIG:Debug>:d>")
    endif ()
endif ()

copy_royale_libs (sampleOpenCV)

target_link_libraries (sampleOpenCV "${royale_LIBS}" "${OpenCV_LIBS}")

add_custom_command(TARGET sampleOpenCV POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:sampleOpenCV> $<TARGET_FILE_DIR:sampleOpenCV>
  COMMAND_EXPAND_LISTS
  )