cmake_minimum_required (VERSION 3.9)

project (kilosim
  VERSION 0.4.0
  DESCRIPTION "TODO"
  LANGUAGES C CXX
)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

find_package(HDF5 COMPONENTS CXX HL REQUIRED)
find_package(Sanitizers)
find_package(Doxygen)

find_package(SFML 2.5 COMPONENTS graphics QUIET)
if (NOT SFML_FOUND)
  find_package(SFMLold REQUIRED)
endif()

find_package(OpenMP)



add_library(kilosim
  src/ConfigParser.cpp
  src/LightPattern.cpp
  src/Logger.cpp
  src/Robot.cpp
  src/Viewer.cpp
  src/World.cpp
  src/random.cpp
)

target_include_directories(kilosim
PUBLIC
  api
  submodules
  ${HDF5_INCLUDE_DIRS}
)

target_link_libraries(kilosim
PUBLIC
  ${HDF5_LIBRARIES}
  ${HDF5_CXX_HL_LIBRARIES}
  ${HDF5_CXX_LIBRARIES}
  OpenMP::OpenMP_CXX
  sfml-graphics
  sfml-window
  sfml-system
)

target_compile_options(kilosim
PRIVATE
  -g
  -march=native
  -ffast-math
  -Wall
  -Wextra
)

target_compile_features(kilosim PRIVATE cxx_std_11)

install(TARGETS kilosim ARCHIVE DESTINATION lib)



if (DOXYGEN_FOUND)
  add_custom_target(
    kilosim_docs
    COMMAND ${DOXYGEN_EXECUTABLE} Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM
  )
else(DOXYGEN_FOUND)
  message("Doxygen needs to be installed to generate the doxygen documentation")
endif(DOXYGEN_FOUND)



add_executable(kilosim_example EXCLUDE_FROM_ALL examples/test.cpp)
# add_executable(example_viewer examples/example_viewer.cpp)
# add_executable(example_logger examples/example_logger.cpp)

target_include_directories(kilosim_example PRIVATE examples)
target_link_libraries(kilosim_example PUBLIC kilosim)
target_compile_options(kilosim_example PRIVATE -g -march=native -Wall -Wextra)
target_compile_features(kilosim_example PRIVATE cxx_std_11)
install(TARGETS kilosim_example RUNTIME DESTINATION bin OPTIONAL)

# target_include_directories(example_viewer PRIVATE examples)
# target_link_libraries(example_viewer PUBLIC kilosim)
# target_compile_options(example_viewer PRIVATE -g -march=native -Wall -Wextra)
# target_compile_features(example_viewer PRIVATE cxx_std_14)
# install(TARGETS example_viewer RUNTIME DESTINATION ${CMAKE_SOURCE_DIR}/bin)

# target_include_directories(example_logger PRIVATE examples)
# target_link_libraries(example_logger PUBLIC kilosim)
# target_compile_options(example_logger PRIVATE -g -march=native -Wall -Wextra)
# target_compile_features(example_logger PRIVATE cxx_std_14)
# install(TARGETS example_logger RUNTIME DESTINATION ${CMAKE_SOURCE_DIR}/bin)

add_custom_target( examples )
add_dependencies( examples kilosim_example )
