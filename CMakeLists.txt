cmake_minimum_required (VERSION 3.9)

project (kilosim
  VERSION 0.4.1
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

# ------------------------------------------------------------------------------



# if (DOXYGEN_FOUND)
#   add_custom_target(
#     docs
#     COMMAND ${DOXYGEN_EXECUTABLE} Doxyfile
#     WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
#     COMMENT "Generating API documentation with Doxygen"
#     VERBATIM
#   )
# else(DOXYGEN_FOUND)
#   message("Doxygen needs to be installed to generate the doxygen documentation")
# endif(DOXYGEN_FOUND)

# ------------------------------------------------------------------------------

add_executable(kilosim_example examples/test.cpp)
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

# ------------------------------------------------------------------------------

# Building documentation with Sphinx (uses cmake/FindSphinx.cmake)

find_package(Doxygen)
find_package(Sphinx)

# Find all the public headers
get_target_property(KILOSIM_PUBLIC_HEADER_DIR kilosim INTERFACE_INCLUDE_DIRECTORIES)
file(GLOB_RECURSE KILOSIM_PUBLIC_HEADERS ${KILOSIM_PUBLIC_HEADER_DIR}/*.h)

# set(DOXYGEN_INPUT_DIR "${PROJECT_SOURCE_DIR}/src ${PROJECT_SOURCE_DIR}/api/kilosim")
set(DOXYGEN_INPUT_DIR "${PROJECT_SOURCE_DIR}/api/kilosim")
set(DOXYGEN_EXAMPLES_DIR ${PROJECT_SOURCE_DIR}/examples)
set(DOXYGEN_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/docs/doxygen)
set(DOXYGEN_INDEX_FILE ${DOXYGEN_OUTPUT_DIR}/xml/index.xml)
set(DOXYFILE_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
set(DOXYFILE_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

#Replace variables inside @@ with the current values
configure_file(${DOXYFILE_IN} ${DOXYFILE_OUT} @ONLY)

# Doxygen won't create this for us
file(MAKE_DIRECTORY ${DOXYGEN_OUTPUT_DIR})

# Only regenerate Doxygen when the Doxyfile or public headers change
add_custom_command(OUTPUT ${DOXYGEN_INDEX_FILE}
                   DEPENDS ${KILOSIM_PUBLIC_HEADERS}
                   COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_OUT}
                   MAIN_DEPENDENCY ${DOXYFILE_OUT} ${DOXYFILE_IN}
                   COMMENT "Generating Doxygen docs"
                   VERBATIM)

# Nice named target so we can run the job easily
add_custom_target(doxygen DEPENDS ${DOXYGEN_INDEX_FILE})


set(SPHINX_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/docs)
set(SPHINX_BUILD ${CMAKE_CURRENT_BINARY_DIR}/docs/sphinx)
set(SPHINX_INDEX_FILE ${SPHINX_BUILD}/index.html)

# Only regenerate Sphinx when:
# - Doxygen has rerun
# - Our doc files have been updated
# - The Sphinx config has been updated
add_custom_command(OUTPUT ${SPHINX_INDEX_FILE}
                   COMMAND
                     ${SPHINX_EXECUTABLE} -b html
                     # Tell Breathe where to find the Doxygen output
                     -Dbreathe_projects.Kilosim=${DOXYGEN_OUTPUT_DIR}/xml
                   ${SPHINX_SOURCE} ${SPHINX_BUILD}
                   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                   DEPENDS
                   # Other docs files you want to track should go here (or in some variable)
                   ${CMAKE_CURRENT_SOURCE_DIR}/docs/index.rst
                   ${DOXYGEN_INDEX_FILE}
                   MAIN_DEPENDENCY ${SPHINX_SOURCE}/conf.py
                   COMMENT "Generating Sphinx documentation")
# add_custom_target(Sphinx ALL
#                   COMMAND ${SPHINX_EXECUTABLE} -b html
#                   # Tell Breathe where to find the Doxygen output
#                   -Dbreathe_projects.Kilosim=${DOXYGEN_OUTPUT_DIR}
#                   ${SPHINX_SOURCE} ${SPHINX_BUILD}
#                   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
#                   COMMENT "Generating documentation with Sphinx")

# Nice named target so we can run the job easily
add_custom_target(docs DEPENDS ${SPHINX_INDEX_FILE})

# Add an install target to install the docs
include(GNUInstallDirs)
install(DIRECTORY ${SPHINX_BUILD}
DESTINATION ${CMAKE_INSTALL_DOCDIR})