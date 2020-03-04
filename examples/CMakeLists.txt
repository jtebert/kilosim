# This is an example of what the CMakeLists.txt file would look like for using
# the static compiled Kilosim library in another project. You will need the
# Kilosim repository downloaded somewhere first.

# Compile your code with:
# mkdir build
# cd build
# cmake ..
# make

cmake_minimum_required (VERSION 3.9)

project(project_name LANGUAGES CXX)

add_subdirectory(/path/to/kilosim build)

add_executable(project_name src/my_code.cpp)
target_link_libraries(project_name PRIVATE kilosim)