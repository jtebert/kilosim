cmake_minimum_required (VERSION 3.9)

project (kilosim
  VERSION 0.0.1
  DESCRIPTION "TODO"
  LANGUAGES C CXX
)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

find_package(HDF5 COMPONENTS CXX HL REQUIRED)
find_package(OpenMP)
find_package(Sanitizers)
find_package(SFML 2.5 COMPONENTS graphics QUIET)

if (NOT TARGET OpenMP::OpenMP_CXX AND APPLE)
    add_library(OpenMP_TARGET INTERFACE)
    add_library(OpenMP::OpenMP_CXX ALIAS OpenMP_TARGET)

    if (ENABLE_MAC_OPENMP)
        message(STATUS "Enabling OpenMP for macOS. NOTE: THIS IS INCOMPATIBLE WITH DISTCC AND PARTLY INCOMPATIBLE WITH CCACHE")
        find_package(Threads REQUIRED)
        set(OpenMP_CXX_FLAGS -Xpreprocessor -fopenmp)
        set(OpenMP_EXE_LINKER_FLAGS -lomp)
        target_compile_options(OpenMP_TARGET INTERFACE ${OpenMP_CXX_FLAGS})

        target_link_libraries(OpenMP_TARGET INTERFACE
            ${OpenMP_CXX_FLAGS}
            ${OpenMP_EXE_LINKER_FLAGS}
            Threads::Threads
        )
    endif()
endif()

if (NOT SFML_FOUND)
  find_package(SFMLold REQUIRED)
endif()


add_library(kilosim
  src/ConfigParser.cpp
  src/LightPattern.cpp
  src/Logger.cpp
  src/MyKilobot.cpp
  src/Robot.cpp
  src/Viewer.cpp
  src/World.cpp
  src/random.cpp
)

target_include_directories(kilosim
PUBLIC
  src
  submodules
  ${HDF5_INCLUDE_DIRS}
)

target_link_libraries(kilosim
PUBLIC
  ${HDF5_LIBRARIES}
  ${HDF5_CXX_HL_LIBRARIES}
  OpenMP::OpenMP_CXX
  sfml-graphics
)

target_compile_options(kilosim
PRIVATE
  -g
  -march=native
  -ffast-math
  -Wall
  -Wextra
)

target_compile_features(kilosim PRIVATE cxx_std_14)

add_executable(kilosim_test
  src/test.cpp
)

target_link_libraries(kilosim_test
PUBLIC
  kilosim
)

# CXXFLAGS = -std=c++11 -g -march=native -O3 -I /usr/include/hdf5/serial/ -L /usr/lib/x86_64-linux-gnu/hdf5/serial/ -Wall -ffast-math -fopenmp #TODO: Reenable -flto
# LIBS = -lhdf5 -lhdf5_hl_cpp -lhdf5_cpp -lsfml-graphics -lsfml-window -lsfml-system

#Flag Index:
# -g            Compile with debug symbols - should always be on, unless using PGI compiler. Does not slow program down.
# -march=native Compile with full use of machine's special CPU instructions (AVX, &c)
# -O3           Use full suite of optimizations
# -Wall         Compile with many code warnings enabled
# -ffast-math   Allow the compiler to reorder mathematics in ways which are not strictly IEEE754 compliant. This often allows for vectorization and, with it, improved performance.
# -fopenmp      Compile with OpenMP parallelism enabled
# -DCHECKSANE   Compile with expensive run-time sanity checks enabled
# -flto         Compilers with Link-Time Optimization. This special mode can squeeze an additional 10% efficiency out of code by optimizing across files. However, it makes debugging harder.
