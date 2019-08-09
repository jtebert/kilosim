cmake_minimum_required (VERSION 3.9)

project (kilosim
  VERSION 0.0.1
  DESCRIPTION "TODO"
  LANGUAGES C CXX
)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

find_package(HDF5 COMPONENTS CXX HL REQUIRED)
find_package(Sanitizers)

find_package(SFML 2.5 COMPONENTS graphics QUIET)
if (NOT SFML_FOUND)
  find_package(SFMLold REQUIRED)
endif()

find_package(OpenMP)
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



if(BUILD_EXAMPLES)
add_executable(kilosim_test
  src/test.cpp
)

target_link_libraries(kilosim_test
PUBLIC
  kilosim
)

target_compile_options(kilosim_test
PRIVATE
  -g
  -march=native
  -Wall
  -Wextra
)
endif()
