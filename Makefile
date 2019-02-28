# Declare phony targets
.PHONY: static exec clean

#Flag Index:
# -g            Compile with debug symbols - should always be on, unless using PGI compiler. Does not slow program down.
# -march=native Compile with full use of machine's special CPU instructions (AVX, &c)
# -O3           Use full suite of optimizations
# -Wall         Compile with many code warnings enabled
# -ffast-math   Allow the compiler to reorder mathematics in ways which are not strictly IEEE754 compliant. This often allows for vectorization and, with it, improved performance.
# -fopenmp      Compile with OpenMP parallelism enabled
# -DCHECKSANE   Compile with expensive run-time sanity checks enabled
# -flto         Compilers with Link-Time Optimization. This special mode can squeeze an additional 10% efficiency out of code by optimizing across files. However, it makes debugging harder.

#Flags
CXXFLAGS = -std=c++11 -g -march=native -O3 -I /usr/include/hdf5/serial/ -L /usr/lib/x86_64-linux-gnu/hdf5/serial/ -Wall -ffast-math -fopenmp #TODO: Reenable -flto
LIBS = -lhdf5 -lhdf5_hl_cpp -lhdf5_cpp -lsfml-graphics -lsfml-window -lsfml-system

# Directories
OUTPUT_DIR = bin
SRC_DIR = src
IDIR = include
OBJ_DIR = obj

# Create the subdirectories if they don't exist
$(info$(shell mkdir -p $(OBJ_DIR) $(OUTPUT_DIR)))

SRC_FILES := $(wildcard $(SRC_DIR)/*.cpp)
OBJ_FILES := $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(SRC_FILES))

static: $(OUTPUT_DIR)/libKilosim.a

exec: $(OUTPUT_DIR)/kilosim

clean:
	rm -f $(OUTPUT_DIR)/libKilosim.a $(OUTPUT_DIR)/kilosim $(OBJ_FILES)

# Build object files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) $(LIBS) -c -o $@ $^

# Build executable (not used right now)
$(OUTPUT_DIR)/kilosim: $(OBJ_FILES)
	$(CXX) -o $@ $^ $(CXXFLAGS) $(LIBS)

# Build library
$(OUTPUT_DIR)/libKilosim.a: $(OBJ_FILES)
	ar rcs $@ $+
