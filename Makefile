# Declare phony targets
.PHONY: static exec clean

# Compiler
CXX = g++

#Flags
CXXFLAGS = -std=c++11 -Ofast -I /usr/include/hdf5/serial/ -L /usr/lib/x86_64-linux-gnu/hdf5/serial/ -fopenmp
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
