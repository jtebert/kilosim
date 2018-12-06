# Started with this: http://www.cs.colby.edu/maxwell/courses/tutorials/maketutor/

#Compiler
CXX = g++

#Flags
CXXFLAGS = -std=c++11 -Ofast -I /usr/include/hdf5/serial/ -L /usr/lib/x86_64-linux-gnu/hdf5/serial/ -fopenmp
LIBS = -lhdf5 -lhdf5_hl_cpp -lhdf5_cpp -lsfml-graphics -lsfml-window -lsfml-system

#Directories
OUTPUT_DIR = bin
SRC_DIR = src
IDIR = include
OBJ_DIR = obj

_DEPS = json.hpp
DEPS = $(patsubst %, $(IDIR)/%, $(_DEPS))
SRC_FILES := $(wildcard $(SRC_DIR)/*.cpp)
OBJ_FILES := $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(SRC_FILES))
# _OBJ = ConfigParser.o KiloSim.o LightPattern.o Logger.o MyKilobot.o Robot.o test.o Viewer.o
# OBJ = $(patsubst %, $(ODIR)/%, $(_OBJ))

CPP_SRC = $(wildcard src/*.cpp)
#OBJ = $(CPP_SRC:.c=.o)

# Build object files

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) $(LIBS) -c -o $@ $^


$(OUTPUT_DIR)/kilosim: $(OBJ_FILES)
	$(CXX) -o $@ $^ $(CXXFLAGS) $(LIBS)

$(OUTPUT_DIR)/libKilosim: $(OBJ_FILES)
	ar rcs $@ $+

static: $(OUTPUT_DIR)/libKilosim

#Declare phony targets
.PHONY: clean

clean:
	rm -f $(OUTPUT_DIR)/libKilosim $(OUTPUT_DIR)/kilosim $(OBJ_FILES)

# $(OUTPUT_DIR)/%.o: $(SRC_DIR)/%.cpp $(SRC_DIR)/%.h
# 	$(CXX) $< $(LIBCXXFLAGS) $@

# # create the library archives
# $(OUTPUT_DIR)/libKilosim.a: $(patsubst %.cpp, $(OUTPUT_DIR)/%.o, $(wildcard *.cpp))
# 	ar rcs $@ $^