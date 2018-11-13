cd src

g++ logger.cpp test.cpp KiloSim.cpp robot.cpp kilobot.cpp viewer.cpp -I /usr/include/hdf5/serial/ -L /usr/lib/x86_64-linux-gnu/hdf5/serial/ -lhdf5 -lhdf5_hl_cpp -lhdf5_cpp -lGL -lGLU -lglut -lsfml-graphics -lsfml-window -lsfml-system -o ../bin/kilosim -std=c++11

cd ..