cd src

g++ Logger.cpp test.cpp KiloSim.cpp Robot.cpp Kilobot.cpp Viewer.cpp ConfigParser.cpp -I /usr/include/hdf5/serial/ -I ../include/ -L /usr/lib/x86_64-linux-gnu/hdf5/serial/ -lhdf5 -lhdf5_hl_cpp -lhdf5_cpp -lsfml-graphics -lsfml-window -lsfml-system -o ../bin/kilosim -std=c++11

cd ..