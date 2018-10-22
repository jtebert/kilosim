// Example/pseudocode for used of simulator + experiment code
// Based on enki minimal example
// Right now this won't run because literally none of this is implemented

#include <kilosim/KiloSim.h>
#include "kilobot.cpp"

int main(int argc, char *argv[])
{
    // Create the world
    KiloSim::World world(1200, 1200, "/path/to/img.png");
    // Turn off displaying output visualization
    world.setDisplay(false);

    // Create a robot
    MyKilobot testKilobot = new MyKilobot();
    // TODO: Configuring this robot with parameters
    world.addRobot(testKilobot);

    // Create a logger
    int trialNum = 1;
    KiloSim::Logger logger("/log/file/name.h5", trialNum);
    logger.addAggregate('mean', 'decision');
    logger.addAggregate('count number detecting');

    // Save the simulation parameters
    // This should probably come from config files (YAML?)
    // https://stackoverflow.com/questions/39883433/create-argc-argv-in-the-code
    std::vector<std::string> paramsToLog = {"--dir", "/some_path"};
    logger.logParams(paramsToLog);

    // TODO: Add/set up logger connected to world
    world.addLogger(logger);
    world.logWorldParams(); // Save general parameters like dimensions

    unsigned int sim_duration = 60 * 60; // Trial duration in minutes
    unsigned int tick = 0;

    while (tick < sim_duration * world.tickRate)
    {
        // Run a simulation step (does all updates, sensing, communication,
        // computation based on the MyKilobot controller)
        world.step();

        // Log the state every 2 seconds
        if ((tick % (2 * world.tickRate)) == 0)
        {
            world.logState();
        }

        tick++;
    }
}