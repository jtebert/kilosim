#include "KiloSim.h"
#include "Logger.h"

std::vector<double> meanAmbientLightFunc(std::vector<Robot *> &robots)
{
    // Example function with the correct contract to be used as an aggregator
    int totalLight = 0;
    for (auto &robot : robots)
    {
        totalLight += robot->light_level;
        // where light_level is defined for your subclass of Robot
    }
    std::vector<double> meanLight(1, (double)total_light / robots.size());
    return meanLight;
}

int main(int argc, char *argv[])
{
    KiloSim::World world(1200.0, 1200.0);

    // Create a Logger that will save to logFilename.h5, in the group trial_1
    KiloSim::Logger logger("logFilename.h5", 1);
    // The values output by meanAmbientLightFunc will be saved in a dataset
    // named meanAmbientLight
    logger.add_aggregator("meanAmbientLight", meanAmbientLightFunc);

    world.add_logger(logger);

    // Run a 10 second simulation
    while (world.get_time() < 10)
    {
        world.step();
        // Save the outputs of all of the aggregator functions in the world's logger
        world.log_state();
    }
    return 0;
}