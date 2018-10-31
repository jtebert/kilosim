#include "logger.h"
#include "KiloSim.h"

uint8_t num_features = 3;

std::vector<double> mean_beliefs(std::set<Robot *> &robots)
{
    // Get the mean belief for all features
    std::vector<double> means(num_features, 0.0);
    for (int feature = 0; feature < num_features; feature++)
    {
        int sum_belief = 0;
        for (auto &robot : robots)
        {
            sum_belief += robot->pattern_belief[feature];
        }
        means[feature] = (double)sum_belief / 255 / robots.size();
    }
    return means;
}

int main(int argc, char *argv[])
{
    // Create logger
    KiloSim::Logger *logger = new KiloSim::Logger("test.h5", 0);
    logger->addAggregator("mean_belief", mean_beliefs);
    logger->logParams({{"test", 1.5}});

    // Create robot(s)

    // Create world
    KiloSim::World *world = new KiloSim::World(1200.0, 1200.0);
    world->addLogger(logger);
    // world->addRobot(robot);

    world->logState();

    std::cout << "Finished" << std::endl;
}