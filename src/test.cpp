#include "logger.h"
#include "KiloSim.h"
#include "kilobot.cpp"

uint8_t num_features = 3;

std::vector<double> mean_beliefs(std::vector<Robot *> &robots)
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
    KiloSim::Logger *logger = new KiloSim::Logger("test.h5", 1);
    logger->addAggregator("mean_belief", mean_beliefs);
    logger->logParams({{"test", 100.25}});

    // Create robot(s)
    Robot *robot = new MyKilobot();
    robot->robot_init(0.0, 0.0, 2.0);

    // Create world
    KiloSim::World *world = new KiloSim::World(1200.0, 1200.0);
    world->addLogger(logger);
    world->addRobot(robot);

    double sim_duration = 15; // Seconds
    while (world->getTime() < sim_duration)
    {
        // Run a simulation step
        // This automatically increments the tick
        world->step();

        // if ((world->getTick() % (2 * world->getTickRate())) == 0)
        // {
        // Log the state of the world every 2 seconds
        // This works because the tickRate (ticks/sec) must be an integer
        std::cout << "TEST " << world->getTime() << std::endl;
        world->logState();
        // }
    }
    std::cout << "Finished" << std::endl;
}