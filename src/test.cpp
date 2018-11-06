#include "KiloSim.h"
#include "logger.h"
#include "viewer.h"
#include "kilobot.cpp"

#include <unistd.h>

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

    // Create world
    KiloSim::World *world = new KiloSim::World(2400.0, 2400.0);
    world->addLogger(logger);

    // Create robot(s)
    int numRobots = 10;
    std::vector<Robot *> robots;
    robots.resize(numRobots);
    for (int n = 0; n < numRobots; n++)
    {
        // std::cout << n * 50 + 20 << std::endl;
        robots[n] = new MyKilobot();
        robots[n]->robot_init(n * 50 + 25, 20, PI / 2 * n);
        world->addRobot(robots[n]);
        // printf("%f, %f (%f)\n", robots[n]->pos[0], robots[n]->pos[1], robots[n]->pos[2]);
    }
    // Robot *robot = new MyKilobot();
    // robot->robot_init(50.0, 50.0, 2.0);

    // Create viewer to visualize the world
    KiloSim::Viewer *viewer = new KiloSim::Viewer(world);

    double sim_duration = 150; // Seconds
    while (world->getTime() < sim_duration)
    {
        // Run a simulation step
        // This automatically increments the tick
        world->step();

        // Draw the world
        viewer->draw();

        if ((world->getTick() % (1 * world->getTickRate())) == 0)
        {
            // Log the state of the world every 2 seconds
            // This works because the tickRate (ticks/sec) must be an integer
            std::cout << "Time: " << world->getTime() << " s" << std::endl;
            world->logState();
        }

        // DEBUG: Delay to see drawing (us)
        usleep(10000);
    }
    std::cout << "Finished" << std::endl;
}