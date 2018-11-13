#include "KiloSim.h"
#include "logger.h"
#include "viewer.h"
#include "kilobot.cpp"

#include <unistd.h>

uint8_t num_features = 3;

std::vector<double> mean_red(std::vector<Robot *> &robots)
{
    // Get the mean belief for all features
    std::vector<double> means(num_features, 0.0);
    for (int feature = 0; feature < num_features; feature++)
    {
        int sum_belief = 0;
        for (auto &robot : robots)
        {
            sum_belief += robot->color[0];
        }
        means[feature] = (double)sum_belief / 255 / robots.size();
    }
    return means;
}

int main(int argc, char *argv[])
{
    // Create world
    KiloSim::World *world = new KiloSim::World(1200.0, 1200.0, "test-bg.png");

    // Create robot(s)
    int numRobots = 10;
    std::vector<Robot *> robots;
    robots.resize(numRobots);
    for (int n = 0; n < numRobots; n++)
    {
        // std::cout << n * 50 + 20 << std::endl;
        robots[n] = new MyKilobot();
        robots[n]->robot_init(n * 80 + 75, 600, PI * n / 2);
        world->add_robot(robots[n]);
    }

    // Create logger
    KiloSim::Logger *logger = new KiloSim::Logger(world, "test.h5", 1);
    logger->add_aggregator("mean_red_led", mean_red);
    logger->log_params({{"test", 100.25}});

    // Create viewer to visualize the world
    KiloSim::Viewer *viewer = new KiloSim::Viewer(world);

    double sim_duration = 300; // Seconds
    while (world->get_time() < sim_duration)
    {
        // Run a simulation step
        // This automatically increments the tick
        world->step();

        // Draw the world
        viewer->draw();

        // std::cout << world->get_light(1100, 1000) << std::endl;

        if ((world->get_tick() % (30 * world->get_tick_rate())) == 0)
        {
            // Log the state of the world every 30 seconds
            // This works because the tickRate (ticks/sec) must be an integer
            std::cout << "Time: " << world->get_time() << " s" << std::endl;
            logger->log_state();
        }
    }
    std::cout << "Finished simulation" << std::endl;
}