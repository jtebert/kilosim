#include "Kilobot.h"
#include "Logger.h"
#include "Viewer.h"
#include "ConfigParser.h"
#include "Kilobot.cpp"

#include <unistd.h>

std::vector<double> mean_colors(std::vector<KiloSim::Robot *> &robots)
{
    // Get the mean color for all 3 LED color components
    std::vector<double> means(3, 0.0);
    for (int c = 0; c < 3; c++)
    {
        int sum_belief = 0;
        for (auto &robot : robots)
        {
            // Downcast to custom class to gain access to custom variables
            KiloSim::MyKilobot *kb = (KiloSim::MyKilobot *)robot;
            sum_belief += kb->light_intensity;
        }
        means[c] = (double)sum_belief / 255 / robots.size();
    }
    return means;
}

int main(int argc, char *argv[])
{
    // Create parser to manage configuration
    KiloSim::ConfigParser *config = new KiloSim::ConfigParser("exampleConfig.json");
    json trial_dur = config->get("trial_duration");
    // std::cout << trial_dur << std::endl;
    // std::cout << trial_dur.get<int>() << std::endl;

    // Create world
    KiloSim::World *world = new KiloSim::World(1200.0, 1200.0, "test-bg.png");

    // Create robot(s)
    int numRobots = 10;
    std::vector<KiloSim::Robot *> robots;
    robots.resize(numRobots);
    for (int n = 0; n < numRobots; n++)
    {
        // std::cout << n * 50 + 20 << std::endl;
        robots[n] = new KiloSim::MyKilobot();
        robots[n]->robot_init(n * 80 + 75, 600, PI * n / 2);
        world->add_robot(robots[n]);
    }

    // Create Logger
    KiloSim::Logger *logger = new KiloSim::Logger(world, "test.h5", 1);
    logger->add_aggregator("mean_led_colors", mean_colors);
    // logger->log_params({{"test", 100.25}});
    logger->log_config(config);

    // Create Viewer to visualize the world
    //KiloSim::Viewer *viewer = new KiloSim::Viewer(world);

    double sim_duration = 30; // Seconds
    while (world->get_time() < sim_duration)
    {
        // Run a simulation step
        // This automatically increments the tick
        world->step();

        // Draw the world
        //viewer->draw();

        // std::cout << world->get_light(1100, 1000) << std::endl;

        if ((world->get_tick() % (5 * world->get_tick_rate())) == 0)
        {
            // Log the state of the world every 30 seconds
            // This works because the tickRate (ticks/sec) must be an integer
            std::cout << "Time: " << world->get_time() << " s" << std::endl;
            logger->log_state();
        }
    }
    std::cout << "Finished simulation" << std::endl;
}
