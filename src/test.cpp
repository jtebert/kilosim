#include "Kilobot.h"
#include "Logger.h"
#include "Viewer.h"
#include "ConfigParser.h"
#include "MyKilobot.cpp"

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
        means[c] = (double)sum_belief / robots.size();
    }
    return means;
}

int main(int argc, char *argv[])
{
    // Create parser to manage configuration
    KiloSim::ConfigParser *config = new KiloSim::ConfigParser("exampleConfig.json");

    uint start_trial = config->get("start_trial");
    uint num_trials = config->get("num_trials");
    double trial_duration = config->get("trial_duration"); // seconds

    for (uint trial = start_trial; trial <= (num_trials + start_trial); trial++)
    {

        // Create world
        KiloSim::World *world = new KiloSim::World(
            config->get("world_width"),
            config->get("world_height"),
            config->get("light_pattern_filename"));

        // Create robot(s)
        int numRobots = config->get("num_robots");
        std::vector<KiloSim::Robot *> robots;
        robots.resize(numRobots);
        for (int n = 0; n < numRobots; n++)
        {
            // std::cout << n * 50 + 20 << std::endl;
            robots[n] = new KiloSim::MyKilobot();
            robots[n]->robot_init(n * 80 + 75, 600, PI * n / 2);
            world->add_robot(robots[n]);
        }

        KiloSim::Logger *logger = new KiloSim::Logger(
            world,
            config->get("log_filename"),
            trial,
            true);
        logger->add_aggregator("mean_led_colors", mean_colors);
        logger->log_config(config);

        // Create Viewer to visualize the world
        // KiloSim::Viewer *viewer = new KiloSim::Viewer(world);

        while (world->get_time() < trial_duration)
        {
            // Run a simulation step
            // This automatically increments the tick
            world->step();

            // Draw the world
            // viewer->draw();

            if ((world->get_tick() % (5 * world->get_tick_rate())) == 0)
            {
                // Log the state of the world every 5 seconds
                // This works because the tickRate (ticks/sec) must be an integer
                std::cout << "Time: " << world->get_time() << " s" << std::endl;
                logger->log_state();
            }
        }
        printf("Completed trial %d\n\n", trial);
    }
    printf("Simulations complete\n\n");
}
