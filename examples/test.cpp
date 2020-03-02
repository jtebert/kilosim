#include <MyKilobot.h>

#include <kilosim/ConfigParser.h>
#include <kilosim/Kilobot.h>
#include <kilosim/Logger.h>
#include <kilosim/Random.h>
#include <kilosim/Timer.h>
#include <kilosim/Viewer.h>

std::vector<double> mean_colors(std::vector<Kilosim::Robot *> &robots)
{
    // Get the mean color for all 3 LED color components
    std::vector<double> means(3, 0.0);
    for (int c = 0; c < 3; c++)
    {
        int sum_belief = 0;
        for (auto &robot : robots)
        {
            // Downcast to custom class to gain access to custom variables
            Kilosim::MyKilobot *kb = (Kilosim::MyKilobot *)robot;
            sum_belief += kb->light_intensity;
        }
        means[c] = (double)sum_belief / robots.size();
    }
    return means;
}

int main(int argc, char *argv[])
{
    Timer timer_overall;
    Timer timer_step;
    timer_overall.start();

    // Create parser to manage configuration
    // Get config file name
    std::vector<std::string> args(argv, argv + argc);
    if (args.size() < 2)
    {
        std::cout << "ERROR: You must provide a config file name" << std::endl;
        exit(1);
    }
    Kilosim::ConfigParser config(args[1]);

    seed_rand(config.get("seed"));

    uint start_trial = config.get("start_trial");
    uint num_trials = config.get("num_trials");
    double trial_duration = config.get("trial_duration"); // seconds
    uint log_freq = config.get("log_freq");

    for (uint trial = start_trial; trial < (num_trials + start_trial); trial++)
    {
        // Create world
        Kilosim::World world(
            config.get("world_width"),
            config.get("world_height"),
            config.get("light_pattern_filename"),
            config.get("num_threads"));

        // Create robot(s)
        // Creates a grid of 23x23 robots (can handle up to 529 robots)
        // That's the most that will fit into a 2.4x2.4 m arena with this spacing
        int num_rows = 23;
        int num_robots = config.get("num_robots");
        std::vector<Kilosim::Robot *> robots;
        robots.resize(num_robots);
        for (int n = 0; n < num_robots; n++)
        {
            // std::cout << n * 50 + 20 << std::endl;
            robots[n] = new Kilosim::MyKilobot();
            world.add_robot(robots[n]);
            robots[n]->robot_init(floor(n / num_rows) * 100 + 75, (n % num_rows) * 100 + 75, PI * n / 2);
        }

        world.check_validity();

        Kilosim::Logger logger(
            world,
            config.get("log_filename"),
            trial,
            true);
        logger.add_aggregator("mean_led_colors", mean_colors);
        logger.log_config(config);

        // Create Viewer to visualize the world
        Kilosim::Viewer viewer(world);

        int step_count = 0;
        while (world.get_time() < trial_duration)
        {
            step_count++;

            // Run a simulation step
            // This automatically increments the tick
            timer_step.start();
            world.step();
            timer_step.stop();

            // Draw the world
            viewer.draw();

            if ((world.get_tick() % (log_freq * world.get_tick_rate())) == 0)
            {
                // Log the state of the world every 5 seconds
                // This works because the tickRate (ticks/sec) must be an integer
                // std::cout << "Time: " << world.get_time() << " s" << std::endl;
                logger.log_state();
            }
        }

        world.printTimes();
        for (int n = 0; n < num_robots; n++)
            delete robots[n];

        printf("Completed trial %d\n\n", trial);
        std::cerr << "m Steps taken = " << step_count << std::endl;
    }
    printf("Simulations complete\n\n");

    std::cerr << "t Step    = " << timer_step.accumulated() << " s" << std::endl;
    std::cerr << "t Overall = " << timer_overall.stop() << " s" << std::endl;
    return 0;
}
