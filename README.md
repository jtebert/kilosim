# KiloSim: Kilobot Simulator

Simulator code: [kilobot-simulator](https://github.com/jtebert/kilobot-simulator)

Experiment scripts: [kilobot-simulator-experiments](https://github.com/jtebert/kilobot-simulator-experiments)

Analysis code: [kilobot-simulator-analysis](https://github.com/jtebert/kilobot-simulator-analysis)

---

## Compiling

**TODO: This needs a proper make/cmake file instead of my hacky bash scripts**

*Temporary:*

`./make_test.sh`

`./bin/kilosim`

## Usage

- General/minimal structure
- Separation of kilobot/simulator code (incl. World class)
- Use the standard functions [Kilolib API](https://www.kilobotics.com/docs/index.html)

## Configuration and Parameters

A `ConfigParser` class is provided to load JSON configuration files and pass them to a `Logger`. The user is responsible for passing any configuration parameters to other objects where necessary (for example, specifying the trial number when creating the `Logger`.) However, an example is provided in `exampleConfig.json`.

- [JSON?](https://github.com/nlohmann/json)
- [YAML?](https://github.com/jbeder/yaml-cpp)
- Explain configuration files... when they exist

## Logger

A `Logger` is used to save [HDF5](https://portal.hdfgroup.org/display/support) files containing parameters and continuous state information for multiple simulation trials.

State information is logged through aggregator functions, which reduce the state of the robots to a vector. This could be a single average value over all the robots (e.g., mean observed ambient light) all the way to saving a value for every robot (e.g., each robot's ambient light value as an element). Each aggregator is saved as an array, where each row is the output of the aggregator function. Whenever the state is logged, the simulator time is also saved as a timeseries.

The resulting file structure would look as follows:

```
logFile.h5
|__ trial_1  (group)
|   |__ time (dataset)  [1 x t]
|   |__ params (group)
|   |   |__ param1 (dataset)
|   |   |__ param2 (dataset)
|   |   |__ ...
|   |__ aggregator_1 (dataset)  [n x t]
|   |__ aggregator_2 (dataset)  [m x t]
|   |__ ...
|__ trial_2
|   |__ time (dataset)  [1 x t]
|   |__ params
|   |   |__ ...
|   |__ ...
...
```

where `t` is the number of time steps when data was logged, and `aggregator_1` and `aggregator_2` were specified by the user.

Example use of an aggregator:

**TODO: Double-check API and add log_params() to example**

```C++
#include "KiloSim.h"
#include "logger.h"

std::vector<double> meanAmbientLightFunc(std::vector<Robot *> & robots)
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

KiloSim::World *world = new KiloSim::World();

// Create a logger that will save to logFilename.h5, in the group trial_1
KiloSim::Logger *logger = new KiloSim::Logger("logFilename.h5", 1);
// The values output by meanAmbientLightFunc will be saved in a dataset
// named meanAmbientLight
logger->add_aggregator("meanAmbientLight", meanAmbientLightFunc);

world->add_logger(logger);

int tick = 0;
while (tick < 100) {
    world->step();
    // Save the outputs of all of the aggregator functions in the world's logger
    world->log_state();
    tick++;
}
```

#### Notes

- If a specified log file does not exist, it will be created. If a group already exists for the specified trial, it will be overwritten. (This is supposed to be changed in the future.)

## Viewer

A `Viewer` displays the contents of a `World`. To use it after creation, you only need to call its `draw()` method.

A minimal example:

```C++
#include "KiloSim.h"
#include "viewer.h"

KiloSim::World *world = new KiloSim::World();
// Construct a Viewer with a pointer to the World you want to draw
KiloSim::Viewer *viewer = new KiloSim::Viewer(world);

int tick = 0;
while (tick < 100) {
    world->step();
    // Draw the current state of the world, up to 144 Hz
    viewer->draw();
    tick++;
}
```

If the viewer is closed, it will not reopen but the simulation will continue to run.

#### Notes

- The Viewer is intended for visualization and debugging only. Do *not* use it when running your real simulations.
- The Viewer is limited 144 frames per second, to prevent it from taking over your computer resources. If you add a call to `viewer.draw()` at every tick, this will come into play.