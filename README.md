# KiloSim: Kilobot Simulator

Simulator code: [kilobot-simulator](https://github.com/jtebert/kilobot-simulator)

Experiment scripts: [kilobot-simulator-experiments](https://github.com/jtebert/kilobot-simulator-experiments)

Analysis code: [kilobot-simulator-analysis](https://github.com/jtebert/kilobot-simulator-analysis)

---

KiloSim is designed to be a fast, lightweight, pseudo-physical simulator for Kilobots. Notable benefits include:

- Directly use code written for Kilobots, using the same Kilolib API
- Included Logger to easily to save experiment parameters in log continuous state data
- Includes support for ambient light sensing
- Cross-platform viewer for debugging and recording simulations
- [Coming soon] Parallelization with OpenMP
- [Coming soon] Easy configuration with JSON files to run multiple and varied experiments

## Compiling

#### Dependencies

HDF5 and SFML. Install on Ubuntu-like systems with:

 `sudo apt install libsfml-dev libhdf5-dev`

**TODO: This needs a proper make/cmake file instead of my hacky bash scripts**

*Temporary:*

`./make_test.sh`

`./bin/kilosim`

## Configuration and Parameters

A `ConfigParser` class is provided to load JSON configuration files and pass them to a `Logger`. The user is responsible for passing any configuration parameters to other objects where necessary (for example, specifying the trial number when creating the `Logger`.) However, an example is provided in `exampleConfig.json`.

- [JSON?](https://github.com/nlohmann/json)
- [YAML?](https://github.com/jbeder/yaml-cpp)
- Explain configuration files... when they exist
