# KiloSim: Kilobot Simulator

This is maintained on GitHub in the [kilosim repository](https://github.com/jtebert/kilosim). You can [submit any issues or questions here](https://github.com/jtebert/kilosim/issues).

[__Full Documentation__](https://jtebert.github.io/kilosim/index.html)

---

KiloSim is designed to be a fast, lightweight, pseudo-physical* simulator for Kilobots. Notable benefits include:

- Pseudo-physical model means fast simulation for high-throughput
- Directly use code written for Kilobots, using the same Kilolib API
- Included Logger to easily to save experiment parameters in log continuous state data
- Includes support for ambient light sensing
- Cross-platform Viewer for debugging and recording simulations
- Parallelized with OpenMP (run a 100-minute, 800 robot simulation in about 10 seconds)
- Easy configuration with JSON files to run multiple and varied experiments

*\* Pseudo-physical means that it is spatial and handles issues like collisions in a functional but hand-wavy manner. We make no attempt to accurately model true physical interactions. If you want to see what this means, run an example simulation with the Viewer.*

## Installing and Using

### Dependencies

[HDF5](https://portal.hdfgroup.org/display/HDF5/HDF5), [SFML](https://www.sfml-dev.org/index.php), and [OpenMP](https://www.openmp.org/). Install on Ubuntu-like systems with: `sudo apt install libsfml-dev libhdf5-dev libomp-dev`

[nlohmann/json](https://github.com/nlohmann/json): Add [`json.hpp`](https://raw.githubusercontent.com/nlohmann/json/develop/single_include/nlohmann/json.hpp) to the `include/` folder.

### Compile

**TODO: This needs a proper make/cmake file instead of my hacky bash scripts**

`./build.sh`

### Run

`./bin/kilosim`

### Build Documentation

(Requires Doxygen to be installed.)

`doxygen`

This will automatically use the settings in `Doxyfile` and save the results to `docs/`. (The latest full documentation is automatically generated and published [here](https://jtebert.github.io/kilosim/index.html).)

## Configuration and Parameters

Configuration files are defined as JSON files and can be loaded with a ConfigParser. The contents of flat JSON files can be automatically saved with your data using the Logger. (Support for saving JSON objects and arrays to HDF5 may be added in the future if someone needs/wants it.)

There are no fixed requirements for the contents of the configuration files; it's an un-opinionated convenience tool for importing and using whatever (atomic) parameters you want.

## License

**TBD**

This shouldn't be publicly used or shared until a license has been set for this software.