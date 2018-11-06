# KiloSim: Kilobot Simulator

Simulator code: [kilobot-simulator](https://github.com/jtebert/kilobot-simulator)

Experiment scripts: [kilobot-simulator-experiments](https://github.com/jtebert/kilobot-simulator-experiments)

Analysis code: [kilobot-simulator-analysis](https://github.com/jtebert/kilobot-simulator-analysis)

## Compiling

**TODO: This needs a proper make/cmake file instead of my hacky bash scripts**

*Temporary:*

`./make_test.sh`

`./bin/kilosim`

## Usage

- General/minimal structure
- Separation of kilobot/simulator code (incl. World class)
- Use the standard functions [Kilolib API](https://www.kilobotics.com/docs/index.html)

## Configuration

- [JSON?](https://github.com/nlohmann/json)
- Explain configuration files... when they exist

## Logger

- HDF5 files (+ reference)
- Note on overwriting
- Explain aggregators (+ addAggregator)
- Main functionality: logParams, world.logState

## Viewer

- How to set up + example
- How to run a step/show
- Note on rate (up to 144 Hz)
- Note: for visualization/debugging only (not large scale simulation; slow)