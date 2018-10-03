# Kilobot Simulator

Simulator code: [kilobot-simulator](https://github.com/jtebert/kilobot-simulator)

Experiment scripts: [kilobot-simulator-experiments](https://github.com/jtebert/kilobot-simulator-experiments)

Analysis code: [kilobot-simulator-analysis](https://github.com/jtebert/kilobot-simulator-analysis)

[Google Doc with info about project](https://docs.google.com/a/harvard.edu/document/d/1zJmLnW7tsqjjS_dx3_AGzUepn_Eh_S9kSxVouFO6sZI/edit?usp=sharing)

## Usage

**Note:** I have only used this on Linux and cannot vouch for whether it will work on Mac or Windows (particularly file creation).

- Install necessary packages (on Ubuntu): `sudo apt install libglew-dev freeglut3-dev`
- Compile: `./build_sh`
- Compile and run: `./build_run.sh`
- Run: `./bin/simulation`
- Run multiple trials: `./run_multi.sh` The number of trials is editable as a variable in the script (default = 10)

All methods of running the simulation take in the parameters described below.

Other scripts for comparing multiple conditions are found in `simulation-scripts\` and should be called from within this subdirectory.

## Logging

3 log files are generated for each simulation run:

- `simulation-#.log`: Logs information at each time step. What is saved to the log files can be changed by changing the input to `log_str(std::string str)` in main.cpp. This function is currently called at every simulation step in `run_simulation_step()`.
- `communication-#.log`: Logs communication that occurred whenever a belief update occurs. This is logged in `update_pattern_beliefs()` in `kilobot.cpp`.
- `params-#.log`: Logs parameters of the simulation (such as communication distance and fill ratios) once at the beginning of the simulation. The full list of saved parameters is in `save_params()` in  `main.cpp`. (Its format is meant to be easily parsable by Matlab.) This is designed for saving global variables declared in `vars.h` and is called after any parameters are modified by user arguments.

*Log files can be overwritten by re-running a simulation with the same filename parameters and trial number, but you will be warned before data is overwritten.*


## Input parameters

*Additional parameters can be added by editing the start of the `main` function in `main.cpp`.*

### General Parameters

- `--num_threads=0` uses dynamic teams. Recommended: set to number of CPU cores (*not* number of hyperthreads). *Note: If using `--draw y`, I recommend using 1 thread, or the display is choppy. I don't totally get why.* (default = `4`)
- `--trial`: [int] Trial number. Appended to the end of the simulation log filename. Useful for multi-trial runs. (default = `0`)
- `--robots`: [int] Number of robots in the arena (default = `120`)
- `--time`: [int] (s) Duration of the simulation (default = `180*60`)
- `--draw`: [y/n] Whether to draw the simulation. Turning off drawing speeds up simulations by ~10%. (default = `y`)
- `--seed`: [int] Seed for random kilobot behavior. (default = random, I think) *Note: This was already in here and I don't exactly understand what it does.*

### Communication

- `--allow_retransmit`: [y/n] Whether to allow robots to re-transmit messages in their memory. (If true, will alternate between sending own information and info from a random neighbor in their memory) (default = `n`)
- `--comm_dist`: [float] (mm) Maximum distance over which kilobots can communicate (default = `96`)
- `--neighbor_dur`: [int] (s) Time that messages (feature estimates) from neighbors will be stored in memory (default = `120`)

### Logging

- `--log`: [y/n] Whether to save logs of the simulation (default = `y`)
- `--logname`: [string] Base name of the log file to save to (trial number will be automatically appended) (default = `simulation-`) 
- `--logdir`: [string] Subdirectory to store the log file. Directories will be created automatically *only for one level deep.* (Otherwise you'll need to pre-create the parent directories) (default = `logs`)

### Arena and Environment

- `--width`: [int] (mm) Arena width (default = `2400`)
- `--height`: [int] (mm) Arena height (default = `2400`)
- `--rects_files`: [string] Base filename for where to get rectangles for arena. Arena rows and fill ratios are automatically appended. If no file is specified, no rectangles will be imported. *Note: Right now, the directory shapes_dir is always "shapes", and the shapes files are imported as rectangles by `gen_color_rects.py`.* (default = ``)
- `--circles_files`: [string] Base filename for where to get circles for arena. Fill ratio is automatically appended. If no file is specified, no circles will be imported. *Note: Right now, the directory shapes_dir is always "shapes", and the shapes files are imported as rectangles by `gen_color_circles.py`.* (default = ``)
- `--circles_radius`: [int] Radius of circles for arena. (This is part of the filename.) In case of circles with random radius, this is the mean. (default = `32`)
- `--polys_files`: [string] Base filename for where to get polygons for arena. If no file is specified, no polygons will be imported. *Note: Right now, there isn't any script that will generate polygon files.* (default = ``)
- `--features`: List of indices of which of the 3 features to use, separated by dashes. (default = `0-1-2`)
- `-r`: [float] (0-1) Ratio of filled red color in the arena. Determines which shapes file is used. (default = `0.3`)
- `-g`: [float] (0-1) Ratio of filled green color in the arena. Determines which shapes file is used. (default = `0.2`)
- `-b`: [float] (0-1) Ratio of filled blue color in the arena. Determines which shapes file is used. (default = `0.8`)
- `--rows`: [int] How many rows of colored blocks to use in the arena. This determines which shapes file is used. *Note: currently a square is assumed, so number of rows = number of columns.* (default = `10`)

### Observation and Dissemination

- `--observation_dur`: [int] (s) Mean observation/exploration duration. Actual value may be drawn from exponential distribution (depending on ) with mean = observation_dur (depending on `--exp_observation`) (default = `60`)
- `--exp_observation`: [y/n] Whether to use an exponential distribution for observation duration (y) or constant duration (n) (default = `y`)
- `--dissemination_dur`: [int] (s) Mean dissemination duration. Actual duration may be drawn from exponential distribution with mean = dissemination_dur * confidence (depending on `--exp_dissemination` and `--use_confidence`). (default = `60`)
- `--exp_dissemination`: [y/n] Whether to use an exponential distribution for dissemination duration (y) or constant duration (n) (default = `y`)
- `--use_confidence`: [y/n] Whether to use the robot's confidence in the feature estimate to determine the dissemination duration (default = `y`)
- `--belief_update_strategy`: [string] (none, DMMD, DMVD) Which strategy to use for belief updates. If `none`, beliefs are skipped and feature estimates alone are transmitted and used for decision-making. [Internally, this is stored as an integer: `none=0`, `DMMD=1`, `DMVD=2`] (default = `DMMD`)

### Decision-making:

- `--diffusion_constant`: [float] Influence of new beliefs on own decision/belief concentration (0-1)  (default = `0.1`)
- `--diffusion_decision_thresh`: [float] Difference from 0/1 at which to lock in feature (default = `0.1`)
- `--diffusion_decision_time`: [uint32_t] (s) Time spent past threshold before locking decision for feature (default = `30` s)

### Agent allocation:

- `--dynamic_allocation`: [y/n] Whether or not agents can change which feature they are detecting (default = `n`)
- `--initial_distribution`: List of ratios of features to use, separated by dashes. Values for used features must sum to at least 1 (uses accumulated values to distribute probability, ignoring values for features not used). This *must* be three elements long. If a feature is not used (from `--features`), set its distribution value to 0. [Example: if `--features 0-2`, then `--initial_distribution 0.9-0-0.1` is valid] (default = uniform distribution over used features)
- `--feature_switch_when`: [string] (decision, observation) Whether to switch feature being detected after making a decision on current feature (default) or for every observation. [Internally, this is stored as an integer: `decision=0`, `observation=1`] (default = `decision`)
- `--feature_switch_to`: [string] (hardest, easiest) Switch to feature with concentration closest to 0.5 (hardest) or furthest from 0.5 (easist). [Internally, this is stored as an integer: `hardest=0`, `easiest=1`, `random=2`] (default = `hardest`)

### Feature types:

- `--which_feature_set`: [mono, color] Use either color features (RGB) or monochrome features (lightness, curvature, pattern). [Internally, this is stored as an integer: `mono=0`, `color=1`] (default = `color`)
