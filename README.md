# Kilosim: Kilobot Simulator

![Kilosim logo](docs/logo.svg)

[![Build Status](https://travis-ci.com/jtebert/kilosim.svg?token=s6ZVW1bvfNjgbZQh2x9M&branch=master)](https://travis-ci.com/jtebert/kilosim)
[![GitHub Issues](https://img.shields.io/github/issues/jtebert/kilosim.svg)](https://github.com/jtebert/kilosim/issues)
[![GitHub release](https://img.shields.io/github/release-pre/jtebert/kilosim?color=yellow)](https://github.com/jtebert/kilosim/releases)
[![DOI](https://zenodo.org/badge/151430556.svg)](https://zenodo.org/badge/latestdoi/151430556)


This is maintained on GitHub in the [kilosim repository](https://github.com/jtebert/kilosim). You can [submit any issues or questions here](https://github.com/jtebert/kilosim/issues).

[**Full Documentation**](https://jtebert.github.io/kilosim/index.html)

---

Kilosim is a fast, lightweight, pseudo-physical* C++ simulator for Kilobot robots. Notable benefits include:

- Pseudo-physical model means fast simulation for high-throughput
- Easily re-use code written for Kilobots, using the same Kilolib API
- Includes support for ambient light sensing
- Included `Logger` to easily to save experiment parameters in log continuous state data
- Cross-platform `Viewer` for debugging and recording simulations
- Easy configuration with JSON files to run multiple trials and varied experiments
- *[In progress]* Parallelization with OpenMP

\* *Pseudo-physical means that it is spatial and handles issues like collisions in a functional but hand-wavy manner. We make no attempt to accurately model true physical interactions. If you want to see what this means, run an example simulation with the Viewer.*

Note that this project is currently in a pre-release state and may still contain bugs, incomplete documentation, and missing features. If you have issues using the code, contact the developers or [open an issue](https://github.com/jtebert/kilosim/issues/new).

## Installing and Using

**NOTE:** At this time, we only support Kilosim on Linux. Other operating systems will be supported by a full release.

You can either [clone the repository from GitHub](https://github.com/jtebert/kilosim) or [download the latest release](https://github.com/jtebert/kilosim/releases).

### Dependencies

[HDF5](https://portal.hdfgroup.org/display/HDF5/HDF5), [SFML](https://www.sfml-dev.org/index.php), and [OpenMP](https://www.openmp.org/). Install on Ubuntu-like systems with:

    sudo apt install libsfml-dev libhdf5-dev libomp-dev

### Compile

To compile, copy and paste the following:

    mkdir build  # Create a build directory
    cd build  # Move into build directory
    cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=../ ..  #Configure for your system
    make  # Build the library (equivalent to `make kilosim`)
    make install  # Install the library to that indicated by `CMAKE_INSTALL_PREFIX` above
    cd ..  # Move back to source directory

If you want to generate more than the library, you can alternatively run:

    make kilosim       # Build library
    make examples      # Build examples (optional)
    make kilosim_docs  # Build documentation (optional)
    make install       # Install all the components generated by the above commands

To clean up the build enter the source directory and run:

    rm -rf build bin lib

Flags:

 * `-DSANITIZE_ADDRESS=On`: Use address sanitizer to check memory safety

Use flags like so:

    cmake -DSANITIZE_ADDRESS=ON ..

Compatibility:

 * g++ 5   - doesn't work
 * g++ 7.3 - works
 * g++ 8.1 - fails due to compiler issue(?)
 * g++ 8.3 - works
 * g++ 9.2.1 - works

### Run

In addition to the static library, we provide an executable example to test your installation.

Run the examples generated by `make examples` and `make install`:

    ./bin/kilosim_example examples/exampleConfig.json

### Using static library

If your project uses `cmake`, then incorporating kilosim is easy!

Just add the following lines:

    add_subdirectory(path/to/kilosim)
    target_link_libraries(your_executable_target PRIVATE kilosim)

And everything will just work.

For a more detailed example, see the [Kilosim Demo repository](https://github.com/jtebert/kilosim-demo).

## Configuration and Parameters

Configuration files are defined as JSON files and can be loaded with a ConfigParser. The contents of flat JSON files can be automatically saved with your data using the Logger. (Support for saving JSON objects and arrays to HDF5 may be added in the future if someone needs/wants it.)

There are no fixed requirements for the contents of the configuration files; it's an un-opinionated convenience tool for importing and using whatever (atomic) parameters you want.

## Support

If you are having issues installing or using the simulator, [open an issue](https://github.com/jtebert/kilosim/issues/new) or [email Julia](mailto:julia@juliaebert.com).

### Known Issues

- Fails with GCC 8 (We suspect this is a GCC bug; see [issue #23](https://github.com/jtebert/kilosim/issues/23).) **Workaround:** Change your GCC version.
- Viewer does not work over SSH

## Citing

If you use this software, please cite it as follows:

**DOI (v0.3):** [10.5281/zenodo.3406865](https://doi.org/10.5281/zenodo.3406865)

**DOI (all/latest version):** [10.5281/zenodo.3406864](https://doi.org/10.5281/zenodo.3406864)

**BibTeX:**

```bibtex
@misc{kilosim_3406865,
  author       = {Ebert, Julia and
                  Barnes, Richard},
  title        = {Kilosim},
  month        = mar,
  year         = 20120,
  doi          = {10.5281/zenodo.3406865},
  url          = {https://doi.org/10.5281/zenodo.3406865}
}
```

## Contributing

We welcome contributions to the repository. To find things we're currently looking for help with, [check out the open issues](https://github.com/jtebert/kilosim/issues?utf8=✓&q=is%3Aissue+is%3Aopen+).

If you discover a bug or have an idea, for an improvement, [open an issue](https://github.com/jtebert/kilosim/issues/new).

If you add something you think would be useful to others, submit a pull request. All code should be [fully documented](http://www.doxygen.nl/manual/docblocks.html). You can check your documentation by [building the documentation locally](#build-documentation) Code reviews will be required for all pull requests, but we currently do not perform unit testing.

See a [full list of the contributors and their contributions here]([CONTRIBUTORS.md](https://github.com/jtebert/kilosim/blob/master/CONTRIBUTORS.md)).

### Build Documentation

Build the HTML documentation: (Requires Doxygen to be installed.)

    make kilosim_docs

This will automatically build the documentation and save the results to `docs/`. The latest full documentation is automatically generated and published [here](https://jtebert.github.io/kilosim/index.html).

## License

This software is released under the [MIT License](https://github.com/jtebert/kilosim/blob/master/LICENSE).

If you use this software for a publication, please cite it.
