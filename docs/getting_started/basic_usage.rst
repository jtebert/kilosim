===========
Basic Usage
===========

.. contents:: On this page
  :local:


Creating a simple robot
=======================

For this first example, we'll use the built-in :cpp:class:`Kilosim::Kilobot` robot. Place this in a ``src`` folder within your project's root directory. If you want to use a different robot model (or make your own), see the :ref:`Advanced Usage` section.

To create the code that runs on your robot, you need to create a class that inherits from :cpp:class:`Kilosim::Kilobot`, as shown below. This needs to implement the following functions (based on the Kilobot API):

- :cpp:func:`Kilosim::Kilobot::setup`: Runs once at the beginning of the simulation. (Note: This is run when the robot is initialized, not when it's placed in the World, so you don't yet have access to World-related values like position.)
- :cpp:func:`Kilosim::Kilobot::update`: Runs once per simulation step.
- :cpp:func:`Kilosim::Kilobot::message_rx`: Runs when the robot receiver a message from another robot.
- :cpp:func:`Kilosim::Kilobot::message_tx`: Creates the message that the robot will transmit.
- :cpp:func:`Kilosim::Kilobot::message_tx_success`: Runs when the robot successfully transmits a message.

For more details of the Kilobot API, see the documentation for the :cpp:class:`Kilosim::Kilobot` class and the `Kilobot API <https://kilobotics.com/docs/>`_.

Below is an example of the minimum requirements for your robot class, which you can use as a template for your own robot. You can also download this directory: :download:`minimal_kilobot.py </../examples/example_kilobot.cpp>`

.. literalinclude:: /../examples/example_kilobot.cpp
  :language: C++
  :linenos:

First Simulation
================


Setting up CMake
================


Using the Viewer
================

- Add it to the main function
- Call it once per step (or less frequently)

Configuration and Parameters
============================

Configuration files are defined as JSON files and can be loaded with a ConfigParser. The contents of flat JSON files can be automatically saved with your data using the Logger. (Support for saving JSON objects and arrays to HDF5 may be added in the future if someone needs/wants it.)

There are no fixed requirements for the contents of the configuration files; it's an un-opinionated convenience tool for importing and using whatever (atomic) parameters you want.

Logging
=======

- Create a Logger
- Creating aggregator functions
- Call every couple steps/seconds