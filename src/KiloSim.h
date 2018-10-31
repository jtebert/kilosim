/*
    Kilobot simulator where the simulator is no longer the master

    Created 2018-10 by Julia Ebert
    Based on Kilobot simulator by Michael Rubenstein
*/

#ifndef __KILOSIM_H
#define __KILOSIM_H

#include <vector>
#include <string>
#include "robot.h"
#include "logger.h"

namespace KiloSim
{
class World
{

protected:
  std::vector<Robot> robots;

public:
  // How many ticks per second in simulation
  // TODO: Set default
  uint16_t tickRate = 30;
  // Current tick of the system (starts at 0)
  uint32_t tick = 0;
  // Height of the arena in mm
  const double arenaWidth;
  // Width of the arena in mm
  const double arenaHeight;
  // Whether or not to display the scene
  bool showScene;

  // 2D image pattern for light
  struct LightPattern
  {
    // Width of the light image (pixels)
    uint16_t width;
    // Height of the light image (pixels)
    uint16_t height;
    // Data of the light pattern (8-bit brayscale 0-255)
    std::vector<uint8_t> data;
    // Build a pattern from an existing reference
    //LightPattern(uint16_t width, uint16_t height, const uint32_t &data);
  };

  // Background light pattern image (as a 2D vector)
  LightPattern lightPattern;

  KiloSim::Logger *logger = nullptr;

protected:
  // Compute the next positions of the robots from positions and motor commands
  double *computeNextStep(double *newPos, double dt);
  // Check to see if motion causes robots to collide
  int findCollisions(double *newPos, int selfID, int time);
  // Draw the scene
  void drawScene();

public:
  // Construct a world of a fixed size with the background light pattern
  World(double arenaWidth, double arenaHeight, std::string lightImg);
  // Construct a world with no background/light pattern
  World(double arenaWidth, double arenaHeight);
  // Destructor, destroy all objects
  virtual ~World();

  // Return whether there is a light pattern
  bool hasLightPattern();
  // Set the light pattern for the world ground surface
  void setLightPattern(std::string lightImg);
  // Add a robot to the world. If the robot is already in the world, do nothing
  void addRobot(Robot &robot);
  // Remove a robot from the world and destroy it. If not in the world, do nothing
  void removeRobot(Robot &robot);
  // Add a Logger object to the World to handle data management
  void addLogger(Logger *lgr);
  // Use the logger to save the current state of the World. Fails if Logger hasn't
  // been added
  void logState();
};
} // namespace KiloSim

#endif