/*
    Kilobot simulator where the simulator is no longer the master

    Created 2018-10 by Julia Ebert
    Based on Kilobot simulator by Michael Rubenstein
*/

#ifndef __KILOSIM_H
#define __KILOSIM_H

#include <set>
#include <string>
#include <iterator>
#include "robot.h"
#include "logger.h"

namespace KiloSim
{
class World
{

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

  struct RobotPose
  {
    // x, y, and theta (rotation) of a robot
    double x;
    double y;
    double theta;
    RobotPose(double x, double y, double theta);
    RobotPose();
  };

protected:
  // Set of (unique) Robots in the world
  std::vector<Robot *> m_robots;
  // How many ticks per second in simulation
  // TODO: Set default
  uint16_t m_tickRate = 30;
  // Current tick of the system (starts at 0)
  uint32_t m_tick = 0;
  // Duration (seconds) of a tick
  double m_tickDeltaT = 1.0 / m_tickRate;
  // Number of ticks between messages (eg, 3 means 10 messages per second)
  uint m_commRate = 3;
  // Height of the arena in mm
  const double m_arenaWidth;
  // Width of the arena in mm
  const double m_arenaHeight;
  // probability of a controller executing its time step
  const double m_pControlExecute = .99;

  // Background light pattern image (as a 2D vector)
  LightPattern m_lightPattern;
  KiloSim::Logger *m_logger = nullptr;

public:
  typedef std::vector<Robot *>::iterator RobotsIterator;
  typedef std::shared_ptr<std::vector<RobotPose>> PosesPtr;

protected:
  // Run the controllers (kilolib) for all robots
  void runControllers();
  // Send messages between robots
  void communicate();
  // Compute the next positions of the robots from positions and motor commands
  PosesPtr computeNextStep();
  // Check to see if motion causes robots to collide
  std::shared_ptr<std::vector<uint8_t>> findCollisions(PosesPtr newPos);
  // Move the robots based on new positions and collisions
  void moveRobots(PosesPtr newPos, std::shared_ptr<std::vector<uint8_t>> collisions);
  // Wrap an angle to in [0, 2*pi)
  double wrapAngle(double angle);
  // Draw the scene
  void drawScene();

public:
  // Construct a world of a fixed size with the background light pattern
  World(double arenaWidth, double arenaHeight, std::string lightImg);
  // Construct a world with no background/light pattern
  World(double arenaWidth, double arenaHeight);
  // Destructor, destroy all objects
  virtual ~World();

  // Run a step of the simulator
  void step();
  // Return whether there is a light pattern
  bool hasLightPattern();
  // Set the light pattern for the world ground surface
  void setLightPattern(std::string lightImg);
  // Add a robot to the world. If the robot is already in the world, do nothing
  void addRobot(Robot *robot);
  // Remove a robot from the world and destroy it. If not in the world, do nothing
  void removeRobot(Robot *robot);
  // Add a Logger object to the World to handle data management
  void addLogger(Logger *lgr);
  // Use the logger to save the current state of the World. Fails if Logger hasn't
  // been added
  void logState();
  // Get/set the tick rate
  void setTickRate(uint16_t newTickRate);
  uint16_t getTickRate();
  // Get the current tick of the simulation (only set by simulator)
  uint32_t getTick();
  // Get the current computed time in seconds (from tick and tickRate)
  double getTime();
  // Return a pointer to the robots in the world
  std::vector<Robot *> &getRobots();
  // Get the dimensions of the world (in mm)
  std::vector<double> getDimensions();
};
} // namespace KiloSim

#endif