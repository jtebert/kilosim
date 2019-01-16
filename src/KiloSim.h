/*
    Kilobot simulator where the simulator is no longer the master

    Created 2018-10 by Julia Ebert
    Based on Kilobot simulator by Michael Rubenstein
*/

#ifndef __KILOSIM_H
#define __KILOSIM_H

#include <omp.h>
#include <set>
#include <string>
#include <iterator>
#include <memory>
#include <SFML/Graphics.hpp>
#include "Robot.h"
#include "LightPattern.h"

namespace KiloSim
{
class World
{
protected:
  struct RobotPose
  {
    // x, y, and theta (rotation) of a robot
    double x;
    double y;
    double theta;
    RobotPose(double x, double y, double theta);
    RobotPose();
  };

  typedef std::shared_ptr<std::vector<RobotPose>> PosesPtr;

protected:
  //! Robots in the world
  std::vector<Robot *> m_robots;
  //! How many ticks per second in simulation
  uint16_t m_tick_rate = 32;
  //! Current tick of the system (starts at 0)
  uint32_t m_tick = 0;
  //! Duration (seconds) of a tick
  double m_tick_delta_t = 1.0 / m_tick_rate;
  //! Number of ticks between messages (eg, 3 means 10 messages per second)
  uint m_comm_rate = 3;
  //! Height of the arena in mm
  const double m_arena_width;
  //! Width of the arena in mm
  const double m_arena_height;
  //! probability of a controller executing its time step
  const double m_prob_control_execute = .99;
  //! Background light pattern image
  LightPattern m_light_pattern;

protected:
  //! Run the controllers (kilolib) for all robots
  void run_controllers();
  //! Send messages between robots
  void communicate();
  //! Compute the next positions of the robots from positions and motor commands
  PosesPtr compute_next_step();
  //! Check to see if motion causes robots to collide
  std::shared_ptr<std::vector<int16_t>> find_collisions(PosesPtr newPos);
  //! Move the robots based on new positions and collisions
  void move_robots(PosesPtr newPos, std::shared_ptr<std::vector<int16_t>> collisions);
  //! Wrap an angle to be within [0, 2*pi)
  double wrap_angle(double angle);

public:
  /*!
   * Construct a world of a fixed size with the background light pattern
   *
   * @param arena_width Width of the rectangular World/arena in mm
   * @param arena_height Height of the rectangular World/arena in mm
   * @param light_img_src Name of image file of the light pattern to use. Aspect
   * ratio should match the arena, but no specific resolution is mandated. If no
   * light_img_src is provided (empty string), the background will be black.
   * @param num_threads How many threads to parallelize the simulation over. If
   * set to 0 (default), dynamic threading will be used.
   */
  World(double arena_width, double arena_height, std::string light_img_src = "", uint num_threads = 0);
  //! Destructor, destroy all objects within the world
  /*!
   * This does not destroy any Robots that have pointers stored in the world.
   */
  virtual ~World();

  /*!
   * Run a step of the simulator.
   * This runs the controllers, communication, pseudo-physics, and movement
   * for all Robots. It also increments the tick time.
   *
   * This is what you should call in your main function to run the simulation.
   */
  void step();

  /*!
   * Get the current light in the world
   * @return SFML Image showing the visible light in the world
   */
  sf::Image get_light_pattern();

  /*!
   * Check whether the World has a light pattern image set
   * @return Whether LightPattern has an image source
   */
  bool has_light_pattern();

  /*!
   * Set the world's light pattern using the given image file.
   * The image type must be supported by SFML's [Image::loadFromFile](https://www.sfml-dev.org/documentation/2.5.1/classsf_1_1Image.php#a9e4f2aa8e36d0cabde5ed5a4ef80290b)
   *
   * @param set_light_pattern Name and location of the image file to use for light pattern
   */
  void set_light_pattern(std::string light_img_src);

  /*!
   * Add a robot to the world by its pointer.
   * WARNING: It is possible right now to add a Robot twice, so be careful.
   */
  void add_robot(Robot *robot);

  /*!
   * Remove a robot from the world by its pointer
   * WARNING: ...I haven't implemented this yet.
   */
  void remove_robot(Robot *robot);

  /*!
   * Get the tick rate (should be 32)
   * @return Number of simulation ticks per second of real-world (wall clock) time
   */
  uint16_t get_tick_rate();

  /*!
   * Get the current tick of the simulation (only set by simulator)
   * @return Number of ticks since simulation started
   */
  uint32_t get_tick();

  /*!
   * Get the current computed time in seconds (from tick and tickRate)
   * @return Time since simulation started (in seconds)
   */
  double get_time();

  /*!
   * Get a reference to a vector of pointers to all robots in the world
   * This is useful for Logger and Viewer functions
   * @return All the Robots added to the world
   */
  std::vector<Robot *> &get_robots();

  /*!
   * Get the dimensions of the world (in mm)
   * @return 2-element [width, height] vector of dimensions in mm
   */
  std::vector<double> get_dimensions();
};
} // namespace KiloSim

#endif