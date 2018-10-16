/*
    Kilobot simulator where the simulator is no longer the master

    Created 2018-10 by Julia Ebert
*/

#ifndef __KILOSIM_H
#define __KILOSIM_H

#include <vector>
#include <string>
#include "robot.h"

namespace KiloSim
{
class World
{
  public:
    // Height of the arena in mm
    const double arenaWidth;
    // Width of the arena in mm
    const double arenaHeight;
    // Background light pattern image (as a 2D vector)
    std::vector<std::vector<u_int8_t>> lightPattern;
    // Whether or not to display the scene
    bool showScene;

  protected:
    // Compute the next positions of the robots form positions and motor commands
    double *computeNextStep(double *newPos, double dt);
    // Check to see if motion causes robots to collide
    int findCollisions(double *newPos, int selfID, int time);
    // Draw the scene
    void draw_scene(void);

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
    void addRobot(Robot *r);
    // Remove a robot from the world and destroy it. If not in the world, do nothing
    void removeRobot(Robot *r);
};
} // namespace KiloSim

#endif