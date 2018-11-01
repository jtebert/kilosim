/*
    KiloSim

    Visualizer for the Kilobot simulator to display robots and light pattern
    (OpenGL-based)

    Created 2018-11 by Julia Ebert
*/

#ifndef __KILOSIM_VIEWER_H
#define __KILOSIM_VIEWER_H

#include "KiloSim.h"
#include "robot.h"

namespace KiloSim
{
class Viewer
{
  protected:
    // Pointer to the World that this viewer draws
    World *m_world;

  public:
    // Create a viewer with the pointer to the given world
    Viewer(World *world);
    // Draw everything in the world at the current state
    void draw();

  protected:
    // Draw a single robot onto the scene
    void drawRobot(Robot *robot);
    // Draw the world's background image
    void drawLightPattern();
};
} // namespace KiloSim

#endif