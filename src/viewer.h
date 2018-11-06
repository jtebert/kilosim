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
#include <SFML/Graphics.hpp>

namespace KiloSim
{
class Viewer
{
protected:
  // Dimensions of the display (in pixels)
  float m_windowWidth = 1200;
  float m_windowHeight = 1200;
  // Pointer to the World that this viewer draws
  World *m_world;
  // SFML window in which the world will be drawn
  sf::RenderWindow m_window;
  // SFML texture containing the image used for background
  sf::Texture m_lightPattern;
  // The background rectangle shape itself
  sf::RectangleShape m_background;
  // Scaling ratio between world and window coordinates
  float m_scale;
  // Texture used for drawing all the robots
  sf::RenderTexture m_robotTexture;
  // Settings for SFML
  sf::ContextSettings m_settings;

public:
  // Create a viewer with the pointer to the given world
  Viewer(World *world);
  // Draw everything in the world at the current state
  void draw();

protected:
  // Draw a single robot onto the scene
  void drawRobot(Robot *robot);
  // Draw the current world time as text in the arena
  void drawTime();
  // Draw the world's background image
  void drawLightPattern();
};
} // namespace KiloSim

#endif