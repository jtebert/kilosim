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
#include <memory>

namespace KiloSim
{
class Viewer
{
protected:
  // Dimensions of the display (in pixels)
  int m_window_width;
  int m_window_height;
  // Pointer to the World that this viewer draws
  World *m_world;
  // SFML window in which the world will be drawn
  sf::RenderWindow m_window;
  // Texture of the World image
  sf::Texture m_bg_texture;
  // The background rectangle shape itself
  sf::RectangleShape m_background;
  // Scaling ratio between world and window coordinates
  float m_scale;
  // Texture used for drawing all the robots
  sf::RenderTexture m_robot_texture;
  // Settings for SFML
  sf::ContextSettings m_settings;

public:
  // Create a viewer with the pointer to the given world
  Viewer(World *world, int window_width = 1080);
  // Draw everything in the world at the current state
  void draw();

protected:
  // Draw a single robot onto the scene
  void draw_robot(Robot *robot);
  // Draw the current world time as text in the arena
  void draw_time();
  // Draw the world's background image
  void draw_light_pattern();
};
} // namespace KiloSim

#endif