/*
    Kilosim

    Visualizer for the Kilobot simulator to display robots and light pattern
    (OpenGL-based)

    Created 2018-11 by Julia Ebert
*/

#ifndef __KILOSIM_VIEWER_H
#define __KILOSIM_VIEWER_H

#include <kilosim/World.h>
#include <kilosim/Robot.h>

#include <SFML/Graphics.hpp>

#include <memory>

namespace Kilosim
{
/*!
 * The Viewer is used to display a Kilosim World. It is instantiated with a
 * pointer to a World, which will be displayed whenever the #draw method is
 * called. After constructing your Viewer, this is the only method you need to
 * call to use it.
 */
class Viewer
{
private:
  //! Reference to the World that this Viewer draws
  World &m_world;
  //! Width of the display window (in pixels)
  const int m_window_width;
  //! Height of the display window (in pixels)
  int m_window_height;
  //! SFML window in which the world will be drawn
  sf::RenderWindow m_window;
  //! Texture of the World image (will be displayed as background)
  sf::Texture m_bg_texture;
  //! The background rectangle shape itself
  sf::RectangleShape m_background;
  //! Scaling ratio between world and window coordinates
  double m_scale;
  //! Texture used for drawing all the robots
  sf::RenderTexture m_robot_texture;
  //! Settings for SFML
  sf::ContextSettings m_settings;

public:
  /*!
   * Create a Viewer with the pointer to the given world
   *
   * @param world Pointe_r to the World that will be displayed
   * @param window_width Width (in pixels) to draw the display window. Height
   * will be automatically determined from the aspect ratio of the World's
   * dimensions.
   */
  Viewer(World &world, const int window_width = 1080);
  /*!
   * Draw everything in the world at the current state
   *
   * This will display all robots and the light pattern (if set; otherwise
   * black). It is frame rate limited to 144 FPS, which limits the overall rate
   * at which the simulation can run. If the window is closed, the simulation
   * will continue to run but the window will not reopen.
   */
  void draw();

private:
  //! Draw a single robot onto the scene
  void draw_robot(Robot *robot);
  //! Add the current world time to the display
  void draw_time();
};

/*! \example example_viewer.cpp
 * Example usage of a Viewer to display a World in a simulation loop.
 */
} // namespace Kilosim

#endif