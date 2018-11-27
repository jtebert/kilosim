/*
    KiloSim

    Created 2018-11 by Julia Ebert
*/

#ifndef __KILOSIM_LIGHTPATTERN_H
#define __KILOSIM_LIGHTPATTERN_H

#include <string>
#include <SFML/Graphics.hpp>

namespace KiloSim
{
/*!
 * A LightPattern represents the visible light intensity of the World. This
 * makes the simplifying assumption that perceived light intensity from the
 * sensors is linearly proportional to the luminosity. Any non-monochrome images
 * will be converted to grayscale by a luminosity for computing light intensity.
 */
class LightPattern
{
  protected:
    //! Ambient light in the world
    sf::Image m_light_pattern;
    //! Width of the World (in mm), as set at initialization
    const double m_arena_width;
    //! Dimensions of internal image (to minimize recomputation)
    sf::Vector2u m_img_dim;
    //! Scaling between world dimensions/coordinates and image coordinates
    double m_scale;

  public:
    /*!
     * Create a new LightPattern from the given source image file
     *
     * The passed image resolution need not match the size of the arena, but the
     * aspect ratio must match. If not, you will encounter errors when detecting
     * values outside the defined image domain.
     * @param img_src Filename (+location) of the new light source image
     * @param arena_width Width of the World in mm
     */
    LightPattern(std::string img_src, double arena_width);
    /*!
     * Get a 10-bit light intensity (0-1023) in the given coordinates in World
     * space Position refers to the position in Kilobot world coordinates, where
     * (0, 0) is the bottom left.
     * @param x Robot x position (from left) in mm
     * @param y Robot y position (from bottom) in mm
     * @return 10-bit light value (matching kilobot API)
     */
    uint16_t get_ambientlight(double x, double y);
    /*!
     * Get the Internal Image representing the light pattern
     * @return Image of the light pattern (full color)
     */
    sf::Image get_light_pattern();
    /*!
     * Set the light pattern to a new image source file
     * @param img_src Filename (+location) of the new light source image
     */
    void set_light_pattern(std::string img_src);
};
} // namespace KiloSim
#endif