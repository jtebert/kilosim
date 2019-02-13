/*
    Kilosim

    Created 2018-11 by Julia Ebert
*/

#include "LightPattern.h"

namespace Kilosim
{
LightPattern::LightPattern()
{
    m_has_source = false;
};

void LightPattern::pattern_init(const double arena_width)
{
    m_arena_width = arena_width;
    m_has_source = false;
}
void LightPattern::pattern_init(const double arena_width, const std::string img_src)
{
    m_arena_width = arena_width;
    set_light_pattern(img_src);
}

uint16_t LightPattern::get_ambientlight(const double x, const double y) const
{
    if (m_has_source)
    {
        // Transform from world coordinates to image coordinates
        uint x_in_img = x * m_scale;
        uint y_in_img = y * m_scale;

        // Get the Color with the y-axis coordinate flip (each is 8-bit)
        sf::Color c = m_light_pattern.getPixel(x_in_img, m_img_dim.y - y_in_img - 1);
        // Convert the color from RGB to grayscale using approximate luminosity
        // Each value is 8-bit, so the resulting value is in the scale [0-255]
        double luminosity = (0.3 * c.r) + (0.59 * c.g) + (0.11 * c.b);
        // Scale to 10-bit [0-1023]
        uint16_t lum_10bit = (uint16_t)luminosity * 4;
        return lum_10bit;
    }
    else
    {
        return 0;
    }
};

sf::Image LightPattern::get_light_pattern() const
{
    return m_light_pattern;
};

bool LightPattern::has_source() const
{
    return m_has_source;
}

void LightPattern::set_light_pattern(const std::string img_src)
{
    if (!m_light_pattern.loadFromFile(img_src))
    {
        //std::cout << "ERROR: Failed to load light pattern: " << img_src << std::endl;
        exit(EXIT_FAILURE);
    }
    // Set scaling and image dimensions when new image loaded
    m_img_dim = m_light_pattern.getSize();
    m_scale = (double)m_img_dim.x / m_arena_width;
    m_has_source = true;
};
} // namespace Kilosim
