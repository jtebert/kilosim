/*
    KiloSim

    Visualizer for the Kilobot simulator to display robots and light pattern
    (OpenGL-based)

    Created 2018-11 by Julia Ebert
*/

#include "Viewer.h"

namespace KiloSim
{
Viewer::Viewer(World *world, int window_width) : m_world(world), m_window_width(window_width)
{
    std::vector<double> world_dim = world->get_dimensions();
    m_scale = m_window_width / world_dim[0];
    m_window_height = world_dim[1] * m_scale;

    //m_settings.antialiasingLevel = 32;
    m_window.create(sf::VideoMode(m_window_width, m_window_height),
                    "KiloSim", sf::Style::Default, m_settings);
    m_window.setFramerateLimit(144);

    m_background.setSize(sf::Vector2f(m_window_width, m_window_height));
    if (world->has_light_pattern())
    {
        m_bg_texture.loadFromImage(world->get_light_pattern());
    }
    else
    {
        // If world doesn't have a light source texture, use blank black
        m_bg_texture.create(m_window_width, m_window_height);
    }
    m_background.setTexture(&m_bg_texture);

    // Create the texture for the Kilobot robots once
    if (!m_robot_texture.create(RADIUS * 2 * m_scale, RADIUS * 2 * m_scale))
        printf("Failed to make robot texture\n");
    sf::CircleShape shape(RADIUS * m_scale);
    m_robot_texture.draw(shape);

    sf::RectangleShape line(sf::Vector2f(RADIUS * m_scale, 2));
    line.setFillColor(sf::Color::Black);
    line.setPosition(RADIUS * m_scale, RADIUS * m_scale - 1);
    m_robot_texture.draw(line);
}

void Viewer::draw()
{
    if (m_window.isOpen())
    {
        sf::Event event;
        while (m_window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                m_window.close();
            }
        }
    }

    m_window.clear();

    // Draw world's lightPattern
    m_window.draw(m_background);
    draw_time();

    // TODO: Implement this
    for (auto &r : m_world->get_robots())
    {
        draw_robot(r);
        // r->draw();
    }

    m_window.display();
}

void Viewer::draw_robot(Robot *r)
{
    // TODO : Implement this
    // Maybe move this to Robot.h (each robot responsible for determining its
    // own representation)

    sf::Sprite sprite;
    sprite.setOrigin(RADIUS * m_scale, RADIUS * m_scale);
    sprite.setTexture(m_robot_texture.getTexture());
    sprite.setColor(sf::Color(r->color[0] * 255, r->color[1] * 255, r->color[2] * 255));
    sprite.setPosition(sf::Vector2f(r->pos[0] * m_scale, m_window_height - (r->pos[1] * m_scale)));
    sprite.setRotation(r->pos[2] * -180 / PI);

    m_window.draw(sprite);
}

void Viewer::draw_time()
{
    int t = m_world->get_time();
    int hour = t / 3600;
    t = t % 3600;
    int minute = t / 60;
    t = t % 60;
    int second = t;

    char buff[100];
    snprintf(buff, sizeof(buff), "%.2d:%.2d:%.2d", hour, minute, second);
    std::string timeStr = buff;
    m_window.setTitle(timeStr);
}

void Viewer::draw_light_pattern()
{
    // TODO: Implement draw_light_pattern()
}
} // namespace KiloSim