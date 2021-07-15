/*
    Kilosim

    Visualizer for the Kilobot simulator to display robots and light pattern
    (OpenGL-based)

    Created 2018-11 by Julia Ebert
*/

#include <kilosim/Viewer.h>

namespace Kilosim
{
Viewer::Viewer(World &world, const int window_width)
    : m_world(world), m_window_width(window_width)
{
    std::vector<double> world_dim = world.get_dimensions();
    m_scale = m_window_width / world_dim[0];
    m_window_height = world_dim[1] * m_scale;

    // m_settings.antialiasingLevel = 32;
    m_window.create(sf::VideoMode(m_window_width, m_window_height),
                    "Kilosim", sf::Style::Default, m_settings);
    m_window.setFramerateLimit(144);

    m_background.setSize(sf::Vector2f(m_window_width, m_window_height));
    if (world.has_light_pattern())
    {
        m_bg_texture.loadFromImage(world.get_light_pattern());
    }
    else
    {
        // If world doesn't have a light source texture, use blank black
        m_bg_texture.create(m_window_width, m_window_height);
    }
    m_background.setTexture(&m_bg_texture);
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

    // Draw the communication network, if enabled
    if (m_show_network) {
        draw_network();
    }

    for (auto &r : m_world.get_robots())
    {
        draw_robot(r);
    }

    m_window.display();

}

void Viewer::set_show_network(const bool show_network)
{
    m_show_network = show_network;
}

void Viewer::draw_network()
{
    std::vector<std::vector<int>> robot_edge_inds = m_world.get_network_edge_inds();
    std::vector<Robot *> robots = m_world.get_robots();
    for (auto i = 0; i < robot_edge_inds.size(); ++i)
    {
        if (robot_edge_inds[i][0] != -1 && robot_edge_inds[i][1] != -1)
        {
            Robot *robot0 = robots[robot_edge_inds[i][0]];
            Robot *robot1 = robots[robot_edge_inds[i][1]];
            std::vector<double> pos0 = {robot0->x, robot0->y};
            std::vector<double> pos1 = {robot1->x, robot1->y};
            if (pos0 != pos1) {
                sf::Vertex line[] = {
                    sf::Vertex(sf::Vector2f(pos0[0] * m_scale, m_window_height - (pos0[1] * m_scale))),
                    sf::Vertex(sf::Vector2f(pos1[0] * m_scale, m_window_height - (pos1[1] * m_scale)))
                };
                m_window.draw(line, 2, sf::Lines);
            }
    }
    }
}

void Viewer::draw_robot(Robot *r)
{
    // Maybe move this to Robot.h (each robot responsible for determining its
    // own representation)

    // Create the robot texture if it hasn't been done yet.
    // Texture is created based on the first robot drawn, but all robots should
    // be the same for the way that the whole simulator is structured
    double radius = r->get_radius();
    if (!m_is_robot_texture_valid)
    {
        // Create the texture for the Robots once
        if (!m_robot_texture.create(radius * 2 * m_scale, radius * 2 * m_scale)){
            printf("Failed to make robot texture\n");
        }
        sf::CircleShape shape(radius * m_scale);
        m_robot_texture.draw(shape);

        sf::RectangleShape line(sf::Vector2f(radius * m_scale, 2));
        line.setFillColor(sf::Color::Black);
        line.setPosition(radius * m_scale, radius * m_scale - 1);
        m_robot_texture.draw(line);
        m_is_robot_texture_valid = true;
    }

    sf::Sprite sprite;
    sprite.setOrigin(radius * m_scale, radius * m_scale);
    sprite.setTexture(m_robot_texture.getTexture());
    sprite.setColor(sf::Color(r->color[0] * 255,
                              r->color[1] * 255,
                              r->color[2] * 255));
    sprite.setPosition(sf::Vector2f(r->x * m_scale,
                                    m_window_height - (r->y * m_scale)));
    sprite.setRotation(r->theta * -180 / PI);

    m_window.draw(sprite);
}

void Viewer::draw_time()
{
    int t = m_world.get_time();
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
} // namespace Kilosim