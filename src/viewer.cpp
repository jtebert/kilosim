/*
    KiloSim

    Visualizer for the Kilobot simulator to display robots and light pattern
    (OpenGL-based)

    Created 2018-11 by Julia Ebert
*/

#include "viewer.h"

namespace KiloSim
{
Viewer::Viewer(World *world) : m_world(world)
{
    float m_windowWidth = 600;
    float m_windowHeight = 800;
    std::vector<double> worldDim = world->getDimensions();
    m_scale = m_windowHeight / worldDim[1];

    m_window.create(sf::VideoMode(m_windowWidth, m_windowHeight), "KiloSim");
    m_background.setSize(sf::Vector2f(m_windowWidth, m_windowHeight));

    if (!m_lightPattern.loadFromFile("test-bg.png"))
    {
        printf("Texture error\n");
    }

    m_background.setTexture(&m_lightPattern);
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

    // TODO: Implement this
    for (auto &r : m_world->getRobots())
    {
        drawRobot(r);
        // r->draw();
    }

    m_window.display();
}

void Viewer::drawRobot(Robot *r)
{
    // TODO : Implement this
    // Maybe move this to robot.h (each robot responsible for determining its
    // own representation)
    sf::CircleShape shape(32 * m_scale);
    shape.setFillColor(sf::Color(r->color[0] * 255, r->color[1] * 255, r->color[2] * 255));
    m_window.draw(shape);
    // sf::Sprite sprite;
    // sprite.setColor(sf::Color(r->color[0], r->color[1], r->color[2]));
    // m_window.draw(sprite);
}

void Viewer::drawLightPattern()
{
    // TODO: Implement drawLightPattern()
}
} // namespace KiloSim