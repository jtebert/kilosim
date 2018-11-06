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
    float m_windowWidth = 1200;
    float m_windowHeight = 1200;
    std::vector<double> worldDim = world->getDimensions();
    m_scale = m_windowHeight / worldDim[1];

    //m_settings.antialiasingLevel = 32;

    m_window.create(sf::VideoMode(m_windowWidth, m_windowHeight), "KiloSim", sf::Style::Default, m_settings);
    m_background.setSize(sf::Vector2f(m_windowWidth, m_windowHeight));

    if (!m_lightPattern.loadFromFile("test-bg.png"))
    {
        printf("Texture error\n");
    }

    m_background.setTexture(&m_lightPattern);

    // Create the texture for the Kilobot robots once
    if (!m_robotTexture.create(RADIUS * 2 * m_scale, RADIUS * 2 * m_scale))
        printf("Failed to make robot texture\n");
    sf::CircleShape shape(RADIUS * m_scale);
    m_robotTexture.draw(shape);

    sf::RectangleShape line(sf::Vector2f(2, RADIUS * m_scale));
    line.setFillColor(sf::Color::Black);
    line.setPosition(RADIUS * m_scale - 1, 0);
    m_robotTexture.draw(line);
    // sf::Vertex line[] = {
    //     sf::Vertex(sf::Vector2f(RADIUS * m_scale, RADIUS * m_scale)),
    //     sf::Vertex(sf::Vector2f(RADIUS * 2 * m_scale, RADIUS * m_scale))};
    // m_robotTexture.draw(line, 2, sf::Lines);
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
    drawTime();

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

    sf::Sprite sprite;
    sprite.setOrigin(RADIUS * m_scale, RADIUS * m_scale);
    sprite.setTexture(m_robotTexture.getTexture());
    sprite.setColor(sf::Color(r->color[0] * 255, r->color[1] * 255, r->color[2] * 255));
    sprite.setPosition(sf::Vector2f(r->pos[0] * m_scale, r->pos[1] * m_scale));
    sprite.setRotation(r->pos[2] * 180 / PI - 90);

    m_window.draw(sprite);
}

void Viewer::drawTime()
{
    int t = m_world->getTime();
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

void Viewer::drawLightPattern()
{
    // TODO: Implement drawLightPattern()
}
} // namespace KiloSim