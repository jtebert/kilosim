#include "KiloSim.h"
#include <stdio.h>
#include <stdlib.h>

// Implementation of Kilobot Arena/World

namespace KiloSim
{
World::World(double arenaWidth, double arenaHeight) : m_arenaWidth(arenaWidth), m_arenaHeight(arenaHeight)
{
    // TODO: Implement constructor without lightImg
}
World::World(double arenaWidth, double arenaHeight, std::string lightImg) : m_arenaWidth(arenaWidth), m_arenaHeight(arenaHeight)
{
    // TODO: Implement constructor with lightImg
}

World::~World()
{
    // TODO: Implement World destructor (any destructors, zB)
}

World::RobotPose::RobotPose() : x(0.0), y(0.0), theta(0.0) {}
World::RobotPose::RobotPose(double x, double y, double theta) : x(x),
                                                                y(y),
                                                                theta(theta)
{
}

void World::step()
{
    // TODO: Implement world.step
    // should sort of come from main.cpp run_simulation_step()

    // Apply robot controller for all robots

    // Communication between all robot pairs

    // Compute potential movement for all robots
    PosesPtr newPoses = computeNextStep(m_tickDeltaT);

    // Check for collisions between all robot pairs
    // And execute move if no collision
    // or turn if collision

    // Increment time
    m_tick++;
}

bool World::hasLightPattern()
{
    return !m_lightPattern.data.empty();
}

void World::setLightPattern(std::string lightImg)
{
    // TODO: Implement setLightPattern
}

void World::addRobot(Robot *robot)
{
    m_robots.insert(robot);
}

void World::removeRobot(Robot *robot)
{
    m_robots.erase(robot);
}

void World::addLogger(Logger *logger)
{
    m_logger = logger;
}

void World::logState()
{
    if (m_logger != nullptr)
    {
        m_logger->logState(getTime(), m_robots);
    }
    else
    {
        printf("YOU SUCK\n");
    }
}

World::PosesPtr World::computeNextStep(double dt)
{
    // TODO: Implement computeNextStep (and maybe change from pointers)
    // TODO: Parallelize... eventually

    // Initialize the new positions to be returned
    std::vector<RobotPose> newPos;
    newPos.resize(m_robots.size());

    int i = 0;
    for (std::set<Robot *>::iterator ri = m_robots.begin(); ri != m_robots.end(); ++ri)
    {
        Robot *r = *ri;
        double x = r->pos[0];
        double y = r->pos[1];
        double theta = r->pos[2];
        double tmp_x, tmp_y;
        double tmp_cos, tmp_sin, phi;
        switch (r->motor_command)
        {
        case 1:
        { // forward
            double speed = r->forward_speed * dt;
            tmp_x = speed * cos(theta) + x;
            tmp_y = speed * sin(theta) + y;
            break;
        }
        case 2:
        { // CW rotation (around back right leg)
            phi = -r->turn_speed * dt;
            theta += phi;
            tmp_cos = radius * cos(theta + 4 * PI / 3);
            tmp_sin = radius * sin(theta + 4 * PI / 3);
            tmp_x = x + tmp_cos - tmp_cos * cos(phi) + tmp_sin * sin(phi);
            tmp_y = y + tmp_sin - tmp_cos * sin(phi) - tmp_sin * cos(phi);
            break;
        }
        case 3:
        { // CCW rotation (around back left leg)
            phi = r->turn_speed * dt;
            theta += phi;
            tmp_cos = radius * cos(theta + 2 * PI / 3);
            tmp_sin = radius * sin(theta + 2 * PI / 3);
            tmp_x = x + tmp_cos - tmp_cos * cos(phi) + tmp_sin * sin(phi);
            tmp_y = y + tmp_sin - tmp_cos * sin(phi) - tmp_sin * cos(phi);
            break;
        }
        }
        std::cout << tmp_x << std::endl;
        newPos[i] = RobotPose(tmp_x, tmp_y, wrapAngle(theta));
        i++;
    }
    return std::make_shared<std::vector<RobotPose>>(newPos);
}

bool World::findCollisions(PosesPtr newPos, int selfID, int time)
{
    // TODO: implement findCollisions (and switch from pointers)
    // TODO: Can make this go 2x faster by only checking if selfID < otherID and copying over
    // Check to see if motion causes robots to collide with their updated positions
    RobotPose selfPos = (*newPos)[selfID];
    RobotPose otherPos;
    double dist_x, dist_y, distance;

    // Check for collision with wall
    if (selfPos.x <= radius || selfPos.x >= m_arenaWidth - radius || selfPos.y <= radius || selfPos.y >= m_arenaHeight - radius)
    {
        return 2;
    }
    bool isCollided = false;

    for (int otherID = 0; otherID < m_robots.size(); otherID++)
    {
        if (otherID != selfID)
        { // Don't compare to self
            otherPos = (*newPos)[otherID];
            // Get distance to other robots
            dist_x = selfPos.x - otherPos.x;
            dist_y = selfPos.y - otherPos.y;
            distance = sqrt(pow(dist_x, 2) + pow(dist_y, 2));

            // Check if new positions are intersecting
            if (distance < 2 * radius)
            {
                return true;
            }
        }
    }
    return false;
}

double World::wrapAngle(double angle)
{
    // Guarantee that angle will be from 0 to 2*pi
    // While loop is fastest option when angles are close to correct range
    // TODO: Should this actually not be in the World class?
    while (angle > TWO_PI)
    {
        angle -= TWO_PI;
    }
    while (angle < 0)
    {
        angle += TWO_PI;
    }
    return angle;
}

void World::drawScene()
{
    // TODO: Implement drawScene
}

void World::setTickRate(uint16_t tickRate)
{
    m_tickRate = tickRate;
    m_tickDeltaT = 1.0 / m_tickRate;
}

uint16_t World::getTickRate()
{
    return m_tickRate;
}

uint32_t World::getTick()
{
    return m_tick;
}

double World::getTime()
{
    return (double)m_tick / m_tickRate;
}

} // namespace KiloSim