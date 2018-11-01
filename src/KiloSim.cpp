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
    std::shared_ptr<std::vector<uint8_t>> collisions = findCollisions(newPoses);
    // And execute move if no collision
    // or turn if collision
    moveRobots(newPoses, collisions);

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
    m_robots.push_back(robot);
}

void World::removeRobot(Robot *robot)
{
    // TODO: Implement this
    printf("This did nothing");
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
    for (std::vector<Robot *>::iterator ri = m_robots.begin(); ri != m_robots.end(); ++ri)
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

std::shared_ptr<std::vector<uint8_t>> World::findCollisions(PosesPtr newPos)
{
    // TODO: Parallelize

    // Check to see if motion causes robots to collide with their updated positions

    // 0 = no collision; 1 = collision w/ robot; 2 = collision w/ wall

    // Initialize the collisions to be returned
    std::vector<uint8_t> collisions(m_robots.size(), 0);
    double r_x, r_y, distance;

    for (uint r = 0; r < m_robots.size(); ++r)
    {
        r_x = (*newPos)[r].x;
        r_y = (*newPos)[r].y;
        // Check for collisions with walls
        if (r_x <= radius || r_x >= m_arenaWidth - radius || r_y <= radius || r_y >= m_arenaHeight - radius)
        {
            // There's a collision with the wall. Don't even bother to check
            // for collisions with other robots
            collisions[r] = 2;
            break;
        }
        for (uint c = 0; c < m_robots.size(); ++c)
        {
            // Check for collisions with other robots
            // Don't do repeat checks, unless the one you're checking against
            // had a wall collision (and therefore didn't check for robot collisions)
            if (r < c || collisions[c] == 2)
            {
                distance = sqrt(pow(r_x - (*newPos)[c].x, 2) +
                                pow(r_y - (*newPos)[c].y, 2));
                if (distance < 2 * radius)
                {
                    collisions[r] = 1;
                }
            }
            else if (r < c)
            {
                // Collisions are symmetric
                collisions[r] = collisions[c];
            }
        }
    }
    return std::make_shared<std::vector<uint8_t>>(collisions);
}

void World::moveRobots(PosesPtr newPos, std::shared_ptr<std::vector<uint8_t>> collisions)
{
    // TODO: Parallelize

    double new_theta;

    for (int ri = 0; ri < m_robots.size(); ++ri)
    {
        Robot *r = m_robots[ri];
        new_theta = (*newPos)[ri].theta;
        switch ((*collisions)[ri])
        {
        case 0:
        { // No collisions
            r->pos[0] = (*newPos)[ri].x;
            r->pos[1] = (*newPos)[ri].y;
            r->collision_timer = 0;
            break;
        }
        case 1:
        { // Collision with another robot
            if (r->collision_turn_dir == 0)
            {
                new_theta = r->pos[2] - r->turn_speed * m_tickDeltaT; // left/CCW
            }
            else
            {
                new_theta = r->pos[2] + r->turn_speed * m_tickDeltaT; // right/CW
            }
            if (r->collision_timer > r->max_collision_timer)
            { // Change turn dir
                r->collision_turn_dir = (r->collision_turn_dir + 1) % 2;
                r->collision_timer = 0;
            }
            r->collision_timer++;
            break;
        }
            // If a bot is touching the wall (collision_type == 2), update angle but not position
            r->pos[2] = wrapAngle(new_theta);
        }
    }
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