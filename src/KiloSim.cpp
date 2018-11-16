#include "KiloSim.h"
#include <stdio.h>
#include <stdlib.h>

// Implementation of Kilobot Arena/World

namespace KiloSim
{
World::World(double arena_width, double arena_height)
    : m_arena_width(arena_width), m_arena_height(arena_height)
{
    // TODO: Implement constructor without light_img_src
}
World::World(double arena_width, double arena_height, std::string light_pattern_src)
    : m_arena_width(arena_width), m_arena_height(arena_height)
{
    set_light_pattern(light_pattern_src);
}

World::~World()
{
    // TODO: Implement World destructor (any destructors, zB)
}

World::RobotPose::RobotPose() : x(0.0), y(0.0), theta(0.0) {}
World::RobotPose::RobotPose(double x, double y, double theta)
    : x(x),
      y(y),
      theta(theta) {}

void World::step()
{
    // Apply robot controller for all robots
    run_controllers();

    // Communication between all robot pairs
    communicate();

    // Compute potential movement for all robots
    PosesPtr newPoses = compute_next_step();

    // Check for collisions between all robot pairs
    std::shared_ptr<std::vector<int16_t>> collisions = find_collisions(newPoses);

    // And execute move if no collision
    // or turn if collision
    move_robots(newPoses, collisions);

    // Increment time
    m_tick++;
}

bool World::has_light_pattern()
{
    // If there is no light pattern, the image is empty (0x0)
    sf::Vector2u img_dim = m_light_pattern.getSize();
    return img_dim.x > 0 && img_dim.y > 0;
}

sf::Image &World::get_light_pattern()
{
    return m_light_pattern;
}

void World::set_light_pattern(std::string light_pattern_src)
{
    if (!m_light_pattern.loadFromFile(light_pattern_src))
    {
        printf("Failed to load light pattern\n");
    }
    // m_light_pattern.flipVertically();
}

uint16_t World::get_light(float x, float y)
{
    // Transform from world coordinates to image coordinates
    sf::Vector2u img_dim = m_light_pattern.getSize();
    double scale = (double)img_dim.x / m_arena_width;
    int x_in_img = x * scale;
    int y_in_img = y * scale;
    // Get the Color with the y-axis coordinate flip
    sf::Color c = m_light_pattern.getPixel(x_in_img, img_dim.y - y_in_img);
    // Convert the color from RGB to grayscale using approximate luminosity
    return (0.3 * c.r) + (0.59 * c.g) + (0.11 * c.b);
}

void World::add_robot(Robot *robot)
{
    robot->add_light_pattern(*m_light_pattern);
    m_robots.push_back(robot);
}

void World::remove_robot(Robot *robot)
{
    // TODO: Implement this
    printf("This does nothing right now");
}

void World::run_controllers()
{
    // TODO: Parallelize
    for (auto &r : m_robots)
    {
        if ((rand()) < (int)(m_prob_control_execute * RAND_MAX))
        {
            r->robot_controller();
        }
    }
}

void World::communicate()
{
    // TODO: Parallelize
    // TODO: Is the shuffling necessary? (I killed it)

    if (m_tick % m_comm_rate == 0)
    {
        //#pragma omp for
        for (auto &tx_r : m_robots)
        {
            // Loop over all transmitting robots
            void *msg = tx_r->get_message();
            if (msg)
            {
                for (auto &rx_r : m_robots)
                {
                    // Loop over receivers if transmitting robot is sending a message
                    if (rx_r != tx_r)
                    {
                        // Check communication range in both directions
                        // (due to potentially noisy communication range)
                        double dist = tx_r->distance(tx_r->pos[0], tx_r->pos[1], rx_r->pos[0], rx_r->pos[1]);
                        if (tx_r->comm_out_criteria(dist) &&
                            rx_r->comm_in_criteria(dist, msg))
                        {
                            rx_r->received();
                        }
                    }
                }
            }
        }
    }
} // namespace KiloSim

World::PosesPtr World::compute_next_step()
{
    // TODO: Implement compute_next_step (and maybe change from pointers)
    // TODO: Parallelize... eventually

    // Initialize the new positions to be returned
    std::vector<RobotPose> newPos;
    newPos.resize(m_robots.size());

    int i = 0;
    float dt = m_tick_delta_t;
    for (auto &r : m_robots)
    {
        double theta = r->pos[2];
        double x = r->pos[0];
        double y = r->pos[1];
        double temp_x = x;
        ;
        double temp_y = y;
        double temp_cos, temp_sin, phi;
        switch (r->motor_command)
        {
        case 1:
        { // forward
            //theta += r->motor_error * dt;
            double speed = r->forward_speed * dt;
            temp_x = speed * cos(theta) + r->pos[0];
            temp_y = speed * sin(theta) + r->pos[1];
            break;
        }
        case 2:
        { // CW rotation
            phi = -r->turn_speed * dt;
            theta += phi;
            temp_cos = RADIUS * cos(theta + 4 * PI / 3);
            temp_sin = RADIUS * sin(theta + 4 * PI / 3);
            temp_x = x + temp_cos - temp_cos * cos(phi) + temp_sin * sin(phi);
            temp_y = y + temp_sin - temp_cos * sin(phi) - temp_sin * cos(phi);
            break;
        }
        case 3:
        { // CCW rotation
            phi = r->turn_speed * dt;
            theta += phi;
            temp_cos = RADIUS * cos(theta + 2 * PI / 3);
            temp_sin = RADIUS * sin(theta + 2 * PI / 3);
            temp_x = x + temp_cos - temp_cos * cos(phi) + temp_sin * sin(phi);
            temp_y = y + temp_sin - temp_cos * sin(phi) - temp_sin * cos(phi);
            break;
        }
        }
        newPos[i] = RobotPose(temp_x, temp_y, wrap_angle(theta));
        i++;
    }
    return std::make_shared<std::vector<RobotPose>>(newPos);
}

std::shared_ptr<std::vector<int16_t>> World::find_collisions(PosesPtr newPos)
{
    // TODO: Parallelize

    // Check to see if motion causes robots to collide with their updated positions

    // 0 = no collision
    // -1 = collision w/ wall
    // other positive # = collision w/ robot of that ind;

    // Initialize the collisions to be returned
    std::vector<int16_t> collisions(m_robots.size(), 0);
    double r_x, r_y, distance;

    for (int r = 0; r < m_robots.size(); r++)
    {
        r_x = (*newPos)[r].x;
        r_y = (*newPos)[r].y;
        // Check for collisions with walls
        if (r_x <= RADIUS || r_x >= m_arena_width - RADIUS || r_y <= RADIUS || r_y >= m_arena_height - RADIUS)
        {
            // There's a collision with the wall.
            // Don't even bother to check for collisions with other robots
            collisions[r] = -1;
        }
        else
        {
            for (int c = 0; c < m_robots.size(); ++c)
            {
                // Check for collisions with other robots
                // Don't do repeat checks, unless the one you're checking against
                // had a wall collision (and therefore didn't check for robot collisions)
                if (r != c)
                {
                    distance = sqrt(pow(r_x - (*newPos)[c].x, 2) +
                                    pow(r_y - (*newPos)[c].y, 2));
                    if (distance < 2 * RADIUS)
                    {
                        collisions[r] = 1; // r is colliding with c
                        // Don't need to worry about more than 1 collision
                        break;
                    }
                }
            }
        }
    }
    return std::make_shared<std::vector<int16_t>>(collisions);
}

void World::move_robots(PosesPtr newPos, std::shared_ptr<std::vector<int16_t>> collisions)
{
    // TODO: Parallelize

    double newTheta;

    for (int ri = 0; ri < m_robots.size(); ++ri)
    {
        Robot *r = m_robots[ri];

        newTheta = (*newPos)[ri].theta;
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
                newTheta = r->pos[2] - r->turn_speed * m_tick_delta_t; // left/CCW
            }
            else
            {
                newTheta = r->pos[2] + r->turn_speed * m_tick_delta_t; // right/CW
            }
            if (r->collision_timer > r->max_collision_timer)
            { // Change turn dir
                r->collision_turn_dir = (r->collision_turn_dir + 1) % 2;
                r->collision_timer = 0;
            }
            r->collision_timer++;
            break;
        }
        }
        // If a bot is touching the wall (collision_type == 2), update angle but not position
        r->pos[2] = wrap_angle(newTheta);
    }
}

double World::wrap_angle(double angle)
{
    // Guarantee that angle will be from 0 to 2*pi
    // While loop is fastest option when angles are close to correct range
    // TODO: Should this actually not be in the World class?
    while (angle > 2 * M_PI)
    {
        angle -= 2 * M_PI;
    }
    while (angle < 0)
    {
        angle += 2 * M_PI;
    }
    return angle;
}

uint16_t World::get_tick_rate()
{
    return m_tick_rate;
}

uint32_t World::get_tick()
{
    return m_tick;
}

double World::get_time()
{
    return (double)m_tick / m_tick_rate;
}

std::vector<Robot *> &World::get_robots()
{
    return m_robots;
}
std::vector<double> World::get_dimensions()
{
    std::vector<double> dimensions{m_arena_width, m_arena_height};
    return dimensions;
}

} // namespace KiloSim