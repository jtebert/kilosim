#include "KiloSim.h"
#include <stdio.h>
#include <stdlib.h>

// Implementation of Kilobot Arena/World

namespace KiloSim
{
World::World(double arena_width, double arena_height, std::string light_pattern_src, uint num_threads)
    : m_arena_width(arena_width), m_arena_height(arena_height)
{
    if (light_pattern_src.size() > 0)
    {
        m_light_pattern.pattern_init(arena_width, light_pattern_src);
    }
    else
    {
        m_light_pattern.pattern_init(arena_width);
    }
    // OpenMP settings
    if (num_threads != 0)
    {
        // Explicitly disable dynamic teams
        omp_set_dynamic(0);
        // Use num_threads for all consecutive parallel regions
        omp_set_num_threads(num_threads);
    }
    else
    {
        omp_set_dynamic(1);
    }
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
    // Initialize vectors that are used in parallelism
    std::vector<RobotPose> new_poses((m_robots.size()));
    // PosesPtr new_poses_ptr = std::make_shared<std::vector<RobotPose>>(new_poses);
    std::vector<int16_t> collisions(m_robots.size(), 0);

    // Apply robot controller for all robots
    run_controllers();

    // Communication between all robot pairs
    communicate();

    // Compute potential movement for all robots
    compute_next_step(&new_poses);

    // Check for collisions between all robot pairs
    find_collisions(&new_poses, &collisions);

    // And execute move if no collision
    // or turn if collision
    move_robots(&new_poses, &collisions);

    // Increment time
    m_tick++;
}

sf::Image World::get_light_pattern()
{
    return m_light_pattern.get_light_pattern();
}

bool World::has_light_pattern()
{
    return m_light_pattern.has_source();
}

void World::set_light_pattern(std::string light_pattern_src)
{
    m_light_pattern.set_light_pattern(light_pattern_src);
}

void World::add_robot(Robot *robot)
{
    robot->add_light(&m_light_pattern);
    m_robots.push_back(robot);
}

void World::remove_robot(Robot *robot)
{
    // TODO: Implement this
    printf("This does nothing right now");
}

void World::run_controllers()
{
    // #pragma omp parallel for schedule(static)
    for (int i = 0; i < m_robots.size(); i++)
    {
        if ((rand()) < (int)(m_prob_control_execute * RAND_MAX))
        {
            m_robots[i]->robot_controller();
        }
    }
}

void World::communicate()
{
    // TODO: Is the shuffling necessary? (I killed it)

    if (m_tick % m_comm_rate == 0)
    {
        // #pragma omp parallel for
        for (int tx_i = 0; tx_i < m_robots.size(); tx_i++)
        {
            Robot *tx_r = m_robots[tx_i];
            // Loop over all transmitting robots
            void *msg = tx_r->get_message();
            if (msg)
            {
                for (int rx_i = 0; rx_i < m_robots.size(); rx_i++)
                {
                    Robot *rx_r = m_robots[rx_i];
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
}

void World::compute_next_step(std::vector<RobotPose> *new_poses_ptr)
{
// TODO: Implement compute_next_step (and maybe change from pointers)

// printf("\nt = %d\n", m_tick);
// #pragma omp parallel for schedule(static)
#pragma omp parallel for
    for (int r_i = 0; r_i < m_robots.size(); r_i++)
    {
        // printf("%d\n", r_i);
        Robot *r = m_robots[r_i];
        double theta = r->pos[2];
        double x = r->pos[0];
        double y = r->pos[1];
        double temp_x = x;

        double temp_y = y;
        double temp_cos, temp_sin, phi;
        switch (r->motor_command)
        {
        case 1:
        { // forward
            //theta += r->motor_error * m_tick_delta_t;
            double speed = r->forward_speed * m_tick_delta_t;
            temp_x = speed * cos(theta) + r->pos[0];
            temp_y = speed * sin(theta) + r->pos[1];
            break;
        }
        case 2:
        { // CW rotation
            phi = -r->turn_speed * m_tick_delta_t;
            theta += phi;
            temp_cos = RADIUS * cos(theta + 4 * PI / 3);
            temp_sin = RADIUS * sin(theta + 4 * PI / 3);
            temp_x = x + temp_cos - temp_cos * cos(phi) + temp_sin * sin(phi);
            temp_y = y + temp_sin - temp_cos * sin(phi) - temp_sin * cos(phi);
            break;
        }
        case 3:
        { // CCW rotation
            phi = r->turn_speed * m_tick_delta_t;
            theta += phi;
            temp_cos = RADIUS * cos(theta + 2 * PI / 3);
            temp_sin = RADIUS * sin(theta + 2 * PI / 3);
            temp_x = x + temp_cos - temp_cos * cos(phi) + temp_sin * sin(phi);
            temp_y = y + temp_sin - temp_cos * sin(phi) - temp_sin * cos(phi);
            break;
        }
        }
        (*new_poses_ptr)[r_i] = RobotPose(temp_x, temp_y, wrap_angle(theta));
    }
}

void World::find_collisions(std::vector<RobotPose> *new_poses_ptr, std::vector<int16_t> *collisions)
{
    // Check to see if motion causes robots to collide with their updated positions

    // 0 = no collision
    // -1 = collision w/ wall
    // 1 = collision w/ robot of that ind;

    // #pragma omp parallel for schedule(static)
    for (int r = 0; r < m_robots.size(); r++)
    {
        double distance;
        double r_x = (*new_poses_ptr)[r].x;
        double r_y = (*new_poses_ptr)[r].y;
        // Check for collisions with walls
        if (r_x <= RADIUS || r_x >= m_arena_width - RADIUS || r_y <= RADIUS || r_y >= m_arena_height - RADIUS)
        {
            // There's a collision with the wall.
            // Don't even bother to check for collisions with other robots
            (*collisions)[r] = -1;
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
                    distance = sqrt(pow(r_x - (*new_poses_ptr)[c].x, 2) +
                                    pow(r_y - (*new_poses_ptr)[c].y, 2));
                    if (distance < 2 * RADIUS)
                    {
                        (*collisions)[r] = 1; // r is colliding with c
                        // Don't need to worry about more than 1 collision
                        break;
                    }
                }
            }
        }
    }
    // return std::make_shared<std::vector<int16_t>>(collisions);
}

void World::move_robots(std::vector<RobotPose> *new_poses_ptr, std::vector<int16_t> *collisions)
{
    // TODO: Parallelize
    // #pragma omp parallel for
    for (int ri = 0; ri < m_robots.size(); ri++)
    {
        // printf("ri=%d\n", ri);
        Robot *r = m_robots[ri];

        double new_theta = (*new_poses_ptr)[ri].theta;
        switch ((*collisions)[ri])
        {
        case 0:
        { // No collisions
            r->pos[0] = (*new_poses_ptr)[ri].x;
            r->pos[1] = (*new_poses_ptr)[ri].y;
            r->collision_timer = 0;
            break;
        }
        case 1:
        { // Collision with another robot
            if (r->collision_turn_dir == 0)
            {
                new_theta = r->pos[2] - r->turn_speed * m_tick_delta_t; // left/CCW
            }
            else
            {
                new_theta = r->pos[2] + r->turn_speed * m_tick_delta_t; // right/CW
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
        r->pos[2] = wrap_angle(new_theta);
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