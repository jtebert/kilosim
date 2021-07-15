#include <kilosim/World.h>
#include <kilosim/Random.h>

#include <stdexcept>

// Implementation of Kilobot Arena/World

namespace Kilosim
{
World::World(const double arena_width, const double arena_height,
             const std::string light_pattern_src, const uint32_t num_threads)
    : m_arena_width(arena_width), m_arena_height(arena_height),
      collision_boxes(arena_width, m_arena_height)
{
    if (light_pattern_src.size() > 0)
    {
        m_light_pattern.pattern_init(arena_width, light_pattern_src);
    }
    else
    {
        m_light_pattern.pattern_init(arena_width);
    }

#ifdef _OPENMP
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
#endif
}

World::~World()
{
    // TODO: Implement World destructor (any destructors, zB)
}

void World::step()
{
    timer_step.start();

    timer_step_memory.start();
    // Initialize vectors that are used in parallelism
    std::vector<RobotPose> new_poses((m_robots.size()));
    std::vector<int16_t> collisions(m_robots.size(), 0);
    timer_step_memory.stop();

    // Apply robot controller for all robots
    timer_controllers.start();
    run_controllers();
    timer_controllers.stop();

    // Communication between all robot pairs
    timer_communicate.start();
    communicate();
    timer_communicate.stop();

    // Compute potential movement for all robots
    timer_compute_next_step.start();
    compute_next_step(new_poses);
    timer_compute_next_step.stop();

    // Check for collisions between all robot pairs
    timer_collisions.start();
    find_collisions(new_poses, collisions);
    timer_collisions.stop();

    // And execute move if no collision
    // or turn if collision
    timer_move.start();
    move_robots(new_poses, collisions);
    timer_move.stop();

    // Increment time
    m_tick++;

    timer_step.stop();
}

sf::Image World::get_light_pattern() const
{
    return m_light_pattern.get_light_pattern();
}

bool World::has_light_pattern() const
{
    return m_light_pattern.has_source();
}

void World::set_light_pattern(std::string light_pattern_src)
{
    m_light_pattern.set_light_pattern(light_pattern_src);
}

void World::add_robot(Robot *robot)
{

    // If this is the first robot added, initialize the collision boxes using
    // its radius
    if (m_robots.size() == 0)
    {
        collision_boxes.init(m_arena_width, m_arena_height, 2 * robot->get_radius());
    }
    robot->add_to_world(m_light_pattern, m_arena_width, m_arena_height, m_tick_delta_t);
    m_robots.push_back(robot);
}

void World::remove_robot(Robot *robot)
{
    // TODO: Implement this
    printf("This does nothing right now");
}

void World::run_controllers()
{
    // #pragma omp parallel for default(none) //schedule(static)
    for (unsigned int i = 0; i < m_robots.size(); i++)
    {
        if (uniform_rand_real(0, 1) < m_prob_control_execute)
        {
            m_robots[i]->controller();
        }
    }
}

void World::communicate()
{
    
    if (m_tick % m_comm_rate == 0)
    {
        std::vector<int> tmp {-1, -1};
        std::fill(m_viewer_comm_lines.begin(), m_viewer_comm_lines.end(), tmp);
        int comm_lines_ind = 0;
        // #pragma omp parallel for
        for (unsigned int tx_i = 0; tx_i < m_robots.size(); tx_i++)
        {
            Robot &tx_r = *m_robots[tx_i];
            // Loop over all transmitting robots
            void *msg = tx_r.get_tx_message();
            if (msg)
            {
                for (unsigned int rx_i = 0; rx_i < m_robots.size(); rx_i++)
                {
                    Robot &rx_r = *m_robots[rx_i];
                    // Loop over receivers if transmitting robot is sending a message
                    if (rx_i != tx_i)
                    {
                        // Check communication range in both directions
                        // (due to potentially noisy communication range)
                        double dist = tx_r.distance(tx_r.x, tx_r.y, rx_r.x, rx_r.y);
                        // Only communicate if robots are within each others'
                        // communication ranges. (Range may be asymmetric/noisy)
                        if (tx_r.comm_criteria(dist) &&
                            rx_r.comm_criteria(dist))
                        {
                            // Receiving robot processes incoming message
                            rx_r.receive_msg(msg, dist);
                            // Tell the sender that the message sent successfully
                            tx_r.received();
                            if (tx_i < rx_i)
                            {
                                std::vector<int> pair = {(int)tx_i, (int)rx_i};
                                // (This makes sure each edge of the pair is only drawn once)
                                if (comm_lines_ind >= m_viewer_comm_lines.size())
                                {
                                    // This only extends vector when necessary
                                    m_viewer_comm_lines.push_back(pair);
                                } else {
                                    m_viewer_comm_lines[comm_lines_ind] = pair;
                                }
                                // std::cout << "Communication line between " << tx_i << " and " << rx_i << std::endl;
                                comm_lines_ind++;
                            }
                        }
                    }
                }
            }
        }
    }
}

void World::compute_next_step(std::vector<RobotPose> &new_poses)
{
    // TODO: Implement compute_next_step (and maybe change from pointers)

    // printf("\nt = %d\n", m_tick);
    // #pragma omp parallel for schedule(static)
    // #pragma omp parallel for
    for (unsigned int r_i = 0; r_i < m_robots.size(); r_i++)
    {
        new_poses[r_i] = m_robots[r_i]->robot_compute_next_step();
    }
}

void World::find_collisions(const std::vector<RobotPose> &new_poses, std::vector<int16_t> &collisions)
{
    // Check to see if motion causes robots to collide with their updated positions

    // 0 = no collision
    // -1 = collision w/ wall
    // 1 = collision w/ robot of that ind;

    //This updates a grid structure which enables robots to quickly identify
    //other robots with whom they might be colliding.
    collision_boxes.update(new_poses);

    //The following checks whether a robot is colliding with a wall or any other
    //robots. Only the collision status of the focal robot is changed. This
    //means that it is possible to accelerate the code by setting the status of
    //both of the robots in a collision; however, this requires careful thought
    //to ensure that wall collisions are still adequately accounted for. It also
    //reduces the potential for parallelism since it introduces a data race.

    // #pragma omp parallel for schedule(static)
    for (unsigned int ci = 0; ci < m_robots.size(); ci++)
    {
        const double radius = m_robots[ci]->get_radius();
        const auto &cr = new_poses[ci];
        // Check for collisions with walls
        if (cr.x <= radius ||
            cr.x >= m_arena_width - radius ||
            cr.y <= radius ||
            cr.y >= m_arena_height - radius)
        {
            // There's a collision with the wall.
            // Don't even bother to check for collisions with other robots
            collisions[ci] = -1;
            continue;
        }

        const auto func = [&](const unsigned int ni) -> bool {
            if (ci == ni)
                return true; //Look at more neighbours
            const auto &nr = new_poses[ni];
            const double distance = pow(cr.x - nr.x, 2) + pow(cr.y - nr.y, 2);

            //Check to see if robots' centers are within 2*radius of each other,
            //since that means their edges would be touching. But we actually
            //check (2*radius)^2 because we don't take the square root of the
            //distance above.
            if (distance < 4 * radius * radius)
            {
                collisions[ci] = 1;
                // Don't need to worry about more than 1 collision
                return false; //I'm done looking at neighbours
            }

            return true; //Look at more neighbours
        };

        collision_boxes.considerNeighbours(cr.x, cr.y, func);
    }

#ifdef CHECKSANE
    for (unsigned int ci = 0; ci < m_robots.size(); ci++)
    {
        const double radius = m_robots[ci]->get_radius();
        const auto &cr = new_poses[ci];
        for (unsigned int ni = ci + 1; ni < m_robots.size(); ni++)
        {
            const auto &nr = new_poses[ni];
            const double distance = pow(cr.x - nr.x, 2) + pow(cr.y - nr.y, 2);
            if (distance < 4 * radius * radius && (collisions[ni] == 0 || collisions[ci] == 0))
            {
                std::cerr << "Robots " << ci << " and " << ni << " overlap!" << std::endl;
                std::cerr << "collisions[" << ci << "] = " << collisions[ci] << std::endl;
                std::cerr << "collisions[" << ni << "] = " << collisions[ni] << std::endl;
                // std::cerr<<"Neighbours found: ";
                // for(const auto &x: cb(cr.x, cr.y))
                // std::cerr<<x<<" ";
                // std::cerr<<std::endl;
                throw std::runtime_error("Robots overlap in find_collisions!");
            }
        }
    }
#endif
}

void World::move_robots(std::vector<RobotPose> &new_poses,
                        const std::vector<int16_t> &collisions)
{
    // TODO: Parallelize
    // #pragma omp parallel for
    for (unsigned int ri = 0; ri < m_robots.size(); ri++)
    {
        m_robots[ri]->robot_move(new_poses[ri], collisions[ri]);
    }
}

void World::set_comm_rate(const uint32_t comm_rate)
{
    m_comm_rate = comm_rate;
}


uint16_t World::get_tick_rate() const
{
    return m_tick_rate;
}

uint32_t World::get_tick() const
{
    return m_tick;
}

double World::get_time() const
{
    return (double)m_tick / m_tick_rate;
}

std::vector<Robot *> &World::get_robots()
{
    return m_robots;
}

std::vector<double> World::get_dimensions() const
{
    std::vector<double> dimensions{m_arena_width, m_arena_height};
    return dimensions;
}

void World::printTimes() const
{
    std::cerr << "t timer_step              = " << timer_step.accumulated() << std::endl;
    std::cerr << "t timer_step_memory       = " << timer_step_memory.accumulated() << std::endl;
    std::cerr << "t timer_controllers       = " << timer_controllers.accumulated() << std::endl;
    std::cerr << "t timer_communicate       = " << timer_communicate.accumulated() << std::endl;
    std::cerr << "t timer_compute_next_step = " << timer_compute_next_step.accumulated() << std::endl;
    std::cerr << "t timer_collisions        = " << timer_collisions.accumulated() << std::endl;
    std::cerr << "t timer_move              = " << timer_move.accumulated() << std::endl;
}

std::vector<std::vector<int>> World::get_network_edge_inds() const
{
    return m_viewer_comm_lines;
}


void World::check_validity() const
{
    // Do any of the robots overlap with each other?
    for (unsigned int ci = 0; ci < m_robots.size(); ci++)
    {
        const auto &cr = *m_robots.at(ci);
        for (unsigned int ni = ci + 1; ni < m_robots.size(); ni++)
        {
            const auto &nr = *m_robots.at(ni);
            const double distance = pow(cr.x - nr.x, 2) + pow(cr.y - nr.y, 2);
            if (distance < 4 * m_robots[ci]->get_radius() * m_robots[ni]->get_radius())
                throw std::runtime_error("Found overlapping robots!");
        }
    }

    std::cerr << "World is valid." << std::endl;
}

} // namespace Kilosim