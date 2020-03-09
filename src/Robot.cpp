#include <kilosim/Robot.h>
#include <kilosim/Random.h>

#include <cmath>
#include <iostream>
#include <numeric>

namespace Kilosim
{
// void Robot::robot_controller()
// {
// 	// Run the user's code
// 	loop();
// 	// Run the robot control code
// 	controller()
// }

void Robot::robot_move(const RobotPose &new_pose, const int16_t &collision)
{
	// printf("ri=%d\n", ri);
	double new_theta = new_pose.theta;
	switch (collision)
	{
	case 0:
	{ // No collisions
		x = new_pose.x;
		y = new_pose.y;
		m_collision_timer = 0;
		break;
	}
	case 1:
	{ // Collision with another robot
		if (m_collision_turn_dir == 0)
		{
			new_theta = theta - m_collision_turn_speed * m_tick_delta_t; // left/CCW
		}
		else
		{
			new_theta = theta + m_collision_turn_speed * m_tick_delta_t; // right/CW
		}
		if (m_collision_timer > m_max_collision_timer)
		{ // Change turn dir
			m_collision_turn_dir = (m_collision_turn_dir + 1) % 2;
			m_collision_timer = 0;
		}
		m_collision_timer++;
		break;
	}
	}
	// If a bot is touching the wall (collision_type == 2), update angle but not position
	theta = wrap_angle(new_theta);
};

void Robot::robot_init(double x0, double y0, double theta0)
{
	// Pick a direction to randomly turn in event of collisions
	m_collision_turn_dir = uniform_rand_int(0, 1);
	m_collision_timer = 0;
	m_max_collision_timer = uniform_rand_int(10, 30) * SECOND;

	// Initialize pose
	x = x0;
	y = y0;
	theta = theta0;

	// Assign unique ID
	id = uniform_rand_int(0, 2147483640);

	// Run implementation-specific initialization
	init();
}

void Robot::add_to_world(LightPattern &light_pattern, const double dt)
{
	m_light_pattern = &light_pattern;
	m_tick_delta_t = dt;
}

double Robot::wrap_angle(double angle) const
{
	// Guarantee that angle will be from 0 to 2*pi
	// While loop is fastest option when angles are close to correct range
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

} // namespace Kilosim