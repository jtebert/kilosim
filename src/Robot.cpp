#include <cmath>
#include <iostream>
#include <numeric>
#include "Robot.h"
#include "random.hpp"

namespace Kilosim
{
void Robot::robot_controller()
{
	if (battery > 0)
	{
		timer++;
		// Run the Kilobot functionality: set sending/receiving messages, setting motor states, and running loop() function
		controller();
		switch (m_motor_command)
		{
		case 1:
		{
			battery -= .5;
			break;
		}
		case 2 | 3:
		{
			battery -= .5;
			break;
		}
		}
	}
	else
	{
		// Robot is dead. Stop movement and don't let it do anything
		m_forward_speed = 0;
		m_turn_speed = 0;
		m_motor_command = 4;
		color[0] = .3;
		color[1] = .3;
		color[2] = .3;
		tx_request = 0;
	}
}

RobotPose Robot::robot_compute_next_step() const
{
	double temp_x = x;
	double temp_y = y;
	double temp_theta = theta;
	switch (m_motor_command)
	{
	case 1:
	{ // forward
		const double speed = m_forward_speed * m_tick_delta_t;
		temp_x = speed * cos(temp_theta) + x;
		temp_y = speed * sin(temp_theta) + y;
		break;
	}
	case 2:
	{ // CW rotation
		const double phi = -m_turn_speed * m_tick_delta_t;
		temp_theta += phi;
		const double temp_cos = RADIUS * cos(temp_theta + 4 * PI / 3);
		const double temp_sin = RADIUS * sin(temp_theta + 4 * PI / 3);
		temp_x = x + temp_cos - temp_cos * cos(phi) + temp_sin * sin(phi);
		temp_y = y + temp_sin - temp_cos * sin(phi) - temp_sin * cos(phi);
		break;
	}
	case 3:
	{ // CCW rotation
		const double phi = m_turn_speed * m_tick_delta_t;
		temp_theta += phi;
		const double temp_cos = RADIUS * cos(temp_theta + 2 * PI / 3);
		const double temp_sin = RADIUS * sin(temp_theta + 2 * PI / 3);
		temp_x = x + temp_cos - temp_cos * cos(phi) + temp_sin * sin(phi);
		temp_y = y + temp_sin - temp_cos * sin(phi) - temp_sin * cos(phi);
		break;
	}
	}
	return {temp_x, temp_y, wrap_angle(temp_theta)};
}

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
			new_theta = theta - m_turn_speed * m_tick_delta_t; // left/CCW
		}
		else
		{
			new_theta = theta + m_turn_speed * m_tick_delta_t; // right/CW
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
	// Initialize robot variables
	x = x0;
	y = y0;
	theta = theta0;

	m_motor_command = 0;
	incoming_message_flag = 0;
	tx_request = 0;
	id = uniform_rand_int(0, 2147483640);
	// Generate CLAMPED motor error (avoid extremes by regenerating)
	m_motor_error = 100;
	double motor_error_clamp = motion_error_std * 1.1;
	while (abs(m_motor_error) > motor_error_clamp)
	{
		m_motor_error = normal_rand(0.0, 1.0) * motion_error_std;
	}
	// Add random variation to forward/turn speeds
	double turn_speed_error = 100;
	double turn_speed_error_std = m_turn_speed * 0.1; // 5% of turn speed
	double turn_speed_error_clamp = turn_speed_error_std * 1.1;
	while (abs(turn_speed_error) > turn_speed_error_clamp)
	{
		turn_speed_error = normal_rand(0.0, 1.0) * turn_speed_error_std;
	}
	m_turn_speed = m_turn_speed + turn_speed_error;
	double forward_speed_error = 100;
	double forward_speed_error_std = m_forward_speed * 0.1; // 5% of turn speed
	double forward_speed_error_clamp = forward_speed_error_std * 1.1;
	while (abs(forward_speed_error) > forward_speed_error_clamp)
	{
		forward_speed_error = normal_rand(0.0, 1.0) * forward_speed_error_std;
	}
	m_forward_speed = m_forward_speed + forward_speed_error;
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