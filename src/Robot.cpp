#include <math.h>
#include <iostream>
#include <numeric>
#include "Robot.h"

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
		color[1] = .3;
		tx_request = 0;
	}
}

std::vector<double> Robot::robot_compute_next_step() const
{
	double theta = pos[2];
	double x = pos[0];
	double y = pos[1];
	double temp_x = x;

	double temp_y = y;
	double temp_cos, temp_sin, phi;
	switch (m_motor_command)
	{
	case 1:
	{ // forward
		double speed = m_forward_speed * m_tick_delta_t;
		temp_x = speed * cos(theta) + pos[0];
		temp_y = speed * sin(theta) + pos[1];
		break;
	}
	case 2:
	{ // CW rotation
		phi = -m_turn_speed * m_tick_delta_t;
		theta += phi;
		temp_cos = RADIUS * cos(theta + 4 * PI / 3);
		temp_sin = RADIUS * sin(theta + 4 * PI / 3);
		temp_x = x + temp_cos - temp_cos * cos(phi) + temp_sin * sin(phi);
		temp_y = y + temp_sin - temp_cos * sin(phi) - temp_sin * cos(phi);
		break;
	}
	case 3:
	{ // CCW rotation
		phi = m_turn_speed * m_tick_delta_t;
		theta += phi;
		temp_cos = RADIUS * cos(theta + 2 * PI / 3);
		temp_sin = RADIUS * sin(theta + 2 * PI / 3);
		temp_x = x + temp_cos - temp_cos * cos(phi) + temp_sin * sin(phi);
		temp_y = y + temp_sin - temp_cos * sin(phi) - temp_sin * cos(phi);
		break;
	}
	}
	return {temp_x, temp_y, wrap_angle(theta)};
}

void Robot::robot_move(const std::vector<double> &new_pose, const int16_t &collision)
{
	// printf("ri=%d\n", ri);
	double new_theta = new_pose[2];
	switch (collision)
	{
	case 0:
	{ // No collisions
		pos[0] = new_pose[0];
		pos[1] = new_pose[1];
		m_collision_timer = 0;
		break;
	}
	case 1:
	{ // Collision with another robot
		if (m_collision_turn_dir == 0)
		{
			new_theta = pos[2] - m_turn_speed * m_tick_delta_t; // left/CCW
		}
		else
		{
			new_theta = pos[2] + m_turn_speed * m_tick_delta_t; // right/CW
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
	pos[2] = wrap_angle(new_theta);
};

void Robot::robot_init(double x, double y, double theta)
{
	// Pick a direction to randomly turn in event of collisions
	m_collision_turn_dir = rand() % 2;
	m_max_collision_timer = (uint32_t)rand() % (20 * SECOND) + (10 * SECOND) + 1; // Max duration 10-30 seconds
	// Initialize robot variables
	pos[0] = x;
	pos[1] = y;
	pos[2] = theta;

	m_motor_command = 0;
	incoming_message_flag = 0;
	tx_request = 0;
	id = rand();
	// Generate CLAMPED motor error (avoid extremes by regenerating)
	m_motor_error = 100;
	double motor_error_clamp = motion_error_std * 1.1;
	while (abs(m_motor_error) > motor_error_clamp)
	{
		timer = rand() / 100;
		m_motor_error = Robot::gauss_rand(timer) * motion_error_std;
	}
	// Add random variation to forward/turn speeds
	double turn_speed_error = 100;
	double turn_speed_error_std = m_turn_speed * 0.1; // 5% of turn speed
	double turn_speed_error_clamp = turn_speed_error_std * 1.1;
	while (abs(turn_speed_error) > turn_speed_error_clamp)
	{
		timer = rand() / 100;
		turn_speed_error = Robot::gauss_rand(timer) * turn_speed_error_std;
	}
	m_turn_speed = m_turn_speed + turn_speed_error;
	double forward_speed_error = 100;
	double forward_speed_error_std = m_forward_speed * 0.1; // 5% of turn speed
	double forward_speed_error_clamp = forward_speed_error_std * 1.1;
	while (abs(forward_speed_error) > forward_speed_error_clamp)
	{
		timer = rand() / 100;
		forward_speed_error = Robot::gauss_rand(timer) * forward_speed_error_std;
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